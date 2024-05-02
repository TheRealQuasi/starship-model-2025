/*
* This is the main file for the Starship model. 
* It contains the setup and loop functions, as well as the functions for initializing and reading the sensors.
* 
* By Emlzdev (Emil Reinfeldt), 
* GunnarEdman (Gunnar Edman)
*/

// =============================================================================================
//  Preprocessor Definitions
// =============================================================================================

#include <Arduino.h>
#include <Wire.h>
#include <Dps3xx.h>
#include <TFMPI2C.h>  // TFMini-Plus I2C Library v1.7.3
#include <settings.h>
#include "RF24.h"
#include "GlobalDecRocket.h"
#include "RadioTransceiverMaster.h"
#include "motorsAndServos.h"
#include "IMU.h"
#include "Barometer.h"
#include "PID_ang_new.h"
#include "PID_altitude_new.h"
#include "LQR.h"
#include <SD.h>



// =============================================================================================
//  Variables/Objects
// =============================================================================================

// Store data on SD card
File dataFile;

// Indicateds if there is new data to be read from the radio
bool newControllerData = false;

// Create a Packet to hold the data
PacketData senderData;

// Acknowledge payload to hold the data coming from the rocket
ControlData ackData;

// Store sensor data
SensorData sensorData;

// Store control data
LqrSignals lqrSignals;


// ========= Status variables =========
bool escCalibrationStatus = false;  // Boolean that informs if ESC calibration is performed or not

// ========= IMU =========
// IMU object
// Object handeling everything with the BMI088 paired with a madgwick filter
Imu6DOF imu;

// ========= Lidar =========
TFMPI2C tfmP;         // Create a TFMini-Plus I2C object

// Variables storing data from TFmini plus
float zMeter = 0 ;
int16_t lidarZ = 0;       // Distance to object in centimeters
int16_t lidarFlux = 0;       // Signal strength or quality of return signal
int16_t lidarTemp = 0;       // Internal temperature of Lidar sensor chip

float zPrev = 0;       // Distance to object in centimeters

// Servo control variables
float xGimb = 0;
float yGimb = 0;
float motorSpeed = 1140;


// Time variables
float t0;
float tTerminate;
float t0Lqr;
float t1Lqr;
float t0Lidar;
float t1Lidar;
float tCheck0 = 0;
float tCheck1 = 0;

// Delta-states
float xDot = 0;
float yDot = 0;
float zDot = 0;

// ======== SD Card =========
// SD file
String sdFile = "";
// Buffer for storing data
String dataBuffer = "";
// Maximum size of the buffer
const int BUFFER_SIZE = 1000;
// Time interval for writing to the SD card (in milliseconds)
const unsigned long WRITE_INTERVAL = 10000;
// Time of the last write operation
unsigned long lastWriteTime = 0;


// =============================================================================================
//  Functions
// =============================================================================================

// Print the data from the ackData object
void printAckData(){
  if(newControllerData){
    Serial.println("Data from ground control: ");
    Serial.print("  Throttle: ");
    Serial.println(ackData.thrustSlider);
    Serial.print("  X: ");
    Serial.println(ackData.lxAxisValue);
    Serial.print("  Y: ");
    Serial.println(ackData.lyAxisValue);
    Serial.print("  Armed: ");
    Serial.println(ackData.armSwitch);
    newControllerData = false;
  }
}

// Initialize the SD card
void initSD(){
  if (!SD.begin(BUILTIN_SDCARD)) {
    #ifdef DEBUG
      Serial.println("SD: Card failed, or not present");
    #endif
    // Don't do anything more
  }

  // Find a unique filename
  int counter = 0;
  String filename;
  do {
    counter++;
    filename = "rocketData" + String(counter) + ".csv";
  } while (SD.exists(filename.c_str()));

  // Create a new data file with a unique name
  dataFile = SD.open(filename.c_str(), FILE_WRITE);
  if (!dataFile) {
    #ifdef DEBUG
      Serial.println("SD: Failed to create data file");
    #endif
  }

  sdFile = filename;
}

void write2SD(){
  // Add the current data point to the buffer
  dataBuffer += String(senderData.timeStamp) + "," +
                // State variables (x)
                String(senderData.xDot) + "," +
                String(senderData.roll) + "," +
                String(senderData.rollDot) + "," +
                String(senderData.yDot) + "," +
                String(senderData.pitch) + "," +
                String(senderData.pitchDot) + "," +
                String(senderData.z) + "," +
                String(senderData.zDot) + "," +
                // Control reference values
                String(senderData.zRef) + "," +
                String(senderData.zDotRef) + "," +
                // Control output values
                String(senderData.motorSpeed) + "," +
                String(senderData.gimb1) + "," +
                String(senderData.gimb2) + "\n";

  // If the buffer is full or the write interval has passed, write the buffer to the SD card
  if (dataBuffer.length() >= BUFFER_SIZE || millis() - lastWriteTime >= WRITE_INTERVAL) {
    // Open the file. Note that only one file can be open at a time,
    // so you have to close this one before opening another.
    dataFile = SD.open(sdFile.c_str(), FILE_WRITE);

    // If the file is available, write to it:
    if (dataFile) {
      dataFile.print(dataBuffer);
      dataFile.close();
    }
    // If the file isn't open, pop up an error:
    else {
      #ifdef DEBUG
        Serial.println("error opening rocketData.csv");
      #endif
    }

    // Clear the buffer
    dataBuffer = "";

    // Update the time of the last write operation
    lastWriteTime = millis();
  }
}

void redLedWarning() {
  for (int i=0; i<BLINK_COUNT; i++) {
    digitalWrite(RED_LED_PIN, HIGH);
    delay(250);
    digitalWrite(RED_LED_PIN, LOW);
    delay(250);
  }
}


// =============================================================================================
//  Main Program
// =============================================================================================

void setup() {
  // Initialize serial communication for debugging
  #ifdef DEBUG
    Serial.begin(BAUDRATE); 
    while (!Serial) {
      ; // wait for serial port to connect. Needed for native USB
    }
    
    Serial.println("==== Starship model initializing... ====");
    Serial.println("");
    Serial.println("Initializing I2C bus...");
  #endif

  // Serial.begin(BAUDRATE); 

  // Initialize the SD card
  initSD();

  // =================== Radio setup =====================

  // Initialize radio module
  #ifndef DISABLE_COM
    initRadio(RF24_PA_LEVEL, RF24_SPEED, RF24_CHANNEL);
  #endif

  // =================== Servo and motor setup ===================

  // Configure digital input for calButton
  pinMode(CAL_BUTTON, INPUT_PULLUP);

  // Configure digital putput for red led
  pinMode(RED_LED_PIN, OUTPUT);
  
  // Initialize servos and ESCs (motors)
  #ifndef DISABLE_COM
    transmitState(SERVO_AND_MOTOR_INIT, ackData);
  #endif

  initServosMotors();

  // ESC calibration phase
  #ifndef DISABLE_COM
    transmitState(ESC_CALIBRATION, ackData);  //Transmit ESC calibration phase message
  #endif

  // Calibrate the ESCs throttle range once calButton has been pressed for at least 2 seconds
  waitESCCalCommand(escCalibrationStatus);
  
  // Gimbal test phase
  #ifndef DISABLE_COM
    transmitState(GIMBAL_TEST, ackData);   //Transmit gimbal test phase message
  #endif

  redLedWarning();
  gimbalTest();

  redLedWarning();
  // motorTest();

  lqrInit();


  // =============== Sensor setup ===============

  // Initialize I2C bus
  Wire.begin();

  // Initialize IMU (needs to happend in the end, to allow for continous IMU sampling)
  imu.init();

  // IMU calibration phase
  #ifndef DISABLE_COM
    transmitState(IMU_CALIBRATION, ackData);    // Transmit IMU calibration phase message
  #endif
  imu.calibrate();

  // Filter warmup phase
  #ifndef DISABLE_COM
    transmitState(FILTER_WARMUP, ackData);    // Transmit filter warmup phase message
  #endif

  // Allow the madgwick filter to start converging on an estimate before flight
  imu.filterWarmup();

  // // Barometer sensor setup
  // if(!initDPS310()){
  //   #ifdef DEBUG
  //     Serial.println("Failed to initialize the pressure sensor. Check the wiring and try again.");
  //   #endif
  // }

  // Initialize the senderData object
  senderData = {0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  // Initialize the ackData object
  ackData.armSwitch = false;
  ackData.calButton = 0;
  ackData.thrustSlider = 0;
  ackData.lxAxisValue = 0;
  ackData.lyAxisValue = 0;

  // Init lqr singal struct
  lqrSignals = {0.0, 0.0, 0.0, 0.0, 0.0};

  #ifdef DEBUG
  Serial.println("Init complete!");
  #endif

  #ifndef DISABLE_COM
    transmitState(SYSTEM_READY, ackData);
  #endif

  // LED arm indication
  // To do! <<<<<<<<<<<<<<<<<<<<<<<<<---------------------------------

  // Arm rocket
  // #ifdef DEBUG
  Serial.print("\n\n Entering main loop - Rocket armed!!!!");
  // #endif
  redLedWarning();
  ackData.armSwitch = true;

  gimbalTest();

  // Set start time
  t0 = micros();
  tTerminate = micros();

  t0Lqr = micros();
  t1Lqr = micros();

  t0Lidar = micros();
  t1Lidar = micros();

  tCheck0 = micros();
  tCheck1 = micros();
}


// =============================================================================================
// =============== MAIN LOOP ===============
// =============================================================================================


void loop() {
  // =============== Time / frequency management =================
  tCheck0 = micros();
  imu.timeUpdate();                         // Record time at start of loop iteration (used in madgwick filters)
  digitalWrite(RED_LED_PIN, HIGH);

  // =============== Safety checks =================
  // Termination timer
  if (tTerminate - t0 >= TIME_LIMIT * 1000) {
    motorsWrite(1100, ackData);
    #ifdef DEBUG
      Serial.print("\nABORT!!!!!!!");
    #endif
    ackData.armSwitch = false;
    digitalWrite(RED_LED_PIN, LOW);
    delay(200000);
  }
  else {
    tTerminate = micros();
  }
  
  // Check arm switch
  // #ifndef DISABLE_COM
  if (!ackData.armSwitch) {
    // motorsWrite(1100, ackData);
    digitalWrite(RED_LED_PIN, LOW);
  } 
  else {
    digitalWrite(RED_LED_PIN, HIGH);
  }
  // #endif

  // Check 
  #ifdef DISABLE_COM
    if (!digitalRead(CAL_BUTTON)) {
      ackData.armSwitch = false;
      motorsWrite(1100, ackData);
      delay(100000);    
    }
  #endif

  // ======================================================
  // ================ Sensors and filters =================
  // ======================================================

  // Read sensors and filter data
  imu.update();                         // Get IMU data and filter it with LP / smoothing and Madgwick-filters
  // Preliminary, rough estimations of the missing states
  // To-do (kalman estimator)
  xDot = (imu.AccX + imu.AccX_prev) * imu.dt;
  yDot = (imu.AccY + imu.AccY_prev) * imu.dt;


  // Get lidar data (100 Hz)
  if (t1Lidar - t0Lidar >= 10000) {
    float dtLidar = (t1Lidar - t0Lidar) / 1000000;
    zPrev = zMeter;
    tfmP.getData(lidarZ, lidarFlux, lidarTemp);    // Get a frame of data from the TFmini
    zMeter = float(lidarZ) * 0.01;
    t0Lidar = micros();
    t1Lidar = micros();

    // Preliminary, rough estimations of the missing states
    // To-do (kalman estimator)az
    zDot = ((zMeter - zPrev) / dtLidar); //imu.dt;

    // Store new state values in senderData struct
    senderData.xDot      = xDot;
    senderData.roll      = imu.roll_IMU;
    senderData.rollDot   = imu.GyroX;
    senderData.yDot      = yDot;
    senderData.pitch     = imu.pitch_IMU; 
    senderData.pitchDot  = imu.GyroY; 
    senderData.z         = zMeter;
    senderData.zDot      = zDot;
  }
  else {
    t1Lidar = micros();
  }

  // Read barometer data
  /* float psReturn = readPS();
  if (psReturn == (-2)){
    #ifdef DEBUG
      Serial.println("No data to be found yet. Please wait...");
    #endif
  }
  else if (psReturn == (-1)){
    #ifdef DEBUG
      Serial.println("Error reading the pressure sensor. Please check the wiring and try again.");
    #endif
  }
  else{
    #ifdef DEBUG
      Serial.print("Height: ");
      Serial.print(psReturn);
      Serial.println(" m");
    #endif

    sensorData.psHeight = psReturn;
  } */


  // ===========================================
  // ================ Control ==================
  // ===========================================

  // =============== PID =================
  // PID altitude
  // motorSpeed = altitude_pid(zMeter*0.01, ALT_REF);
  // Serial.println(motorSpeed);

  // PID angle 
  // yGimb = angleControlY(imu.pitch_IMU, imu.roll_IMU, -MAX_GIMBAL, MAX_GIMBAL);
  // xGimb = angleControlX(imu.pitch_IMU, imu.roll_IMU, -MAX_GIMBAL, MAX_GIMBAL);

  // PID actuation
  // motorsWrite(motorSpeed, ackData);

  // =============== LQR control =================
  
  // Regulate at predefined frequency
  if (t1Lqr - t0Lqr >= (1/CONTROLLER_FREQUENCY) * 1000000) {
    #ifdef DEBUG
      Serial.print("\n z = ");
      Serial.print(zMeter - 0.10);

      Serial.print("\t");
      Serial.print("(Xr, Yr): ");
      Serial.print(imu.roll_IMU);
      Serial.print("  ");
      Serial.print(imu.pitch_IMU);
      Serial.print("  ");
    #endif


    // Calculate current time (passed since t0)
    float currentTime = (micros() - t0) / 1000000;
    senderData.timeStamp = micros() - t0;

    lqr(xDot, imu.roll_IMU, imu.GyroX, yDot, imu.pitch_IMU, imu.GyroY, zMeter, zDot, currentTime, lqrSignals);
    xGimb = lqrSignals.gimb1;
    yGimb = lqrSignals.gimb2;
    
    // Actuation
    // Serial.print("----------------------------------------------------------------");
    // Serial.print("\n");
    // Serial.print("\n");
    // Serial.print("motorSpeed: ");
    // Serial.print(lqrSignals.motorSpeed);
    // Serial.print("\n");
    // Serial.print("\n");

    #ifdef DEBUG
      Serial.print("  g1: ");
      Serial.print(xGimb);
      Serial.print("  g2: ");
      Serial.print(yGimb);

      Serial.print("   PWM: ");
      Serial.print(lqrSignals.motorSpeed);
      Serial.print("\t ");
      Serial.print("t: ");
      Serial.print(currentTime);
      Serial.print("   dt: ");
      Serial.print(t1Lqr - t0Lqr);
      
    #endif

    motorsWrite(lqrSignals.motorSpeed, ackData);
    setServo1Pos(-xGimb);
    setServo2Pos(-yGimb);

    // Store control inputs
    senderData.zRef = lqrSignals.zRef;
    senderData.zDotRef = lqrSignals.zDotRef;
    
    // Store control outputs in senderData struct
    senderData.motorSpeed = lqrSignals.motorSpeed;
    senderData.gimb1 = lqrSignals.gimb1;
    senderData.gimb2 = lqrSignals.gimb2;

    // Log to SD-card at 100 Hz
    // write2SD();

    t0Lqr = micros();
    t1Lqr = micros();
  }
  else {
    t1Lqr = micros();
  }


  // Ground control
  // --------------
  // Send the data to the ground controller via radio
  // currentMillis = millis();
  // if (currentMillis - prevMillis >= txIntervalMillis) {
  //   if (!transmitData(radio, senderData, ackData, newControllerData, prevMillis)) {
  //     // Connection lost to the ground controller
  //     // Set flag and try reconnecting in the next loop
  //     // TODO: Initialize landing script and dearm the rocket
  //     #ifdef DEBUG
  //       Serial.println("Connection lost to the ground controller. Trying to reconnect...");
  //     #endif
  //   }
  //   else {
  //     //Print data from ackData
  //     #ifdef DEBUG
  //       printAckData();
  //     #endif
  //   }
  //   prevMillis = currentMillis;
  // }


  #ifndef DISABLE_COM
    transmitFlightData(senderData, ackData);
  #endif

  tCheck1 = micros();
  if((tCheck1 - tCheck0) > 500) {
    Serial.print("\n ****************************************** \n Loop too slow: ");
    Serial.print(tCheck1 - tCheck0);
    Serial.print("Microseconds \n \n");
  }

  // #ifdef DEBUG
  //   if((tCheck1 - tCheck0) > 500) {
  //     Serial.print("\n ****************************************** \n Loop too slow: ");
  //     Serial.print(tCheck1 - tCheck0);
  //     Serial.print("Microseconds \n \n");
  //   }
  // #endif

  // Regulate looprate to predefined loop frequency (the teeensy runs much faster then what is suitable for this)
  imu.loopRate();     // <<<<<<<<<<<<<<<------------------------------------------------------------------------------ To do (Gunnar): Tweak this to prevent lag when transmitting data etc.
  
}