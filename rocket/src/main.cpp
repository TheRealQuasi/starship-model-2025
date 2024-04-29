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

int16_t zPrev = 0;       // Distance to object in centimeters

// Servo control variables
float xGimb = 0;
float yGimb = 0;
float motorSpeed = 1140;


// Time variables
float t0;
float tTerminate;
float t0Lqr;
float t1Lqr;
float tCheck0 = 0;
float tCheck1 = 0;

// Delta-states
float xDot = 0;
float yDot = 0;
float zDot = 0;

// SD file
String sdFile = "";


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
  // Open the file. Note that only one file can be open at a time,
  // so you have to close this one before opening another.
  dataFile = SD.open(sdFile.c_str(), FILE_WRITE);

  // If the file is available, write to it:
  if (dataFile) {
    // Time stamp
    dataFile.print(senderData.timeStamp);
    dataFile.print(",");

    // State variables (x)
    dataFile.print(senderData.xDot);
    dataFile.print(",");
    dataFile.print(senderData.roll);
    dataFile.print(",");
    dataFile.print(senderData.rollDot);
    dataFile.print(",");
    dataFile.print(senderData.yDot);
    dataFile.print(",");
    dataFile.print(senderData.pitch);
    dataFile.print(",");
    dataFile.print(senderData.pitchDot);
    dataFile.print(",");
    dataFile.print(senderData.z);
    dataFile.print(",");
    dataFile.print(senderData.zDot);
    dataFile.print(",");

    // Control reference values
    dataFile.print(senderData.zRef);
    dataFile.print(",");
    dataFile.print(senderData.zDotRef);
    dataFile.print(",");


    // Control output values
    dataFile.print(senderData.motorSpeed);
    dataFile.print(",");
    dataFile.print(senderData.gimb1);
    dataFile.print(",");
    dataFile.println(senderData.gimb2);
    dataFile.close();
  }
  // If the file isn't open, pop up an error:
  else {
    #ifdef DEBUG
      Serial.println("error opening rocketData.csv");
    #endif
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
  motorTest();

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
  redLedWarning();
  ackData.armSwitch = true;

  gimbalTest();

  // Set start time
  t0 = micros();
  tTerminate = micros();

  t0Lqr = micros();
  t1Lqr = micros();

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
    motorsWrite(1100, ackData);
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
  imu.update();                          // Get IMU data and filter it with LP / smoothing and Madgwick-filters
  zPrev = zMeter;
  tfmP.getData(lidarZ, lidarFlux, lidarTemp);    // Get a frame of data from the TFmini
  zMeter = float(lidarZ) * 0.01;

  #ifdef DEBUG
    Serial.print("Altitude = ");
    Serial.print(zMeter);   
    // Serial.print("\t");
    // Serial.print(lidarFlux);   
  #endif

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

  // Preliminary, rough estimations of the missing states
  // To-do (kalman estimator)
  xDot = (imu.AccX + imu.AccX_prev) * imu.dt;
  yDot = (imu.AccY + imu.AccY_prev) * imu.dt;
  zDot = (zMeter - zPrev)/imu.dt;

  // Store new state values in senderData struct
  senderData.xDot      = xDot;
  senderData.roll      = imu.roll_IMU;
  senderData.rollDot   = imu.GyroX;
  senderData.yDot      = yDot;
  senderData.pitch     = imu.pitch_IMU; 
  senderData.pitchDot  = imu.GyroY; 
  senderData.z         = zMeter;
  senderData.zDot      = zDot;

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
    // Calculate current time (passed since t0)
    float currentTime = (micros() - t0) * pow(10.0, -6);
    senderData.timeStamp = micros() - t0;

    lqr(xDot, imu.roll_IMU, imu.GyroX, yDot, imu.pitch_IMU, imu.GyroY, zMeter, zDot, currentTime, lqrSignals);
    
    // Actuation
    Serial.print("----------------------------------------------------------------");
    Serial.print("\n");
    Serial.print("Time: ");
    Serial.print(currentTime);
    Serial.print("\n");
    Serial.print("motorSpeed: ");
    Serial.print(lqrSignals.motorSpeed);
    Serial.print("\n");
    Serial.print("\n");
        

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
    write2SD();

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
  #ifdef DEBUG
    if((tCheck1 - tCheck0) / pow(10, 6) > 0.00051) {
      Serial.print(tCheck1 - tCheck0);
    }
  #endif

  // Regulate looprate to predefined loop frequency (the teeensy runs much faster then what is suitable for this)
  imu.loopRate();     // <<<<<<<<<<<<<<<------------------------------------------------------------------------------ To do (Gunnar): Tweak this to prevent lag when transmitting data etc.
  
}