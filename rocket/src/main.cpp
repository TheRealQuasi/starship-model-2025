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
#include "RollControl.h"



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
float zMeter = 0;
float zCalibration = 0;
int16_t lidarZ = 0;       // Distance to object in centimeters
int16_t lidarFlux = 0;       // Signal strength or quality of return signal
int16_t lidarTemp = 0;       // Internal temperature of Lidar sensor chip

float zPrev = 0;       // Distance to object in centimeters

// Servo control variables
float xGimb = 0;
float yGimb = 0;
float motorSpeed = 1140;


// Timing variables
unsigned long t0;
unsigned long tTerminate;
unsigned long t0Lqr;
unsigned long t1Lqr;
unsigned long t0Lidar;
unsigned long t1Lidar;
unsigned long tCheck0 = 0;
unsigned long tCheck1 = 0;
unsigned long t0IMU = 0;
unsigned long t1IMU = 0;

// Inverse sampling frequencies
unsigned long madgwickFrekvInv = 0;
unsigned long controllerFrekvInv = 0;
unsigned long imuSampleInv = 0;

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
unsigned int bufferSize = 56000;//500000; //1000;
// Time interval for writing to the SD card (in milliseconds)
const unsigned long WRITE_INTERVAL = TIME_LIMIT-1;
// Time of the last write operation
unsigned long lastWriteTime = 0;

// Debug variables
float maxDeltaT = 0; // Maximum recorded iteration step time [us]


// =============================================================================================
//  Functions
// =============================================================================================

// // Barometer sampling
// void getBarometer() {
//   // Read barometer data
//   float psReturn = readPS();
//   if (psReturn == (-2)){
//     #ifdef DEBUG
//       Serial.println("No data to be found yet. Please wait...");
//     #endif
//   }
//   else if (psReturn == (-1)){
//     #ifdef DEBUG
//       Serial.println("Error reading the pressure sensor. Please check the wiring and try again.");
//     #endif
//   }
//   else{
//     #ifdef DEBUG
//       Serial.print("Height: ");
//       Serial.print(psReturn);
//       Serial.println(" m");
//     #endif

//     sensorData.psHeight = psReturn;
//   }
// }

void getLidar() {
  float dtLidar = (t1Lidar - t0Lidar) / 1000000;
  zPrev = zMeter;
  // tfmP.getData(lidarZ, lidarFlux, lidarTemp);    // Get a frame of data from the TFmini
  tfmP.getData(lidarZ);    // Get a frame of data from the TFmini
  if(tfmP.status == TFMP_CHECKSUM){
    lidarZ = tfmP.frame[ 2] + ( tfmP.frame[ 3] << 8);
  }

  zMeter = float(float(lidarZ) * float(0.01)) - zCalibration;

  if (zMeter < 0.00) {
    zMeter = 0.00;
  }
  
  // Get distance to ground
  // if (abs(imu.roll_IMU) < 80 && abs(imu.pitch_IMU) < 80) {
  //   zMeter = sqrt(zMeter * zMeter * (1 - pow( sin(imu.roll_IMU), 2 ) - pow( sin(imu.pitch_IMU), 2 )) );
  // }
  // Preliminary, rough estimations of the missing states
  // To-do (kalman estimator)
  zDot = ((zMeter - zPrev) / dtLidar);
}


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

  bufferSize = sizeof(senderData) * TIME_LIMIT / 10;
  Serial.println("Buffersize: " + String(bufferSize));
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
  if (dataBuffer.length() >= bufferSize) {
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

void redLedWarningV2() {
  t0 = micros();
  unsigned long t1 = micros();
  t0IMU = micros();  
  t1IMU = micros();  

  bool state = HIGH;
  
  int c = 1;

  while (c < 5) {
    // LED blink
    if (t1 - t0 >= 250000) {
      digitalWrite(RED_LED_PIN, state);

      if (state) {
        state = LOW;
      } 
      else {
        if (c >= 5) {
          break;
        }
        else {
          state = HIGH;
          c++;
        }
      }
    }

    // Maintain attitude estimation
    if (t1IMU - t0IMU >= imuSampleInv) {
      imu.sample();
    }

    // Madgwick step    
    imu.timeUpdate();                         // Record time at start of loop iteration (used in madgwick filters)
    imu.madgwickStep();
    #ifdef LOOP_RATE
      imu.loopRate();
    #endif
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

  // =================== Radio setup =====================

  // Initialize radio module
  #ifndef DISABLE_COM
    initRadio(RF24_PA_LEVEL, RF24_SPEED, RF24_CHANNEL);
  #endif

  // =================== Servo and motor setup ===================

  #ifdef MOTORS_SERVOS
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
  #endif

  lqrInit();

  // Initialize the SD card
  initSD();


  // =============== Sensor setup ===============
  // Inverse sampling frequencies for timing (in microseconds)
  madgwickFrekvInv = (1 / MADGWICK_FREQUENCY) * 1000000;
  controllerFrekvInv = (1 / CONTROLLER_FREQUENCY) * 1000000;
  imuSampleInv = (1 / IMU_SAMPLE_FREQUENCY) * 1000000;

  // Initialize I2C bus
  // Wire.setSpeed(I2C_CLOCKSPEED);
  Wire.begin();
  Wire.setClock(I2C_CLOCKSPEED);

  // Lidar calibration (measure offset to ground at standstill)
  tfmP.getData(lidarZ, lidarFlux, lidarTemp);    // Get a frame of data from the TFmini
  if(tfmP.status == TFMP_CHECKSUM){
    lidarZ = tfmP.frame[ 2] + ( tfmP.frame[ 3] << 8);
  }
  zMeter = float(lidarZ) * 0.01;
  zCalibration = zMeter;

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
  lqrSignals = {0.0, 0.0, 0, 0, 0.0, 0.0};

  // Allow the madgwick filter to start converging on an estimate before flight
  // (This needs to happen without other uninteruptions right before entering the loop)
  imu.filterWarmup();

  #ifdef DEBUG
    Serial.println("Init complete!");
  #endif

  #ifndef DISABLE_COM
    transmitState(SYSTEM_READY, ackData);
  #endif

  // Arm rocket
  #ifdef DEBUG
    Serial.print("\n\n Entering main loop - Rocket armed!!!!");
  #endif
  
  // Red blinking to warn before startup
  // redLedWarningV2();

  ackData.armSwitch = true;

  // Set start time
  t0 = micros();
  tTerminate = micros();

  t0Lqr = micros();
  t1Lqr = micros();

  t0Lidar = micros();
  t1Lidar = micros();

  tCheck0 = micros();
  tCheck1 = micros();

  t0IMU = micros();
  t1IMU = micros();
}


// =============================================================================================
// =============== MAIN LOOP ===============
// =============================================================================================


void loop() {
  // =============== Time / frequency management =================
  tCheck0 = micros();

  // =============== Safety checks =================
  // Termination timer
  if (tTerminate - t0 >= TIME_LIMIT * 1000) {
    motorsWrite(1, 1100, ackData);
    motorsWrite(2, 1100, ackData);
    #ifdef DEBUG
      Serial.print("\nABORT!!!!!!!");
    #endif
    ackData.armSwitch = false;
    digitalWrite(RED_LED_PIN, LOW);

    //write2SD();

    // Print largest deltaT in main loop
    Serial.print("\n\n\n =================================== \n Biggest delta T (bellow 10 ms): ");
    Serial.print(maxDeltaT);
    Serial.print(" [us] \n \n");

    // Do nothing until the teensy is reset
    delay(1000000);

    // Infinite loop with do nothing (arduino can't do exit(0) since the loop() is infinite)
    while(1) {}    
  }
  else {
    tTerminate = micros();
  }
  
  // // Check arm switch
  // if (!ackData.armSwitch) {
  //   digitalWrite(RED_LED_PIN, LOW);
  // } 
  // else {
  //   digitalWrite(RED_LED_PIN, HIGH);
  // }

  // // Check emergency stop
  // #ifdef DISABLE_COM
  //   if (!digitalRead(CAL_BUTTON)) {
  //     ackData.armSwitch = false;
  //     motorsWrite(1, 1100, ackData);
  //     motorsWrite(2, 1100, ackData);

  //     // Do nothing until the teensy is reset
  //     delay(1000000);

  //     // Infinite loop with do nothing (arduino can't do exit(0) since the loop() is infinite)
  //     while(0 == 0) {}    
  //   }
  // #endif

  // ======================================================
  // ================ Sensors and filters =================
  // ======================================================
  
  if (t1IMU - t0IMU >= imuSampleInv) {
    // Read IMU
    unsigned long t0Madg = micros();
    imu.sample();
    unsigned long t1Madg = micros();

    Serial.print("\n IMU sample took: ");
    Serial.print(t1Madg - t0Madg);

    t0IMU = micros();
    t1IMU = micros();
  }
  else {
    t1IMU = micros();
  }

  // Madgwick iteration
  imu.timeUpdate();                         // Record time at start of loop iteration (used in madgwick filters)

  imu.madgwickStep();

  // Preliminary, rough estimations of the missing states
  // To-do (kalman estimator)
  xDot = (imu.AccX + imu.AccX_prev) * imu.dt;
  yDot = (imu.AccY + imu.AccY_prev) * imu.dt;


  // Get lidar data (100 Hz)
  if (t1Lidar - t0Lidar >= 10000) {
    unsigned long t0Lid = micros();
    getLidar();
    unsigned long t1Lid = micros();

    Serial.print("\n Lidar sample took: ");
    Serial.print(t1Lid - t0Lid);

  }
  else {
    t1Lidar = micros();
  }

  // getBarometer();

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
  
  // Run control-update at predefined frequency
  if (t1Lqr - t0Lqr >= controllerFrekvInv) {
    #ifdef DEBUG
      Serial.print("\n z = ");
      Serial.print(zMeter);

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
    
    #ifdef DEBUG
      Serial.print("  g1: ");
      Serial.print(xGimb);
      Serial.print("  g2: ");
      Serial.print(yGimb);

      Serial.print("   PWM: ");
      Serial.print(lqrSignals.motor1Speed);
      Serial.print("\t ");
      Serial.print("t: ");
      Serial.print(currentTime);
      Serial.print("   dt: ");
      Serial.print(t1Lqr - t0Lqr);
      
    #endif

    motorsWrite(1, lqrSignals.motor1Speed, ackData);
    #ifdef ROLLCONTROLLER
      motorsWrite(2, roll_p_controller(imu.yaw_IMU, lqrSignals.motor2Speed), ackData);
    #endif

    #ifndef ROLLCONTROLLER
      motorsWrite(2, lqrSignals.motor1Speed, ackData);
    #endif

    setServo1Pos(-xGimb);
    setServo2Pos(-yGimb);

    //Store new state values in senderData struct
    senderData.xDot      = xDot;
    senderData.roll      = imu.roll_IMU;
    senderData.rollDot   = imu.GyroX;
    senderData.yDot      = yDot;
    senderData.pitch     = imu.pitch_IMU; 
    senderData.pitchDot  = imu.GyroY; 
    senderData.z         = zMeter;
    senderData.zDot      = zDot;

    // Store control inputs
    senderData.zRef = lqrSignals.zRef;
    senderData.zDotRef = lqrSignals.zDotRef;
    
    // Store control outputs in senderData struct
    senderData.motorSpeed = lqrSignals.motor1Speed;
    senderData.gimb1 = lqrSignals.gimb1;
    senderData.gimb2 = lqrSignals.gimb2;

    // Log to SD-card at 100 Hz
    //write2SD();

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

  // Calculate loop iteration deltaT and record max value if too big
  #ifdef DEBUG
    tCheck1 = micros();
    float bigDeltaT = tCheck1 - tCheck0;
    if((bigDeltaT) > madgwickFrekvInv) {
      if (bigDeltaT > maxDeltaT && bigDeltaT < 10000) {
        maxDeltaT = bigDeltaT;
      }
      
        // Serial.print("\n ****************************************** \n Loop too slow: ");
        // Serial.print(bigDeltaT);
        // Serial.print(" [us] \n \n");
    }
  #endif

  // Regulate looprate to predefined loop frequency (the teeensy runs much faster then what is suitable for this)
  #ifdef LOOP_RATE
    imu.loopRate();     // <<<<<<<<<<<<<<<------------------------------------------------------------------------------ To do (Gunnar): Tweak this to prevent lag when transmitting data etc.
  #endif
}

