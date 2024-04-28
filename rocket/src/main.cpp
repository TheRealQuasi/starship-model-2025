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
LQR_outputs lqrOutputs;


// ========= Status variables =========
bool escCalibrationStatus = false;  // Boolean that informs if ESC calibration is performed or not

// ========= IMU =========
// IMU object
// Object handeling everything with the BMI088 paired with a madgwick filter
Imu6DOF imu;

// ========= Lidar =========
TFMPI2C tfmP;         // Create a TFMini-Plus I2C object

// Variables storing data from TFmini plus
int16_t lidarZ = 0;       // Distance to object in centimeters
int16_t lidarFlux = 0;       // Signal strength or quality of return signal
int16_t lidarTemp = 0;       // Internal temperature of Lidar sensor chip

int16_t zPrev = 0;       // Distance to object in centimeters

// Servo control variables
float xGimb = 0;
float yGimb = 0;
float motorSpeed = 1140;


// Time variables
int t0;
int t1;
int tLqr;

// Delta-states
float xDot = 0;
float yDot = 0;
float zDot = 0;


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

void write2SD(){
  // Open the file. Note that only one file can be open at a time,
  // so you have to close this one before opening another.
  dataFile = SD.open("rocketData.csv", FILE_WRITE);

  // If the file is available, write to it:
  if (dataFile) {
    dataFile.print(senderData.timeStamp);
    dataFile.print(",");
    dataFile.print(senderData.posXValue);
    dataFile.print(",");
    dataFile.print(senderData.posYValue);
    dataFile.print(",");
    dataFile.print(senderData.posZValue);
    dataFile.print(",");
    dataFile.print(senderData.accXValue);
    dataFile.print(",");
    dataFile.print(senderData.accYValue);
    dataFile.print(",");
    dataFile.print(senderData.accZValue);
    dataFile.print(",");
    dataFile.print(senderData.gamValue);
    dataFile.print(",");
    dataFile.print(senderData.accGamValue);
    dataFile.print(",");
    dataFile.print(senderData.betaValue);
    dataFile.print(",");
    dataFile.println(senderData.accBetaValue);
    dataFile.close();
  }
  // If the file isn't open, pop up an error:
  else {
    #ifdef DEBUG
      Serial.println("error opening data.csv");
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

  // =================== Radio setup =====================

  // Initialize radio module
  #ifndef DISABLE_COM
    initRadio(RF24_PA_LEVEL, RF24_SPEED, RF24_CHANNEL);
  #endif

  // =================== Servo and motor setup ===================

  // Configure digital input for calButton
  pinMode(CAL_BUTTON, INPUT_PULLUP);

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

  gimbalTest();

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
  senderData.timeStamp = 0;
  senderData.posXValue = 0.0;
  senderData.posYValue = 0.0;
  senderData.posZValue = 0.0;
  senderData.accXValue = 0.0;
  senderData.accYValue = 0.0;
  senderData.accZValue = 0.0;
  senderData.gamValue = 0.0;
  senderData.accGamValue = 0.0;
  senderData.betaValue = 0.0;
  senderData.accBetaValue = 0.0;

  // Initialize the ackData object
  // ackData.armSwitch = 0;
  ackData.calButton = 0;
  ackData.thrustSlider = 0;
  ackData.lxAxisValue = 0;
  ackData.lyAxisValue = 0;

  #ifdef DEBUG
  Serial.println("Init complete!");
  #endif

  SD.begin(BUILTIN_SDCARD);

  #ifndef DISABLE_COM
    transmitState(SYSTEM_READY, ackData);
  #endif

  ackData.armSwitch = true;

  gimbalTest();

  // Set start time
  t0 = millis();
  t1 = millis();

  tLqr = millis();

}


// =======================================
// ===============MAIN LOOP===============
// =======================================


void loop() {
  // Time management

  Serial.print("\t");
  Serial.print(t1);

  if (t1 - t0 >= TIME_LIMIT) {
    motorsWrite(1100, ackData);
    Serial.print("\nABORT!!!!!!!");
    delay(100000);
  }
  else {
    t1 = millis();
  }
  
  #ifndef DISABLE_COM
    if (!ackData.armSwitch) {
      motorsWrite(1100, ackData);
    }
  #endif

  #ifdef DISABLE_COM
    if (!digitalRead(CAL_BUTTON)) {
      ackData.armSwitch = false;
      motorsWrite(1100, ackData);
      delay(100000);    
    }
  #endif

  imu.timeUpdate();                         // Record time at start of loop iteration (used in madgwick filters)

  // Read sensors and filter data
  imu.update();                          // Get IMU data and filter it with LP / smoothing and Madgwick-filters
  zPrev = lidarZ;
  tfmP.getData(lidarZ, lidarFlux, lidarTemp);    // Get a frame of data from the TFmini
  

  #ifdef DEBUG
    Serial.print("Altitude = ");
    Serial.print(lidarZ);   
  #endif


  // ===========================================
  // ================ Control ==================
  // ===========================================

  // =============== PID =================
  // PID altitude
  // motorSpeed = altitude_pid(lidarZ*0.01, ALT_REF);
  // Serial.println(motorSpeed);

  // PID angle 
  // yGimb = angleControlY(imu.pitch_IMU, imu.roll_IMU, -MAX_GIMBAL, MAX_GIMBAL);
  // xGimb = angleControlX(imu.pitch_IMU, imu.roll_IMU, -MAX_GIMBAL, MAX_GIMBAL);

  // =============== LQR =================

  // Preliminary, rough estimations of the missing states
  // To-do (kalman estimator)
  xDot = (imu.AccX + imu.AccX_prev) * imu.dt;
  yDot = (imu.AccY + imu.AccY_prev) * imu.dt;
  zDot = (lidarZ - zPrev)/imu.dt;

  lqr(xDot, imu.roll_IMU, imu.GyroX, yDot, imu.pitch_IMU, imu.GyroY, lidarZ, zDot, t0, lqrOutputs);
  
  // Actuation
  motorsWrite(motorSpeed, ackData);
  setServo1Pos(-xGimb);
  setServo2Pos(-yGimb);

  


  //Read barometer data
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

  //delay(240); // Delay for 1 second (1000 milliseconds)

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

  write2SD();

  // Regulate looprate to predefined loop frequency (the teeensy runs much faster then what is suitable for this)
  imu.loopRate();     // <<<<<<<<<<<<<<<------------------------------------------------------------------------------ To do (Gunnar): Tweak this to prevent lag when transmitting data etc.
}