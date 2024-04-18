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



// =============================================================================================
//  Variables/Objects
// =============================================================================================

// Instantiate an object for the nRF24L01 transceiver
RF24 radio(CE_PIN, CSN_PIN);

// Indicateds if there is new data to be read from the radio
bool newControllerData = false;

// For when to send packets
unsigned long currentMillis;
unsigned long prevMillis = 0;
unsigned long txIntervalMillis = 50; // send once per every 250 milliseconds

// Create a Packet to hold the data
PacketData senderData;

// Acknowledge payload to hold the data coming from the rocket
ControlData ackData;

// Store sensor data
SensorData sensorData;

// ========= Status variables =========
bool escCalibrationStatus = false;  // Boolean that informs if ESC calibration is performed or not

// ========= IMU =========
// IMU object
// Objects handeling everything with the BMI088 paired with a madgwick filter
Imu6DOF imu;

// ========= Lidar =========
TFMPI2C tfmP;         // Create a TFMini-Plus I2C object

// Variables storing data from TFmini plus
int16_t tfDist = 0;       // Distance to object in centimeters
int16_t tfFlux = 0;       // Signal strength or quality of return signal
int16_t tfTemp = 0;       // Internal temperature of Lidar sensor chip


// Servo control variables
float xGimb = 0;
float yGimb = 0;
float motorSpeed = 1140;

// =============================================================================================
//  Functions
// =============================================================================================

// Calibration procedure called after calButton is pressed for at least 2 seconds
void waitESCCalCommand() {
  // OLd implementation
  // ------------------

  // // Only execute this code if the ESC is in calibration mode (never in armed mode)
  // if (!escCalibrationStatus) {
  //   // Wait for calibration to be granted by the pilot (holding calButton for at least 2 seconds)
  //   unsigned long t0 = millis();
  //   unsigned long tPress = millis();

  //   // Check for calButton press and duration off press
  //   while (tPress - t0 < CAL_BUTTON_DURATION) {
  //     // Check inpus
  //     transmitData(radio, senderData, ackData, newControllerData, prevMillis);

  //     // Check calButton status
  //     if (ackData.calButton) {
  //       tPress = millis();
  //     }

  //     // Reset duration if button is not pressed
  //     else {
  //       t0 = millis();
  //       tPress = millis();
  //     }
  //   }

  //   // When button press duration is enough, run ESC calibation
  //   escCalibration(escCalibrationStatus);
  // }


  // Only execute this code if the ESC is in calibration mode (never in armed mode)
  if (!escCalibrationStatus) {
    #ifdef DEBUG
      Serial.print("Avaiting cal button");
    #endif

    // Wait for calibration to be granted by the pilot (holding calButton for at least 2 seconds)
    unsigned long t0 = millis();
    unsigned long tPress = millis();

    // Check for calButton press and duration off press
    while (tPress - t0 < CAL_BUTTON_DURATION) {
      // Check calButton status
      if (!digitalRead(CAL_BUTTON)) {
        tPress = millis();
      }

      // Reset duration if button is not pressed
      else {
        t0 = millis();
        tPress = millis();
      }
    }

    delay(1000);

    // When button press duration is enough, run ESC calibation
    escCalibration(escCalibrationStatus);
  }
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

  // Initialize radio module
  // initRadio(radio, RF24_PA_LEVEL, RF24_SPEED, RF24_CHANNEL);

  // Initialize servos and ESCs (motors)
  initServosMotors();

  // Configure digital input for calButton
  pinMode(CAL_BUTTON, INPUT_PULLUP);

  // Wait for ESC calibration command
  waitESCCalCommand();
  

  #ifdef DEBUG
  setServo1Pos(-MAX_GIMBAL);
  setServo2Pos(SERVO_2_HOME);
  delay(1500);    

  setServo1Pos(SERVO_1_HOME);
  setServo2Pos(SERVO_2_HOME);
  delay(1500);    
  #endif

  setServo1Pos(0);
  setServo2Pos(0);

  // Initialize I2C bus
  Wire.begin();

  // Initialize sensors (needs to happend in the end, to allow for continous IMU sampling)
  imu.initIMU();

  if(!initDPS310()){
    #ifdef DEBUG
      Serial.println("Failed to initialize the pressure sensor. Check the wiring and try again.");
    #endif
  }

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
  ackData.armSwitch = 0;
  ackData.calButton = 0;
  ackData.thrustSlider = 0;
  ackData.lxAxisValue = 0;
  ackData.lyAxisValue = 0;

  #ifdef DEBUG
  Serial.println("Init complete!");
  #endif
}

void loop() {
  // Time management
  imu.timeUpdate();                         // Record time at start of loop iteration (used in madgwick filters)

  // Read sensors and filter data
  imu.imuUpdate();                          // Get IMU data and filter it with LP / smoothing and Madgwick-filters
  tfmP.getData( tfDist, tfFlux, tfTemp);    // Get a frame of data from the TFmini
  #ifdef DEBUG
    Serial.print("Altitude = ");
    Serial.print(tfDist);
  #endif

  // PID altitude
  motorSpeed = altitude_pid(tfDist, ALT_REF);
  motorsWrite(motorSpeed, ackData);

  // PID angle 
  yGimb = angleControlY(imu.pitch_IMU, imu.roll_IMU, -MAX_GIMBAL, MAX_GIMBAL);
  xGimb = angleControlX(imu.pitch_IMU, imu.roll_IMU, -MAX_GIMBAL, MAX_GIMBAL);

  // Actuation
  setServo1Pos(xGimb);
  setServo2Pos(yGimb);

  
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

  // Regulate looprate to predefined loop frequency (the teeensy runs much faster then what is suitable for this)
  imu.loopRate();     // <<<<<<<<<<<<<<<------------------------------------------------------------------------------ To do (Gunnar): Tweak this to prevent lag when transmitting data etc.
}
