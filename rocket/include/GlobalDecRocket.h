/*
* 
* 
* By Emlzdev (Emil Reinfeldt)
*/

#pragma once

#ifndef GLOBALDECROCKET_H
#define GLOBALDECROCKET_H

#include <Arduino.h>

struct PacketData
{
  float timeStamp;

  // State variables (x)
  float xDot;
  float roll;    // xRot
  float rollDot;
  float yDot;    // yRot
  float pitch; 
  float pitchDot; 
  float z;
  float zDot;

  // Control reference values
  float zRef;
  float zDotRef;

  // Control output values
  float motorSpeed;
  float gimb1;
  float gimb2;
};

struct ControlData {
  bool armSwitch;     // Arm switch status
  byte calButton;     // Calibration button status  <<<<<<<<<---------- (Gunnar): Added variable for calButton
  byte thrustSlider;  // 0-255
  byte lxAxisValue;   // 0-255
  byte lyAxisValue;   // 0-255
};

struct SensorData {
  float psHeight; // Height from ground (Pressure sensor)

  // // State variables (x)
  // float xDot;
  // float roll; // xRot
  // float rollDot;
  // float yDot;
  // float pitch; 
  // float pitchDot; 
  // float z;
  // float zDot;
};

enum States {
  SERVO_AND_MOTOR_INIT = 1,
  ESC_CALIBRATION = 2,
  GIMBAL_TEST = 3,
  IMU_CALIBRATION = 4,
  FILTER_WARMUP = 5,
  SYSTEM_READY = 6
};

// LQR signals
struct LqrSignals {
  // Input (reference)
  float zRef;
  float zDotRef;

  // Output
  float motorSpeed; // Motorspeed in % of max RPM
  float gimb1;
  float gimb2;
};



#endif
