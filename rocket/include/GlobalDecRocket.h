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
  byte timeStamp;
  float posXValue;
  float posYValue;
  float posZValue;
  float accXValue;
  float accYValue; 
  float accZValue; 
  float gamValue;
  float accGamValue;
  float betaValue;
  float accBetaValue; 
};

struct ControlData {
  byte armSwitch;     // Arm switch status
  byte calButton;     // Calibration button status  <<<<<<<<<---------- (Gunnar): Added variable for calButton
  byte thrustSlider;  // 0-255
  byte lxAxisValue;   // 0-255
  byte lyAxisValue;   // 0-255
};

struct SensorData {
  float psHeight; // Height from ground (Pressure sensor)
};

enum States {
  SERVO_AND_MOTOR_INIT,
  ESC_CALIBRATION,
  GIMBAL_TEST,
  IMU_CALIBRATION,
  FILTER_WARMUP,
  SYSTEM_READY
};


#endif
