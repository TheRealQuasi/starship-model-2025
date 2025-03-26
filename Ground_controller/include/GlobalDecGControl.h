/*
* 
* 
* By Emlzdev (Emil Reinfeldt)
*/

#pragma once

#ifndef GLOBALDECGCONTROL_H
#define GLOBALDECGCONTROL_H

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
  byte calButton;     // Calibration button status
  byte thrustSlider;  // 0-255
  byte lxAxisValue;   // 0-255
  byte lyAxisValue;   // 0-255
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