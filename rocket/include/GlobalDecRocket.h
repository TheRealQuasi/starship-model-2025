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
  byte thrustSlider;  // 0-255
  byte lxAxisValue;   // 0-255
  byte lyAxisValue;   // 0-255
};


#endif