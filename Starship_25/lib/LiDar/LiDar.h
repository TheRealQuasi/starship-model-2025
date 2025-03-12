#ifndef LIDAR_H
#define LIDAR_H

#include "pins.h"
#include <Wire.h>
#include <Arduino.h>
#include <TFMPI2C.h>
//#include "printf.h"

class LiDAR {
public:
    bool begin();
    bool getData( int16_t &dist, int16_t &flux, int16_t &temp);
    
private:
    const int PIN_WIRE1_SDA = 16; // SDA on Teensy 4.1
    const int PIN_WIRE1_SCL = 17; // SCL on Teensy 4.1
    TFMPI2C tfmP;
};

#endif
