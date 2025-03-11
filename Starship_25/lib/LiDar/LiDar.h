#ifndef LIDAR_H
#define LIDAR_H

#include <Wire.h>
#include <Arduino.h>
#include <TFMPI2C.h>
//#include "printf.h"

class LiDAR {
public:
    bool begin();
    bool getData( int16_t &dist, int16_t &flux, int16_t &temp, uint8_t addr);
    
private:
    TFMPI2C sensor; // Create a TFMini-Plus I2C object
};

#endif
