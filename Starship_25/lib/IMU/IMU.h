// Header file for the IMU class

#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Wire.h>
#include "BMI088.h"

const int PIN_WIRE1_SDA = 18; // SDA on Teensy 4.1
const int PIN_WIRE1_SCL = 19; // SCL on Teensy 4.1

class IMU{
    public:
    bool begin();
    void read(float &ax, float &ay, float &az, float &gx, float &gy, float &gz);

    private:
};

#endif