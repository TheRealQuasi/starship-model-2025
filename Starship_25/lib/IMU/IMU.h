// Header file for the IMU class

#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Wire.h>
#include "BMI088.h"

class IMU{
    public:
    bool begin();
    void read(float &ax, float &ay, float &az, float &gx, float &gy, float &gz);

    private:
};

#endif