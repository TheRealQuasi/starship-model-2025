// Header file for the IMU class

#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <SPI.h>

class IMU {
public:
    IMU();
    bool begin();
    void read(float& accel_x, float& accel_y, float& accel_z, 
              float& gyro_x, float& gyro_y, float& gyro_z);

private:
    Adafruit_BNO055 bno;
};

#endif