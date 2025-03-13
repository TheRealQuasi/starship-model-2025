// Header file for Sensor_handler.cpp
#ifndef SENSOR_HANDLER_H
#define SENSOR_HANDLER_H

#include "IMU.h"
#include <Wire.h>
#include <Arduino.h>
#include "LiDar.h"
#include "Optical_flow.h"

struct SensorData {
    float imu_accel_x, imu_accel_y, imu_accel_z;
    float imu_gyro_x, imu_gyro_y, imu_gyro_z;
    int16_t imu_temp;
    int16_t lidar_dist, lidar_flux, lidar_temp;
    //int16_t flow_x, flow_y;
};

class SensorHandler {
public:
    SensorHandler();
    bool begin();
    SensorData readSensors();

private:
    IMU imu;
    LiDAR lidar;
    //Optical_flow flow;
};

#endif