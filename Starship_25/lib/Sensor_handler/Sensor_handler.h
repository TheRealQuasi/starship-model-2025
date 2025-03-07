// Header file for Sensor_handler.cpp
#ifndef SENSOR_HANDLER_H
#define SENSOR_HANDLER_H

#include "IMU.h"
//#include "lidar.h"
//#include "optical_flow.h"

struct SensorData {
    float imu_accel_x, imu_accel_y, imu_accel_z;
    float imu_gyro_x, imu_gyro_y, imu_gyro_z;
    //int16_t flow_x, flow_y;
    //uint16_t lidar_distance;
};

class SensorHandler {
public:
    SensorHandler();
    bool begin();
    SensorData readSensors();

private:
    IMU imu;
    //LiDAR lidar;
    //OpticalFlow opticalFlow;
};

#endif