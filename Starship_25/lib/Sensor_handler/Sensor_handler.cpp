// Makes use of the IMU, LiDAR and OpticalFlow classes to read data from the sensors

#include "Sensor_handler.h"

SensorHandler::SensorHandler() {}

bool SensorHandler::begin() {
    return imu.begin(); //&& lidar.begin() && opticalFlow.begin();
}

SensorData SensorHandler::readSensors() {
    SensorData data = {0};

    // Read IMU
    imu.read(data.imu_accel_x, data.imu_accel_y, data.imu_accel_z,
             data.imu_gyro_x, data.imu_gyro_y, data.imu_gyro_z);

    // Read Optical Flow
    //opticalFlow.readMotion(data.flow_x, data.flow_y);

    // Read LiDAR
    //data.lidar_distance = lidar.readDistance();

    return data;
}

