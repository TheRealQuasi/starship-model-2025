// Sensor data handling functions for the IMU sensor

#include "IMU.h"

IMU::IMU() : bno(55, 0x28) {}

bool IMU::begin() {
    return bno.begin();
}

void IMU::read(float& accel_x, float& accel_y, float& accel_z, 
               float& gyro_x, float& gyro_y, float& gyro_z) {
    sensors_event_t accelData, gyroData;
    bno.getEvent(&accelData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno.getEvent(&gyroData, Adafruit_BNO055::VECTOR_GYROSCOPE);

    accel_x = accelData.acceleration.x;
    accel_y = accelData.acceleration.y;
    accel_z = accelData.acceleration.z;

    gyro_x = gyroData.gyro.x;
    gyro_y = gyroData.gyro.y;
    gyro_z = gyroData.gyro.z;
}
