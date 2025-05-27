
// imu_filter.h
#ifndef IMU_FILTER_H
#define IMU_FILTER_H

extern float pitch;
extern float yaw;

void updateIMU(float ax, float ay, float az, float gx, float gy, float gz);

#endif