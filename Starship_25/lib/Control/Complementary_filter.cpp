#include <math.h>
#include "Complementary_filter.h"

// Constants
const float alpha = 0.98f;         // Complementary filter constant
const float dt = 0.005f;           // Time step (5 ms)

// Orientation state (in radians)
float roll = 0.0f;
float pitch = 0.0f;


void updateIMU(float ax, float ay, float az, float gx, float gy, float gz) {

    // --- Accelerometer angle estimates (radians) ---
    float acc_pitch = atan2(ay, sqrt(ax * ax + az * az));
    float acc_roll  = atan2(-ax, az);

    // --- Integrate gyroscope data ---
    float roll_gyro  = roll  + gx * dt;
    float pitch_gyro = pitch + gy * dt;

    // --- Complementary filter ---
    roll  = alpha * roll_gyro  + (1.0f - alpha) * acc_roll;
    pitch = alpha * pitch_gyro + (1.0f - alpha) * acc_pitch;
}