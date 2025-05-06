// Sensor data handling functions for the IMU sensor

#include "IMU.h"
#include <math.h>

float ax = 0, ay = 0, az = 0;
float gx = 0, gy = 0, gz = 0;
int16_t temp = 0;

// Assume these calibration values are set elsewhere
float ax_offset = 0.0f, ay_offset = 0.0f, az_offset = 0.0f;
float gx_offset = 0.0f, gy_offset = 0.0f, gz_offset = 0.0f;

const float accel_scale = 9.81f / 5460.0f;     // LSB to m/s^2
const float gyro_scale = (M_PI / 180.0f) / 131.072f; // LSB to rad/s

BMI088 bmi088( BMI088_ACC_ADDRESS, BMI088_GYRO_ADDRESS );

bool IMU::begin(){
    Wire.begin();
    Serial.begin(115200);

    while (!Serial);
    Serial.println("BMI088 Raw Data");
    

    while (1) {
        if (bmi088.isConnection()) {
            bmi088.initialize();
            Serial.println("BMI088 is connected");
            return true;
        } else {
            Serial.println("BMI088 is not connected");
        }

        delay(2000);
        return true;
    }
} 

void IMU::read(float &ax, float &ay, float &az, float &gx, float &gy, float &gz, int16_t &temp){
    // Raw sensor data
    float raw_ax, raw_ay, raw_az;
    float raw_gx, raw_gy, raw_gz;

    bmi088.getAcceleration(&raw_ax, &raw_ay, &raw_az);
    bmi088.getGyroID();
    bmi088.getGyroscope(&raw_gx, &raw_gy, &raw_gz);
    temp = bmi088.getTemperature();

    // Apply calibration
    ax = (raw_ax - ax_offset) * accel_scale;
    ay = (raw_ay - ay_offset) * accel_scale;
    az = (raw_az - az_offset) * accel_scale;

    gx = (raw_gx - gx_offset) * gyro_scale;
    gy = (raw_gy - gy_offset) * gyro_scale;
    gz = (raw_gz - gz_offset) * gyro_scale;
}


