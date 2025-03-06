#include "starship_imu.h"

float ax = 0, ay = 0, az = 0;
float gx = 0, gy = 0, gz = 0;
int16_t temp = 0;

BMI088 bmi088( BMI088_ACC_ADDRESS, BMI088_GYRO_ADDRESS );


bool STARSHIP_IMU::start_imu(){
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