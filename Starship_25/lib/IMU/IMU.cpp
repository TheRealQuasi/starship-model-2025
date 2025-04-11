// Sensor data handling functions for the IMU sensor

#include "IMU.h"

float ax = 0, ay = 0, az = 0;
float gx = 0, gy = 0, gz = 0;
int16_t temp = 0;

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
    bmi088.getAcceleration(&ax, &ay, &az);
    bmi088.getGyroID();
    bmi088.getGyroscope(&gx, &gy, &gz);
    temp = bmi088.getTemperature();
}
