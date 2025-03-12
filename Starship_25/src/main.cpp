#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "Sensor_handler.h"

// if standard library not found,
// use this to check: ls ~/.platformio/packages/framework-arduinoteensy/libraries/
// Wire and SPI are standard libraries for Teensy so they should be found
// Teensy 4.1 has 2 I2C buses, Wire and Wire1
// Wire is on pins 18 and 19
// Wire1 is on pins 16 and 17
// SPI is on pins 11, 12, 13, and 10
/* Try running: platformio run --target clean
platformio run
*/

SensorHandler sensorHandler;

void setup() {
    Serial.begin(115200);
    while (!Serial); // Wait for serial connection
    
    if (!sensorHandler.begin()) {
        Serial.println("Sensor initialization failed!");
        while (1);
    }

    Serial.println("All sensors initialized!");
}

void loop() {
    SensorData data = sensorHandler.readSensors();

    Serial.printf("IMU: Ax: %.2f Ay: %.2f Az: %.2f Gx: %.2f Gy: %.2f Gz: %.2f | "
                  "Flow: X: %d Y: %d | LiDAR: %dmm\n",
                  data.imu_accel_x, data.imu_accel_y, data.imu_accel_z,
                  data.imu_gyro_x, data.imu_gyro_y, data.imu_gyro_z,
                  data.flow_x, data.flow_y,
                  data.lidar_dist, data.lidar_flux, data.lidar_temp);

    delay(100);  // Send data every 100ms
}
