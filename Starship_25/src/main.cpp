#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "Sensor_handler.h"
#include "Control.h"
#include "Killswitch.h"
#include "Motor_controller.h"

// if standard library not found,
// use this to check: ls ~/.platformio/packages/framework-arduinoteensy/libraries/
// Wire and SPI are standard libraries for Teensy so they should be found
// Teensy 4.1 has 2 I2C buses, Wire and Wire1
// Wire is on pins 18 and 19
// Wire1 is on pins 16 and 17
// SPI is on pins 11, 12, 13, and 10
/* 
Try running: platformio run --target clean
platformio run
*/

SensorHandler sensorHandler;
Control droneControl;

void setup() {
    // Initialize serial communication to sensors
    Serial.begin(115200);
    while (!Serial); // Wait for serial connection
    
    if (!sensorHandler.begin()) {
        Serial.println("Sensor initialization failed!");
        while (1);
    }

    Serial.println("All sensors initialized!");

    // Initialize kill switch
    setupKillSwitch(); // This is a non-blocking function that sets up the interrupt

    // Initialize drone control
    //droneControl.initialize();
}

void loop() {
    //int pulseWidth = pulseIn(KILL_SWITCH_PIN, HIGH, 250000);  // Read PWM pulse width (max timeout: 2.5ms)
    //Serial.print("Kill Switch PWM Pulse Width: ");
    //Serial.println(pulseWidth);

    if (emergencyStop) {
        Serial.println("Emergency stop triggered!");

        // Stop motors

        //dc_motor_1.write(1100); // Motor 1
        //dc_motor_2.write(1100); // Motor 2
        //analogWrite(MOTOR_1_PIN, 1100); // Motor 1
        //analogWrite(MOTOR_2_PIN, 1100); // Motor 2
        break;
    }

    //droneControl.update();

    SensorData data = sensorHandler.readSensors();

    Serial.printf(">Ax:");
    Serial.println(data.imu_accel_x);
    Serial.printf(">Ay:");
    Serial.println(data.imu_accel_y);
    Serial.printf(">Az:");
    Serial.println(data.imu_accel_z);
    Serial.printf(">Gx:");
    Serial.println(data.imu_gyro_x);
    Serial.printf(">Gy:");
    Serial.println(data.imu_gyro_y);
    Serial.printf(">Gz:");
    Serial.println(data.imu_gyro_z);
    Serial.printf(">Temp:");
    Serial.println(data.imu_temp);
    
    Serial.printf(">FlowX:");
    Serial.println(data.flow_x);
    Serial.printf(">FlowY:");
    Serial.println(data.flow_y);
    
    Serial.printf(">LiDAR:");
    Serial.println(data.lidar_dist);
    Serial.printf(">Flux:");
    Serial.println(data.lidar_flux);
    Serial.printf(">LiDAR Temp:");
    Serial.println(data.lidar_temp);

    delay(100);  // Send data every 100ms
}
