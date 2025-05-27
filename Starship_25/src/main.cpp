#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "Sensor_handler.h"
#include "Control.h"
#include "Killswitch.h"
#include "Motor_controller.h"
#include "Complementary_filter.h"
#include "Dead_reckoning_position.h"
#include "settings.h"
#include "Servo_controller.h"
#include <math.h>
#include "Madgwick6DOF.h"

//#define MADGWICK // Uncomment for Madgwick filter
#define COMPLEMENTARY // Uncomment for complementary filter
#define DEAD_RECKONING // Uncomment for dead reckoning

Madgwick6DOF filter; // Initialize the Madgwick filter with a beta value of 0.1

SensorHandler sensorHandler;
Control droneControl;

const float dt = 0.005f;
PositionEstimator estimator(dt);

void setup() {
    pinMode(9, INPUT);
    // Initialize serial communication to sensors
    Serial.begin(BAUDRATE);
    while (!Serial); // Wait for serial connection
    
    if (!sensorHandler.begin()) {
        Serial.println("Sensor initialization failed!");
        while (1);
    }
    Serial.println("All sensors initialized!");

    // Initialize kill switch
    setupKillSwitch(); // This is a non-blocking function that sets up the interrupt

    // Initialize drone control
    droneControl.initialize();
    //droneControl.testComponents();  // Test motors and servos
}

void printFixed(int value, bool end = false) {
    char buffer[7];  // 5 chars + null terminator
    snprintf(buffer, sizeof(buffer), "%5d", value);  // Right-aligned, 5 wide
    Serial.print(buffer);
    if (!end) {
        Serial.print(",");
    }
}

void loop() {

    if (emergencyStop) {
        // Stop motors
        motorsWrite(1,1050);
        motorsWrite(2,1050);
        while(emergencyStop){
            // Wait for the emergency stop to be released
            //Serial.println("Waiting for emergency stop to be released...");
            delay(100);
        };  // Infinite loop
        Serial.println("Resuming operation...");
    }

    SensorData data = sensorHandler.readSensors();

    #ifdef MADGWICK
        filter.update(data.imu_gyro_x, data.imu_gyro_y, data.imu_gyro_z,
                    data.imu_accel_x, data.imu_accel_y, data.imu_accel_z, dt);

        float yaw_m, pitch_m, roll_m;
        filter.getEulerAngles(yaw_m, pitch_m, roll_m);
        printFixed(yaw_m);
        printFixed(pitch_m);
        printFixed(roll_m);
    #endif

    #ifdef COMPLEMENTARY
        updateIMU(data.imu_accel_x, data.imu_accel_y, data.imu_accel_z,
                data.imu_gyro_x/131.072, data.imu_gyro_y/131.072, data.imu_gyro_z/131.072);
        // convert to degrees
        float yaw_deg = 2 * yaw * 180.0 / M_PI;
        float pitch_deg = 2 * pitch * 180.0 / M_PI;
        Serial.print(yaw);
        Serial.print(",");
        Serial.print(pitch);
        Serial.print(",");
    #endif

    // Update estimator
    #ifdef DEAD_RECKONING
        float flow_x, flow_y;
        flowToVelocityXY(data.flow_x, data.flow_y, data.lidar_dist/100.0, flow_x, flow_y);
        estimator.update({data.imu_accel_x, data.imu_accel_y, data.imu_accel_z},
                        pitch, yaw, data.lidar_dist/100.0, flow_x, flow_y);
        auto pos = estimator.getPosition();
        Serial.print(pos[0]);
        Serial.print(",");
        Serial.print(pos[1]);
        Serial.print(",");
        Serial.println(pos[2]); 
    #endif

    //float state[10] = {pos[0], vel[0], pos[1], vel[1], pos[2], vel[2],
    //                        pitch_m, yaw_m, data.imu_gyro_x, data.imu_gyro_y};
    //droneControl.lqrControl(state);

    if (manualControl) {
        droneControl.manualControl();
    } else {
        droneControl.autoControl(pitch_deg, yaw_deg);
    }
}
