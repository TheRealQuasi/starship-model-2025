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

#define MADGWICK // Uncomment for Madgwick filter
//#define COMPLEMENTARY // Uncomment for complementary filter
#define DEAD_RECKONING // Uncomment for dead reckoning
//#define time_test // Uncomment for time test

Madgwick6DOF filter; // Initialize the Madgwick filter with a beta value of 0.1

volatile uint32_t start_time = 0; // Time in microseconds
volatile uint32_t counter = 0; // Flag for emergency stop

SensorHandler sensorHandler;
Control droneControl;

volatile bool TestStart = true; // Flag for emergency stop
volatile bool logging = false; // Flag for logging
volatile uint32_t temp_times = 0; // counter for logging

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
    if (manualControl) {
        droneControl.manualControl();
    }

    if (emergencyStop) {
        //uint32_t time_change = (micros()-start_time)/1000000;
        //Serial.println("time: " + String(time_change) + " seconds");
        //Serial.println("counter: " + String(counter));
        //Serial.println("Data/second: " + String(counter/time_change));  
        //Serial.println("Emergency stop triggered!");

        // Stop motors
        motorsWrite(1,1050);
        motorsWrite(2,1050);

        while(emergencyStop){
            // Wait for the emergency stop to be released
            //Serial.println("Waiting for emergency stop to be released...");
            delay(100);
        };  // Infinite loop
        Serial.println("Resuming operation...");

        #ifdef time_test
            start_time = micros();  // Reset time
            counter = 0;
            logging = true;
        #endif
    }

    SensorData data = sensorHandler.readSensors();

    #ifdef MADGWICK
        filter.update(data.imu_gyro_x, data.imu_gyro_y, data.imu_gyro_z,
                    data.imu_accel_x, data.imu_accel_y, data.imu_accel_z, dt);

        float roll_m, pitch_m, yaw_m;
        filter.getEulerAngles(roll_m, pitch_m, yaw_m);
        Serial.print(roll_m);
        Serial.print(pitch_m);
        Serial.print(yaw_m);
    #endif

    #ifdef COMPLEMENTARY
        updateIMU(data.imu_accel_x, data.imu_accel_y, data.imu_accel_z,
                data.imu_gyro_x/131.072, data.imu_gyro_y/131.072, data.imu_gyro_z/131.072);

        printFixed(pitch); // rad
        printFixed(roll); // rad/s
    #endif

    // Update estimator
    #ifdef DEAD_RECKONING
        float flow_x, flow_y;
        flowToVelocityXY(data.flow_x, data.flow_y, data.lidar_dist/100.0, flow_x, flow_y);
        estimator.update({data.imu_accel_x, data.imu_accel_y, data.imu_accel_z},
                        pitch_m, roll_m, data.lidar_dist/100.0, flow_x, flow_y);
        auto pos = estimator.getPosition();
        auto vel = estimator.getVelocity();
        printFixed(pos[0]);
        printFixed(pos[1]);
        printFixed(pos[2]); 
        Serial.print("\n"); //,\n
    #endif

    float state[10] = {pos[0], vel[0], pos[1], vel[1], pos[2], vel[2],
                            pitch_m, roll_m, data.imu_gyro_x, data.imu_gyro_y};
    droneControl.lqrControl(state);

    #ifdef time_test
        counter++;
        if (counter > 1000000) {
            counter = 0;
            start_time = micros();
        }
        if (logging && micros()-start_time > 50000*temp_times){
            temp_times++;
            float time_change = float((micros()-start_time)/1000000);
            //Serial.println("time: " + String(time_change) + " seconds");
            //Serial.println("counter: " + String(counter));
            //Serial.println("Data/second: " + String(counter/time_change));  
            Serial.println(String(counter/time_change));
        }
    #endif
}
