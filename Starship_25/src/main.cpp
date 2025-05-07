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
    } else {
        motorsWrite(1,1050);
        motorsWrite(2,1050);
        //droneControl // Update motors and servos
    }

    if (emergencyStop) {
        //uint32_t time_change = (micros()-start_time)/1000000;
        //Serial.println("time: " + String(time_change) + " seconds");
        //Serial.println("counter: " + String(counter));
        //Serial.println("Data/second: " + String(counter/time_change));  
        //Serial.println("Emergency stop triggered!");

        // Stop motors

        //dc_motor_1.write(1100); // Motor 1
        //dc_motor_2.write(1100); // Motor 2
        //analogWrite(MOTOR_1_PIN, 1100); // Motor 1
        //analogWrite(MOTOR_2_PIN, 1100); // Motor 2
        while(emergencyStop){
            //analogWrite(MOTOR_1_PIN, 1050); // Motor 1
            //analogWrite(MOTOR_2_PIN, 1050); // Motor 2
        };  // Infinite loop
        Serial.println("Resuming operation...");
        start_time = micros();  // Reset time
        counter = 0;
        logging = true;
    }
    // Test stop condition
    //droneControl.testComponents(); 
    //if (TestStart) {
    //    start_time = micros();  // Start time
    //    counter = 0;  // Reset counter
    //    TestStart = false;  // Reset test stop flag
    //    Serial.println("Test Start!");
    //}
    //droneControl.update();

    SensorData data = sensorHandler.readSensors();
    //SensorData data = {0};
    //delay(1);
    
    if (logging && micros()-start_time > 50000*temp_times){
        temp_times++;
        float time_change = float((micros()-start_time)/1000000);
        //Serial.println("time: " + String(time_change) + " seconds");
        //Serial.println("counter: " + String(counter));
        //Serial.println("Data/second: " + String(counter/time_change));  
        Serial.println(String(counter/time_change));
    }

    // Read IMU data in real-time to matlab
    
    // Serial.print(data.imu_accel_x); Serial.print(",");
    // Serial.print(data.imu_accel_y); Serial.print(",");
    // Serial.print(data.imu_accel_z); Serial.print(",");
    // Serial.print(data.imu_gyro_x); Serial.print(",");
    // Serial.print(data.imu_gyro_y); Serial.print(",");
    // Serial.println(data.imu_gyro_z); Serial.print(",");
    // Serial.print(data.flow_x); Serial.print(",");
    // Serial.print(data.flow_y); Serial.print(",");  
    // Serial.println(data.lidar_dist);

    // Serial.print("\r");  // Return to the beginning of the line

    // printFixed(data.imu_accel_x);
    // printFixed(data.imu_accel_y);
    // printFixed(data.imu_accel_z);
    // printFixed(data.imu_gyro_x);
    // printFixed(data.imu_gyro_y);
    // printFixed(data.imu_gyro_z);
    // printFixed(data.flow_x);    
    // printFixed(data.flow_y);    
    // printFixed(data.lidar_dist);
    //printFixed(data.lidar_flux);
    //printFixed(manualControl); // No comma at the end
    // //delay(200);
    // printFixed(MotorControlPWM,1);
    // counter++;

    updateIMU(data.imu_accel_x, data.imu_accel_y, data.imu_accel_z,
              data.imu_gyro_x, data.imu_gyro_y, data.imu_gyro_z);

    //Serial.print("Pitch: ");
    Serial.print(pitch); Serial.print(","); // rad/s
    //Serial.print(" Roll: ");
    Serial.print(roll); Serial.print(","); // rad/s

    // Update estimator
    estimator.update({data.imu_accel_x, data.imu_accel_y, data.imu_accel_z},
                     pitch, roll, data.lidar_dist, data.flow_x, data.flow_y);
    auto pos = estimator.getPosition();

    Serial.print(pos[0]); Serial.print(",");
    Serial.print(pos[1]); Serial.print(",");
    Serial.println(pos[2]);


    // Serial.printf(">Ax:");
    // Serial.println(data.imu_accel_x / 5460) * 9.81;
    // Serial.printf(">Ay:");
    // Serial.println(data.imu_accel_y / 5460) * 9.81;
    // Serial.printf(">Az:");
    // Serial.println(data.imu_accel_z / 5460) * 9.81;
    // Serial.printf(">Gx:");
    // Serial.println((data.imu_gyro_x / 131.072) * M_PI / 180);
    // Serial.printf(">Gy:");
    // Serial.println((data.imu_gyro_y / 131.072) * M_PI / 180);
    // Serial.printf(">Gz:");
    // Serial.println((data.imu_gyro_z / 131.072) * M_PI / 180);
    // Serial.printf(">FlowX:");
    // Serial.println(data.flow_x);
    // Serial.printf(">FlowY:");
    // Serial.println(data.flow_y);
    // Serial.printf(">LiDAR:");
    // Serial.println(data.lidar_dist);
    //Serial.printf(">Flux:");
    //Serial.println(data.lidar_flux);
    //Serial.printf(">LiDAR Temp:");
    //Serial.println(data.lidar_temp);
    

    //delay(100);  // Send data every 100ms
    //if ((micros() - start_time) > (1000000* 20)) {  // 1 second elapsed
    //    Serial.println();  // New line
    //    Serial.print("Data/second: ");
    //    int32_t time_sec = (micros() - start_time)/1000000;  // Convert to seconds
    //    Serial.println(counter/time_sec);
    //    start_time = micros();  // Reset time
    //    counter = 0;  // Reset counter
    //    TestStart = true;
    //}
}
