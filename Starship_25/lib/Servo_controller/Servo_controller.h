
// =======================
// Servo control
// =======================

#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include <Arduino.h>
#include <Servo.h>

// Constants
// ---------
#define SERVO_1_PIN 2       // Lower servo (xRot = imu.roll_IMU)
#define SERVO_2_PIN 3       // Upper servo (yRot = imu.pitch_IMU)
#define SERVO_1_HOME 55 //82;          // 0 position [degrees]
#define SERVO_2_HOME 71 //107; // 88;  // 0 position[degrees]
#define MAX_GIMBAL 30

// Functions
// ---------
void setServo1Pos(int theta1);

void setServo2Pos(int theta2);

void gimbalTest();

void initServos();

#endif
