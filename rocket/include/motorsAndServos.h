// =======================
// Servo and motor control
// =======================

/*
* 
* 
* By GunnarEdman (Gunnar Edman)
*/

#ifndef SERVO_MOTOR_CONTROL_H
#define SERVO_MOTOR_CONTROL_H

#include <Arduino.h>
#include <Servo.h>
#include <settings.h>
#include <GlobalDecRocket.h>


// Functions
// ---------
void setServo1Pos(int theta1);

void setServo2Pos(int theta2);

int speedMapping(int thrustLevel);

void motorsWrite(int motor, int s, ControlData& ackData);

void gimbalTest();

void motorTest();

void escCalibration(bool &escCalibrationStatus);

void waitESCCalCommand(bool &escCalibrationStatus);

void initServosMotors();

#endif
