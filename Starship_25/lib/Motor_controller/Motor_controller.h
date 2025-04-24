// =======================
// Motor control
// =======================

#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>
#include <Servo.h>
#include "Control.h"
#include <settings.h>

// Constants

#define MOTOR_1_PIN 7  //4  // Lower motor
#define MOTOR_2_PIN 8  //5  // Upper motor
#define SPEED_LIMIT 1700    // Maximum allowed motor speed [us]
#define SPEED_MIN 1100      // Lowest allowed motor speed
//#define SPEED_MAX 1940      // Highest allowed motor speed
// #define SPEED_PROCENT_LIMIT 100 
#define RED_LED_PIN 41     // Red LED pin
#define CAL_BUTTON  37 //6 // Calibration button pin
#define CAL_BUTTON_DURATION 2000  // How long the botton needs to hold to enter esc calibration [ms]


extern volatile uint32_t MotorControlPWM;  

// Functions
// ---------
int speedMapping(int thrustLevel);

void motorsWrite(int motor, int s);

void motorTest();

void escCalibration(bool &escCalibrationStatus);

void waitESCCalCommand(bool &escCalibrationStatus);

void initMotorController();


#endif
