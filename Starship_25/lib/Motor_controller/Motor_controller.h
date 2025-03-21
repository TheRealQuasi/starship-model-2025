// =======================
// Motor control
// =======================

#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>
#include <Servo.h>

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

struct ControlData {
  bool armSwitch;     // Arm switch status
  byte calButton;     // Calibration button status  <<<<<<<<<---------- (Gunnar): Added variable for calButton
  byte thrustSlider;  // 0-255
  byte lxAxisValue;   // 0-255
  byte lyAxisValue;   // 0-255
};

// Functions
// ---------
int speedMapping(int thrustLevel);

void motorsWrite(int motor, int s, ControlData& ackData);

void motorTest();

void escCalibration(bool &escCalibrationStatus);

void waitESCCalCommand(bool &escCalibrationStatus);

void initMotorController();

#endif
