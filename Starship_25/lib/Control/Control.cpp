
#include <Arduino.h>
#include <Servo.h>
#include "Control.h"
#include "Motor_controller.h"
#include "Servo_controller.h"

// Constructor
Control::Control() {
    // Nothing to do in constructor
}

// Initialize both motors and servos
void Control::initialize() {
    initMotorController();
    initServos();
}

// Run test functions for motors and servos
void Control::testComponents() {
    //motorTest();
    gimbalTest();
}

// Example LQR Gain Matrix (replace with actual values)
const float K[4][10] = {
  { /* row 0 for gimbal 1 angle control */ },
  { /* row 1 for motor 1 PWM control */ },
  { /* row 2 for gimbal 2 angle control */ },
  { /* row 3 for motor 2 PWM control */ }
};

void Control::lqrControl(float state[10]) {
    float u[4] = {0, 0, 0, 0};

    // Compute u = -K * x
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 10; j++) {
            u[i] -= K[i][j] * state[j];
        }
    }

    // Convert angles from radians to degrees if needed
    float gimbalAngle1 = constrain(u[0], -30, 30); // degrees
    float pwm1 = constrain(u[1], 1100, 1950);       // microseconds

    float gimbalAngle2 = constrain(u[2], -30, 30); // degrees
    float pwm2 = constrain(u[3], 1100, 1950);       // microseconds

    // Apply control signals
    setServo1Pos((int)gimbalAngle1); // assuming function expects degrees
    setServo2Pos((int)gimbalAngle2);

    motorsWrite(1, (int)pwm1);
    motorsWrite(2, (int)pwm2);
}

void Control::manualControl() {
    int theata = map(Servo1ControlPWM, 1100, 1950, -30, 30); // Map the angle to the servo range
    setServo1Pos(theata); // Set servo 1 to 0 degrees
    theata = map(Servo2ControlPWM, 1100, 1950, -30, 30); // Map the angle to the servo range
    setServo2Pos(theata); 

    motorsWrite(1, MotorControlPWM); // Set motor 1 speed
    motorsWrite(2, MotorControlPWM); // Set motor 2 speed
}