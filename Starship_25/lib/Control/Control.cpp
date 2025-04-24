
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

void Control::manualControl() {
    int theata = map(Servo1ControlPWM, 1100, 1950, -30, 30); // Map the angle to the servo range
    setServo1Pos(theata); // Set servo 1 to 0 degrees
    theata = map(Servo2ControlPWM, 1100, 1950, -30, 30); // Map the angle to the servo range
    setServo2Pos(theata); 

    motorsWrite(1, MotorControlPWM); // Set motor 1 speed
    motorsWrite(2, MotorControlPWM); // Set motor 2 speed
}