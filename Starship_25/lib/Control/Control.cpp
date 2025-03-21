
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
    motorTest();
    gimbalTest();
}