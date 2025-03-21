// =======================
// Control Theory
// =======================

#ifndef CONTROL_H
#define CONTROL_H

#include <Arduino.h>
#include <Servo.h>

class Control {
public:
    Control();  // Constructor

    void initialize();  // Initialize motors and servos
    void testComponents();  // Run test functions for motors and servos
};

#endif
