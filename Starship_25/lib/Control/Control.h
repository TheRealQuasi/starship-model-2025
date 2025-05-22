// =======================
// Control Theory
// =======================

#ifndef CONTROL_H
#define CONTROL_H

#include <Arduino.h>
#include <Servo.h>

struct ControlData {
  bool armSwitch;     // Arm switch status
  byte calButton;     // Calibration button status  <<<<<<<<<---------- (Gunnar): Added variable for calButton
  byte thrustSlider;  // 0-255
  byte lxAxisValue;   // 0-255
  byte lyAxisValue;   // 0-255
};

class Control {
public:
    Control();  // Constructor


    void initialize();  // Initialize motors and servos
    void testComponents();  // Run test functions for motors and servos
    void lqrControl(float state[10]);  // LQR control function
    void manualControl();  // Manual control function
};

#endif
