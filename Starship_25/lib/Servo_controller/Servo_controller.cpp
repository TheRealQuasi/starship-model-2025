
#include <Arduino.h>
#include <Servo.h>
#include "Servo_controller.h"

Servo servo1;
Servo servo2;

// ====== Servos ======

// Functions that sends position commands to the respective servo
// --------------------------------------------------------------
// Takes an angle theta 1 in approx. the range -60 to 60 (depends on the servo calibration parameter)

// Servo range: 900, 2100

void setServo1Pos(int theta1) {             // <<<<<<<<---------- To do: Combine these to one function
//  servo1.write(servo1Home + theta1*1.5);

  int thetaMapped = theta1 + SERVO_1_HOME;

  // Constraints (servo can't move out of actuation range)
  if (thetaMapped > SERVO_1_HOME + MAX_GIMBAL) {
    thetaMapped = SERVO_1_HOME + MAX_GIMBAL;
  }
  else if (thetaMapped < SERVO_1_HOME - MAX_GIMBAL) {
    thetaMapped = SERVO_1_HOME - MAX_GIMBAL;
  }

  int tMapped = map(thetaMapped, 0, 120, 900, 2100);
  servo1.writeMicroseconds(tMapped);
}

void setServo2Pos(int theta2) {
//  servo1.write(servo1Home + theta1*1.5);
  int thetaMapped = theta2 + SERVO_2_HOME;

  // Constraints (servo can't move out of actuation range)
  if (thetaMapped > SERVO_2_HOME + MAX_GIMBAL) {
    thetaMapped = SERVO_2_HOME + MAX_GIMBAL;
  }
  else if (thetaMapped < SERVO_2_HOME - MAX_GIMBAL) {
    thetaMapped = SERVO_2_HOME - MAX_GIMBAL;
  }

  int tMapped = map(thetaMapped, 0, 120, 900, 2100);
  servo2.writeMicroseconds(tMapped);
}

// Gimbal test during startup (+ / - 30 degrees gimbal on both motors)
void gimbalTest() {
  setServo1Pos(-MAX_GIMBAL);
  setServo2Pos(-MAX_GIMBAL);
  delay(1500);    

  setServo1Pos(MAX_GIMBAL);
  setServo2Pos(MAX_GIMBAL);
  delay(1500);    

  setServo1Pos(0);
  setServo2Pos(0);
}

void initServos() {
  #ifdef DEBUG
    Serial.print("Servo init\n");
    Serial.print("--------------------\n \n");
  #endif                         

  // Attatch pins to servo obejcts (with On-time [us] range specified)
  servo1.attach(SERVO_1_PIN, 900, 2100);
  servo2.attach(SERVO_2_PIN, 900, 2100);

  // Gimbal both motors to the home position (0 degrees)
  setServo1Pos(0);
  setServo2Pos(0);
}