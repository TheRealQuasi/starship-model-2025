#ifndef KILL_SWITCH_H
#define KILL_SWITCH_H

#include <Arduino.h>
#include <Servo.h>
#include "Motor_controller.h"

#define KILL_SWITCH_PIN 3  // Pin connected to kill switch channel

extern volatile bool emergencyStop;

void setupKillSwitch();
void killSwitchISR();

#endif
