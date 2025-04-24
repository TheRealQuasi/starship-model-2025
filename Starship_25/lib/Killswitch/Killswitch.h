#ifndef KILL_SWITCH_H
#define KILL_SWITCH_H

#include <Arduino.h>
#include <Servo.h>
#include "Motor_controller.h"
#include "settings.h"

extern volatile bool emergencyStop;
extern volatile unsigned long pulseWidth;
extern volatile bool manualControl;

void setupKillSwitch();
void killSwitchISR();

#endif
