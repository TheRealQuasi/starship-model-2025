#include "Killswitch.h"
#include "Motor_controller.h"

volatile bool emergencyStop = false;

void killSwitchISR() {
    if (pulseIn(KILL_SWITCH_PIN, HIGH, 25000) < 1000) {  
        emergencyStop = true;

        Serial.println("!!! KILL SWITCH ACTIVATED !!!");

        // Stop motors
        analogWrite(MOTOR_1_PIN, 1100); // Motor 1
        analogWrite(MOTOR_2_PIN, 1100); // Motor 2

        // Stay in infinite loop
        while (true);
    }
}

void setupKillSwitch() {
    pinMode(KILL_SWITCH_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(KILL_SWITCH_PIN), killSwitchISR, CHANGE);
}