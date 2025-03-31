#include "Killswitch.h"
#include "Motor_controller.h"

volatile bool emergencyStop = false;
volatile unsigned long pulseStart = 0;
volatile unsigned long pulseWidth = 0;

void killSwitchISR() {

    if (digitalRead(KILL_SWITCH_PIN) == HIGH) {
        pulseStart = micros();  // Record the timestamp at the rising edge
    } else {
        pulseWidth = micros() - pulseStart;  // Compute pulse width on falling edge
        if (pulseWidth > 1600) {  // Adjust threshold based on your WFLY signal range
            emergencyStop = true;
            //NVIC_SystemReset();  // Force a system reset

            // Stop motors
            analogWrite(MOTOR_1_PIN, 1100); // Motor 1
            analogWrite(MOTOR_2_PIN, 1100); // Motor 2

        }
    }
}

void setupKillSwitch() {
    pinMode(KILL_SWITCH_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(KILL_SWITCH_PIN), killSwitchISR, CHANGE);
}