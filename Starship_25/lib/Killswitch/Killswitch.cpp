#include "Killswitch.h"

volatile bool emergencyStop = false;
volatile unsigned long pulseStart = 0;
volatile unsigned long pulseWidth = 0;
volatile unsigned int start_count = 0;
volatile bool manualControl = false;

void killSwitchISR() {

    if (digitalRead(KILL_SWITCH_PIN) == HIGH) {
        pulseStart = micros();  // Record the timestamp at the rising edge
    } else {
        pulseWidth = micros() - pulseStart;  // Compute pulse width on falling edge
        if (pulseWidth > 1500) {  // Adjust threshold based on your WFLY signal range
            emergencyStop = true;
            start_count = 0;  // Reset the start count

            // Stop motors
            //analogWrite(MOTOR_1_PIN, 1050); // Motor 1
            //analogWrite(MOTOR_2_PIN, 1050); // Motor 2
            motorsWrite(1, 1050);
            motorsWrite(2, 1050);

        
        } else if (start_count > KILL_SWITCH_START_THRESHOLD) {  //Reduce possibility of false trigger
            emergencyStop = false;  // Reset the emergency stop flag if pulse width is below threshold
            start_count = 0;
        }
        start_count++;
        
        if (pulseWidth < 1000) {  // Adjust threshold based on your WFLY signal range
            // manual control
            manualControl = true;
        } else if (pulseWidth < 1500 ){
            //automated control
            manualControl = false;
        }
    }
}

void setupKillSwitch() {
    pinMode(KILL_SWITCH_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(KILL_SWITCH_PIN), killSwitchISR, CHANGE);
}


