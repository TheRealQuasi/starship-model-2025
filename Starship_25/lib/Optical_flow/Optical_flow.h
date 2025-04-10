
#include <Wire.h>
#include <Arduino.h>
#include "Bitcraze_PMW3901.h"

#define PMW3901_INT_PIN 2 // Interrupt pin for the PMW3901 sensor

// Interrupt service routine for optical flow sensor
extern volatile bool flowDataReady; // Flag to indicate if new data is available

void flowISR();
void setupFlowISR(); // Function to set up the interrupt for the optical flow sensor


class Optical_flow {
public:
    bool begin();
    bool readMotion( int16_t &dx, int16_t &dy);  //, int16_t &qual);

private:
    //Bitcraze_PMW3901 flow;
};