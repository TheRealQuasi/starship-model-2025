
#include "Optical_flow.h"

volatile bool flowDataReady = false; // Flag to indicate if new data is available

// Using digital pin 10 for chip select
Bitcraze_PMW3901 flow(10); // SPI CS pin 10 for optical flow sensor
// MOSI: 11, MISO: 12, SCK: 13, INT: 2

 // Interrupt service routine for optical flow sensor
void flowISR() {
    flowDataReady = true; // Set the flag to indicate new data is available
}

void setupFlowISR() {
    pinMode(PMW3901_INT_PIN, INPUT); // Set the interrupt pin as input
    attachInterrupt(digitalPinToInterrupt(PMW3901_INT_PIN), flowISR, RISING); // Attach interrupt on rising edge
}

bool Optical_flow::begin() {
    // Set up the interrupt for the optical flow sensor
    setupFlowISR();
    Serial.println("Optical flow sensor interrupt initiated.");
    return flow.begin();
}

bool Optical_flow::readMotion( int16_t &deltaX, int16_t &deltaY) { //, int16_t &qual)
    // Get motion count since last call
    flow.readMotionCount(&deltaX, &deltaY);

    delay(100);
    return true;
}
 
 //The  begin()  function initializes the I2C bus and the PMW3901 sensor. The  getData()  function reads the delta X, delta Y, and quality values from the sensor. 
 //The  Optical_flow  class is used in the  main.cpp  file to read the optical flow sensor data and print it to the serial monitor.

