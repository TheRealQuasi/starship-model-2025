
#include "Optical_flow.h"

// Using digital pin 10 for chip select
Bitcraze_PMW3901 flow(10); // SPI CS pin 10 for optical flow sensor
// MOSI: 11, MISO: 12, SCK: 13

bool Optical_flow::begin() {
    return flow.begin();
}

bool Optical_flow::readMotion( int16_t &deltaX, int16_t &deltaY) { //, int16_t &qual)
    // Get motion count since last call
    flow.readMotionCount(&deltaX, &deltaY);

    Serial.print("X: ");
    Serial.print(deltaX);
    Serial.print(", Y: ");
    Serial.print(deltaY);
    Serial.print("\n");

    delay(100);
    return true;
}
 
 //The  begin()  function initializes the I2C bus and the PMW3901 sensor. The  getData()  function reads the delta X, delta Y, and quality values from the sensor. 
 //The  Optical_flow  class is used in the  main.cpp  file to read the optical flow sensor data and print it to the serial monitor.