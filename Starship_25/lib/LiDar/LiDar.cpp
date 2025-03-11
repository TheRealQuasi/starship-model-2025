#include "lidar.h"

TFMPI2C::TFMPI2C(){}
TFMPI2C::~TFMPI2C(){}

//sensor.recoverI2CBus();

bool LiDAR::begin() {
    Wire.begin();            // Initialize two-wire interface
    Serial.begin( 115200);   // Initialize terminal serial port
    //printf_begin();          // Initialize printf library.
	delay(20);

    Serial.flush();          // Flush serial write buffer
    while( Serial.available())Serial.read();  // flush serial read buffer

    // Say hello
    Serial.println();
    Serial.println( "*****************************");
    Serial.println( "Will scan the I2C bus for all devices");
    Serial.println( "and display the first address found.");
    Serial.println( "Enter a new address in decimal format.");
    Serial.println( "Confirm 'Y/N' in 5 seconds. Default is 'N'.");
    Serial.println( "When done, close this window to halt program.");
    delay(1000);

    return true;
}

/*
uint16_t LiDAR::getData( int16_t &dist, int16_t &flux, int16_t &temp, uint8_t addr) {
    //uint8_t addr = 0x62; // Default I2C address for TFMini
    uint8_t buf[9]; // Array to hold measurement data
    uint16_t distance = 0; // Reset distance
    uint16_t strength = 0; // Reset signal strength
    uint16_t temperature = 0; // Reset temperature

    // Send command packet to LiDAR
    Wire.beginTransmission( addr); // Start transmission to device
    Wire.write( (uint8_t)0x02); // Send distance command
    Wire.write( (uint8_t)0x00); // Send data byte 1
    Wire.write( (uint8_t)0x00); // Send data byte 2
    Wire.write( (uint8_t)0x00); // Send data byte 3
    Wire.write( (uint8_t)0x01); // Send data byte 4
    Wire.endTransmission(); // End transmission

    // Wait for LiDAR to process data
    delay( 100);

    // Read data from LiDAR
    Wire.requestFrom( addr, 9); // Request 9 bytes from LiDAR
    if( Wire.available() >= 9) { // If 9 bytes are available
        for( uint8_t i = 0; i < 9; i++) { // Loop through all 9 bytes
            buf[i] = Wire.read(); // Read byte from LiDAR
        }

        // Calculate distance
        distance = (uint16_t)buf[2] + (uint16_t)buf[3] * 256;

        // Calculate signal strength
        strength = (uint16_t)buf[4] + (uint16_t)buf[5] * 256;

        // Calculate temperature
        temperature = (uint16_t)buf[6] + (uint16_t)buf[7] * 256;

        // Assign values to variables
        dist = distance;
        flux = strength;
        temp = temperature;
    }

    return distance;
}
*/

bool LiDAR::getData( int16_t &dist, int16_t &flux, int16_t &temp, uint8_t addr) {
    return sensor.getData(dist, flux, temp, addr);
}


