#include <Arduino.h>
#include <Wire.h>

// ========= Constants ==========

// Sensor adresses for I2C
#define IMU_ADR 0x68 // Replace with sensor's address
#define PRESSURE_SENSOR_ADR 0x69 // Replace with sensor's address

// put function declarations here:
int myFunction(int, int);



// ========= Setup ==========

void setup() {
  Wire.begin(); // Initialize I2C bus
  Serial.begin(9600); // Initialize serial communication for debugging
}



// ========= Loop ==========

void loop() {
  // Request data from IMU
  Wire.requestFrom(IMU_ADR, 1); // Request 1 byte
  if(Wire.available()) {
    char c = Wire.read(); // Read the byte
    Serial.print("IMU: ");
    Serial.println(c, HEX); // Print the byte
  }

  // Request data from Pressure Sensor
  Wire.requestFrom(PRESSURE_SENSOR_ADR, 1); // Request 1 byte
  if(Wire.available()) {
    char c = Wire.read(); // Read the byte
    Serial.print("Pressure Sensor: ");
    Serial.println(c, HEX); // Print the byte
  }

  delay(1000); // Delay between reads
}
