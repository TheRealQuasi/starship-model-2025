#include <Arduino.h>
#include <Wire.h>
#include <Dps310.h>
#include "I2Cdev.h"
#include "MPU6050.h"


// TODO: Pressuresensor: The 0 Ohm resistors are soldered as shown on the right picture
// TODO: Create a pull-up resistor for I2C bus



// ========= Constants ==========

// Sensor adresses for I2C
#define IMU_ADR 0x68 //b1101000
//#define PRESSURE_SENSOR_ADR 0x77 // Default and does not need to be given
#define BAUDRATE 9600 // Baudrate for serial communication to terminal

// LED pin
#define LED_PIN 13



// ========= Variables/Objects ==========

// Pressure sensor object
Dps310 Dps310PressureSensor = Dps310();

//temperature measure rate (value from 0 to 7)
//2^temp_mr temperature measurement results per second
int16_t temp_mr = 2;
//temperature oversampling rate (value from 0 to 7)
//2^temp_osr internal temperature measurements per result
//A higher value increases precision
int16_t temp_osr = 2;
//pressure measure rate (value from 0 to 7)
//2^prs_mr pressure measurement results per second
int16_t prs_mr = 2;
//pressure oversampling rate (value from 0 to 7)
//2^prs_osr internal pressure measurements per result
//A higher value increases precision
int16_t prs_osr = 2;

// IMU object
MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

// LED pin
bool blinkState = false;



// ========= Functions ==========

void initSensors(){
  // join I2C bus
  Wire.begin();

  // initialize IMU
  Serial.println("Initializing IMU over I2C...");
  accelgyro.initialize();

  // verify connection
  Serial.println("Testing IMU connection...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  
  // Initialize pressure sensor on I2C bus
  Serial.println("Initializing Pressuresensor over I2C...");
  Dps310PressureSensor.begin(Wire);
  //startMeasureBothCont enables background mode
  //temperature and pressure ar measured automatically
  //High precision and hgh measure rates at the same time are not available.
  //Consult Datasheet (or trial and error) for more information
  int16_t ret = Dps310PressureSensor.startMeasureBothCont(temp_mr, temp_osr, prs_mr, prs_osr);
  // NOTE: Removed third parameter ", 1U" from function on row 567 and 777 in .pio\libdeps\teensy41\DigitalPressureSensor\src\DpsClass.cpp. 
  // This is a bug in the library i guess
  //Use one of the commented lines below instead to measure only temperature or pressure
  //int16_t ret = Dps310PressureSensor.startMeasureTempCont(temp_mr, temp_osr);
  //int16_t ret = Dps310PressureSensor.startMeasurePressureCont(prs_mr, prs_osr);
  if(ret != 0){
    Serial.print("Init failed! ret: ");
    Serial.println(ret);
  }
  else{
    Serial.println("Init successful!");
  }
  
  // configure microcontroller LED for TX/RX status
  pinMode(LED_PIN, OUTPUT);
}

void readIMU(){
  // Read sensor data from I2C bus
  // read raw accel/gyro measurements from device
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

  // these methods are also available
  //accelgyro.getAcceleration(&ax, &ay, &az);
  //accelgyro.getRotation(&gx, &gy, &gz);

  // display tab-separated accel/gyro x/y/z values
  Serial.print("a/g/m:\t");
  Serial.print(ax); Serial.print("\t");
  Serial.print(ay); Serial.print("\t");
  Serial.print(az); Serial.print("\t");
  Serial.print(gx); Serial.print("\t");
  Serial.print(gy); Serial.print("\t");
  Serial.print(gz); Serial.print("\t");
  Serial.print(mx); Serial.print("\t");
  Serial.print(my); Serial.print("\t");
  Serial.println(mz);  
}

void readPS(){
  //read temperature and pressure

  uint8_t pressureCount = 20;
  float pressure[pressureCount];
  uint8_t temperatureCount = 20;
  float temperature[temperatureCount];

  //This function writes the results of continuous measurements to the arrays given as parameters
  //The parameters temperatureCount and pressureCount should hold the sizes of the arrays temperature and pressure when the function is called
  //After the end of the function, temperatureCount and pressureCount hold the numbers of values written to the arrays
  //Note: The Dps310 cannot save more than 32 results. When its result buffer is full, it won't save any new measurement results
  int16_t ret = Dps310PressureSensor.getContResults(temperature, temperatureCount, pressure, pressureCount);

  if (ret != 0)
  {
    Serial.println();
    Serial.println();
    Serial.print("FAIL! ret = ");
    Serial.println(ret);
  }
  else
  {
    Serial.println();
    Serial.println();
    Serial.print(temperatureCount);
    Serial.println(" temperature values found: ");
    for (int16_t i = 0; i < temperatureCount; i++)
    {
      Serial.print(temperature[i]);
      Serial.println(" degrees of Celsius");
    }

    Serial.println();
    Serial.print(pressureCount);
    Serial.println(" pressure values found: ");
    for (int16_t i = 0; i < pressureCount; i++)
    {
      Serial.print(pressure[i]);
      Serial.println(" Pascal");
    }
  }

  //Wait some time, so that the Dps310 can refill its buffer
  delay(10000);
}



// ========= Setup ==========

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(BAUDRATE); 

  // Initialize sensors
  initSensors();

  Serial.println("Init complete!");
}



// ========= Loop ==========

void loop() {
  readIMU();
  readPS();

  // blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
}
