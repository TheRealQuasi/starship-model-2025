/*
* This is the file for the Barometric sensor of the Starship model. 
* 
* 
* By Emlzdev (Emil Reinfeldt)
*/

// =============================================================================================
//  Preprocessor Definitions
// =============================================================================================

#include "Barometer.h"



// =============================================================================================
//  Variables/Objects
// =============================================================================================

// Pressure sensor object
Dps3xx Dps3xxPressureSensor = Dps3xx();

int16_t temp_mr = TEMP_MR;      // Temperature measure rate

int16_t temp_osr = TEMP_OSR;    // Temperature oversampling rate
 
int16_t prs_mr = PRS_MR;        // Pressure measure rate

int16_t prs_osr = PRS_OSR;      // Pressure oversampling rate

float seaLevelPressure = 1013.25; // Pressure at sea level in hPa



// =============================================================================================
//  Functions
// =============================================================================================

// Initialize pressure sensor
bool initDPS310(){

  #ifdef DEBUG
    Serial.println("Initializing Pressuresensor over I2C...");
  #endif
  Dps3xxPressureSensor.begin(Wire);
  #ifdef DEBUG
    Serial.println("Pressuresensor I2C initialized successfully");
  #endif

  /*
   * startMeasureBothCont enables background mode
   * temperature and pressure ar measured automatically
   * High precision and hgh measure rates at the same time are not available.
   * Consult Datasheet (or trial and error) for more information
   * int16_t ret = Dps3xxPressureSensor.startMeasureBothCont(temp_mr, temp_osr, prs_mr, prs_osr);
   */
  /*
   * Use one of the commented lines below instead to measure only temperature or pressure
   * int16_t ret = Dps3xxPressureSensor.startMeasureTempCont(temp_mr, temp_osr);
   * int16_t ret = Dps3xxPressureSensor.startMeasurePressureCont(prs_mr, prs_osr);
   */
  int16_t ret = Dps3xxPressureSensor.startMeasurePressureCont(prs_mr, prs_osr);

  if (ret != 0)
  {
    #ifdef DEBUG
        Serial.print("Init Dsp310 FAILED! ret = ");
        Serial.println(ret);
    #endif
    // TODO: Initialization failed, send error to computer

    return 0;
  }
  else
  {
    #ifdef DEBUG
        Serial.println("Init Dsp310 complete!");
    #endif

    return 1;
  }
}

// Read temperature and pressure
float readPS(){

  static unsigned long lastTime = 0; // Keep track of the last time we read the sensor
  unsigned long currentTime = millis(); // Get the current time
  
  // Only read the sensor if at least the specified delay in milliseconds have passed since the last reading
  if (currentTime - lastTime >= PS_DELAY) {

    uint8_t pressureCount = 20;
    float pressure[pressureCount];
    uint8_t temperatureCount = 20;
    float temperature[temperatureCount];
    float height = 0; // Meters above ground level

    /*
    * This function writes the results of continuous measurements to the arrays given as parameters
    * The parameters temperatureCount and pressureCount should hold the sizes of the arrays temperature and pressure when the function is called
    * After the end of the function, temperatureCount and pressureCount hold the numbers of values written to the arrays
    * Note: The Dps3xx cannot save more than 32 results. When its result buffer is full, it won't save any new measurement results
    */
    int16_t ret = Dps3xxPressureSensor.getContResults(temperature, temperatureCount, pressure, pressureCount);

    if (ret != 0)
    {
      #ifdef DEBUG
        Serial.println();
        Serial.println();
        Serial.print("FAIL! ret = ");
        Serial.println(ret);
      #endif

      // TODO: Send error message to ground control


      return -1;

    }
    else
    {
      #ifdef DEBUG
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
      #endif

      height = calculateHeight(pressure[0]);
    }
    lastTime = currentTime; // Update the last time we read the sensor
    return height;
  }
  return -1;
}

// Calculate the height from ground level in meters
float calculateHeight(float pressure){
  return 44330 * (1.0 - pow((pressure / seaLevelPressure), 0.1903)); // Height in meters
}
