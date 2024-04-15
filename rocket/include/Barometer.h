/*
* 
* By Emlzdev (Emil Reinfeldt)
*/


#ifndef BAROMETER_H
#define BAROMETER_H

#include <Arduino.h>
#include <Wire.h>
#include <Dps3xx.h>
#include "settings.h"
#include "GlobalDecRocket.h"

// Functions
bool initDPS310();
float readPS();
float calculateHeight(float pressure);

#endif // BAROMETER_H