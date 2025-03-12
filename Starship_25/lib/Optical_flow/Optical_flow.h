
#include <Wire.h>
#include <Arduino.h>
#include "Bitcraze_PMW3901.h"

class Optical_flow {
public:
    bool begin();
    bool readMotion( int16_t &dx, int16_t &dy);  //, int16_t &qual);

private:
    //Bitcraze_PMW3901 flow;
};