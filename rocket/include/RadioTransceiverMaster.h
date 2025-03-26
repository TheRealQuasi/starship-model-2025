/*
* 
* 
* By Emlzdev (Emil Reinfeldt)
*/
#pragma once

#ifndef RADIOTRANSCEIVERMASTER_H
#define RADIOTRANSCEIVERMASTER_H

#include <Arduino.h>
#include <SPI.h>
#include "RF24.h"
#include <nRF24L01.h>

/**
 * @brief Initializes the radio transceiver with the given parameters.
 * 
 * @param radio A reference to the RF24 object representing the radio transceiver.
 * @param level The power level of the radio transceiver.
 * @param speed The data rate of the radio transceiver.
 * @param channel The channel on which the radio transceiver operates.
 */
void initRadio(uint8_t level, 
                rf24_datarate_e speed,
                uint8_t channel
                );

/**
 * @brief Transmits data using the radio transceiver and receives acknowledgment data.
 * 
 * @param radio A reference to the RF24 object representing the radio transceiver.
 * @param dataFrame The data to be transmitted.
 * @param ackData The acknowledgment data received after transmission.
 * @param newControllerData A flag indicating whether new controller data is available.
 * @param prevMillis The object previous time in milliseconds.
 * @return true if the data transmission was successful, false otherwise.
 */
/* bool transmitData(  PacketData& dataFrame, 
                    ControlData& ackData, 
                    bool& newControllerData, 
                    unsigned long& prevMillis
                    ); */


void transmitFlightData(PacketData& dataToSend, ControlData& ackData);
void transmitState(States state, ControlData& ackData);
// void transmitErrorMsg(Errors errCode, ControlData& ackData);

#endif // RADIOTRANSCEIVERMASTER_H