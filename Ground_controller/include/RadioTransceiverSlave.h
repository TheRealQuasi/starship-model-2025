/*
* 
* 
* By Emlzdev (Emil Reinfeldt)
*/
#pragma once

#ifndef RADIOTRANSCEIVERSLAVE_H
#define RADIOTRANSCEIVERSLAVE_H

#include <Arduino.h>
#include <SPI.h>
#include "RF24.h"
#include <nRF24L01.h>

/**
 * @brief Initializes the radio with the given parameters.
 * 
 * @param radio The RF24 radio to initialize.
 * @param level The power level to set.
 * @param speed The data rate to set.
 * @param channel The channel to use.
 * @param controllerData The controller data object to use.
 */
void initRadio( uint8_t level, 
                rf24_datarate_e speed,
                uint8_t channel,
                ControlData& controllerData
                );

/**
 * @brief Receives data from the radio.
 * 
 * @param radio The RF24 radio to receive data from.
 * @param receiverData The data received.
 * @param controllerData The controller data object to use.
 * @return true if data was received successfully, false otherwise.
 */
//bool receiveData(PacketData& receiverData, ControlData& controllerData);

bool receivePacket(PacketData& receiverData, ControlData& controllerData);


#endif // RADIOTRANSCEIVERSLAVE_H
