/*
* This is the main file for the Groundcontroller to the Starship model. 
* It contains the setup and loop functions, as well as the functions for initializing and reading the joysticks.
* Recieves live flight data from the Starship model and prints it to the serial monitor.
* 
* By Emlzdev (Emil Reinfeldt)
*/


#include <Arduino.h>
#include "RF24.h"
#include <nRF24L01.h>
#include "GlobalDecGControl.h"
#include "RadioTransceiverSlave.h"


// =============================================================================================
//  Definitions and Configuration
// =============================================================================================
 
// Define the pins used for the nRF24L01 transceiver module (CE, CSN)
#define CE_PIN 7
#define CSN_PIN 8
// Define transmit power level | RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
#define RF24_PA_LEVEL RF24_PA_MIN
// Define speed of transmission | RF24_250KBPS, RF24_1MBPS, RF24_2MBPS
#define RF24_SPEED RF24_2MBPS
// What radio channel to use (0-127). The same on all nodes must match exactly.
#define RF24_CHANNEL 124

// Baudrate for serial communication 
#define BAUDRATE 115200

// Instantiate an object for the nRF24L01 transceiver
RF24 radio(CE_PIN, CSN_PIN);

// Indicateds if there is new data to be read from the radio
bool newData = false;

/* This variable is used to store the received data from the Starship model, such as timestamps, 
positions, accelerations, and other relevant flight data. 
*/
PacketData receiverData;

/* This variable is used to store control data that will be sent to the Starship model 
via the radio communication module. It could include information such as joystick input
values, arm switch status, thrust level, and other control parameters needed for the 
Starship model to operate or be overridden. 
*/
ControlData controllerData;


// =============================================================================================
//  Functions
// =============================================================================================

//Assign default input received values
/* void setInputDefaultValues()
{
  // TODO: implement controller with joystick input
  // The middle position for joystick. (254/2=127)
  senderData.armSwitch = LOW;
  senderData.thrustSlider = 0;
  senderData.lxAxisValue = 127;
  senderData.lyAxisValue = 127; 
} */

/**
 * The function `printData` prints received data values if `newData` is true and then sets `newData` to
 * false.
 */
void printData() {
  if (newData == true) {
    // Print the received data
    Serial.print("Time Stamp: ");
    Serial.println(receiverData.timeStamp);
    Serial.print("Position X: ");
    Serial.println(receiverData.posXValue);
    Serial.print("Position Y: ");
    Serial.println(receiverData.posYValue);
    Serial.print("Position Z: ");
    Serial.println(receiverData.posZValue);
    Serial.print("Acceleration X: ");
    Serial.println(receiverData.accXValue);
    Serial.print("Acceleration Y: ");
    Serial.println(receiverData.accYValue);
    Serial.print("Acceleration Z: ");
    Serial.println(receiverData.accZValue);
    Serial.print("Gamma: ");
    Serial.println(receiverData.gamValue);
    Serial.print("Acceleration Gamma: ");
    Serial.println(receiverData.accGamValue);
    Serial.print("Beta: ");
    Serial.println(receiverData.betaValue);
    Serial.print("Acceleration Beta: ");
    Serial.println(receiverData.accBetaValue);
    Serial.println("");
    newData = false;
  }
}



// =============================================================================================
//  Main Program
// =============================================================================================

void setup(){
  Serial.begin(BAUDRATE); 
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }

  Serial.println("==== Controller initializing... ====");
  Serial.println("");
  delay(1000); // Delay for 1 second before starting the program
  // Initialize the radio communication module
  initRadio(radio, RF24_PA_LEVEL, RF24_SPEED, RF24_CHANNEL, controllerData);
  
  Serial.println("Init complete!");
}

void loop(){

  // Receive data
  newData = receiveData(radio, receiverData, controllerData);

  printData();

}