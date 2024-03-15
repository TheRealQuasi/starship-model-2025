#include <Arduino.h>
#include <SPI.h>
#include "printf.h"
#include "RF24.h"
#include <nRF24L01.h>


// =============================================================================================
//  Definitions and Configuration
// =============================================================================================

// Define the pins used for the nRF24L01 transceiver module (CE, CSN)
#define CE_PIN 7
#define CSN_PIN 8

// Define the maximum number of bytes to be sent in a packet
#define PACKET_SIZE 32

// Signal timeout in milli seconds. We will reset the data if no signal
#define SIGNAL_TIMEOUT 500 
// Baudrate for serial communication 
#define BAUDRATE 250000 

// Instantiate an object for the nRF24L01 transceiver
RF24 radio(CE_PIN, CSN_PIN);

// Let these addresses be used for the pair
uint8_t address[][6] = { "1Node", "2Node" };

// This is last received time of the signal
unsigned long lastRecvTime = 0; 

// Create a Packet to hold the data
struct PacketData
{
  byte timeStamp;
  float posXValue;
  float posYValue;
  float posZValue;
  float accXValue;
  float accYValue; 
  float accZValue; 
  float gamValue;
  float accGamValue;
  float betaValue;
  float accBetaValue; 
};
PacketData receiverData;

struct Packet {
  byte sequenceNumber;
  byte data[PACKET_SIZE - sizeof(byte)]; // subtract the size of sequenceNumber
};





// =============================================================================================
//  Functions
// =============================================================================================

//Assign default input received values
void setInputDefaultValues()
{
  // TODO: implement controller with joystick input
  // The middle position for joystick. (254/2=127)
  /* receiverData.lxAxisValue = 127;
  receiverData.lyAxisValue = 127;
  receiverData.rxAxisValue = 127;
  receiverData.ryAxisValue = 127;
  receiverData.lPotValue = 0;  
  receiverData.rPotValue = 0;    
  receiverData.switch1Value = LOW;
  receiverData.switch2Value = LOW;
  receiverData.switch3Value = LOW;
  receiverData.switch4Value = LOW;  */
}


// Receive Data
void receiveData()
{
  Packet packet;

  // Check if RF is connected and packet is available 
  if(radio.isChipConnected() && radio.available())
  {
    radio.read(&packet, sizeof(Packet));
    if (packet.sequenceNumber == 1) {
      // this is the first packet, store the data
      memcpy(&receiverData, packet.data, PACKET_SIZE - sizeof(byte));
    } 
    else if (packet.sequenceNumber == 2) {
      // this is the second packet, combine it with the first packet's data to form a PacketData
      memcpy(((byte*)&receiverData) + PACKET_SIZE - sizeof(byte), packet.data, sizeof(PacketData) - PACKET_SIZE + sizeof(byte));
    } 
    else {
      Serial.println("Unknown packet type received.");
    }

    lastRecvTime = millis();
  }
  else
  {
    //Check Signal lost.
    unsigned long now = millis();
    if ( now - lastRecvTime > SIGNAL_TIMEOUT ) 
    {
      // Signal lost. Reset the input values
      Serial.print("RF Signal lost");


      //setInputDefaultValues();
    }
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
  

  // Initiate the radio object
  Serial.println("Initializing radio...");
  radio.begin();

  // Set the transmit power to lowest available to prevent power supply related issues (not needed with power breakout board)
  Serial.println("\tSetting PA Level...");
  radio.setPALevel(RF24_PA_MIN);

  // Set the speed of the transmission to the quickest available
  Serial.println("\tSetting data rate...");
  radio.setDataRate(RF24_2MBPS);

  // Use a channel unlikely to be used by Wifi, Microwave ovens etc
  Serial.println("\tSetting channel...");
  radio.setChannel(124);

  // Open a writing and reading pipe on each radio, with opposite addresses
  Serial.println("\tSetting pipe addresses...");
  radio.openWritingPipe(address[0]);
  radio.openReadingPipe(1, address[1]);

  // Start the radio listening for data
  Serial.println("\tStart listening...");
  radio.startListening();

  Serial.println("Init complete!");
}

void loop(){

  // Receive data
  receiveData();

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

  
}