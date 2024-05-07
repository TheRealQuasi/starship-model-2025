/*
* This is wireless communication interface for the Groundcontroller to the Starship model. 
* It contains functions for initializing, reading the joysticks and sending data.
* Recieves live flight data from the Starship model and prints it to the serial monitor.
* 
* By Emlzdev (Emil Reinfeldt)
*/


// =============================================================================================
//  Preprocessor Definitions
// =============================================================================================

#include "GlobalDecGControl.h"
#include "RadioTransceiverSlave.h"



// =============================================================================================
//  Definitions and Configuration
// =============================================================================================

// Define the maximum number of bytes to be sent in a packet
#define PACKET_SIZE 32

// Signal timeout in milli seconds. We will reset the data if no signal
#define SIGNAL_TIMEOUT 5000

// Define the pins used for the nRF24L01 transceiver module (CE, CSN)
#define CE_PIN 2
#define CSN_PIN 4

// Instantiate an object for the nRF24L01 transceiver
RF24 radio(CE_PIN, CSN_PIN);

// Let these addresses be used for the pair
uint8_t address[][6] = { "1Node" };

// This is last received time of the signal
unsigned long lastRecvTime = 0; 

// This is last time "signal lost" message was printed
unsigned long lastSignalLostPrinted = 0;

// Data type
enum DataType {
  FLIGHT_DATA = 1,
  STATE_DATA = 2,
  ERROR_MSG = 3
};

// Create a Packet structure to hold the data that will be received from the other node
struct Packet {
  byte type;
  byte sequenceNumber;
  byte data[PACKET_SIZE - sizeof(byte) - sizeof(byte)]; // subtract the size of type and sequenceNumber
};



// =============================================================================================
//  Functions
// =============================================================================================

/**
 * The function `initRadio` initializes an RF24 radio object with specified power level, data rate,
 * channel, and other settings.
 * The idea is that the slave puts data in the ackPayload buffer BEFORE it receives a message from 
 * the master and then the data in the ackPayload buffer is sent automatically as part of the normal 
 * acknowledgment process. This is the slave node.
 * 
 * @param radio The `radio` parameter is an object of type `RF24`, which is a class representing an
 * NRF24L01(+) radio transceiver. This object is used to communicate with the NRF24 module and
 * configure its settings for wireless communication.
 * @param level The `level` parameter in the `initRadio` function is used to set the transmit power
 * level of the radio. It is a uint8_t type parameter, which means it is an unsigned 8-bit integer. The
 * transmit power level can be set to the lowest available level to prevent power supply
 * @param speed The `speed` parameter in the `initRadio` function is used to specify the data rate for
 * transmission. It is of type `rf24_datarate_e`, which is an enumeration representing different data
 * rates supported by the RF24 radio module. The available options for `speed` typically include values
 * @param channel The `channel` parameter in the `initRadio` function is used to set the radio
 * frequency channel on which the RF24 module will operate. This parameter allows you to specify a
 * specific channel for communication. It is important to choose a channel that is not being used by
 * other devices in the vicinity to minimize interference.
 * @param controllerData The `controllerData` parameter in the `initRadio` function is of type `ControlData`,
 * which represents the control data struct defined in the `GlobalDecGControl.h` file. This struct contains
 * information about the state of the Arduino-based controller, such as joystick positions, switch states, etc.
 */
void initRadio( uint8_t level, 
                rf24_datarate_e speed,
                uint8_t channel,
                ControlData& controllerData
                )
{
  // Initialize the transceiver on the SPI bus
  Serial.println("Initializing radio...");
  while (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
    delay(3000);
  }

  /* 
  * Set the transmit power to lowest available to prevent power supply related issues
  * (can use higher power with adapter board) 
  */
  Serial.println("\tSetting PA Level...");
  radio.setPALevel(level);

  // Set the speed of the transmission to the quickest available
  Serial.println("\tSetting data rate...");
  radio.setDataRate(speed);

  // Open a reading pipe on the radio
  Serial.println("\tSetting pipe address...");
  radio.openReadingPipe(1, address[0]);

  // Set the auto acknowledge feature to true (may not be necessary)
  //radio.setAutoAck(true);

  // Enable payload by acknowledgement
  Serial.println("\tEnabling ack payload...");
  radio.enableAckPayload();

  /* 
  * When dynamic payloads are enabled, it allows the radio module to automatically 
  * adjust the payload size based on the actual data being sent or received. 
  */
  Serial.println("\tEnabling dynamic payloads...");
  radio.enableDynamicPayloads();

  // Use a channel unlikely to be used by Wifi, Microwave ovens etc
  Serial.println("\tSetting channel...");
  radio.setChannel(channel);

  // Print the details of the radio
  Serial.println("\tRadio settings:");
  //radio.printPrettyDetails();

  /*   // Clear the buffer (DEBUG)
  Packet clearPacket;
  while (radio.available()) {
    // Read each available payload, but don't do anything with it
    radio.read(&clearPacket, sizeof(Packet));
    Serial.println("Clearing buffer");
  } */

  // Start handshake with the rocket
  /* Serial.println("\tStarting handshake...");
  char ackMsg[] = "ACK";
  if (!radio.writeAckPayload(1, &ackMsg, sizeof(ackMsg))){
    Serial.println("Failed to create acknowledgment message");
  }
  char handshakeMsg[6];  */

  // Start the radio listening for data
  Serial.println("\tStart listening...");
  radio.startListening();

  /* while ( true ) {
    if (radio.available()) {
      
      radio.read(&handshakeMsg, sizeof(handshakeMsg));

      if (strcmp(handshakeMsg, "HELLO") == 0) {
        Serial.println("Handshake message received");
        break;
      } else {
        Serial.println("Unexpected message received");
      }
    }
    else {
      Serial.println("Handshake failed: no handshake received");
    }
    delay(1000); // wait a second before trying again
  } */

  // Pre-load data to send from buffer
  Serial.println("\tPre-loading data...");
  radio.writeAckPayload(1, &controllerData, sizeof(ControlData));

}

/**
 * The function `receiveData` reads incoming packets from an RF module, processes them based on
 * sequence number, and handles signal loss detection.
 * 
 * @param radio The `radio` parameter is of type `RF24&`, which is a reference to an object of the RF24
 * class. This object is used for communication with an RF module, for sending and receiving
 * data wirelessly.
 * @param receiverData The `receiverData` parameter in the `receiveData` function is of type
 * `PacketData`, which is a custom data structure used to store received packet data. It is a
 * struct or class that contains data fields related to the received packets.
 * 
 * @return The function `receiveData` is returning a boolean value - `true` or `false` based on whether
 * the RF24 chip is connected and a packet is available for reading.
 */
/* bool receiveData(PacketData& receiverData, ControlData& controllerData)
{
  Packet packet;
  bool newData = false;

  // Check if RF is connected and packet is available 
  if(radio.isChipConnected() && radio.available())
  {
    radio.read(&packet, sizeof(Packet));
    if (packet.sequenceNumber == 1) {
      Serial.println("Packet 1 received");
      // this is the first packet, store the data
      memcpy(&receiverData, packet.data, PACKET_SIZE - sizeof(byte));
      newData = true;
    } 
    else if (packet.sequenceNumber == 2) {
      Serial.println("Packet 2 received");
      // this is the second packet, combine it with the first packet's data to form a PacketData
      memcpy(((byte*)&receiverData) + PACKET_SIZE - sizeof(byte), packet.data, sizeof(PacketData) - PACKET_SIZE + sizeof(byte));
      newData = true;
    } 
    else {
      Serial.println("Unknown packet type received.");
    }

    // Load the payload for the next sendoff
    radio.writeAckPayload(1, &controllerData, sizeof(ControlData));

    lastRecvTime = millis();
    return newData;
  }
  else
  {
    //Check Signal lost.
    unsigned long now = millis();
    if ( now - lastRecvTime > SIGNAL_TIMEOUT ) 
    {
      // Signal lost. TODO: Reset the input values
      if (now - lastSignalLostPrinted > 5000) { // only print the message if at least 5 seconds have passed
        Serial.println("RF Signal lost");
        lastSignalLostPrinted = now; // update the last print time
      }
    }
    return false;
  }
} */

void receiveFlightData(PacketData& receiverData, Packet& packet)
{
  if (packet.sequenceNumber == 1) {
    Serial.println("Data Packet 1 received");
    // this is the first packet, store the data
    memcpy(&receiverData, packet.data, PACKET_SIZE - sizeof(byte) - sizeof(byte));
  } 
  else if (packet.sequenceNumber == 2) {
    Serial.println("Data Packet 2 received");
    // this is the second packet, combine it with the first packet's data to form a PacketData
    size_t packet2DataSize = sizeof(PacketData) - (PACKET_SIZE - sizeof(byte) - sizeof(byte));
    memcpy(((byte*)&receiverData) + (PACKET_SIZE - sizeof(byte) - sizeof(byte)), packet.data, packet2DataSize);
  } 
  else {
    Serial.println("Unknown packet type received.");
  }
}

void receiveState(Packet& packet)
{
  switch (*reinterpret_cast<int*>(packet.data)) { //May need to use packet.data[0] instead of packet.data for byte access
    case SERVO_AND_MOTOR_INIT:
      Serial.println("State: SERVO_AND_MOTOR_INIT");
      break;
    case ESC_CALIBRATION:
      Serial.println("State: ESC_CALIBRATION");
      break;
    case GIMBAL_TEST:
      Serial.println("State: GIMBAL_TEST");
      break;
    case IMU_CALIBRATION:
      Serial.println("State: IMU_CALIBRATION. Please wait for the calibration to finish. You will see disconnected RF radio message until the calibration is done.");
      break;
    case FILTER_WARMUP:
      Serial.println("State: FILTER_WARMUP");
      break;
    case SYSTEM_READY:
      Serial.println("State: SYSTEM_READY");
      break;
    default:
      Serial.println("Unknown state received.");
      break;
  }
}

bool receivePacket(PacketData& receiverData, ControlData& controllerData)
{
  Packet packet;
  bool newData = false;

  if(radio.isChipConnected() && radio.available())
  {
    radio.read(&packet, sizeof(Packet));
    
    switch(packet.type) {
      case FLIGHT_DATA:
        receiveFlightData(receiverData, packet);
        newData = true;
        break;
      case STATE_DATA:
        receiveState(packet);
        newData = true;
        break;
      case ERROR_MSG:
        // this is an error message
        Serial.print("Error message received: ");
        /* Serial.println(packet.data); */
        break;
      default:
        Serial.println("Unknown packet type received.");
        break;
    }

    // Load the payload for the next sendoff
    radio.writeAckPayload(1, &controllerData, sizeof(ControlData));

    lastRecvTime = millis();
    return newData;
  }
  else
  {
    //Check Signal lost.
    unsigned long now = millis();
    if ( now - lastRecvTime > SIGNAL_TIMEOUT ) 
    {
      // Signal lost. TODO: Reset the input values
      if (now - lastSignalLostPrinted > 5000) { // only print the message if at least 5 seconds have passed
        Serial.println("RF Signal lost");
        lastSignalLostPrinted = now; // update the last print time
      }
    }
    return false;
  }
}
