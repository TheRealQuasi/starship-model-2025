/*
* This is wireless communication interface for the Rocket to the Starship model. 
* It contains functions for ...
* Sends live flight data to groundcontrol.
* 
* By Emlzdev (Emil Reinfeldt)
*/


// =============================================================================================
//  Preprocessor Definitions
// =============================================================================================

#include "GlobalDecRocket.h"
#include "RadioTransceiverMaster.h"


// =============================================================================================
//  Definitions and Configuration
// =============================================================================================

// Define the maximum number of bytes to be sent in a packet
#define PACKET_SIZE 32

// Signal timeout in milli seconds. We will reset the data if no signal
#define SIGNAL_TIMEOUT 5000 

// Let these addresses be used for the pair
uint8_t address[][6] = { "1Node" };

// This is last received time of the signal
unsigned long lastRecvTime = 0; 

// This is last time "signal lost" message was printed
unsigned long lastSignalLostPrinted = 0;

// Create a Packet structure to hold the data that will be received from the other node
struct Packet {
  byte sequenceNumber;
  byte data[PACKET_SIZE - sizeof(byte)]; // subtract the size of sequenceNumber
};



// =============================================================================================
//  Functions
// =============================================================================================

/**
 * The function `initRadio` initializes an RF24 radio object with specified power level, data rate,
 * channel, and other settings.
 * The idea is that the slave puts data in the ackPayload buffer BEFORE it receives a message from 
 * the master and then the data in the ackPayload buffer is sent automatically as part of the normal 
 * acknowledgment process. This is the master node.
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
 * other devices in the vicinity to
 */
void initRadio( RF24& radio,
                uint8_t level, 
                rf24_datarate_e speed,
                uint8_t channel
                )
{
  // Initiate the radio object
  #ifdef DEBUG
    Serial.println("Initializing radio...");
  #endif
  radio.begin();

  // Set the transmit power to lowest available to prevent power supply related issues (not needed with power breakout board)
  #ifdef DEBUG
    Serial.println("\tSetting PA Level...");
  #endif
  radio.setPALevel(level);

  // Set the speed of the transmission to the quickest available
  #ifdef DEBUG
    Serial.println("\tSetting data rate...");
  #endif
  radio.setDataRate(speed);

  // Enable payload by acknowledgement
  #ifdef DEBUG
    Serial.println("\tEnabling ack payload...");
  #endif
  radio.enableAckPayload();

  /*
  * Sets the delay and number of retries upon failed transmission.
  * 5 gives a 1500 µsec delay which is needed for a 32 byte ackPayload
  */
  #ifdef DEBUG
    Serial.println("\tSetting retries...");
  #endif
  radio.setRetries(5,5); // delay, count

  // Use a channel unlikely to be used by Wifi, Microwave ovens etc
  #ifdef DEBUG
    Serial.println("\tSetting channel...");
  #endif
  radio.setChannel(channel);

  // Open a writing and reading pipe on each radio, with opposite addresses
  #ifdef DEBUG
    Serial.println("\tSetting pipe addresses...");
  #endif
  radio.openWritingPipe(address[0]);
}

/**
 * The function `checkSignalLoss` checks for signal loss based on a timeout condition and can perform
 * actions if the signal is lost.
 */
void checkSignalLoss(){
  unsigned long now = millis();
  if ( now - lastRecvTime > SIGNAL_TIMEOUT ) 
  {
    // Signal lost. TODO: Reset the input values or execute some script
    #ifdef DEBUG
      if (now - lastSignalLostPrinted > 5000) { // only print the message if at least 5 seconds have passed
        Serial.println("RF Signal lost");
        lastSignalLostPrinted = now; // update the last print time
      }
    #endif
  }
}

/**
 * The function `transmitData` sends data in two packets via an RF24 radio module, receives
 * acknowledgment and data from ground control, and handles potential transmission failures.
 * 
 * @param radio The `radio` parameter is a reference to an RF24 object, which is used for communication
 * with a wireless module.
 * @param dataFrame `dataFrame` is a reference to an object of type `PacketData`, which likely contains
 * the data you want to transmit.
 * @param ackData `ackData` is a reference to a `ControlData` object where the acknowledge data
 * received from ground control will be stored.
 * @param newControllerData The `newControllerData` parameter is a reference to a boolean variable that
 * is used to indicate whether new data has been received from the ground control. It is set to `true`
 * if new data is received during the transmission process.
 * @param prevMillis `prevMillis` is a reference to an `unsigned long` variable that is used to store
 * the previous time in milliseconds when the data transmission function was called. This variable is
 * updated within the `transmitData` function to keep track of the time of the most recent data
 * transmission.
 * 
 * @return The function `transmitData` returns a boolean value - `true` if the data transmission was
 * successful, and `false` if there was a failure during transmission.
 */
bool transmitData(  RF24& radio, 
                    PacketData& dataFrame, 
                    ControlData& ackData, 
                    bool& newControllerData, 
                    unsigned long& prevMillis
                    )
{
  PacketData dataToSend; // The data we want to send
  dataToSend = dataFrame; 

  // Split the data into two packets
  Packet packet1 = {1, {}};
  memcpy(packet1.data, &dataToSend, PACKET_SIZE - sizeof(byte));

  Packet packet2 = {2, {}};
  memcpy(packet2.data, ((byte*)&dataToSend) + PACKET_SIZE - sizeof(byte), sizeof(PacketData) - PACKET_SIZE + sizeof(byte));

  // Send the first packet
  bool rslt1;
  rslt1 = radio.write( &packet1, sizeof(Packet) );
  #ifdef DEBUG
    Serial.println("Dataframe 1 Sent ");
    Serial.print(packet1);
  #endif

  // Recive the acknowledge and data from ground control
  if (rslt1) {
    if ( radio.isAckPayloadAvailable() ) {
      radio.read(&ackData, sizeof(ackData));
      newControllerData = true;
      #ifdef DEBUG
        Serial.println("  Acknowledged and recived data: ");
        Serial.println(ackData);
      #endif
    }
    else {
      #ifdef DEBUG
        Serial.println("  Acknowledge but no data ");
      #endif
    }
  }
  else {
    #ifdef DEBUG
      Serial.println("  Tx failed");
    #endif
    return false;
  }

  // Send the second packet (if TX of first packet was successful)
  bool rslt2;
  rslt2 = radio.write( &packet2, sizeof(Packet) );
  #ifdef DEBUG
    Serial.print("Dataframe 2 Sent ");
    Serial.print(packet2);
  #endif

  // Recive the acknowledge and data from ground control
  if (rslt2) {
    if ( radio.isAckPayloadAvailable() ) {
      radio.read(&ackData, sizeof(ackData)); // To clear FIFO
    }
    else {
      #ifdef DEBUG
        Serial.println("  Acknowledge but no data ");
      #endif
    }
  }
  else {
    #ifdef DEBUG
      Serial.println("  Tx failed");
    #endif
    checkSignalLoss();
    return false;
  }

  prevMillis = millis();
  return true;
}

