// Project: TFMini-Plus I2C Library
#include "lidar.h"

// Initialize LiDAR
bool LiDAR::begin() {
    Serial.begin( 115200);   // Initialize terminal serial port
    //printf_begin();          // Initialize printf library.
    delay(20);
    
    printf("\n");            // say 'hello'
    printf( "TFMPlus I2C Library Example - 14JAN2022");
    printf("\n\n");

    // - - - - -   RECOVER I2C BUS  - - - - - - - - - - - - - - - 
    // An I2C device that quits unexpectedly can leave the I2C bus hung,
    // waiting for a transfer to finish.  This function bypasses the Wire
    // library and sends 8 pseudo clock cycles, a NAK, and a STOP signal
    // to the SDA and SCL pin numbers. It flushes any I2C data transfer
    // that may have been in progress, and ends by calling `Wire.begin()`.
    tfmP.recoverI2CBus(PIN_WIRE1_SDA, PIN_WIRE1_SCL);

    //  Wire.begin();            // Called in previous function.
    //  Wire.setClock( 400000);  // Set I2C bus speed to 'Fast' if
                                 // your Arduino supports 400KHz.

    // Send some example commands to the TFMini-Plus
    // - - Perform a system reset - - - - - - - - - - -
    printf( "System reset: ");
    if( tfmP.sendCommand( SOFT_RESET, 0))
    {
        printf( "passed.\r\n");
    }
    else tfmP.printReply();  // This response and 'printStatus()' are for
                             // troubleshooting and not strictly necessary.
    //
    // - - Display the firmware version - - - - - - - - -
    printf( "Firmware version: ");
    if( tfmP.sendCommand( GET_FIRMWARE_VERSION, 0))
    {
        printf( "%1u.",  tfmP.version[ 0]); // print three single numbers
        printf( "%1u.",  tfmP.version[ 1]); // each separated by a dot
        printf( "%1u\n", tfmP.version[ 2]);
    }
    else tfmP.printReply();
    //
    // - - Set the data frame-rate to 20 - - - - - - - - -
    printf( "Data-Frame rate: ");
    if( tfmP.sendCommand( SET_FRAME_RATE, FRAME_1000))
    {
        printf( "%2uHz.\n", FRAME_1000);
    }
    else tfmP.printReply();
    // - - - - -   End of example commands- - - - - - - - - -

/*  // - - - - - - - - - - - - - - - - - - - - - - - -
    // The next two commands may be used to switch the device to
    // UART (serial) mode.  If used, this sketch will no longer
    // receive I2C data.  The 'TFMPlus_example' sketch in the TFMPlus
    // (serial) Library can be used to switch the device back to
    // I2C mode.   Don't forget to switch the cables, too.
    // - - - - - - - - - - - - - - - - - - - - - - - -

    // - - Set Serial Mode - - - - - - - - - - -
    printf( "Set Serial Mode: ");
    if( tfmP.sendCommand( SET_SERIAL_MODE, 0))
    {
        printf( "mode set.\r\n");
    }
    else tfmP.printReply();
    printf( "Save Settings: ");
    if( tfmP.sendCommand( SAVE_SETTINGS, 0))
    {
        printf( "saved.\r\n");
    }
    else tfmP.printReply();
*/

    delay(500);            // And wait for half a second.

    return true;
}

// Initialize data variables
int16_t tfDist = 0;       // Distance to object in centimeters
int16_t tfFlux = 0;       // Signal strength or quality of return signal
int16_t tfTemp = 0;       // Internal temperature of Lidar sensor chip

// Read data from LiDAR
bool LiDAR::getData( int16_t &tfDist, int16_t &tfFlux, int16_t &tfTemp) {
    Wire1.setSDA(PIN_WIRE1_SDA);
    Wire1.setSCL(PIN_WIRE1_SCL);
    tfmP.getData(tfDist, tfFlux, tfTemp); // Get a frame of data

    if( tfmP.status == TFMP_READY)         // If no error...
    {
        /*
        printf( "Dist:%04icm ", tfDist);   // display distance,
        printf( "Flux:%05i ", tfFlux);     // display signal strength/quality,
        printf( "Temp:%2i%s", tfTemp, "°C" );   // display temperature,
        printf( "\n");                     // end-of-line.
        */
    }
    else
    {
        tfmP.printFrame();                 // Display error and data frame
        if( tfmP.status == TFMP_I2CWRITE)  // If I2C error...
        {
            tfmP.recoverI2CBus(PIN_WIRE1_SDA, PIN_WIRE1_SCL);          // recover hung bus.
        }
    }
    //delay(50);    //  Run loop at approximately 20Hz.
    return true;
}


