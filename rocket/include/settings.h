// ==============================
// Global settings for the rocket
// ==============================

/*
* This file contains all the global settings and neasessery parameters
* It is a common place that makes adjustments easier
* 
* By GunnarEdman (Gunnar Edman),
* Emlzdev (Emil Reinfeldt)
*/

// =================================================================
// Change debug mode | COMMENT OUT WHEN NO COMPUTER CONNECTED
    #define DEBUG
// =================================================================




// =============================================================================================
//  Pin assignments 
// =============================================================================================

// // ====== Motor and servo pin assignment ======
#define MOTOR_1_PIN 4
#define MOTOR_2_PIN 5
#define SERVO_1_PIN 2
#define SERVO_2_PIN 3

// // ===== Radio pin assignment =====
// Define the pins used for the nRF24L01 transceiver module (CE, CSN)
#define CE_PIN 9    //9 teensy, 2 arduino uno (lighter color)
#define CSN_PIN 10  //10 teensy, 4 arduino uno (lighter color)


// // =============================================================================================
// //  Constants 
// // =============================================================================================

// ====== Motor constants ======
// #define SPEED_PROCENT_LIMIT 100
#define SPEED_LIMIT 1940                // Maximum allowed motor speed [us]

// ====== Motor constants ======
#define SERVO_1_HOME 55 //82;          // 0 position [degrees]
#define SERVO_2_HOME 71 //107; // 88;  // 0 position[degrees]
#define MAX_GIMBAL 30

// ====== Ground control specs ======
#define CAL_BUTTON_DURATION 2000        // How long the botton needs to hold to enter esc calibration [ms]


// ====== Wireless interface ======
// Sensor adresses for I2C
#define IMU_ADR 0x68 //b1101000
//#define PRESSURE_SENSOR_ADR 0x77 // Default and does not need to be given


// ====== Radio Configuration ======
// Define transmit power level | RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
#define RF24_PA_LEVEL RF24_PA_LOW
// Define speed of transmission | RF24_250KBPS, RF24_1MBPS, RF24_2MBPS
#define RF24_SPEED RF24_2MBPS
// What radio channel to use (0-127). The same on all nodes must match exactly.
#define RF24_CHANNEL 124 


// ====== Other ======
// Baudrate for serial communication to terminal on computer
#define BAUDRATE 115200 

/* // Timeout to wait before skipping a task
#define TIMEOUT_DURATION 15000000 // 15 seconds */


// =============================================================================================
//  Sensor configuration
// =============================================================================================
// ====== IMU Configuration ======


// ====== Pressure sensor configuration ======
// Delay between pressure sensor readings
#define PS_DELAY 240 // 240 milliseconds


