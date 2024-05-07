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

// Enable / disable settings
// =================================================================
// Change debug mode | COMMENT OUT WHEN NO COMPUTER CONNECTED
#define DEBUG
#define DISABLE_COM
#define TIME_LIMIT 5000//10000
#define LOOP_RATE
// #define ROLLCONTROLLER

// =================================================================


// Reference values
#define ALT_REF 1


// =============================================================================================
//  Pin assignments 
// =============================================================================================

// // ====== Motor and servo pin assignment ======
#define MOTOR_1_PIN 7  //4
#define MOTOR_2_PIN 8  //5
#define SERVO_1_PIN 2
#define SERVO_2_PIN 3
#define CAL_BUTTON  37//6
#define RED_LED_PIN 41 

// // ===== Radio pin assignment =====
// Define the pins used for the nRF24L01 transceiver module (CE, CSN)
#define CE_PIN 9    //9 teensy, 2 arduino uno (lighter color)
#define CSN_PIN 10  //10 teensy, 4 arduino uno (lighter color)



// // =============================================================================================
// //  Constants 
// // =============================================================================================

// ===== Div settings =====
#define BLINK_COUNT 5

// ====== Motor and servo constants ======
// #define SPEED_PROCENT_LIMIT 100
#define SPEED_LIMIT 1700                // Maximum allowed motor speed [us]
#define SPEED_MIN 1100                  // Lowest allowed motor speed
#define SERVO_1_HOME 55 //82;          // 0 position [degrees]
#define SERVO_2_HOME 71 //107; // 88;  // 0 position[degrees]
#define MAX_GIMBAL 30

// ====== Ground control specs ======
#define CAL_BUTTON_DURATION 2000        // How long the botton needs to hold to enter esc calibration [ms]

// ====== I2C interface ======
#define IMU_ADR 0x68 //b1101000 // Sensor adress for I2C communication
//#define PRESSURE_SENSOR_ADR 0x77 // Default adress and does not need to be given

// ====== Radio Configuration ======
// Define transmit power level | RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
#define RF24_PA_LEVEL RF24_PA_LOW
// Define speed of transmission | RF24_250KBPS, RF24_1MBPS, RF24_2MBPS
#define RF24_SPEED RF24_2MBPS
// What radio channel to use (0-127). The same on all nodes must match exactly.
#define RF24_CHANNEL 124 

// #define TX_INTERVAL_MILLIS 

// ====== Other ======
// Baudrate for serial communication to terminal on computer
#define BAUDRATE 576000//500000//115200 

/* // Timeout to wait before skipping a task
#define TIMEOUT_DURATION 15000000 // 15 seconds */

// ======== Roll Configuration =================================
#define ROLL_Kp 3     // Proportional gain



// =============================================================================================
//  Sensor configuration
// =============================================================================================
// ====== IMU Configuration ======
// BMI088 settings

// Filter parameters - Defaults tuned for 2kHz loop rate; Do not touch unless you know what you are doing:

// Default
// -----------------------------
// #define MADGWICK_FREQUENCY 2000          // Main loop frequency (the same as mdagwick filter frequency)
// #define CONTROLLER_FREQUENCY 100         // The frequency at which the LQR recalculates the control values
// #define CALIBRATION_COUNT 10000 //20000;
// #define WARMUP_TIME 100//21000 //10000

// #define B_MADGWCIK 0.04 //0.02; //0.04;    // Madgwick filter parameter (tuned for MPU MPU6050 or MPU9250)
// #define B_ACCEL 0.25//0.25//0.14                       // Accelerometer LP filter paramter, (MPU6050 default: 0.14. MPU9250 default: 0.2)
// #define B_GYRO 0.25//0.25//0.1                         // Gyro LP filter paramter, (MPU6050 default: 0.1. MPU9250 default: 0.17)

// #define ACC_RANGE_SETTING RANGE_6G
// #define ACC_RATE_SETTING ODR_1600

// #define GYRO_RANGE_SETTING RANGE_2000
// #define GYRO_RATE_SETTING ODR_2000_BW_532
// -----------------------------
// Default

// Test 1
// -----------------------------
#define MADGWICK_FREQUENCY 800          // Main loop frequency (the same as mdagwick filter frequency)
#define CONTROLLER_FREQUENCY 100         // The frequency at which the LQR recalculates the control values
#define IMU_SAMPLE_FREQUENCY 400
#define CALIBRATION_COUNT 10000 //20000;
#define WARMUP_TIME 20000

#define B_MADGWCIK 0.033//0.04 //0.02    // Madgwick filter parameter (tuned for MPU MPU6050 or MPU9250)
#define B_ACCEL 0.25//0.25//0.14                       // Accelerometer LP filter paramter, (MPU6050 default: 0.14. MPU9250 default: 0.2)
#define B_GYRO 0.25//0.25//0.1                         // Gyro LP filter paramter, (MPU6050 default: 0.1. MPU9250 default: 0.17)

#define ACC_RANGE_SETTING RANGE_3G
#define ACC_RATE_SETTING ODR_400//ODR_200    // bwp = 145 Hz

#define GYRO_RANGE_SETTING RANGE_500
#define GYRO_RATE_SETTING ODR_400_BW_47
// -----------------------------
// Test 1


// ====== Pressure sensor configuration ======
// Delay between pressure sensor readings
#define PS_DELAY 240 // In milliseconds (to use 128 Hz (max for sensor) => 7.8125 ms)

/*
  * temperature measure rate (value from 0 to 7)
  * 2^temp_mr temperature measurement results per second
  */
#define TEMP_MR 2

/*
  * temperature oversampling rate (value from 0 to 7)
  * 2^temp_osr internal temperature measurements per result
  * A higher value increases precision
  */
#define TEMP_OSR 2
 
/*
  * pressure measure rate (value from 0 to 7)
  * 2^prs_mr pressure measurement results per second
  */
#define PRS_MR 2

/*
  * pressure oversampling rate (value from 0 to 7)
  * 2^prs_osr internal pressure measurements per result
  * A higher value increases precision
  */
#define PRS_OSR 2


