// ==============================
// Global settings for the rocket
// ==============================

/*
* This file contains all the global settings and neasessery parameters
* It is a common place that makes adjustments easier
* 
* By GunnarEdman (Gunnar Edman)
*/

// Change debug mode | COMMENT OUT WHEN NO COMPUTER CONNECTED
#define DEBUG true


// =============================================================================================
//  Pin assignments 
// =============================================================================================

// // ====== Motor and servo pin assignment ======
#define MOTOR_1_PIN 4
#define MOTOR_2_PIN 5
#define SERVO_1_PIN 2
#define SERVO_2_PIN 3


// // =============================================================================================
// //  Constants 
// // =============================================================================================

// ====== Motor and servo constants ======
// #define SPEED_PROCENT_LIMIT 100
#define SPEED_LIMIT 1940                   // Maximum allowed motor speed [us]
#define SERVO_1_HOME 55 //82;              // 0 position [degrees]
#define SERVO_2_HOME 71 //107; // 88;      // 0 position[degrees]
#define MAX_GIMBAL 30


// ====== Ground control specs ======
#define CAL_BUTTON_DURATION 2000           // How long the botton needs to hold to enter esc calibration [ms]

// ====== IMU constants ======
// BMI088 settings

//Filter parameters - Defaults tuned for 2kHz loop rate; Do not touch unless you know what you are doing:
#define CONTROLLER_FREQUENCY 2000          // Main loop frequency (the same as mdagwick filter frequency)
#define CALIBRATION_COUNT 10000 //20000;
#define WARMUP_TIME 10000

#define B_MADGWCIK 0.04 //0.02; //0.04;    // Madgwick filter parameter (tuned for MPU MPU6050 or MPU9250)
#define B_ACCEL 0.14                       // Accelerometer LP filter paramter, (MPU6050 default: 0.14. MPU9250 default: 0.2)
#define B_GYRO 0.1                         // Gyro LP filter paramter, (MPU6050 default: 0.1. MPU9250 default: 0.17)
