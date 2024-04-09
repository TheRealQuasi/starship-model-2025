// ==============================
// Global settings for the rocket
// ==============================

/*
* This file contains all the global settings and neasessery parameters
* It is a common place that makes adjustments easier
* 
* By GunnarEdman (Gunnar Edman)
*/

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

// ====== Motor constants ======
// #define SPEED_PROCENT_LIMIT 100
#define SPEED_LIMIT 1940                // Maximum allowed motor speed [us]

// ====== Motor constants ======
#define SERVO_1_HOME 55 //82;          // 0 position [degrees]
#define SERVO_2_HOME 71 //107; // 88;  // 0 position[degrees]
#define MAX_GIMBAL 30

// ====== Ground control specs ======
#define CAL_BUTTON_DURATION 2000        // How long the botton needs to hold to enter esc calibration [ms]