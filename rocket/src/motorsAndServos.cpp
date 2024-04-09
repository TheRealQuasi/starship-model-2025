// =======================
// Servo and motor control
// =======================

/*
* These functions handle control signals to the servos and ESCs on the rocket
* They have built in safety features that complies with the physical and electrical restrictions of the hardware
* By GunnarEdman (Gunnar Edman)
*/


// =============================================================================================
//  Preprocessor Definitions
// =============================================================================================
#include <Arduino.h>
#include <Servo.h>
#include <settings.h>
#include <GlobalDecRocket.h>

// =============================================================================================
//  Definitions
// =============================================================================================

// Create motor and servo objects
Servo dc_motor_1;
Servo dc_motor_2;
Servo servo1;
Servo servo2;


// =============================================================================================
//  Functions
// =============================================================================================

// ====== Servos ======

// Functions that sends position commands to the respective servo
// --------------------------------------------------------------
// Takes an angle theta 1 in approx. the range -60 to 60 (depends on the servo calibration parameter)

// Servo range: 900, 2100

void setServo1Pos(int theta1) {             // <<<<<<<<---------- To do: Combine these to one function
//  servo1.write(servo1Home + theta1*1.5);
  int thetaMapped = theta1 + SERVO_1_HOME;

  // Constraints (servo can't move out of actuation range)
  if (thetaMapped > 120) {
    thetaMapped = 120;
  }
  else if (thetaMapped < 0) {
    thetaMapped = 0;
  }

  int tMapped = map(thetaMapped, 0, 120, 900, 2100);
  servo1.writeMicroseconds(tMapped);
}

void setServo2Pos(int theta2) {
//  servo1.write(servo1Home + theta1*1.5);
  int thetaMapped = theta2 + SERVO_2_HOME;

  // Constraints (servo can't move out of actuation range)
  if (thetaMapped > 120) {
    thetaMapped = 120;
  }
  else if (thetaMapped < 0) {
    thetaMapped = 0;
  }

  int tMapped = map(thetaMapped, 0, 120, 900, 2100);
  servo2.writeMicroseconds(tMapped);
}

// ====== Motors ======
// Speed mapping - takes % of thrust max thrust, returns on-time [us]
int speedMapping(int thrustLevel) {                  // <<<<<<<<<--------------- ToDo: This should be changed so it's maps from [N] to [us] according to the "thrust-map" curve
    return int(map(thrustLevel, 0, 100, 1100, 1940));
}

// Sets PWM on-time in [micro-seconds]
void motorsWrite(int speed, ControlData& ackData) {
  // Check arming status, set speed to zero in case unarmed
  if(!(ackData.armSwitch)) {                                 // <<<<<<<<<-------This armed check might need tweaking to prevent shutdown mid air
    dc_motor_1.write(1100);
    dc_motor_2.write(1100);
    
    // ToDo: Here, a shoudown procedure should be called
  }

  // If armed, set motor speed
  else {
    // Constraints
    if (speed > SPEED_LIMIT) {
      speed = SPEED_LIMIT;
    }
   if (speed < SPEED_LIMIT) {
      speed = 1100;
    }

    dc_motor_1.write(speed);
    dc_motor_2.write(speed);  
    // ToDo: Here, a shoudown procedure should be called
  }
}

// ====== ESC throttle calibration sequence ======
// Sends full throttle on-time for a few seconds and then zero throttle on-time
void escCalibration(bool &escCalibrationStatus) {           // <<<<<<-------------- To do: Link the buttom of the controller to this to support the calibration sequence

  // Max throttle for 5 seconds
  dc_motor_1.write(1940);
  dc_motor_2.write(1940);
  delay(6000);                                    

  // Zero throttle for 3 seconds
  dc_motor_1.write(1100); 
  dc_motor_2.write(1100);                           
  delay(3000);

  escCalibrationStatus = true;

  // Limit thrust range for safety                      // <<<<<-------------- Might not be needed in the final version
  dc_motor_1.attach(MOTOR_1_PIN, 1100, SPEED_LIMIT);
  dc_motor_2.attach(MOTOR_2_PIN, 1100, SPEED_LIMIT);
  dc_motor_1.write(1100);
  dc_motor_2.write(1100);
}


// ====== Setup function ======
void initServosMotors() {
  // Attatch pins to motor obejcts (with On-time [us] range specified)
  dc_motor_1.attach(SERVO_1_PIN, 1100, 1940);
  dc_motor_2.attach(SERVO_2_PIN, 1100, 1940);

  // Set speed to zero
  dc_motor_1.write(1100); 
  dc_motor_2.write(1100);                           

  // Attatch pins to servo obejcts (with On-time [us] range specified)
  servo1.attach(SERVO_1_PIN, 900, 2100);
  servo2.attach(SERVO_2_PIN, 900, 2100);

  // Gimbal both motors to the home position (0 degrees)
  setServo1Pos(0);
  setServo1Pos(0);
}