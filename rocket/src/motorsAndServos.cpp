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
  if (thetaMapped > SERVO_1_HOME + MAX_GIMBAL) {
    thetaMapped = SERVO_1_HOME + MAX_GIMBAL;
  }
  else if (thetaMapped < SERVO_1_HOME - MAX_GIMBAL) {
    thetaMapped = SERVO_1_HOME - MAX_GIMBAL;
  }

  int tMapped = map(thetaMapped, 0, 120, 900, 2100);
  servo1.writeMicroseconds(tMapped);
}

void setServo2Pos(int theta2) {
//  servo1.write(servo1Home + theta1*1.5);
  int thetaMapped = theta2 + SERVO_2_HOME;

  // Constraints (servo can't move out of actuation range)
  if (thetaMapped > SERVO_1_HOME + MAX_GIMBAL) {
    thetaMapped = SERVO_1_HOME + MAX_GIMBAL;
  }
  else if (thetaMapped < SERVO_1_HOME - MAX_GIMBAL) {
    thetaMapped = SERVO_1_HOME - MAX_GIMBAL;
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
    
    //   // ToDo: Here, a shoudown procedure should be called
    delay(200000);
  }

  // If armed, set motor speed
  else {
    // Constraints
    if (speed > SPEED_LIMIT) {
      speed = SPEED_LIMIT;
    }
    if (speed < 1100) {
      speed = 1100;
    }

    // // ToDo: slider mapping
    // int sliderMapped = ackData.thrustSlider;

    // if (sliderMapped <= 5) {
    //   sliderMapped = 0;
    // }
    
    // if (sliderMapped >= 250) {
    //   sliderMapped = 255;
    // }
    
    // sliderMapped = map(ackData.thrustSlider, 0, 255, 1100, 1940); 

    // // Slider constraints
    // if (speed > sliderMapped) {
    //   speed = sliderMapped;
    // }

    dc_motor_1.write(speed);
    dc_motor_2.write(speed);  
    // ToDo: Here, a shoudown procedure should be called
    }
}

// Gimbal test during startup (+ / - 30 degrees gimbal on both motors)
void gimbalTest() {
  setServo1Pos(-MAX_GIMBAL);
  setServo2Pos(-MAX_GIMBAL);
  delay(1500);    

  setServo1Pos(MAX_GIMBAL);
  setServo2Pos(MAX_GIMBAL);
  delay(1500);    

  setServo1Pos(0);
  setServo2Pos(0);
}

// Brief test of BLCD motors at low RPM
void motorTest() {
  // LED warning

  // Test of motor 1
  dc_motor_1.write(1150);
  delay(500);

  dc_motor_1.write(1100);
  delay(500);

  // Test of motor 1
  dc_motor_2.write(1150);
  delay(500);

  dc_motor_2.write(1100);
  delay(500);
}

// ====== ESC throttle calibration sequence ======
// Sends full throttle on-time for a few seconds and then zero throttle on-time
void escCalibration(bool &escCalibrationStatus) {           // <<<<<<-------------- To do: Link the buttom of the controller to this to support the calibration sequence
  #ifdef DEBUG
    Serial.print("Calibrating ESCs \n");
    Serial.print("---------------- \n \n");
    Serial.print("1940 [us] for 5 seconds\n");
  #endif

  // Max throttle for some seconds (5)
  dc_motor_1.write(1940);
  dc_motor_2.write(1940);
  delay(5000);                                    

  // Zero throttle for 3 seconds
  #ifdef DEBUG
    Serial.print("1100 [us] for 5 seconds\n");
  #endif
  dc_motor_1.write(1100); 
  dc_motor_2.write(1100);                           
  delay(5000);

  escCalibrationStatus = true;
  #ifdef DEBUG
    Serial.print("ESC calibration completed - ESCs are now armed!!!! \n");
    Serial.print("================================================== \n");
  #endif

  // Limit thrust range for safety                      // <<<<<-------------- Might not be needed in the final version
  dc_motor_1.attach(MOTOR_1_PIN, 1100, SPEED_LIMIT);
  dc_motor_2.attach(MOTOR_2_PIN, 1100, SPEED_LIMIT);
  dc_motor_1.write(1100);
  dc_motor_2.write(1100);

  #ifdef DEBUG
    Serial.print("Testing motors at low speed for 2 seconds\n");
    Serial.print("-----------------------------------------\n \n");

    dc_motor_1.write(1150);
    dc_motor_2.write(1150);

    delay(2000);    

    dc_motor_1.write(1100);
    dc_motor_2.write(1100);

  #endif


}


// Calibration procedure called after calButton is pressed for at least 2 seconds
void waitESCCalCommand(bool &escCalibrationStatus) {
  // Only execute this code if the ESC is in calibration mode (never in armed mode)
  if (!escCalibrationStatus) {
    #ifdef DEBUG
      Serial.print("Avaiting cal button");
    #endif

    // Wait for calibration to be granted by the pilot (holding calButton for at least 2 seconds)
    unsigned long t0 = millis();
    unsigned long tPress = millis();

    // Check for calButton press and duration off press
    while (tPress - t0 < CAL_BUTTON_DURATION) {
      // Check calButton status
      if (!digitalRead(CAL_BUTTON)) {
        tPress = millis();
      }

      // Reset duration if button is not pressed
      else {
        t0 = millis();
        tPress = millis();
      }
    }

    delay(1000);

    // When button press duration is enough, run ESC calibation
    escCalibration(escCalibrationStatus);
  }
}


// ====== Setup function ======
void initServosMotors() {
  #ifdef DEBUG
    Serial.print("Motor and servo init\n");
    Serial.print("--------------------\n \n");
  #endif

  // Attatch pins to motor obejcts (with On-time [us] range specified)
  dc_motor_1.attach(MOTOR_1_PIN, 1100, 1940);
  dc_motor_2.attach(MOTOR_2_PIN, 1100, 1940);

  // Set speed to zero
  dc_motor_1.write(1100); 
  dc_motor_2.write(1100);                           

  // Attatch pins to servo obejcts (with On-time [us] range specified)
  servo1.attach(SERVO_1_PIN, 900, 2100);
  servo2.attach(SERVO_2_PIN, 900, 2100);


  // Gimbal both motors to the home position (0 degrees)
  setServo1Pos(0);
  setServo2Pos(0);
}