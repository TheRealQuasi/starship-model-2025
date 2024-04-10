#pragma once

#ifndef IMU_H
#define IMU_H

// =======================
// IMU and madgwick filter
// =======================

/*
* Class that handles both the BMI088 IMU and the madgwick filter estimating the attitude
* Also stores all nescessary variables
*
* By GunnarEdman (Gunnar Edman)
*/

// =============================================================================================
//  Preprocessor Definitions
// =============================================================================================
#include <Arduino.h>
#include <BMI088.h>
#include <settings.h>

// =============================================================================================
//  IMU class
// =============================================================================================

// Class declaration for BMI088 (6DOF) IMU (including madgwick and LP filtering)
class Imu6DOF {
public:

  // ============ Constructor ==============
  Imu6DOF() : bmi088(BMI088_ACC_ADDRESS, BMI088_GYRO_ADDRESS) {}

  // ============ Public attributes ==============
  // Create BMI088 object
  BMI088 bmi088;

  // Tempeture for BMI088
  int16_t temp = 0;

  // Time management
  int freq = CONTROLLER_FREQUENCY;   // Frequency of the madgwick filter
  float dt;                          // Stores dt that is required in the madgwick method
  unsigned long prev_time;
  unsigned long start_time, current_time;

  float B_madgwick = 0.04;  // Madgwick filter parameter
  float B_accel = 0.14;     // Accelerometer LP filter paramter
  float B_gyro = 0.1;       // Gyro LP filter paramter, (MPU6050 default: 0.1. MPU9250 default: 0.17)

  // IMU states (raw sensor data from the BMI088)
  float AccX, AccY, AccZ = 0;                     // Acceleration [g]
  float AccX_prev, AccY_prev, AccZ_prev = 0;      
  float GyroX, GyroY, GyroZ = 0;                  // Rotation [dps]
  float GyroX_prev, GyroY_prev, GyroZ_prev = 0;

  // Madgwick states (calculated in every iteration of the madgwick filter)
  float roll_IMU, pitch_IMU, yaw_IMU = 0;
  // float roll_IMU_prev, pitch_IMU_prev = 0;   // Potentially useful in kalman filter
  float q0 = 1.0f; //Initialize quaternion for madgwick filter
  float q1 = 0.0f;
  float q2 = 0.0f;
  float q3 = 0.0f;

  // ============ Public methods ==============
  // void loopRate(int freq, float current_time) {
  void loopRate() {
      //DESCRIPTION: Regulate main loop rate to specified frequency in Hz
      /*
      * It's good to operate at a constant loop rate for filters to remain stable and whatnot. Interrupt routines running in the
      * background cause the loop rate to fluctuate. This function basically just waits at the end of every loop iteration until 
      * the correct time has passed since the start of the current loop for the desired loop rate in Hz. 2kHz is a good rate to 
      * be at because the loop nominally will run between 2.8kHz - 4.2kHz. This lets us have a little room to add extra computations
      * and remain above 2kHz, without needing to retune all of our filtering parameters.
      */
      float invFreq = 1.0/freq*1000000.0;
      unsigned long checker = micros();

      //Sit in loop until appropriate time has passed
      while (invFreq > (checker - current_time)) {
          checker = micros();
      }
  }

  void timeUpdate() {
    prev_time = current_time;      
    current_time = micros();      
    dt = (current_time - prev_time)/1000000.0;
  }

  // Inits for the entire IMU object
  void initIMU () {
    init_BMI088();
    delay(500);

    start_time = millis();

  // Calibration
  // -----------
  calculate_IMU_error_BMI088(); // 

  filterIMUWarmup(); // Simulates a main loop for a few seconds (to get the madgwick filter to converge before the main loop)
  }

  // Gets new readings from BMI088 and sends them to the madgwick filter that estimates the attitude
  void imuUpdate () {
    getIMUdata_BMI088();
    madgwickStep();
  }

  // <<<<<<<<------------------- To do: Might need to add "get" functions to feed values to global variables in main

void getAttitude(float* roll, float* pitch, float* yaw) {
    *roll = roll_IMU;
    *pitch = pitch_IMU;
    *yaw = yaw_IMU;
}

private:
  // ============ Private attributes ==============

  // IMU calibration parameters (are set in the "calculate_IMU_error_BMI088" merhod)
  float AccErrorX = 0.0;
  float AccErrorY = 0.0;
  float AccErrorZ = 0.0;
  float GyroErrorX = 0.0;
  float GyroErrorY= 0.0;
  float GyroErrorZ = 0.0;


  // ============ Private methods ==============
  // Helper methods 
  // ----------------
  float invSqrt(float x) {
  return 1.0/sqrtf(x); //Teensy is fast enough to just take the compute penalty lol suck it arduino nano
  }

  // Madgwick filter
  // ---------------
  void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq) {
    //DESCRIPTION: Attitude estimation through sensor fusion - 6DOF
    /*
    * See description of Madgwick() for more information. This is a 6DOF implimentation for when magnetometer data is not
    * available (for example when using the recommended MPU6050 IMU for the default setup).
    */
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    //Convert gyroscope degrees/sec to radians/sec
    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;

    //Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    //Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
      //Normalise accelerometer measurement
      recipNorm = invSqrt(ax * ax + ay * ay + az * az);
      ax *= recipNorm;
      ay *= recipNorm;
      az *= recipNorm;

      //Auxiliary variables to avoid repeated arithmetic
      _2q0 = 2.0f * q0;
      _2q1 = 2.0f * q1;
      _2q2 = 2.0f * q2;
      _2q3 = 2.0f * q3;
      _4q0 = 4.0f * q0;
      _4q1 = 4.0f * q1;
      _4q2 = 4.0f * q2;
      _8q1 = 8.0f * q1;
      _8q2 = 8.0f * q2;
      q0q0 = q0 * q0;
      q1q1 = q1 * q1;
      q2q2 = q2 * q2;
      q3q3 = q3 * q3;

      //Gradient decent algorithm corrective step
      s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
      s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
      s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
      s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
      recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); //normalise step magnitude
      s0 *= recipNorm;
      s1 *= recipNorm;
      s2 *= recipNorm;
      s3 *= recipNorm;

      //Apply feedback step
      qDot1 -= B_madgwick * s0;
      qDot2 -= B_madgwick * s1;
      qDot3 -= B_madgwick * s2;
      qDot4 -= B_madgwick * s3;
    }

    //Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * invSampleFreq;
    q1 += qDot2 * invSampleFreq;
    q2 += qDot3 * invSampleFreq;
    q3 += qDot4 * invSampleFreq;

    //Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    //Compute angles
    roll_IMU = atan2(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2)*57.29577951; //degrees
    pitch_IMU = -asin(constrain(-2.0f * (q1*q3 - q0*q2),-0.999999,0.999999))*57.29577951; //degrees
    yaw_IMU = -atan2(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3)*57.29577951; //degrees
  }

  void madgwickStep() {
    // Estimate states with madgwick filter
    Madgwick6DOF(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, dt); //Updates roll_IMU, pitch_IMU, and yaw_IMU angle estimates (degrees)
  }

  // IMU methods
  // ---------------
  // Find IMU error avrage over a number of samples (while IMU lays still)
  void calculate_IMU_error_BMI088() {
    //DESCRIPTION: Computes IMU accelerometer and gyro error on startup. Note: vehi cle should be powered up on flat surface
    /*
    * Don't worry too much about what this is doing. The error values it computes are applied to the raw gyro and 
    * accelerometer values AccX, AccY, AccZ, GyroX, GyroY, GyroZ in getIMUdata(). This eliminates drift in the
    * measurement. 
    */
    AccErrorX = 0.0;
    AccErrorY = 0.0;
    AccErrorZ = 0.0;
    GyroErrorX = 0.0;
    GyroErrorY= 0.0;
    GyroErrorZ = 0.0;
    
    //Read IMU values 12000 times

    int c = 0;
    while (c < CAL_BUTTON_DURATION) {
      // BMI088 - Get values
      bmi088.getAcceleration(&AccX, &AccY, &AccZ);
      bmi088.getGyroscope(&GyroX, &GyroY, &GyroZ);
      temp = bmi088.getTemperature();

      // Acceleration [mg]
      AccX = AccX - AccErrorX;
      AccY = AccY - AccErrorY;
      AccZ = AccZ - AccErrorZ;
    
      // Rotation [dps]
      GyroX = GyroX - GyroErrorX;
      GyroY = GyroY - GyroErrorY;
      GyroZ = GyroZ - GyroErrorZ;
    
      //Sum all readings
      AccErrorX  = AccErrorX + AccX;
      AccErrorY  = AccErrorY + AccY;
      AccErrorZ  = AccErrorZ + AccZ;
      GyroErrorX = GyroErrorX + GyroX;
      GyroErrorY = GyroErrorY + GyroY;
      GyroErrorZ = GyroErrorZ + GyroZ;
      c++;
    }

    //Divide the sum by 12000 to get the error value
    AccErrorX  = AccErrorX / c;
    AccErrorY  = AccErrorY / c;
    AccErrorZ  = AccErrorZ / c - 1.0;
    GyroErrorX = GyroErrorX / c;
    GyroErrorY = GyroErrorY / c;
    GyroErrorZ = GyroErrorZ / c;

    #ifdef DEBUG
        Serial.print("float AccErrorX = ");
        Serial.print(AccErrorX);
        Serial.println(";");
        Serial.print("float AccErrorY = ");
        Serial.print(AccErrorY);
        Serial.println(";");
        Serial.print("float AccErrorZ = ");
        Serial.print(AccErrorZ);
        Serial.println(";");
        
        Serial.print("float GyroErrorX = ");
        Serial.print(GyroErrorX);
        Serial.println(";");
        Serial.print("float GyroErrorY = ");
        Serial.print(GyroErrorY);
        Serial.println(";");
        Serial.print("float GyroErrorZ = ");
        Serial.print(GyroErrorZ);
        Serial.println(";");

        Serial.println("Paste these values in user specified variables section and comment out calculate_IMU_error() in void setup.");
    #endif
  }

  // Sensor data fetching
  void getIMUdata_BMI088() {
    // Fetch IMU data
    bmi088.getAcceleration(&AccX, &AccY, &AccZ);
    bmi088.getGyroscope(&GyroX, &GyroY, &GyroZ);
    temp = bmi088.getTemperature();

    // Acceleration [mg]
    // -----------------
    AccX = AccX - AccErrorX;
    AccY = AccY - AccErrorY;
    AccZ = AccZ - AccErrorZ;

    //LP filter accelerometer data
    AccX = (1.0 - B_accel)*AccX_prev + B_accel*AccX;
    AccY = (1.0 - B_accel)*AccY_prev + B_accel*AccY;
    AccZ = (1.0 - B_accel)*AccZ_prev + B_accel*AccZ;
    AccX_prev = AccX;
    AccY_prev = AccY;
    AccZ_prev = AccZ;


    // Rotation [dps]
    // --------------
    GyroX = GyroX - GyroErrorX;
    GyroY = GyroY - GyroErrorY;
    GyroZ = GyroZ - GyroErrorZ;

    //LP filter gyro data
    GyroX = (1.0 - B_gyro)*GyroX_prev + B_gyro*GyroX;
    GyroY = (1.0 - B_gyro)*GyroY_prev + B_gyro*GyroY;
    GyroZ = (1.0 - B_gyro)*GyroZ_prev + B_gyro*GyroZ;
    GyroX_prev = GyroX;
    GyroY_prev = GyroY;
    GyroZ_prev = GyroZ;
  }

  // IMU and filter warmup
  void filterIMUWarmup() {
    //DESCRIPTION: Used to warm up the main loop to allow the madwick filter to converge before commands can be sent to the actuators
    //Assuming vehicle is powered up on level surface!
    /*
      * This function is used on startup to warm up the attitude estimation and is what causes startup to take a few seconds
      * to boot. 
      */
    //Warm up IMU and madgwick filter in simulated main loop
    for (int i = 0; i <= 10000; i++) {
      prev_time = current_time;      
      current_time = micros();      
      dt = (current_time - prev_time)/1000000.0; 
      getIMUdata_BMI088();
      Madgwick6DOF(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, dt);
      loopRate(); //do not exceed 2000Hz
    }
  }

  // Get methods
  // Inits BMI088 by connecting via I2C, setting predefined settings and checking connection
  void init_BMI088() {
    #ifdef DEBUG
      while (!Serial);
      Serial.println("BMI088 Raw Data");          // To do: <<<<<<<<<-------- Add "if debug" to serial prints

      while (1) {
        if (bmi088.isConnection()) {
          bmi088.initialize();
          Serial.println("BMI088 is connected");
          break;
        } 
        else {
          Serial.println("BMI088 is not connected");
        }

        delay(2000);
      }

      Serial.print('\n');
  #else
        while (1) {
          if (bmi088.isConnection()) {
            bmi088.initialize();
            break;
          }
          delay(2000);
        }
  #endif

  // Adjust IMU-settings (will be adjusted during testing)
  bmi088.setAccScaleRange(RANGE_6G);
  bmi088.setAccOutputDataRate(ODR_1600);
  bmi088.setGyroScaleRange(RANGE_2000);
  bmi088.setGyroOutputDataRate(ODR_2000_BW_532);
  }
};

# endif