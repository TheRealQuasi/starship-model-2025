#include "RollControl.h"

float const reference_angle = 0.0;

/**
 * The function calculates a new PWM value based on the error between a reference angle and the current
 * yaw angle, with constraints on the output value.
 * 
 * @param yaw_angle The `yaw_angle` parameter represents the current yaw angle in the system.
 * @param pwm The `pwm` parameter in the `roll_p_controller` function represents the current pulse
 * width modulation value that is being used to control a motor or actuator. It is a signal that is
 * typically used to control the speed or position of a motor in a system.
 * 
 * @return The function `roll_p_controller` returns the calculated value `p_val`, which is the result
 * of the proportional control calculation for the roll angle controller. If `p_val` is within the
 * specified speed limits (`SPEED_LIMIT` and `SPEED_MIN`), it is returned as is. Otherwise, it is
 * adjusted to be within the limits before being returned.
 */
int roll_p_controller(float yawRef, float yaw_angle, int pwm){

  // Roll-error (zRot orientation in degrees)
  float roll_error = yawRef - yaw_angle;
  int p = int(ROLL_Kp * roll_error);

  int p_val = pwm + p;

  // Constrain speed difference between motors to +/- 10 %
  if(p_val<=pwm*0.9){
     p_val = pwm*0.9;
  }
  else if(p_val>=pwm*1.1){
     p_val = pwm*1.1;
  }

  // Constrain to range of possible pulse-widths to the ESC
  if(p_val>=SPEED_LIMIT){
    return SPEED_LIMIT;
  }
  else if(p_val<=SPEED_MIN){
    return SPEED_MIN;
  }
  
  return p_val; // Default return statement
}