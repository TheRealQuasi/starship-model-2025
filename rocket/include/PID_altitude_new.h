#ifndef ALTITUDE_PID_CONTROLLER_H
#define ALTITUDE_PID_CONTROLLER_H

//PID error variables------------------------------------------------------------------------------
extern double alt_cumulative_error;
extern double alt_previous_error;

//PID limits----------------------------------------------------------------------------------------
extern int max_motor_speed;
extern int min_motor_speed;

//PID constants-------------------------------------------------------------------------------------
extern double alt_Kp;
extern double alt_Ki;
extern double alt_Kd;

//PID controllers----------------------------------------------------------------------------------
double altitude_pid(double altitude, double reference_altitude);

#endif // ALTITUDE_PID_CONTROLLER_H
