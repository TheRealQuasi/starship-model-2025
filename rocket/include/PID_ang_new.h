#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

//PID error variables------------------------------------------------------------------------------
extern double tv_cumulative_error_pitch;
extern double tv_previous_error_pitch;

extern double tv_cumulative_error_roll;
extern double tv_previous_error_roll;

extern double tv_cumulative_error_x;
extern double tv_previous_error_x;

extern double tv_cumulative_error_y;
extern double tv_previous_error_y;

extern double rot_cumulative_error;
extern double rot_previous_error;

//PID constants-------------------------------------------------------------------------------------

// pitch pid gain values
extern double tv_pitch_Kp;
extern double tv_pitch_Ki;
extern double tv_pitch_Kd;

extern double tv_stability_Kp;
extern double tv_stability_Ki;
extern double tv_stability_Kd;

//PID controllers----------------------------------------------------------------------------------

double thrust_vector_pid_pitch(double dir, double reference_dir, int min_servo_angle_tv_stab ,int max_servo_angle_tv_stab);
double thrust_vector_pid_roll(double dir, double reference_dir, int min_servo_angle_tv_stab, int max_servo_angle_tv_stab);

//Main_control_function----------------------------------------------------------------------------------
int angleControlY(int pitch_filtered, int roll_filtered , int min_servo_angle_tv_stab , int max_servo_angle_tv_stab);
int angleControlX(int pitch_filtered, int roll_filtered , int min_servo_angle_tv_stab , int max_servo_angle_tv_stab);

#endif // PID_CONTROLLER_H
