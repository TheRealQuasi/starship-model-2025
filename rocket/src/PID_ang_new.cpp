
//PID error variables------------------------------------------------------------------------------

double tv_cumulative_error_pitch;
double tv_previous_error_pitch;

double tv_cumulative_error_roll;
double tv_previous_error_roll;

double tv_cumulative_error_x;
double tv_previous_error_x;

double tv_cumulative_error_y;
double tv_previous_error_y;

double rot_cumulative_error;
double rot_previous_error;




//PID constants-------------------------------------------------------------------------------------

// pitch pid gain values
double tv_pitch_Kp = 2;//0.5;//3.000;
double tv_pitch_Ki = 0.01;//0.080;
double tv_pitch_Kd = 10; //55.00;

double tv_stability_Kp =0.5; //3.000;
double tv_stability_Ki = 0.01;//0.080;
double tv_stability_Kd = 10;//55.00;


//PID controllers----------------------------------------------------------------------------------

double thrust_vector_pid_pitch(double dir, double reference_dir, int min_servo_angle_tv_stab ,int max_servo_angle_tv_stab){
  double tv_dir_error = reference_dir - dir;
  double p = tv_pitch_Kp * tv_dir_error;
  double i = tv_pitch_Ki * tv_cumulative_error_pitch;
  double d = tv_pitch_Kd * (tv_dir_error-tv_previous_error_pitch);

  float pid_val = p+i+d;

  if (!((pid_val<=min_servo_angle_tv_stab && tv_dir_error<=0)|| (pid_val>=max_servo_angle_tv_stab && tv_dir_error>=0))){
    tv_cumulative_error_pitch += tv_dir_error;
  }
  
  tv_previous_error_pitch = tv_dir_error;

  if(pid_val>=max_servo_angle_tv_stab){
    return max_servo_angle_tv_stab;
  }
  else if(pid_val<=min_servo_angle_tv_stab){
    return min_servo_angle_tv_stab;
  }
  else{
    return pid_val;
  }
}

double thrust_vector_pid_roll(double dir, double reference_dir, int min_servo_angle_tv_stab, int max_servo_angle_tv_stab){
  double tv_dir_error = reference_dir - dir;
  double p = tv_stability_Kp * tv_dir_error;
  double i = tv_stability_Ki * tv_cumulative_error_roll;
  double d = tv_stability_Kd * (tv_dir_error-tv_previous_error_roll);

  float pid_val = p+i+d;

  if (!((pid_val<=min_servo_angle_tv_stab && tv_dir_error<=0)|| (pid_val>=max_servo_angle_tv_stab && tv_dir_error>=0))){
    tv_cumulative_error_roll += tv_dir_error;
  }
  tv_previous_error_roll = tv_dir_error;

  if(pid_val>=max_servo_angle_tv_stab){
    return max_servo_angle_tv_stab;
  }
  else if(pid_val<=min_servo_angle_tv_stab){
    return min_servo_angle_tv_stab;
  }
  else{
    return pid_val;
  }
}


//Main_control_function----------------------------------------------------------------------------------
int angleControlY(int pitch_filtered, int roll_filtered, int min_servo_angle_tv_stab, int max_servo_angle_tv_stab) {
  float yGimb = float(thrust_vector_pid_pitch(pitch_filtered, 0 , min_servo_angle_tv_stab ,max_servo_angle_tv_stab));
  return yGimb;
}

int angleControlX(int pitch_filtered, int roll_filtered, int min_servo_angle_tv_stab, int max_servo_angle_tv_stab) {
  float xGimb = float(thrust_vector_pid_roll(roll_filtered, 0 , min_servo_angle_tv_stab , max_servo_angle_tv_stab));
  return xGimb;
}

