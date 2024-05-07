
//PID error variables------------------------------------------------------------------------------

double alt_cumulative_error;
double alt_previous_error;

//PID limits----------------------------------------------------------------------------------------

int max_motor_speed = 1940;
int min_motor_speed = 1100;

//PID constants-------------------------------------------------------------------------------------

double alt_Kp = 4.5;//10; // 200; //450;
double alt_Ki = 1; //100.00;
double alt_Kd = 2.5; //200.00;

//PID controllers----------------------------------------------------------------------------------
// denna kontroller returnerar vad som ska skrivas till motorerna, så 
// sätt bara "speed = altitude_pid".... och sen "dc_motor.write(speed)"

double altitude_pid(double altitude, double reference_altitude){
  double alt_error = reference_altitude - altitude;
  double p = alt_Kp * alt_error;
  double i = alt_Ki * alt_cumulative_error;
  double d = alt_Kd * (alt_error-alt_previous_error);

  alt_cumulative_error += alt_error;
  alt_previous_error = alt_error;

  double pid_val = min_motor_speed + p+i+d;

  if(pid_val>=max_motor_speed){
    return max_motor_speed;
  }
  else if(pid_val<=min_motor_speed){
    return min_motor_speed;
  }
  else{
    return pid_val;
  }
}