/***************角度环**************/
void angle_PID_compute(void)
{
  double angle_error = angle - Setpoint;
  angle_output = kp * angle_error +  kd * angle_dot;
}

/***************速度环**************/
void speed_PID_compute(void)
{
  double speed_error, speed_last;

  int L = count_L; count_L = 0;
  int R = count_R; count_R = 0;

  if (Setpoints == 0 && e == 0)
  {
    if (speed_integral > 40)   speed_integral -= 4;
    if (speed_integral < -40)  speed_integral += 4;
  }
  if (Setpoints > 0 && speed_n <= Setpoints)
  {
    speed_n += 5;
  }
  if (Setpoints < 0 && speed_n >= Setpoints)
  {
    speed_n -= 5;
  }
  if (Setpoints == 0)
  {
    if (speed_n > 0) speed_n -= 15;
    if (speed_n < 0) speed_n += 15;
    if (abs(speed_n) < 15) speed_n = 0;
  }

  speed_last = L + R;
  speed_error *= 0.7;
  speed_error += speed_last * 0.3;  //一阶低通滤波
  speed_integral += speed_error;
  speed_integral -= speed_n; //控制前进后退

  speed_output = sp * speed_error + si * speed_integral;

  if (e == 0)
  {
    if (speed_integral > 100)  speed_integral = 100;
    if (speed_integral < -100)  speed_integral = -100;
  }
}

/***************转向环**************/
void turn_PID_compute(void)
{
  if (turn != 0.035 && Setpoints != 0 && flag == 0)
  {
    if (turn < 0) turn = -4;
    else turn = 4;
  }
  turn_integral += gyroZ;
  turn_i += turn; //控制左右转向
  turn_output = P_turn * gyroZ + I_turn * turn_integral - turn_i ;
}
