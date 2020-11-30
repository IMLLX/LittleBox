void IMU_read(void)
{
  // while (i2cRead(0x3B, i2cData, 14));
  // accX = ((i2cData[0] << 8) | i2cData[1]);
  // accZ = ((i2cData[4] << 8) | i2cData[5]);
  // gyroY = (i2cData[10] << 8) | i2cData[11];
  // gyroZ = (i2cData[12] << 8) | i2cData[13];
  mpu.getMotion6(&accX, &ay, &accZ, &gx, &gyroY, &gyroZ);

  double roll = atan2(accX, accZ) * RAD_TO_DEG;
  double gyroYrate = -gyroY / 131.0;

  Kalman_Filter(roll, gyroYrate); //卡尔曼滤波

#ifdef PID_DEBUG //PID调试
  Serial.print(angle);
  Serial.print("  ");
  Serial.println(count_L + count_R);
#endif
}

void Kalman_Filter(double angle_m, double gyro_m)
{
  angle += (gyro_m - q_bias) * dtt;
  angle_err = angle_m - angle;
  Pdot[0] = Q_angle - P[0][1] - P[1][0];
  Pdot[1] = -P[1][1];
  Pdot[2] = -P[1][1];
  Pdot[3] = Q_gyro;
  P[0][0] += Pdot[0] * dtt;
  P[0][1] += Pdot[1] * dtt;
  P[1][0] += Pdot[2] * dtt;
  P[1][1] += Pdot[3] * dtt;
  PCt_0 = C_0 * P[0][0];
  PCt_1 = C_0 * P[1][0];
  E = R_angle + C_0 * PCt_0;
  K_0 = PCt_0 / E;
  K_1 = PCt_1 / E;
  t_0 = PCt_0;
  t_1 = C_0 * P[0][1];
  P[0][0] -= K_0 * t_0;
  P[0][1] -= K_0 * t_1;
  P[1][0] -= K_1 * t_0;
  P[1][1] -= K_1 * t_1;
  angle += K_0 * angle_err;
  q_bias += K_1 * angle_err;
  angle_dot = gyro_m - q_bias; //也许应该用last_angle-angle
}
