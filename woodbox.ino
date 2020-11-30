#include "U8glib.h"
#include "MPU6050.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "Bitmap.h"
/*****************************调试选项********************************/
//#define PID_DEBUG    //PID参数调试,关闭后用宏取代变量值节省动态内存
#define MOTOR_ENABLE //电机使能

/*********************引脚定义*********************/
#define LFT 0       //左电机
#define RHT 1       //右电机
#define BUZZER A0   //蜂鸣器+LED
#define SERVO 12    //舵机
#define TRIG_PIN A1 //超声波模块触发脚
#define ECHO_PIN A2 //超声波模块接收脚

void (*resetFunc)(void) = 0;                                                          //重启函数
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_DEV_0 | U8G_I2C_OPT_NO_ACK | U8G_I2C_OPT_FAST); //实例化OLED

/***************MPU6050变量定义**********************/
int16_t accX, ay, accZ;
int16_t gx, gyroY, gyroZ;
int16_t tempRaw;
double gyroYangle;
uint8_t i2cData[14];

/***************卡尔曼滤波变量*****************/
double P[2][2] = {{1, 0}, {0, 1}};
double Pdot[4] = {0, 0, 0, 0};
static const double Q_angle = 0.001, Q_gyro = 0.005, R_angle = 0.5, dtt = 0.005, C_0 = 1;
double q_bias, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
double angle, angle_dot;

/***************PID变量定义**********************/
uint32_t PID_timer, music_timer;
double angle_output;
double Setpoints = 0, speed_output, speed_integral, speed_n;
double turn = 0.035, turn_output, turn_integral, turn_i, turn_last;

#ifdef PID_DEBUG                        //PID调试
double Setpoint = -8.7;                 //机械平衡点 需要你修改的参数
byte dl = 18;                           //电机死区
double kp = 37, kd = 0.62;              //角度环PD 需要你修改的参数
double sp = 6, si = 1.5;                //速度环PI 需要你修改的参数
double P_turn = 0.001, I_turn = 0.0003; //转向环PI 需要你修改的参数
#else
#define Setpoint -6.75
#define dl 18
#define kp 37
#define kd 0.62
#define sp 6
#define si 1.5
#define P_turn 0.001
#define I_turn 0.0003
#endif

/*********************其他变量**********************/
short yindiao = 800, i;
int music_count;
byte num = 0, num1 = 0, num2 = 0, a, b, c = -1, d, e, f, yindiao_flag, pos_flag, flag = 0, flag2 = 0, pos = 37, restart = 0;
volatile int count_L, count_R; //左右编码器计数
MPU6050 mpu;
/*********************初始化**********************/
void setup()
{
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(11, INPUT);
  pinMode(SERVO, OUTPUT);
  // pinMode(BUZZER, OUTPUT);
  /*********************初始化通信********************/
  // Serial.begin(115200);  //蓝牙串口
  Wire.begin(); //I2C总线
  Serial.begin(9600);
  delay(1000);
  /*********************MPU6050初始化********************/
  TWBR = ((F_CPU / 400000L) - 16) / 2;
  mpu.initialize();
  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  // i2cData[0] = 7;
  // i2cData[1] = 0x00;
  // i2cData[2] = 0x00;
  // i2cData[3] = 0x00;
  // while (i2cWrite(0x19, i2cData, 4, false));
  // while (i2cWrite(0x6B, 0x00, true));
  // while (i2cRead(0x75, i2cData, 1));
  //if (i2cData[0] != 0x68)
  //  while (1);
  delay(20);
  /*********************开机logo********************/
  dis_init();
  /*********************PID初始化********************/

  attachInterrupt(0, ENCODER_L, CHANGE); //开启编码器中断
  attachInterrupt(1, ENCODER_R, CHANGE); //开启编码器中断

  /*********************开机********************/
  randomSeed(analogRead(A5));
  //  tone(BUZZER, 2500, 400); //开机响一声
  delay(50);
  servo(37);
  for (int i = 0; i <= 10; i++)
  {
    d = 1;
    c = random(15); //生成一个随机表情
    tone(BUZZER, 2500, 15);
    biaoqing();
  }
  // tone(BUZZER, 2500, 150);
}

void loop()
{

  double output_L, output_R;
  //蓝牙调试和控制
  servo_ctrl(); //舵机控制
  // mingdi(); //鸣笛
  // music(); //音乐
  if (micros() - PID_timer >= 5000)
  {

    IMU_read(); //角度计算
    // protect();  //异常状态关机

    angle_PID_compute(); //角度环控制 周期5ms

    num++;
    num1++;
    num2++;
    if (num >= 8)
    {
      num = 0;
      speed_PID_compute(); //速度环控制 周期40ms
    }
    if (num1 >= 4)
    {
      num1 = 0;
      turn_PID_compute(); //转向环控制 周期20ms
    }
    if (num2 >= 20)
    {
      num2 = 0;
      distance_ctrl(); //距离跟踪控制 周期100ms
      biaoqing();      //做表情 周期100ms
    }

    /***************电机输出**************/
    output_L = angle_output + speed_output - turn_output;
    output_R = angle_output + speed_output + turn_output;

    if (output_L > 0)
      output_L += dl;
    if (output_L < 0)
      output_L -= dl;
    if (output_R > 0)
      output_R += dl;
    if (output_R < 0)
      output_R -= dl; //电机死区设置

    if (output_L > 255)
      output_L = 255;
    if (output_L < -255)
      output_L = -255;
    if (output_R > 255)
      output_R = 255;
    if (output_R < -255)
      output_R = -255; //输出限幅

#ifdef MOTOR_ENABLE
    if (restart == 0)
    {
      Motor(LFT, output_L);
      Motor(RHT, output_R);
    }
#endif
    PID_timer = micros();
  }
}

/********************蓝牙调试和控制*******************/
void serialEvent()
{
  char val = 0;

#ifdef PID_DEBUG //PID调试
  if (Serial.available() > 0)
  {
    val = Serial.read();
  }
  switch (val)
  {
  case 'q':
    kp *= 10;
    break;
  case 'w':
    kp *= 1.1;
    break;
  case 'e':
    kp *= 0.9;
    break;
  case 'a':
    kd *= 10;
    break;
  case 's':
    kd *= 1.1;
    break;
  case 'd':
    kd *= 0.9;
    break;
  case 'z':
    sp *= 10;
    break;
  case 'x':
    sp *= 1.1;
    break;
  case 'c':
    sp *= 0.9;
    break;
  case 'r':
    si *= 10;
    break;
  case 't':
    si *= 1.1;
    break;
  case 'y':
    si *= 0.9;
    break;
  case 'f':
    P_turn *= 10;
    break;
  case 'g':
    P_turn *= 1.1;
    break;
  case 'h':
    P_turn *= 0.9;
    break;
  case 'v':
    I_turn *= 10;
    break;
  case 'b':
    I_turn *= 1.1;
    break;
  case 'n':
    I_turn *= 0.9;
    break;
  case '1':
    Setpoints = 30;
    break; //前进
  case '2':
    Setpoints = -30;
    break; //后退
  case '3':
    Setpoints = 0;
    flag2 = 0;
    break; //停止
  }
  Serial.print("SP=");
  Serial.print(sp);
  Serial.print("  SI=");
  Serial.print(si);
  Serial.print("  KP=");
  Serial.print(kp);
  Serial.print("  KD=");
  Serial.print(kd);
  Serial.print("  TP=");
  Serial.print(P_turn * 1000);
  Serial.print("  TI=");
  Serial.println(I_turn * 1000);
#else
  if (Serial.available() > 0)
  {
    val = Serial.read();
  }
  switch (val)
  {
  case 'q':
    b = 1;
    break; //舵机
  case 'e':
    b = 2;
    break; //舵机转向
  case 'h':
    b = 0;
    break; //舵机停止
  case 'w':
    Setpoints = 30;
    break; //前进
  case 's':
    Setpoints = -30;
    break; //后退
  case 'f':
    Setpoints = 0;
    flag2 = 0;
    break; //停止
  case 'g':
    if (flag != 1)
    {
      turn = 0.035;
      servo(37);
    }
    break; //停止
  case 'a':
    if (flag != 1)
    {
      turn = 8;
      servo(72);
    }
    break; //左转
  case 'd':
    if (flag != 1)
    {
      turn = -8;
      servo(2);
    }
    break; //右转
  case 'r':
    a = 0;
    break; //超声波关闭
  case 't':
    a = 1;
    break; //距离感应
  case 'y':
    a = 2;
    break; //避障
  case 'z':
    f = 1;
    break; //蜂鸣器
  case 'b':
    f = 2;
    break; //黑人抬棺
  case 'x':
    f = 0;
    noTone(BUZZER);
    break;
  case 'c':
    c = 1;
    d = 1;
    tone(BUZZER, 2500, 100);
    break;
  case 'v':
    resetFunc();
    break;
  case 'm':
    c = 0;
    d = 1;
    tone(BUZZER, 2500, 100);
    break;
  case 'n':
    c = random(16);
    d = 1;
    tone(BUZZER, 2500, 100);
    break; //做表情
  case 'j':
    Motor(LFT, 0); //关电机
    Motor(RHT, 0); //关电机
    u8g.firstPage();
    do
    {
      dis_yun();
    } while (u8g.nextPage());
    restart = 1;
    break;
  case 'k':
    e = 1;
    break; //定点模式
  case 'l':
    e = 0;
    break; //移动模式
  }
  //Serial.print(angle);
  //Serial.print("  ");
  //Serial.println(speed_integral);
#endif
}

/********************中断函数*******************/
void ENCODER_L()
{
  if (digitalRead(2) == LOW)
  {
    if (digitalRead(4) == LOW)

      count_L--;
    else
    {
      count_L++;
    }
  }
}

void ENCODER_R()
{
  if (digitalRead(3) == LOW)
  {
    if (digitalRead(11) == LOW)

      count_R++;
    else
    {
      count_R--;
    }
  }
}

/********************异常状态关机*******************/
void protect()
{
  if (abs(angle - Setpoint) > 25 && restart == 0)
  {
    Motor(LFT, 0); //关电机
    Motor(RHT, 0); //关电机
    u8g.firstPage();
    do
    {
      dis_die();
    } while (u8g.nextPage());
    restart = 1;
  }
  if (restart == 1)
  {
    IMU_read();
    if (abs(angle - Setpoint) < 0.1 && abs(((i2cData[2] << 8) | i2cData[3]) - 450) < 2000)
    {
      speed_integral = 0;
      Setpoints = 0;
      turn = 0.035;
      turn_integral = 0;
      turn_i = 0;  //积分清零
      restart = 0; //开电机
    }
  }
}
