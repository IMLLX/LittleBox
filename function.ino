#define LD1 349 //低音
#define LD2 392
#define LD3 440
#define LD4 464
#define LD5 523
#define LD6 587
#define LD7 659
#define D0 0  //中音
#define D1 698
#define D2 784
#define D3 880
#define D4 932
#define D5 1046
#define D6 1175
#define D7 1318
#define HD1 1396 //高音
#define HD2 1568
#define HD3 1760
#define HD4 1865
#define HD5 2092
#define HD6 2347
#define HD7 2632
const int tune[] PROGMEM =
{
  D4, D4, D4, D4, D6, D6, D6, D6, D5, D5, D5, D5, HD1, HD1, HD1, HD1, HD2, HD2, HD2, HD2, HD2, HD2, HD2, HD2, D5, D4, D3, D1, D2, D0, D2, D6, D5, D0, D4, D0, D3, D0, D3, D3, D5, D0,
  D4, D3, D2, D0, D2, HD4, HD3, HD4, HD3, HD4, D2, D0, D2, HD4, HD3, HD4, HD3, HD4, D2, D0, D2, D6, D5, D0, D4, D0, D3, D0, D3, D3, D5, D0, D4, D3, D2, D0, D2, HD4, HD3, HD4, HD3,
  HD4, D2, D0, D2, HD4, HD3, HD4, HD3, HD4, D4, D4, D4, D4, D6, D6, D6, D6, D5, D5, D5, D5, HD1, HD1, HD1, HD1, HD2, HD2, HD2, HD2, HD2, HD2, HD2, HD2, D5, D4, D3, D1, D2, D0, D2, D6, D5,
  D0, D4, D0, D3, D0, D3, D3, D5, D0, D4, D3, D2, D0, D2, HD4, HD3, HD4, HD3, HD4, D2, D0, D2, HD4, HD3, HD4, HD3, HD4, D2, D0, D2, D6, D5, D0, D4, D0, D3, D0, D3, D3, D5, D0, D4, D3, D2,
  D0, D2, HD4, HD3, HD4, HD3, HD4, D2, D0, D2, HD4, HD3, HD4, HD3, HD4, D0
};


void servo(int POS)  //舵机控制函数，不用定时器
{
  if (POS < 2) POS = 2;
  if (POS > 72) POS = 72;  //舵机角度限制
  int pulsewidth = (POS * 11) + 500; //将角度转化为500-2480的脉宽值
  digitalWrite(SERVO, HIGH);   //将舵机接口电平至高
  delayMicroseconds(pulsewidth);  //延时脉宽值的微秒数
  digitalWrite(SERVO, LOW);    //将舵机接口电平至低

}

void music() //黑人抬棺音乐
{
  if (f == 2)
  {
    if (micros() - music_timer >= 230000)
    {
      if (i > 184) i = 0;
      if (pgm_read_word_near(tune + i) == 0)    digitalWrite(BUZZER, LOW);
      else
      {
        /*
          bool pos_flag;
          if (pos <= 2) pos_flag = 0;
          if (pos >= 72) pos_flag = 1;
          if (pos_flag == 0) pos += pgm_read_word_near(tune + i) / 200;
          else pos -= pgm_read_word_near(tune + i) / 200;
        */
        music_count++;
        if (music_count % 2 == 0) {
          pos = 37 + pgm_read_word_near(tune + i) / 75;
        }
        else {
          pos = 37 - pgm_read_word_near(tune + i) / 75;
        }
        servo(pos);
        tone(BUZZER, pgm_read_word_near(tune + i));
      }
      i++;
      music_timer = micros();
    }
    if (micros() - music_timer >= 200000)
    {
      noTone(BUZZER);
    }
  }
}

void mingdi() //鸣笛
{

  if (f == 1)
  {
    if (yindiao <= 1000) yindiao_flag = 0;
    if (yindiao >= 3000) yindiao_flag = 1;
    if (yindiao_flag == 0) yindiao += 3;
    else yindiao -= 3;

    tone(BUZZER, yindiao);
  }
}

void servo_ctrl()   //舵机控制
{
  if (b == 1)
  {
    pos++;
    if (pos >= 72)   //最大旋转角度35度
    {
      pos = 72;
    }
    servo(pos);
  }
  if (b == 2)
  {
    pos--;
    if (pos <= 2)
    {
      pos = 2;
    }
    servo(pos);
  }
}

void distance_ctrl()  //sr04距离控制
{

  if (a == 1)   //距离感应
  {

    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    long distance = pulseIn(ECHO_PIN, HIGH, 5000) * 0.017;
    if (distance > 10 && distance < 15 )
    {
      Setpoints = 20;
    }
    else if (distance < 8  && distance > 1)
    {
      Setpoints = -20;
    }
    else
    {
      Setpoints = 0;
    }
  }

  if (a == 2)  //避障功能
  {

    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    long distance = pulseIn(ECHO_PIN, HIGH, 5000) * 0.017;

    if (distance < 14 && distance != 0 && flag == 0)
    {
      if (Setpoints != 0) flag2 = 1;
      flag = 1;
      turn_last = turn_i;
      turn = 9;
      Setpoints = 0;
    }

    if (turn_i - turn_last > 235 && flag == 1)
    {
      turn = 0.035;
      flag = 0;
      //dis_happy();
      if (flag2 != 0)
      {
        Setpoints = 25;
      }
      tone(BUZZER, 3000, 100);
    }
  }
}

void biaoqing()  //表情功能
{
  if (c > 15) c = 0;

  if (d == 1)
  {
    u8g.firstPage();
    do {
      if (c == 0)
      {
        u8g.drawBitmapP( 19, 0, 12, 64, woodbox);
      }
      if (c == 1)
      {
        dis_huaji();
      }
      if (c == 2)
      {
        dis_buman();
      }
      if (c == 3)
      {
        dis_deyi();
      }
      if (c == 4)
      {
        dis_falldown();
      }
      if (c == 5)
      {
        dis_happy();
      }
      if (c == 6)
      {
        dis_yun();
      }
      if (c == 7)
      {
        dis_yihuo();
      }
      if (c == 8)
      {
        dis_weixiao();
      }
      if (c == 9)
      {
        dis_die();
      }
      if (c == 10)
      {
        dis_shengqi();
      }
      if (c == 11)
      {
        dis_shangxin();
      }
      if (c == 12)
      {
        dis_piezui();
      }
      if (c == 13)
      {
        dis_xiao();
      }
      if (c == 14)
      {
        dis_jusang();
      }
      if (c == 15)
      {
        dis_niupi();
      }
      d = 0;
    } while ( u8g.nextPage() );
  }
}
