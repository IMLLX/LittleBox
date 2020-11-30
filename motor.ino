void Motor(char LR, int SPEED) //电机驱动函数
{
  if (LR)
  {
    if (SPEED <= 0)    //右电机
    {
      analogWrite(5, 0);
      analogWrite(6, -SPEED);
    }
    else if (SPEED > 0)
    {
      analogWrite(5, SPEED);
      analogWrite(6, 0);
    }
  }
  else
  {
    if (SPEED <= 0)   //左电机
    {
      analogWrite(9, 0);
      analogWrite(10, -SPEED);
    }
    else if (SPEED > 0)
    {
      analogWrite(9, SPEED);
      analogWrite(10, 0);
    }
  }
}
