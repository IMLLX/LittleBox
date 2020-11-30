void dis_init()
{
  u8g.firstPage();
  do {
    u8g.drawBitmapP( 19, 0, 12, 64, woodbox);
  } while ( u8g.nextPage() );
}

void dis_falldown()
{
  u8g.drawBitmapP( 32, 0, 9, 64, fall_down);
}

void dis_happy()
{
  u8g.drawBitmapP( 32, 0, 8, 64, happy);
}

void dis_yun()
{
  u8g.drawBitmapP( 27, 0, 10, 64, yun);
}
void dis_yihuo()
{
  u8g.drawBitmapP( 34, 0, 8, 64, yihuo);
}
void dis_buman()
{
  u8g.drawBitmapP( 12, 0, 13, 64, buman);
}
void dis_huaji()
{
  u8g.drawBitmapP( 32, 0, 8, 64, huaji);
}
void dis_weixiao()
{
  u8g.drawBitmapP( 32, 0, 9, 64, weixiao);
}
void dis_die()
{
  u8g.drawBitmapP( 29, 0, 9, 64, die);
}
void dis_shengqi()
{
  u8g.drawBitmapP( 27, 0, 10, 63, shengqi);
}
void dis_shangxin()
{
  u8g.drawBitmapP( 20, 0, 10, 63, shangxin);
}
void dis_piezui()
{
  u8g.drawBitmapP( 17, 0, 12, 64, piezui);
}
void dis_deyi()
{
  u8g.drawBitmapP( 34, 0, 8, 63, deyi);
}
void dis_xiao()
{
  u8g.drawBitmapP( 32, 0, 9, 63, xiao);
}
void dis_jusang()
{
  u8g.drawBitmapP( 22, 0, 11, 63, jusang);
}
void dis_niupi()
{
  u8g.drawBitmapP( 32, 0, 8, 64, niupi);
}
