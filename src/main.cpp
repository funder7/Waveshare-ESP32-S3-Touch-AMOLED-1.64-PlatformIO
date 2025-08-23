#include "lcd_bsp.h"
#include "FT3168.h"
void setup()
{
  Serial.begin(115200);
  Touch_Init();
  lcd_lvgl_Init();
}
void loop()
{
  //delay(1000);
  //set_amoled_backlight(0xff);
  //delay(1000);
  //set_amoled_backlight(200);
  //delay(1000);
  //set_amoled_backlight(150);
  //delay(1000);
  //set_amoled_backlight(100);
  //delay(1000);
  //set_amoled_backlight(50);
  //delay(1000);
  //set_amoled_backlight(0);
}
