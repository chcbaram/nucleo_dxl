/*
 *  lcd.c
 *
 *  Created on: 2016. 7. 13.
 *      Author: Baram
 */
#include "lcd/lcd.h"
#include "hw.h"
#include "st7735.h"


#ifdef _USE_HW_LCD


int lcdCmdif(int argc, char **argv);




void lcdInit(void)
{

  st7735Init();

  lcdSetRotation(1);

  cmdifAdd("lcd", lcdCmdif);
}

void lcdFillRect(int16_t x, int16_t y, int16_t w, int16_t h,  uint16_t color)
{
  st7735FillRect(x, y, w, h, color);
}

void lcdFillScreen(uint16_t color)
{
  st7735FillRect(0, 0,  st7735GetWidth(), st7735GetHeight(), color);
}

void lcdDrawFrame(void)
{
  st7735DrawFrame(true);
}

void lcdSetRotation(uint8_t mode)
{
  st7735SetRotation(mode);
}



//-- lcdCmdif
//
int lcdCmdif(int argc, char **argv)
{
  bool ret = true;


  if (argc == 2)
  {
    if(strcmp("demo", argv[1]) == 0)
    {
    }
    else
    {
      ret = false;
    }
  }
  else
  {
    ret = false;
  }

  if (ret == false)
  {
    cmdifPrintf( "lcd demo  ...\n");
  }

  return 0;
}



#endif
