/*
 *  lcd.h
 *
 *  Created on: 2016. 5. 14.
 *      Author: Baram
 */

#ifndef LCD_H
#define LCD_H


#ifdef __cplusplus
 extern "C" {
#endif



#include "def.h"
#include "bsp.h"


#ifdef _USE_HW_LCD


#include "st7735.h"



void lcdInit();

void lcdFillRect(int16_t x, int16_t y, int16_t w, int16_t h,  uint16_t color);
void lcdFillScreen(uint16_t color);
void lcdDrawFrame(void);
void lcdSetRotation(uint8_t mode);

#endif

#ifdef __cplusplus
}
#endif


#endif
