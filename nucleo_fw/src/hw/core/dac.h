/*
 *  dac.h
 *
 *  Created on: 2016. 5. 14.
 *      Author: Baram
 */

#ifndef DAC_H
#define DAC_H


#ifdef __cplusplus
 extern "C" {
#endif



#include "def.h"
#include "bsp.h"



#define DAC_CH_MAX      2





void dacInit();
void dacSetup(uint32_t hz);
void dacStart(void);
void dacStop(void);
uint32_t dacAvailable(void);
void     dacPutch(uint8_t data);
void     dacWrite(uint8_t *p_data, uint32_t length);

uint32_t dacGetDebug(void);


#ifdef __cplusplus
}
#endif


#endif
