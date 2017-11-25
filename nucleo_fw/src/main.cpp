/*
 * main.c
 *
 *  Created on: 2017. 3. 19.
 *      Author: baram
 */
#include "main.h"
#include <math.h>


void mainInit(void);







static void threadMain(void const *argument);




int main(void)
{
  mainInit();


  osThreadDef(threadMain, threadMain, osPriorityNormal, 0, 4*1024 / 4);

  osThreadCreate(osThread(threadMain), NULL);



  osKernelStart();

  for (;;);

  return 0;
}

void mainInit(void)
{
  bspInit();
  hwInit();
  apInit();
}


static void threadMain(void const *argument)
{
  (void) argument;

  apMain();
}

