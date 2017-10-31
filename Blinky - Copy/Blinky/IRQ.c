/******************************************************************************/
/* IRQ.c: IRQ Handler                                                         */
/******************************************************************************/
/* This file is part of the uVision/ARM development tools.                    */
/* Copyright (c) 2010 Keil - An ARM Company. All rights reserved.             */
/* This software may only be used under the terms of a valid, current,        */
/* end user licence from KEIL for a compatible version of KEIL software       */
/* development tools. Nothing else gives you the right to use this software.  */
/******************************************************************************/

#include "stm32f10x.h"                  /* STM32F10x.h definitions            */


unsigned char  clock_1s;                /* Flag activated each second         */

/*----------------------------------------------------------------------------
  Systick Interrupt Handler
  SysTick interrupt happens every 10 ms
 *----------------------------------------------------------------------------*/
void SysTick_Handler (void) {           /* SysTick Interrupt Handler (10ms)   */
	static unsigned long ticks = 0;
	
	//1 sec = 100 x 10 msec, hence the 10 msec counter 'ticks'
	if (ticks++ >= 99) {                /* Set clock_1s every 1 second    	  */
		ticks    = 0;
		clock_1s = 1;
	}
}

