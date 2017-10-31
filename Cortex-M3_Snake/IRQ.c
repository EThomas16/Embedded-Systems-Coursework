/******************************************************************************/
/* IRQ.c: IRQ Handler                                                         */
/******************************************************************************/
/* This file is part of the uVision/ARM development tools.                    */
/* Copyright (c) 2010 Keil - An ARM Company. All rights reserved.             */
/* This software may only be used under the terms of a valid, current,        */
/* end user licence from KEIL for a compatible version of KEIL software       */
/* development tools. Nothing else gives you the right to use this software.  */
/******************************************************************************/
/******************************************************************************/
/* 			The following code is the modified version of IRQ.c               */
/* 			It has been modified by Besim Mustafa for use with the            */
/* 			Embedded Systems module (CIS3105) - March 2012                    */
/******************************************************************************/

#include "stm32f10x.h"                  /* STM32F10x.h definitions            */

//Joystick direction constants (should be defined in a common .h file)
#define JOYSTICK_UP			1
#define JOYSTICK_DOWN		2
#define JOYSTICK_LEFT		3
#define JOYSTICK_RIGHT		4
#define JOYSTICK_SELECT		5

unsigned long ticks;                	/* Tick counter    					  */
unsigned int user_btn_int;				/* USER button						  */
unsigned int joystick_int;				/* JOYSTICK							  */

/*----------------------------------------------------------------------------
  Systick Interrupt Handler
  SysTick interrupt happens every 1 ms
  The calling function sets the tick counter to the number of milliseconds
  it wishes to delay the program and then goes in a loop waiting for this
  to be reduced to zero, meaning the required milliseconds have passed.
  The SysTick handler decrements the tick counter every millisecond.
 *----------------------------------------------------------------------------*/
void SysTick_Handler (void) {
	if (ticks > 0) {
		//Decrement the tick counter every 1 msec as long as it is greater than 0.
		ticks -= 1;
	}
}

/*----------------------------------------------------------------------------
  EXTI Line 3 Interrupt Handler - Used to handle interrupts from
  the Joystick switch on PortD input source PD3 (see the board schematics)
 *----------------------------------------------------------------------------*/ 
void EXTI3_IRQHandler (void) {
  	if (EXTI->PR & (1<<3)) {
		//Yes, it is, so clear (i.e. reset) it.
		//If you don't do this then the interrupt
		//will keep occuring again and again as the
		//system thinks it is still pending!
    	EXTI->PR |= (1<<3);

		//Joystick DOWN
	   joystick_int = JOYSTICK_DOWN;
   	}
}

/*----------------------------------------------------------------------------
  EXTI Line 9..5 Interrupt Handler - Used to handle interrupts from
  the User button on PortG input source PG8 (see the board schematics)
  and Joystick switch on PortG input source PG7.
 *----------------------------------------------------------------------------*/ 
void EXTI9_5_IRQHandler (void) {
	//Check to see if PG8 interrupt is pending
  	if (EXTI->PR & (1<<8)) {
		//Yes, it is, so clear (i.e. reset) it.
		//If you don't do this then the interrupt
		//will keep occuring again and again as the
		//system thinks it is still pending!
    	EXTI->PR |= (1<<8);

		//Set the flag to indicate button pressed status - need resetting later
		user_btn_int = 1;
   	}

	//Check to see if PG7 interrupt is pending
  	if (EXTI->PR & (1<<7)) {
		//Yes, it is, so clear (i.e. reset) it.
		//If you don't do this then the interrupt
		//will keep occuring again and again as the
		//system thinks it is still pending!
    	EXTI->PR |= (1<<7);

		//Joystick SELECT
	   joystick_int = JOYSTICK_SELECT;
   	}
}

/*----------------------------------------------------------------------------
  EXTI Line 15..9 Interrupt Handler - Used to handle interrupts from
  the Joystick switches on PortG input sources PG15-13 (see the board schematics)
 *----------------------------------------------------------------------------*/ 
void EXTI15_10_IRQHandler (void) {
	//Check to see if PG15 interrupt is pending
  	if (EXTI->PR & (1<<15)) {
		//Yes, it is, so clear (i.e. reset) it.
		//If you don't do this then the interrupt
		//will keep occuring again and again as the
		//system thinks it is still pending!
    	EXTI->PR |= (1<<15);

		//Joystick UP
		joystick_int = JOYSTICK_UP;
   	}

	//Check to see if PG14 interrupt is pending
  	if (EXTI->PR & (1<<14)) {
		//Yes, it is, so clear (i.e. reset) it.
		//If you don't do this then the interrupt
		//will keep occuring again and again as the
		//system thinks it is still pending!
    	EXTI->PR |= (1<<14);

		//Joystick LEFT
		joystick_int = JOYSTICK_LEFT;
   	}

	//Check to see if PG13 interrupt is pending
  	if (EXTI->PR & (1<<13)) {
		//Yes, it is, so clear (i.e. reset) it.
		//If you don't do this then the interrupt
		//will keep occuring again and again as the
		//system thinks it is still pending!
    	EXTI->PR |= (1<<13);

		//Joystick RIGHT
	    joystick_int = JOYSTICK_RIGHT;
   	}
}
