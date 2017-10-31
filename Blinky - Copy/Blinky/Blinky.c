/******************************************************************************/
/* Blinky.c: LED Flasher                                                      */
/******************************************************************************/
/* This file is part of the uVision/ARM development tools.                    */
/* Copyright (c) 2010 Keil - An ARM Company. All rights reserved.             */
/* This software may only be used under the terms of a valid, current,        */
/* end user licence from KEIL for a compatible version of KEIL software       */
/* development tools. Nothing else gives you the right to use this software.  */
/******************************************************************************/

//The following include files are pre-defined for you.
//Standard C library definitions
#include <stdio.h>
#include <string.h>
//STM32 MCU specific definitions
#include "stm32f10x.h"                  /* STM32F10x.h definitions            */
#include "GLCD.h"						/* Required for the LCD screen		  */

#define TRUE		1
#define FALSE		0

#define __FI        1                   /* Font index 16x24 - used for LCD display */

#define LED_NUM     8                   /* Number of user LEDs  */

extern unsigned char clock_1s;

//This function delays program by secs seconds. This function relies on
//a timer which is used to produce an interrupt at regular intervals. See
//further down for the way this timer is activated. The timer interrupt
//handler is in IRQ.c source file. You need to supply the code for this
//interrupt (this is a vectored interrupt) in order for this function
//to work. 
void delay (int secs) {
	int n;

	//Repeat the following secs times
	for (n = 0; n < secs; n++) {
		//First reset clock_1s
		clock_1s = 0;
		//Wait for it to be set after 1 sec	(refer to IRQ.c)
		while (!clock_1s);
	}
}

//The following function is used to display a test banner
//and show the number of cycles the test is repeated (see the main body).
void displayTestMessage(int lineNo, int colNo, unsigned char* msgText, int loopCount) {
	static int oldLineNo = 0;
	char loopNoStr [4] = {0,0,0,0};			/* Assigns a 3-digit counter display string */

	if (oldLineNo > 0) {
		GLCD_DisplayString(oldLineNo, colNo, __FI, "  ");
	}
	
	GLCD_SetTextColor(Red);
	//Show the currently executing test
	GLCD_DisplayString(lineNo, colNo, __FI, ">>");
	GLCD_SetTextColor(Blue);

	GLCD_DisplayString(lineNo, colNo + 2, __FI, msgText);
	sprintf(loopNoStr, "%d", loopCount);	/* Converts integer value to its string format */
	GLCD_SetBackColor(Red);
	GLCD_SetTextColor(White);
  	GLCD_DisplayString(lineNo, strlen((const char*)msgText) + colNo + 3, __FI, (unsigned char*)loopNoStr);
	GLCD_SetBackColor(White);
	GLCD_SetTextColor(Blue);

	oldLineNo = lineNo;
}

/*----------------------------------------------------------------------------
  Function that initializes LEDs - kind of a LED driver!
 *----------------------------------------------------------------------------*/
void LED_init(void) {
	//Peripheral clock enable register (APB2ENR) is part of
	//Reset and Clock Control (RCC) register. Bit 3 when set
	//enables the I/O Port B clock which is required for Port B
	//to function.
  	RCC->APB2ENR |= (1UL << 3);           /* Enable GPIOB clock                 */

	//The KEIL evaluation board MCBSTM32EXL has 8 green LEDs.
	//These LEDs are connected to MCU's PB8 to PB15 (see the board shematic).

	//The procedure for configuring PB8 to PB15 as output pins:
	//1. PortB's data register ODR defines bits 0 to 15 as output pins.
	//   The following statement resets PB8 to PB15, i.e. LEDs are off.
  	GPIOB->ODR   &= ~0x0000FF00;          /* switch off LEDs                    */
	//2. PortB's configuration register CRH is used to define which PortB
	//   data pins will be configured as output pins. First clear the CRH
	//   register - this sets the pins to input mode.
  	GPIOB->CRH   &= ~0xFFFFFFFF;          /* Configure the GPIO for LEDs        */
	//3. Now configure pins PB8 to PB15 to general purpose output mode (see MCU spec.).
	//   Mode bits set to bin 11 (i.e. output mode) and the configuration bits are set
	//   to bin 00 (i.e. general purpose output).
  	GPIOB->CRH   |=  0x33333333;
}

/*----------------------------------------------------------------------------
  Function that turns on requested LED
 *----------------------------------------------------------------------------*/
void LED_On (unsigned int num) {

  	//Do the following to switch on a LED:
	//Set PortB Bit Set/Reset Register (BSSR) to output a signal on corresponding port pin
	//which is used to active the LED (i.e. switch it on) connected to that pin.
	//Here, num is a number from 0 to 8 for LED0 to LED7. Since register bits for LEDs
	//start at bit 8, we need to add 8 to num in order to set the correct LED output data
	//bit using the following bit shift statement.
  	GPIOB->BSRR |= (1UL << num + 8);
}

/*----------------------------------------------------------------------------
  Function that turns off requested LED
 *----------------------------------------------------------------------------*/
void LED_Off (unsigned int num) {

  	//PortB Bit Reset Register (BRR) is used to switch off output data signals to
	//the LEDs thus effectively turning them off. If you set a bit in this register
	//then the corresponding output pin is reset.
  	GPIOB->BRR |= (1UL << num + 8);
}

void All_LEDs_On() {
   GPIOB->BSRR |= 0x0000FF00;
}

void All_LEDs_Off() {
   GPIOB->BRR |= 0x0000FF00;
}

/*----------------------------------------------------------------------------
  Main Program
 *----------------------------------------------------------------------------*/
int main (void) {
	int loopCount = 1;
	int i;

	//Following statement sets the timer interrupt frequency
	//The clock rate on this boards MCU is 72 Mhz - this means
	//every second there will be 72 million clocks. So, if use this
	//as the parameter to the function the interrupt rate is going
	//to be every second. If it is reduced to 100 times less then
	//it will do that many clocks in 100 times less time, i.e. 10 msecs.
	//NOTE: We could have chosen to generate a timer interrupt every 1 msec
	//if we wished and that would be quite acceptable in which case we would
	//need to modify the interrupt handler routine in IRQ.c file.
  	SysTick_Config(SystemCoreClock/100);  /* Generate interrupt each 10 ms      */

   	//Here we initialise the LED driver
  	LED_init();                           /* LED Initialization                 */

#ifdef __USE_LCD
	//In order to use the LCD display, the device needs to be
	//properly configured and initialized (refer to GLCD_16bitIF_STM32.c file).
	//This is a complex process and is beyond the scope of this module.
  	GLCD_Init();                          /* Initialize graphical LCD display   */

	//The following functions demonstrate how the LCD display can be used.
	//This is a 240 x 320 pixel colour screen and the pixels can be individually
	//manipulated allowing graphics shapes to be constructed and displayed.
	//This configuration allows 20 characters per line and 10 lines (16x24 pixel characters).
	GLCD_Clear(White);                    /* Clear graphical LCD display        */
	GLCD_SetBackColor(Blue);
	GLCD_SetTextColor(White);
	GLCD_DisplayString(0, 0, __FI, "< CIS3105 Tutorial >");
	GLCD_DisplayString(1, 0, __FI, " ARM Cortex-M3 MCU  ");
	GLCD_DisplayString(2, 0, __FI, "====================");
	GLCD_SetBackColor(White);
	GLCD_SetTextColor(Blue);
	GLCD_DisplayString(3, 0, __FI, "Ex1:");
	GLCD_DisplayString(4, 0, __FI, "Ex2:");
	GLCD_DisplayString(5, 0, __FI, "Ex3:");
	GLCD_DisplayString(6, 0, __FI, "Ex4:");
	GLCD_DisplayString(7, 0, __FI, "Ex5:");
	GLCD_DisplayString(8, 0, __FI, "Ex6:");
#endif // __USE_LCD

	//The main body of the program requires a continuous loop which
	//generally executes various functions and monitors various device
	//states. It will be interrupted at regular intervals by the
	//vectored timer interrupt as explained above!
  	while (TRUE) {                           /* Loop forever                       */

		displayTestMessage(3, 4, "LED Test1", loopCount);
		//For all 8 LEDs do
	  	for (i = 0; i < LED_NUM; i++) {
			//Switch a LED on
			LED_On(i);
			//Wait for a second
			delay(1);6
			//Switch LED off
			LED_Off(i);
			//Wait for a second
			delay(1);
		}                                           

		displayTestMessage(4, 4, "LED Test2", loopCount);
		//Slow flash all LEDs 4 times
		for (i = 0; i != 3; i++) {
			All_LEDs_On();
			delay(1);
			All_LEDs_Off();
			delay(1);
		}


		/* PUT MORE LED DISPLAY PATTERNS BELOW */


		//Increment the loop count
		loopCount += 1;
  	}
}
