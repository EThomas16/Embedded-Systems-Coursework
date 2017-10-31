/******************************************************************************/
/* Blinky.c: LED Flasher                                                      */
/******************************************************************************/
/* This file is part of the uVision/ARM development tools.                    */
/* Copyright (c) 2010 Keil - An ARM Company. All rights reserved.             */
/* This software may only be used under the terms of a valid, current,        */
/* end user licence from KEIL for a compatible version of KEIL software       */
/* development tools. Nothing else gives you the right to use this software.  */
/******************************************************************************/
/******************************************************************************/
/* 			The following code is the modified version of Blinky.c            */
/* 			It has been modified by Besim Mustafa for use with the            */
/* 			Embedded Systems module (CIS3105) - March 2012                    */
/******************************************************************************/

//The following include files are pre-defined for you.
//Standard C library definitions
#include <stdio.h>
#include <string.h>

//STM32 MCU specific definitions
#include "stm32f10x.h" 					/* STM32F10x.h definitions            */                
#include "GLCD.h"						/* Required for the LCD screen		  */

#define TRUE				1
#define FALSE				0

#define __FI        		1           /* Font index 16x24 - used for LCD display */

#define LED_NUM     		8           /* Number of user LEDs  		*/
#define HALF_LED_NUM		4			/* Half number of user LEDs     */

//Test selector constants
#define TEST_VECT_INTR		0
#define TEST_POLL_INPT		1
#define TEST_LED_OUTPT		2

//Joystick direction constants (should be defined in a common .h file)
#define JOYSTICK_UP			1
#define JOYSTICK_DOWN		2
#define JOYSTICK_LEFT		3
#define JOYSTICK_RIGHT		4
#define JOYSTICK_SELECT		5

#define __AFIO_EXTICR1_JS_PD3     0x00003000  //EXTI3 interrupt
#define __EXTI_IMR_JS_PD3         0x00000008  //Interrupt Mask Register - Unmask intr from src EXTI3
#define __EXTI_EMR_JS_PD3         0x00000000  //Event Mask Register - Mask events from all EXTI
#define __EXTI_RTSR_JS_PD3        0x00000000  //Rising Trigger Selection Register - Rising trigger disabled for all EXTI
#define __EXTI_FTSR_JS_PD3        0x00000008  //Falling Trigger Selection register - Falling trigger enabled for EXTI3
#define EXTI3_IRQChannel           ((u8)0x09)  // EXTI Line3 Interrupt (position 9  in interrupt table)

/* Definitions for PG7 (Joystick ) interrupt handling setup */
#define __AFIO_EXTICR2_JS_PG7     0x00006000  //EXTI7 interrupt
#define __EXTI_IMR_JS_PG7         0x00000080  //Interrupt Mask Register - Unmask intr from src EXTI7
#define __EXTI_EMR_JS_PG7         0x00000000  //Event Mask Register - Mask events from all EXTI
#define __EXTI_RTSR_JS_PG7        0x00000000  //Rising Trigger Selection Register - Rising trigger disabled for all EXTI
#define __EXTI_FTSR_JS_PG7        0x00000080  //Falling Trigger Selection register - Falling trigger enabled for EXTI7
#define EXTI9_5_IRQChannel        ((u8)0x17)  //External Line[9:5] Interrupts (position 23  in interrupt table)

/* Definitions for PG8 (User button) interrupt handling setup */
#define __AFIO_EXTICR3_UB_PG8     0x00000006  //EXTI8 interrupt
#define __EXTI_IMR_UB_PG8         0x00000100  //Interrupt Mask Register - Unmask intr from src EXTI8
#define __EXTI_EMR_UB_PG8         0x00000000  //Event Mask Register - Mask events from all EXTI
#define __EXTI_RTSR_UB_PG8        0x00000000  //Rising Trigger Selection Register - Rising trigger disabled for all EXTI
#define __EXTI_FTSR_UB_PG8        0x00000100  //Falling Trigger Selection register - Falling trigger enabled for EXTI8
//#define EXTI9_5_IRQChannel        ((u8)0x17)  //External Line[9:5] Interrupts

#define __AFIO_EXTICR4_PG15_13	  0x00006660  //EXTI15,14,14 interrupts
#define __EXTI_IMR_JS_PG15_13     0x0000E000  //Interrupt Mask Register - Unmask intr from src EXTI15, EXTI14, EXTI13
#define __EXTI_EMR_JS_PG15_13     0x00000000  //Event Mask Register - Mask events from all EXTI
#define __EXTI_RTSR_JS_PG15_13    0x00000000  //Rising Trigger Selection Register - Rising trigger disabled for all EXTI
#define __EXTI_FTSR_JS_PG15_13    0x0000E000  //Falling Trigger Selection register - Falling trigger enabled for EXTI15, EXTI14, EXTI13
#define EXTI15_10_IRQChannel        ((u8)0x28)  //External Line[15:10] Interrupts (position 40 in interrupt table)

//The following are declared external to this file (in IRQ.c)
extern unsigned int user_btn_int;
extern unsigned long ticks;
extern unsigned int joystick_int;

int snakeMove = 0;
char snakeArray[3] = { 'o', 'o', 'o' };
char snakeHead = 'X';
int snakeDirection = JOYSTICK_RIGHT;
int headX = 11;
int headY = 5;


//This function delays program by msecs milliseconds. This function relies on
//a timer which is used to produce an interrupt at regular intervals. See
//further down for the way this timer is activated. The timer interrupt
//handler is in IRQ.c source file. You need to supply the code for this
//interrupt (this is a vectored interrupt) in order for this function
//to work.
void delay(unsigned long msecs) {
	//Set the tick counter
	ticks = msecs;
	//Wait until the tick counter is 0 (zero)
	while (ticks) {}
}

void delaySecs(int secs) {
	delay(1000 * secs); 
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
  	RCC->APB2ENR |= (1UL << 3);           //Enable GPIOB clock

	//The KEIL evaluation board MCBSTM32EXL has 8 green LEDs.
	//These LEDs are connected to MCU's PB8 to PB15 (see the board shematic).

	//The procedure for configuring PB8 to PB15 as output pins:
	//1. PortB's data register ODR defines bits 0 to 15 as output pins.
	//   The following statement resets PB8 to PB15, i.e. LEDs are off.
  	GPIOB->ODR   &= ~0x0000FF00;          //Switch off LEDs
	//2. PortB's configuration register CRH is used to define which PortB
	//   data pins will be configured as output pins. First clear the CRH
	//   register - this sets the pins to input mode.
  	GPIOB->CRH   &= ~0xFFFFFFFF;          //Configure the GPIO for LEDs
	//3. Now configure pins PB8 to PB15 to general purpose output mode (see MCU spec.).
	//   Mode bits set to bin 11 (i.e. output mode) and the configuration bits are set
	//   to bin 00 (i.e. general purpose output).
  	GPIOB->CRH   |=  0x33333333;
}

/*----------------------------------------------------------------------------
  Function that initializes the User button	as an input
 *----------------------------------------------------------------------------*/
void userButton_Init(void) {
	RCC->APB2ENR |= (1UL << 8);         /* Enable GPIOG clock                 	*/

  	GPIOG->CRH   &= ~0x0000000F;        /* Configure the GPIO for the User button 	*/
  	GPIOG->CRH   |=  0x00000004;
}

/*----------------------------------------------------------------------------
  Function that initializes the Joystick switches as inputs
 *----------------------------------------------------------------------------*/
void joyStick_Init(void) {
	RCC->APB2ENR |= (1UL << 8);         /* Enable GPIOG clock                 	        */

   	GPIOG->CRL   &= ~0xF000F000;		/* Configure PG7 and PD3 as joystick inputs	    */
	GPIOG->CRL   |=  0x40004000; 		

  	GPIOG->CRH   &= ~0xFFF00000;        /* Configure the GPIO for the joystick buttons PG15, PG14, PG13	*/
  	GPIOG->CRH   |=  0x44400000;
}

/*----------------------------------------------------------------------------
  Function that enables the User button for interrupt handling
 *----------------------------------------------------------------------------*/
void userButton_IntrEnable(void) {
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;           			// Enable clock for Alternate Function register
    AFIO->EXTICR[2] &= 0xFFF0;                    			// Clear
    AFIO->EXTICR[2] |= (0x000F & __AFIO_EXTICR3_UB_PG8);	// Use PG8 as source of interrupt

    EXTI->IMR       |= ((1 << 8) & __EXTI_IMR_UB_PG8);      // Unmask interrupt	only from source PG8
	//EXTI->IMR     	= 0x00000000;            				// Mask all interrupt sources
    EXTI->EMR       |= ((1 << 8) & __EXTI_EMR_UB_PG8);      // Mask ALL events
    EXTI->RTSR      |= ((1 << 8) & __EXTI_RTSR_UB_PG8);     // Rising edge for PG8 interrupt - not set
    EXTI->FTSR      |= ((1 << 8) & __EXTI_FTSR_UB_PG8);     // Falling edge	for PG8 interrupt - set

	NVIC->ISER[0] |= (1 << (EXTI9_5_IRQChannel & 0x1F));  	// Enable interrupt handler for EXTI 9..5
}

/*----------------------------------------------------------------------------
  Function that enables the Joystick switches for interrupt handling
 *----------------------------------------------------------------------------*/
void joyStick_IntrEnable_PG15_13(void) {
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;           				// Enable clock for Alternate Function register
    AFIO->EXTICR[3] &= 0x000F;                    				// Clear
    AFIO->EXTICR[3] |= (0xFFF0 & __AFIO_EXTICR4_PG15_13);		// Use PG15, PG14, PG13 as interrupt sources

    EXTI->IMR       |= ((1 << 15) & __EXTI_IMR_JS_PG15_13);  	// Unmask interrupt	only from source PG15
    EXTI->IMR       |= ((1 << 14) & __EXTI_IMR_JS_PG15_13);  	// Unmask interrupt	only from source PG14
	EXTI->IMR       |= ((1 << 13) & __EXTI_IMR_JS_PG15_13);  	// Unmask interrupt	only from source PG13

    EXTI->EMR       |= ((1 << 15) & __EXTI_EMR_JS_PG15_13);  	// Mask ALL events
    EXTI->RTSR      |= ((1 << 15) & __EXTI_RTSR_JS_PG15_13); 	// Rising edge for PG15 interrupt - not set
    EXTI->FTSR      |= ((1 << 15) & __EXTI_FTSR_JS_PG15_13); 	// Falling edge for PG15 interrupt - set

    EXTI->RTSR      |= ((1 << 14) & __EXTI_RTSR_JS_PG15_13); 	// Rising edge for PG14 interrupt - not set
    EXTI->FTSR      |= ((1 << 14) & __EXTI_FTSR_JS_PG15_13); 	// Falling edge for PG14 interrupt - set

    EXTI->RTSR      |= ((1 << 13) & __EXTI_RTSR_JS_PG15_13);  	// Rising edge for PG13 interrupt - not set
    EXTI->FTSR      |= ((1 << 13) & __EXTI_FTSR_JS_PG15_13);  	// Falling edge for PG13 interrupt - set

	NVIC->ISER[1] |= (1 << (EXTI15_10_IRQChannel & 0x1F));    	// Enable interrupt handler for EXTI 15..10
}

/*----------------------------------------------------------------------------
  Function that enables the Joystick switch for interrupt handling
 *----------------------------------------------------------------------------*/
void joyStick_IntrEnable_PG7(void) {
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;           			// Enable clock for Alternate Function register
    AFIO->EXTICR[1] &= 0x0FFF;                    			// Clear
    AFIO->EXTICR[1] |= (0xF000 & __AFIO_EXTICR2_JS_PG7);	// Use PG7 as source of interrupt

    EXTI->IMR       |= ((1 << 7) & __EXTI_IMR_JS_PG7);      // Unmask interrupt	only from source PG7
	//EXTI->IMR     	= 0x00000000;            				// Mask all interrupt sources
    EXTI->EMR       |= ((1 << 7) & __EXTI_EMR_JS_PG7);      // Mask ALL events
    EXTI->RTSR      |= ((1 << 7) & __EXTI_RTSR_JS_PG7);     // Rising edge for PG7 interrupt - not set
    EXTI->FTSR      |= ((1 << 7) & __EXTI_FTSR_JS_PG7);     // Falling edge	for PG7 interrupt - set

	NVIC->ISER[0] |= (1 << (EXTI9_5_IRQChannel & 0x1F));  	// Enable interrupt handler for EXTI 9..5
}

/*----------------------------------------------------------------------------
  Function that enables the Joystick switch for interrupt handling
 *----------------------------------------------------------------------------*/
void joyStick_IntrEnable_PD3(void) {
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;                     // Enable clock for Alternate Function
    AFIO->EXTICR[0] &= 0x0FFF;                              // Clear used pin
    AFIO->EXTICR[0] |= (0xF000 & __AFIO_EXTICR1_JS_PD3);    // Use PD3 as source of interrupt

    EXTI->IMR       |= ((1 << 3) & __EXTI_IMR_JS_PD3);      // Unmask interrupt	only from source PD3
    EXTI->EMR       |= ((1 << 3) & __EXTI_EMR_JS_PD3);      // Mask ALL events
    EXTI->RTSR      |= ((1 << 3) & __EXTI_RTSR_JS_PD3);     // Rising edge for PD3 interrupt - not set
    EXTI->FTSR      |= ((1 << 3) & __EXTI_FTSR_JS_PD3);     // Falling edge	for PD3 interrupt - set

    NVIC->ISER[0] |= (1 << (EXTI3_IRQChannel & 0x1F));    	// Enable interrupt EXTI 3
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

/*----------------------------------------------------------------------------
  Function that turns on all LEDs
 *----------------------------------------------------------------------------*/
void All_LEDs_On() {
   GPIOB->BSRR |= 0x0000FF00;
}

/*----------------------------------------------------------------------------
  Function that turns off all LEDs
 *----------------------------------------------------------------------------*/
void All_LEDs_Off() {
   GPIOB->BRR |= 0x0000FF00;
}

void snakeRefresh(void) {
	//Updates the position of the snake
	GLCD_DisplayString(headY, headX, __FI, "x");
	delay(1);
	GLCD_ClearLn(headX, Black);
}

void snakeMovement(void) {
	//Handles basic movement of the snake on the LCD screen
	//This is an interrupt
	if (snakeDirection == JOYSTICK_RIGHT) {
		//move right
		headX++;
	}
	else if (snakeDirection == JOYSTICK_LEFT) {
		//move left
		headX--;
	}
	else if (snakeDirection == JOYSTICK_UP) {
		//move up
		headY--;
	}
	else if (snakeDirection == JOYSTICK_DOWN) {
		//move down
		headY++;
	}
	snakeRefresh();
}

/*----------------------------------------------------------------------------
  Main Program
 *----------------------------------------------------------------------------*/
int main (void) {
	int loopCount = 1;
	int i, n;
	int Button_Flag;
	unsigned char testSel = TEST_VECT_INTR;
	//unsigned char testSel = TEST_POLL_INPT;
	//unsigned char testSel = TEST_LED_OUTPT;
	 
	//Following statement sets the timer interrupt frequency.
	//The clock rate on this boards MCU is 72 Mhz - this means
	//every second there will be 72 million clocks. So, if we use this
	//as the parameter to the function the interrupt rate is going
	//to be every second. If this is divided by 1000 then each clock
	//interrupt will be 1000 times more often, i.e. every 1 msec.
  	SysTick_Config(SystemCoreClock/1000);  //Generate interrupt every 1 ms

   	//Here we initialise the LED driver
  	LED_init();
	//Here we initialise the Joystick driver
	joyStick_Init();

	//Here we initialise the User button as an input source
	//If you wish to poll this source (PG8) for input you need
	//this initialisation.
	userButton_Init();
    //Here we initialise the User button as an interrupt source
	//If you wish to use this source (PG8) for interrupt vectoring
	//then you need	the above initialisation and this initialisation.
	userButton_IntrEnable();

    //Here we initialise the Joystick switches as interrupt sources
   	joyStick_IntrEnable_PG15_13();
	joyStick_IntrEnable_PG7();
	joyStick_IntrEnable_PD3();

#ifdef __USE_LCD
	//In order to use the LCD display, the device needs to be
	//properly configured and initialized (refer to GLCD_16bitIF_STM32.c file).
	//This is a complex process and is beyond the scope of this module.
  	GLCD_Init();                          //Initialize graphical LCD display

	//The following functions demonstrate how the LCD display can be used.
	//This is a 240 x 320 pixel colour screen and the pixels can be individually
	//manipulated allowing graphics shapes to be constructed and displayed.
	//This configuration allows 20 characters per line and 10 lines (16x24 pixel characters).
	GLCD_Clear(Black);                    /* Clear graphical LCD display        */
	GLCD_SetBackColor(Black);
	GLCD_SetTextColor(White);
	GLCD_DisplayString(0, 0, __FI, "<--PierreSoft Ltd-->");
	GLCD_DisplayString(1, 0, __FI, "<-----Presents----->");
	GLCD_DisplayString(2, 0, __FI, "Snake-Pierre Edition");
#endif // __USE_LCD

	//Used to select the test
	switch (testSel) {

		case 0:
			//Vectored interrupt routines
			displayTestMessage(3, 0, "Vect User Btn:", testSel);
			displayTestMessage(5, 0, "Vect Joystick:", testSel);
			loopCount = 1;
			user_btn_int = 0;
			joystick_int = 0;
			while (TRUE) {  //Loop forever
				if (user_btn_int == 1) {
					//Count the number of times the User button is pressed
					displayTestMessage(4, 0, "Button Down", loopCount);
					user_btn_int=0;
					loopCount += 1;
				}

				//Check to see if we got a joystick switch interrupt
				switch (joystick_int) {
					case JOYSTICK_UP:
						displayTestMessage(6, 0, "Joystick Up    ", joystick_int);
						joystick_int=0;
						break;
					case JOYSTICK_DOWN:
						displayTestMessage(6, 0, "Joystick Down  ", joystick_int);
						joystick_int=0;
						break;
					case JOYSTICK_LEFT:
						displayTestMessage(6, 0, "Joystick Left  ", joystick_int);
						joystick_int=0;
						break;
					case JOYSTICK_RIGHT:
						displayTestMessage(6, 0, "Joystick Right ", joystick_int);
						joystick_int=0;
						break;
					case JOYSTICK_SELECT:
						displayTestMessage(6, 0, "Joystick Select", joystick_int);
						joystick_int=0;
						break;
					default:
						break;
				}

			}
			break;

		case 1:
			//Polled button output state
			displayTestMessage(3, 0, "Poll User Btn", testSel);
			Button_Flag = 0;
			while (TRUE) {   //Loop forever
			    // Button inputs - Check to see if there is input signal available on PG8
				//Note: PG8 = 0 (input available), PG8 = 1 (input not available) - inverted logic!
			    if ((GPIOG->IDR & (1 <<  8)) == 0) {			
					if (Button_Flag == 0) {
						Button_Flag = 1;  			//Button freshly pressed
					}
				}
				else {
					if (Button_Flag == 2) {			//Button is released
					 	Button_Flag = 3;
					}
				}
		
				//Check to see if the button is freshly pressed
				//since the last release
				if (Button_Flag == 1) {
					displayTestMessage(4, 0, "Button Down", 0);
					Button_Flag = 2;
				}
				//Check to see if the button is released
				//since the last pressing
				else if (Button_Flag == 3) {
					displayTestMessage(4, 0, "Button Up  ", 0);
					Button_Flag = 0;
				}
			}
			break;

		case 2:
			//LED lights showcase!
			//The main body of the program requires a continuous loop which
			//generally executes various functions and monitors various device
			//states. It will be interrupted at regular intervals by the
			//vectored timer interrupt as explained above!

			GLCD_DisplayString(3, 0, __FI, "Ex1:");
			GLCD_DisplayString(4, 0, __FI, "Ex2:");
			GLCD_DisplayString(5, 0, __FI, "Ex3:");
			GLCD_DisplayString(6, 0, __FI, "Ex4:");
			GLCD_DisplayString(7, 0, __FI, "Ex5:");
			GLCD_DisplayString(8, 0, __FI, "Ex6:");

		  	while (TRUE) {    //Loop forever
		
				displayTestMessage(3, 4, "LED Test1", loopCount);
				//For all 8 LEDs do
			  	for (i = 0; i < LED_NUM; i++) {
					//Switch a LED on
					LED_On(i);
					//Wait for msecs
					delay(200);
					//Switch LED off
					LED_Off(i);
					//Wait for msecs 
					delay(200);
				}                                           
		
				displayTestMessage(4, 4, "LED Test2", loopCount);
				//Flash all LEDs 4 times
				for (i = 0; i != 3; i++) {
					All_LEDs_On();
					delay(200);
					All_LEDs_Off();
					delay(200);
				}
		
				displayTestMessage(5, 4, "LED Test3", loopCount);
				for (n = 1; n <= 4; n++) {
					for (i = 0; i < LED_NUM; i++) {
			   			LED_On(i);
						delay(n * 50);
					}
		
					delay(200);
		
					for (i = 0; i < LED_NUM; i++) {
			   			LED_Off(i);
						delay(n * 50);
					}
		
					delay(200);
				}
		
				displayTestMessage(6, 4, "LED Test4", loopCount);
				for (n = 0; n < 4; n++) {
					for (i = 0; i < HALF_LED_NUM; i++) {
						LED_On(LED_NUM-i-1);
			   			LED_On(i);
						delay(100);
					}
		
					delay(200);
		
					for (i = HALF_LED_NUM ; i >= 0; i--) {
						LED_Off(LED_NUM-i-1);
			   			LED_Off(i);
						delay(100);
					}
		
					delay(200);
				}
		
				displayTestMessage(7, 4, "LED Test5", loopCount);
				for (n = 0; n < 256; n ++) {
					for (i = 0; i < LED_NUM;  i++) {
						if ((1 << i) & n) {
						   LED_On(i);
						}
						else {
						   LED_Off(i);
						}
						delay(10);
					} 
				}
				break;
		
				/* PUT MORE LED DISPLAY PATTERNS BELOW */
	
	
				//Increment the loop count
				loopCount += 1;
	  		}
			break;

			default:
				break;		
	}
}
