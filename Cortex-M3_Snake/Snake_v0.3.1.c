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
/******************************************************************************/
/* 			The following code is the modified version of Blinky.c            */
/* 			It has been modified by Erik Thomas & Zachary Wharton             */
/* 			for the coursework one task for CIS3105 Embedded Systems          */
/******************************************************************************/

/*Version 0.2.1
 *Authors - Erik Thomas & Zach Wharton
 *TODO:
 *		Add difficulty scaling
 *		Lights up whenever the snake length increases 
 */

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
#define TEST_ADC_TONE		1
#define TEST_POLL_INPT		2
#define TEST_LED_OUTPT		3
	
//Joystick direction constants (should be defined in a common .h file)
#define STATIONARY			0
#define JOYSTICK_UP			1
#define JOYSTICK_DOWN		2
#define JOYSTICK_LEFT		3
#define JOYSTICK_RIGHT		4
#define JOYSTICK_SELECT		5

#define INSANE	1
#define HARD 		2
#define MEDIUM	3
#define EASY		4

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

#define ADC1_IRQn                 18     		/*ADC1 global Interrupt                                */

//The following are declared external to this file (in IRQ.c)
extern unsigned int user_btn_int;
extern unsigned long ticks;
extern unsigned int joystick_int;

//Following are used for ADC
extern unsigned short AD_last;	 		//Last ADC conversion value
extern unsigned char  AD_done;			//ADC conversion complete flag

int gameOver;
int width = 20;
int height = 10;
//int tailMax = 100;
int tailX[100];
int tailY[100];
//time_t time;
//Initialises RNG
//srand((unsigned) time(&t));

struct food {
    unsigned char shape;
    int x;
    int y;
};

struct snake {
    unsigned char shape;
    int x;
    int y;
    int xD;
    int yD;
    //Tail segment set to allow increase in length of snake
    int tailLen;
};

struct snake sSnake;
struct food sFood;

//This function delays program by msecs milliseconds. This function relies on
//a timer which is used to produce an interrupt at regular intervals. See
//further down for the way this timer is activated. The timer interrupt
//handler is in IRQ.c source file. You need to supply the code for this
//interrupt (this is a vectored interrupt) in order for this function
//to work.

void delay10th(unsigned long tenthMsecs) {
	//Set the tick counter in number of 10th of msecs
	ticks = tenthMsecs;
	//Wait until the tick counter is 0 (zero)
	while (ticks) {}
}

void delay(unsigned long msecs) {
	//Set the tick counter in number of msecs
	ticks = 10 * msecs;
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
  Function that initializes ADC
 *----------------------------------------------------------------------------*/
void ADC_init (void) {
  RCC->APB2ENR |= ( 1UL <<  4);         /* enable periperal clock for GPIOC   */
  GPIOC->CRL &= ~0x000F0000;            /* Configure PC4 as ADC.14 input      */

  /* Setup and initialize ADC converter                                       */
  RCC->APB2ENR |= ( 1UL <<  9);         /* enable periperal clock for ADC1    */

  ADC1->SQR1    =  0;                   /* Regular channel 1 conversion       */
  ADC1->SQR2    =  0;                   /* Clear register                     */
  ADC1->SQR3    = (14UL <<  0);         /* SQ1 = channel 14                   */
  ADC1->SMPR1   = ( 5UL << 12);         /* sample time channel 14 55,5 cycles */
  ADC1->CR1     = ( 1UL <<  8);         /* Scan mode on                       */
  ADC1->CR2     = ( 7UL << 17)|         /* select SWSTART                     */
                  ( 1UL << 20) ;        /* enable ext. Trigger                */

  ADC1->CR1    |= ( 1UL <<  5);         /* enable for EOC Interrupt           */
  NVIC_EnableIRQ(ADC1_IRQn);            /* enable ADC Interrupt               */

  ADC1->CR2    |= ( 1UL <<  0);         /* ADC enable                         */

  ADC1->CR2    |=  1 <<  3;             /* Initialize calibration registers   */
  while (ADC1->CR2 & (1 << 3));         /* Wait for initialization to finish  */
  ADC1->CR2    |=  1 <<  2;             /* Start calibration                  */
  while (ADC1->CR2 & (1 << 2));         /* Wait for calibration to finish     */
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

//Initialises the speaker output pin PA4
void pin_PA4_For_Speaker(void) {
	RCC->APB2ENR |= (1UL << 2);         /* Enable GPIOA clock                 	*/

	GPIOA->ODR   &= ~0x00000100;        //Switch off PA4
	GPIOA->CRL   &= ~0xFFFFFFFF;        
  	GPIOA->CRL   |=  0x00030000;	   	//Configure pins PA4 to general purpose output mode
}

//Produces a square wave of frequency f Hz and duration of d msecs
void generate_Tone_On_Speaker(int f, int d) {
	int n;
	int t;

	t = (10000 / f);					//10thMsecs
	for (n = ((d * 10) / t); n; n--) {
		GPIOA->BSRR |= (1UL << 4);   	//Pin PA4 on
		delay10th(t >> 1);
		GPIOA->BRR |= (1UL << 4);    	//Pin PA4 off
		delay10th(t >> 1);
	}
}

//Frequency Shift Key modulation example - takes a string and produces FSK tones!
void generate_FSK_Tone(unsigned char *s) {
	int i = 0;
	int n;
	int v;
	unsigned char c;
	unsigned char m[2] = {' ','\0'};

	//Get each character of the string
	
	GLCD_DisplayString(7, i, __FI, "                   ");
	
	GLCD_SetBackColor(Blue);
	GLCD_SetTextColor(White);

	while (s[i]) {
		c = s[i];
		m[0] = c;
		GLCD_DisplayString(7, i + 1, __FI, (unsigned char *)m);
		//Check each bit of the character
		for (n = 0; n < 8; n++) {
			//Determine if a 1 or a 0
			v = ((c >> n) & 1);
			if (v) {
				//If a 1 then generate 600 Hz tone for 20 msecs
				generate_Tone_On_Speaker(600, 20);
			}
			else {
				//If a 0 then generate 1200 Hz tone for 20 msecs
				generate_Tone_On_Speaker(1200, 20);
			}
		}
		i++;
	}
	
	GLCD_SetBackColor(White);
	GLCD_SetTextColor(Blue);
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

void displayInit(void) {
	//This is a separate method to snakeInit due to the LCD screen being drawn to before logic is performed
	
}

int snakeInit(void) {
	//Initialises the relevant values for drawing the snake and its food
	const char *difficulties[4] = { "Easy", "Medium", "Hard", "Insane" };
	
	GLCD_Clear(Black);                    /* Clear graphical LCD display        */
	GLCD_SetBackColor(Black);
	GLCD_SetTextColor(White);
	GLCD_DisplayString(0, 0, __FI, "CIS-3108");
	GLCD_DisplayString(1, 0, __FI, "Snake-Game");
	GLCD_DisplayString(2, 0, __FI, "Press joystick to");
	GLCD_DisplayString(3, 0, __FI, "Select difficulty:");
	for (int i=5; i<sizeof(difficulties); i++) {
		GLCD_DisplayString(i, 5, __FI, (unsigned char*)difficulties[i-5]);
	}

	gameOver = FALSE;

	sSnake.x = width/2;
	sSnake.y = height/2;
	sSnake.x = 10;
	sSnake.y = 5;
	sSnake.xD = STATIONARY;
	sSnake.yD = STATIONARY;

	sFood.x = rand() % width;
	sFood.y = rand() % height;
}

void snakeDrawing(void) {
    //Draws the snake to the LCD screen
    sSnake.shape = 'o';
    sFood.shape = 'x';

    GLCD_DisplayString(sSnake.y, sSnake.x, __FI, &sSnake.shape);
    GLCD_DisplayString(sFood.y, sFood.x, __FI, &sFood.shape);
    for (int i=0; i<sSnake.tailLen; i++) {
        GLCD_DisplayString(tailY[i], tailX[i], __FI, &sSnake.shape);
    }   
}

void snakeLogic(xDir, yDir) {
    //Performs the logical operations to move the snake, read jostick inputs etc.
    int lastX = tailX[0];
    int lastY = tailY[0];
    int lastX_, lastY_;

    tailX[0] = sSnake.x;
    tailY[0] = sSnake.y;

    for (int i=1; i<sSnake.tailLen; i++) {
        lastX_ = tailX[i];
        lastY_ = tailY[i];
        tailX[i] = lastX;
        tailY[i] = lastY;
        lastX = lastX_;
        lastY = lastY_;
    }
		
		if (xDir == 0 && yDir == -1) {
			sSnake.y--;
		}
		else if (xDir == 0 && yDir == 1) {
			sSnake.y++;
		}
		else if (xDir == -1 && yDir == 0) {
			sSnake.x--;
		}
		else if (xDir == 1 && yDir == 0) {
			sSnake.x++;
		}
		
    if (sSnake.x >= width) {
        sSnake.x = 0;
    } else if (sSnake.x < 0) {
        sSnake.x = width - 1;
    }
    if (sSnake.y >= width) {
        sSnake.y = 0;
    } else if (sSnake.y < 0) {
        sSnake.y = height - 1;
    }

    if (sSnake.x == sFood.x && sSnake.y == sFood.y) {
        sFood.x = rand() % width;
        sFood.y = rand() % height;
        sSnake.tailLen++;
    }
    //Checks if the snake is colliding with itself. If true, game over
    for (int i=0; i<sSnake.tailLen; i++) {
        if (tailX[i] == sSnake.x && tailY[i] == sSnake.y) {
            gameOver = TRUE;
        }
    }
}

/*----------------------------------------------------------------------------
  Main Program
 *----------------------------------------------------------------------------*/
int main (void) {
	int loopCount = 1;
	int c, i, n;
	int Button_Flag;
	int doTone;
	 
	//Following statement sets the timer interrupt frequency.
	//The clock rate on this boards MCU is 72 Mhz - this means
	//every second there will be 72 million clocks. So, if we use this
	//as the parameter to the function the interrupt rate is going
	//to be every second. If this is divided by 1000 then each clock
	//interrupt will be 1000 times more often, i.e. every 1 msec.
  	//SysTick_Config(SystemCoreClock/1000);  //Generate interrupt every 1 ms
	SysTick_Config(SystemCoreClock/10000);  //Generate interrupt every 0.1 ms

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
   	
	//Initialise speaker output
	pin_PA4_For_Speaker();

	//Initialise ADC
	ADC_init();

#ifdef __USE_LCD
	//In order to use the LCD display, the device needs to be
	//properly configured and initialized (refer to GLCD_16bitIF_STM32.c file).
	//This is a complex process and is beyond the scope of this module.
  	GLCD_Init();                          //Initialize graphical LCD display

	//The following functions demonstrate how the LCD display can be used.
	//This is a 240 x 320 pixel colour screen and the pixels can be individually
	//manipulated allowing graphics shapes to be constructed and displayed.
	//This configuration allows 20 characters per line and 10 lines (16x24 pixel characters).
#endif // __USE_LCD

	AD_done = 0;
	ADC1->CR2 |= (1UL << 22);
	while(TRUE) {
		if (AD_done) {
			srand(AD_last);
			break;
		}
	}

	//Used to select the test
	//Make this adaptable using input (?)
	int difficulty;
	int difficultySelected = FALSE;
	int nDelay = 50;
	loopCount = 1;
	user_btn_int = 0;
	joystick_int = 0;
	snakeInit();
	//Work in progress, needs refining
	while(!difficultySelected) {
		switch(joystick_int) {
			case JOYSTICK_UP:
				difficulty = INSANE;
				nDelay = difficulty*nDelay;
				difficultySelected = TRUE;
				joystick_int=0;
				break;
			case JOYSTICK_DOWN:
				difficulty = MEDIUM;
				nDelay = difficulty*nDelay;
				difficultySelected = TRUE;
				joystick_int=0;
				break;
			case JOYSTICK_LEFT:
				difficulty = EASY;
				nDelay = difficulty*nDelay;
				difficultySelected = TRUE;
				joystick_int=0;
				break;
			case JOYSTICK_RIGHT:
				difficulty = HARD;
				nDelay = difficulty*nDelay;
				difficultySelected = TRUE;
				joystick_int=0;
				break;
		}
		GLCD_Clear(Black);
		break;
	}
	

	
	while(TRUE) {
		if (!gameOver) {
			snakeDrawing();
			switch(joystick_int) {
				case JOYSTICK_UP:
					sSnake.xD=0;
					sSnake.yD=-1;
					snakeLogic(sSnake.xD, sSnake.yD);
					joystick_int=0;
					break;
				case JOYSTICK_DOWN:
					sSnake.xD=0;
					sSnake.yD=1;
					snakeLogic(sSnake.xD, sSnake.yD);
					joystick_int=0;
					break;
				case JOYSTICK_LEFT:
					sSnake.xD=-1;
					sSnake.yD=0;
					snakeLogic(sSnake.xD, sSnake.yD);
					joystick_int=0;
					break;
				case JOYSTICK_RIGHT:
					sSnake.xD=1;
					sSnake.yD=0;
					snakeLogic(sSnake.xD, sSnake.yD);
					joystick_int=0;
					break;
			}
			//Can alter this to increase/decrease difficulty
			delay(nDelay);
			snakeLogic(sSnake.xD, sSnake.yD);
			GLCD_Clear(Black);
		} else {
			gameOver = FALSE;
			snakeInit();
		}
	}
}
