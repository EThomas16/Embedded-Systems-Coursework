/******************************************************************************/
/* Serial.c: Low Level Serial Routines                                        */
/******************************************************************************/
/* This file is part of the uVision/ARM development tools.                    */
/* Copyright (c) 2010 Keil - An ARM Company. All rights reserved.             */
/* This software may only be used under the terms of a valid, current,        */
/* end user licence from KEIL for a compatible version of KEIL software       */
/* development tools. Nothing else gives you the right to use this software.  */
/******************************************************************************/

#include "stm32f10x.h"                  /* STM32F10x.h definitions            */

#define USARTx  USART2

/*----------------------------------------------------------------------------
  Initialize UART pins, Baudrate
 *----------------------------------------------------------------------------*/
void SER_init (void) {
#ifndef __DBG_ITM
  int i;


  /* Configure UART2 for 115200 baud                                          */
  RCC->APB2ENR |= ( 1UL <<  2);         /* Enable GPIOA clock                 */
  GPIOA->CRL   &= 0xFFFF00FF;
  GPIOA->CRL   |= 0x00004B00;           /* PA2 = USART2_TX, PA3 = USART2_RX   */

  RCC->APB1ENR |= ( 1UL << 17);         /* Enable USART#2 clock               */

  USARTx->BRR   = 0x0135;               /* Configure 115200 baud @ 72MHz      */
  USARTx->CR3   = 0x0000;               /*           8 bit, 1 stop bit,       */     
  USARTx->CR2   = 0x0000;               /*           no parity                */
  for (i = 0; i < 0x1000; i++) __NOP(); /* avoid unwanted output              */
  USARTx->CR1 = 0x200C;
#endif
}


/*----------------------------------------------------------------------------
  Write character to Serial Port
 *----------------------------------------------------------------------------*/
int sendchar (int c) {

#ifdef __DBG_ITM
    ITM_SendChar(c);
#else
  while (!(USARTx->SR & 0x0080));
  USARTx->DR = (c & 0x1FF);
#endif
  return (c);
}


/*----------------------------------------------------------------------------
  Read character from Serial Port   (blocking read)
 *----------------------------------------------------------------------------*/
int getkey (void) {
#ifdef __DBG_ITM
  return(-1);
#else
  while (!(USARTx->SR & 0x0020));
  return (USARTx->DR);
#endif
}
