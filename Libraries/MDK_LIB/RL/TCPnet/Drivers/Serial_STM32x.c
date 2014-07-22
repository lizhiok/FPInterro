/*----------------------------------------------------------------------------
 *      RL-ARM - TCPnet
 *----------------------------------------------------------------------------
 *      Name:    SERIAL.C
 *      Purpose: Serial Port Driver for STM32F10x
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <Net_Config.h>
#include <stm32f10x.h>                              /* STM32F10x definitions */

/* Local variables */
struct buf_st  {
  U8 in;
  U8 out;
  U8 buf [256];
};

static struct buf_st rbuf;
static struct buf_st tbuf;
static BIT    tx_active;

/*----------------------------------------------------------------------------
 *      Serial Driver Functions
 *----------------------------------------------------------------------------
 *  Required functions for Serial driver module:
 *   - void init_serial ()
 *   - int  com_get_char ()
 *   - BOOL com_putchar (U8 c)
 *   - BOOL com_tx_active ()
 *   - interrupt function(s)
 *---------------------------------------------------------------------------*/


/*--------------------------- init_serial -----------------------------------*/

void init_serial (void) {
  /* Initialize the serial interface */
  rbuf.in   = 0;
  rbuf.out  = 0;
  tbuf.in   = 0;
  tbuf.out  = 0;
  tx_active = __FALSE;

  /* Enable the clocks for AFIO, GPIOA, USART1 */
  RCC->APB2ENR |=  0x00004005;

  /* Enable USART1_Rx and USART1_Tx pins. */
  AFIO->MAPR &= ~(1 << 2);
  GPIOA->CRH &= 0xFFFFF00F;
  GPIOA->CRH |= 0x000004B0;

  /* Configure UART1 for 115200 baud, 8 bits, no Parity, 1 Stop bit. */
  /* PCLK2 = 72MHz */
  USART1->BRR  = 0x0271;           
  USART1->CR1  = 0x002C;
  USART1->CR2  = 0x0000;
  USART1->CR3  = 0x0000;
  USART1->CR1 |= 0x2000;

  /* Enable USART1 interrupts. */
  NVIC->ISER[1]|= (1 << 5);
}


/*--------------------------- com_putchar -----------------------------------*/

BOOL com_putchar (U8 c) {
  /* Write a byte to serial interface. */
  struct buf_st *p = &tbuf;

  /* Write a byte to serial interface */
  if ((U8)(p->in + 1) == p->out) {
    /* Serial transmit buffer is full. */
    return (__FALSE);
  }
  NVIC->ICER[1] |= (1 << 5);
  __nop ();
  if (tx_active == __FALSE) {
    /* Send directly to UART. */
    USART1->DR   = (U8)c;
    USART1->CR1 |= 0x0080;
    tx_active = __TRUE;
  }
  else {
    /* Add data to transmit buffer. */
    p->buf [p->in++] = c;
  }
  NVIC->ISER[1] |= (1 << 5);
  return (__TRUE);
}


/*--------------------------- com_getchar -----------------------------------*/

int com_getchar (void) {
  /* Read a byte from serial interface */
  struct buf_st *p = &rbuf;

  if (p->in == p->out) {
    /* Serial receive buffer is empty. */
    return (-1);
  }
  return (p->buf[p->out++]);
}


/*--------------------------- com_tx_active ---------------------------------*/

BOOL com_tx_active (void) {
  /* Return status Transmitter active/not active.         */
  /* When transmit buffer is empty, 'tx_active' is FALSE. */
  return (tx_active);
}


/*--------------------------- handler_UART1 ---------------------------------*/

void USART1_IRQHandler (void) {
  /* Serial Rx and Tx interrupt handler. */
  struct buf_st *p;
  U32 sr;

  sr = USART1->SR;
  if (sr & 0x0020) {
    /* Receive Buffer Not Empty */
    p = &rbuf;
    if ((U8)(p->in + 1) != p->out) {
      p->buf [p->in++] = (U8)USART1->DR;
    }
  }
  if ((sr & 0x0080) && (tx_active == __TRUE)) {
    /* Transmit Data Register Empty */
    p = &tbuf;
    if (p->in != p->out) {
      USART1->DR = p->buf [p->out++];
    }
    else {
      USART1->CR1 &= ~0x0080;
      tx_active = __FALSE;
    }
  }
}


/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/

