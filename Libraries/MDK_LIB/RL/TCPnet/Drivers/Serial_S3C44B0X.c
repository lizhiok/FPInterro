/*----------------------------------------------------------------------------
 *      RL-ARM - TCPnet
 *----------------------------------------------------------------------------
 *      Name:    SERIAL.C
 *      Purpose: Serial Port Driver for Samsung S3C44B0X
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include "Net_Config.h"
#include <S3C44B0X.H>                         /* S3C44B0X definitions        */

#define MCLK 60000000                         /* Master Clock                */
#define BR      38400                         /* Baud Rate                   */

#define BRD ((int)(MCLK/16.0/BR+0.5)-1)       /* Baud Rate Divisor           */


/* Local variables */
struct buf_st  {
  U8 in;
  U8 out;
  U8 buf [256];
};

static struct buf_st rbuf;
static struct buf_st tbuf;
static BOOL   tx_active;

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

  /* Normal mode, 8-bits, no parity, 1 stop bit */
  pUART0->ULCON  = NORMAL_MODE | PARITY_NONE | STOP_ONE   | DATA_8;
  pUART0->UCON   = RXM_INTPOL  | TXM_INTPOL  | RX_TIMEOUT;
  pUART0->UFCON  = FIFO_EN     | TXF_RST     | RXF_RST;
  pUART0->UBRDIV = BRD;

  /* Enable UART0 interrupts. */
  pIC->INTMSK &= ~(INT_GLOBAL | INT_URXD0  | INT_UTXD0);
  pIC->INTCON  =  FIQ_DISABLE | IRQ_ENABLE | VECT_IRQ;
}


/*--------------------------- com_putchar -----------------------------------*/

BOOL com_putchar (U8 c) {
  /* Write a byte to serial interface */
  struct buf_st *p = &tbuf;

  if ((U8)(p->in + 1) == p->out) {
    /* Serial transmit buffer is full. */
    return (__FALSE);
  }
  pIC->INTMSK |= INT_UTXD0;
  if (tx_active == __FALSE) {
    /* Send directly to UART FIFO. */
    pUART0->UTXH = c;
    tx_active = __TRUE;
  }
  else {
    /* Add data to the transmit buffer. */
    p->buf [p->in++] = c;
  }
  pIC->INTMSK &= ~INT_UTXD0;
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


/*--------------------------- com_getchar -----------------------------------*/

BOOL com_tx_active (void) {
  /* Return status Transmitter active/not active.        */
  /* When transmit buffer is empty, 't_active' is FALSE. */
  return (tx_active);
}


/*--------------------------- HandlerURXD0 ----------------------------------*/

void HandlerURXD0 (void) __irq  {
  /* Serial receive interrupt function. */
  struct buf_st *p = &rbuf;

  if ((p->in + 1) != p->out) {
    p->buf [p->in++] = pUART0->URXH;
  }
  pIC->I_ISPC = INT_URXD0;
}


/*--------------------------- HandlerUTXD0 ----------------------------------*/

void HandlerUTXD0 (void) __irq {
  /* Serial transmit interrupt function. */
  struct buf_st *p = &tbuf;

  if (p->in != p->out) {
    pUART0->UTXH = p->buf [p->out++];
  }
  else {
    tx_active = __FALSE;
  }
  pIC->I_ISPC = INT_UTXD0;
}


/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/

