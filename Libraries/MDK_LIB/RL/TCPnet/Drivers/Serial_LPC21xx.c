/*----------------------------------------------------------------------------
 *      RL-ARM - TCPnet
 *----------------------------------------------------------------------------
 *      Name:    SERIAL.C
 *      Purpose: Serial Port Driver for Philips LPC21xx
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <Net_Config.h>
#include <LPC21xx.H>                 /* LPC21xx definitions                  */

/* Local variables */
struct buf_st  {
  U8 in;
  U8 out;
  U8 buf [256];
};

static struct buf_st rbuf;
static struct buf_st tbuf;
static BOOL   tx_active;

void handler_UART1 (void) __irq;

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

  /* Enable RxD1 and TxD1 pins. */
  PINSEL0 &= ~0x000F0000;
  PINSEL0 |=  0x00050000;
  /* 8-bits, no parity, 1 stop bit */
  U1LCR = 0x83;
  /* 19200 Baud Rate @ 15MHz VPB Clock */
  U1DLL = 49;
  U1DLM = 0;
  U1LCR = 0x03;
  /* Enable RDA and THRE interrupts. */
  U1IER = 0x03;
  /* Enable UART1 interrupts. */
  VICVectAddr14 = (U32)handler_UART1;
  VICVectCntl14 = 0x27;
  VICIntEnable  = (1 << 7);
}


/*--------------------------- com_putchar -----------------------------------*/

BOOL com_putchar (U8 c) {
  struct buf_st *p = &tbuf;

  /* Write a byte to serial interface */
  if ((U8)(p->in + 1) == p->out) {
    /* Serial transmit buffer is full. */
    return (__FALSE);
  }
  VICIntEnClr = (1 << 7);
  if (tx_active == __FALSE) {
    /* Send directly to UART. */
    U1THR = (U8)c;
    tx_active = __TRUE;
  }
  else {
    /* Add data to transmit buffer. */
    p->buf [p->in++] = c;
  }
  VICIntEnable = (1 << 7);
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

void handler_UART1 (void) __irq {
  /* Serial Rx and Tx interrupt handler. */
  struct buf_st *p;
  U8 rbr;

  if ((U1IIR & 0x0E) == 0x04) {
    /* Serial Receive Interrupt. */
    p = &rbuf;
    /* Read a character to clear RDA Interrupt. */
    rbr = U1RBR;
    if ((p->in + 1) != p->out) {
      p->buf [p->in++] = (U8)rbr;
    }
  }
  if (U1LSR & 0x20)  {
    /* Serial Transmit Interrupt. */
    p = &tbuf;
    if (p->in != p->out) {
      U1THR = p->buf [p->out++];
    }
    else {
      tx_active = __FALSE;
    }
  } 
  /* Acknowledge Interrupt. */
  VICVectAddr = 0;
}


/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/

