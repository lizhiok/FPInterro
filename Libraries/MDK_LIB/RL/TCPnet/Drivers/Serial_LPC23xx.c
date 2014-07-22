/*----------------------------------------------------------------------------
 *      RL-ARM - TCPnet
 *----------------------------------------------------------------------------
 *      Name:    SERIAL.C
 *      Purpose: Serial Port Driver for Philips LPC23xx
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <Net_Config.h>
#include <LPC23xx.H>                 /* LPC214x definitions                  */

/* Local variables */
struct buf_st  {
  U8 in;
  U8 out;
  U8 buf [256];
};

static struct buf_st rbuf;
static struct buf_st tbuf;
static BIT    tx_active;

void handler_UART1 (void) __irq;
void def_interrupt (void) __irq;

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
  PINSEL0 &= ~0xC0000000;
  PINSEL0 |=  0x40000000;
  PINSEL1 &= ~0x00000003;
  PINSEL1 |=  0x00000001;
  /* 8-bits, no parity, 1 stop bit */
  U1LCR = 0x83;
  /* 115200 Baud Rate @ 12MHz PLL Clock */
  U1DLL = 3;
  U1DLM = 0;
  U1FDR = 6<<4 | 7;
  U1LCR = 0x03;
  /* Enable FIFO with 8-byte trigger level. */
  U1FCR = 0x87;
  /* Enable RDA and THRE interrupts. */
  U1IER = 0x03;
  /* Enable UART1 interrupts. */
  VICVectAddr7  = (U32)handler_UART1;
  VICIntEnable  = 1 << 7;
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
  VICIntEnClr = 1 << 7;
  if (tx_active == __FALSE) {
    /* Send directly to UART. */
    U1THR = (U8)c;
    tx_active = __TRUE;
  }
  else {
    /* Add data to transmit buffer. */
    p->buf [p->in++] = c;
  }
  VICIntEnable = 1 << 7;
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
  volatile U8 dummy;
  U32 iir,rbr;

  /* Repeat while there is at least one interrupt source. */
  while (((iir = U1IIR) & 0x01) == 0) {
    switch (iir & 0x0E) {
      case 0x06:
        /* Receive Line Status, just clear the source. */
        dummy = U1LSR;
        break;

      case 0x04:
        /* Receive Data Available */
      case 0x0C:
        /* Character Time-Out */
        p = &rbuf;
        /* Read a character to clear RDA Interrupt. */
        rbr = U1RBR;
        if ((U8)(p->in + 1) != p->out) {
          p->buf [p->in++] = (U8)rbr;
        }
        break;

      case 0x02:
        /* THRE Interrupt */
        p = &tbuf;
        if (p->in != p->out) {
          U1THR = p->buf [p->out++];
        }
        else {
          tx_active = __FALSE;
        }
        break;

      case 0x00: 
        /* Modem Interrupt, just clear the source. */
        dummy = U1MSR;
        break;
    }
  }
  /* Acknowledge Interrupt. */
  VICVectAddr = 0;
}


/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/

