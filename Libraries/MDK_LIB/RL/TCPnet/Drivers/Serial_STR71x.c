/*----------------------------------------------------------------------------
 *      RL-ARM - TCPnet
 *----------------------------------------------------------------------------
 *      Name:    SERIAL.C
 *      Purpose: Serial Port Driver for ST STR71x
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <Net_Config.h>
#include <71x_lib.H>

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
void __swi(8) swi_putchar (U8 c);

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
  GPIO_Config(GPIO0, 0x0800, GPIO_AF_PP);
  GPIO_Config(GPIO0, 0x0400, GPIO_IN_TRI_CMOS);

  /* Configure UART1 for 115200 baud, 8 bits, no Parity, 1 Stop bit. */
  UART_OnOffConfig(UART1, ENABLE);
  UART_FifoConfig (UART1, DISABLE);
  UART_FifoReset  (UART1, UART_RxFIFO);
  UART_FifoReset  (UART1, UART_TxFIFO);
  UART_LoopBackConfig(UART1, DISABLE);
  UART_Config(UART1, 115200, UART_NO_PARITY, UART_1_StopBits, UARTM_8D);
  UART_RxConfig(UART1 ,ENABLE);
  /* Enable RBNE and TE interrupts. */
  UART_ItConfig (UART1, UART_RxBufFull, ENABLE);

  /* Configure EIC interrut controller. */
  EIC->IVR = (U32)handler_UART1;
  EIC->SIR[UART1_IRQChannel] = ((U32)handler_UART1 << 16);
  EIC_IRQChannelConfig(UART1_IRQChannel, ENABLE);
  EIC_IRQChannelPriorityConfig(UART1_IRQChannel, 1);
  EIC_IRQConfig(ENABLE);
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
  swi_putchar (c);
  return (__TRUE);
}

void __SWI_8 (U8 c) {
  /* IRQ Protected putchar */
  struct buf_st *p = &tbuf;

  if (tx_active == __FALSE) {
    /* Send directly to UART. */
    UART1->TxBUFR = (U8)c;
    UART1->IER |= UART_TxEmpty;
    tx_active = __TRUE;
  }
  else {
    /* Add data to transmit buffer. */
    p->buf [p->in++] = c;
  }
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
  U32 sr,rxb;

  sr = UART1->SR;
  if (sr & UART_RxBufFull) {
    /* Receive Data Available */
    p = &rbuf;
    rxb = UART1->RxBUFR;
    if ((U8)(p->in + 1) != p->out) {
      p->buf [p->in++] = (U8)rxb;
    }
  }
  if ((sr & UART_TxEmpty) && (tx_active == __TRUE)) {
    p = &tbuf;
    if (p->in != p->out) {
      UART1->TxBUFR = p->buf [p->out++];
    }
    else {
      UART1->IER &= ~UART_TxEmpty;
      tx_active = __FALSE;
    }
  }
  /* Acknowledge Interrupt. */
  EIC->IPR = 1 << UART1_IRQChannel;
}


/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/

