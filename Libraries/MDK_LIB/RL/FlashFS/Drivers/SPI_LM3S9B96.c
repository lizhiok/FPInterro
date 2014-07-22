/*-----------------------------------------------------------------------------
 *      RL-ARM - FlashFS
 *-----------------------------------------------------------------------------
 *      Name:    SPI_LM3S9B96.c
 *      Purpose: Serial Peripheral Interface Driver for Luminary LM3S9B96
 *      Rev.:    V4.60
 *-----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2012 KEIL - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include <lm3s9b96.h>                   /* LM3S9B96 Definitions               */
#include <File_Config.h>

#define __SSI_CLOCK 64000000            /* SSI peripheral clock               */

#define __DRV_ID  spi0_drv

/* SPI Driver Interface functions */
static BOOL Init        (void);
static BOOL UnInit      (void);
static U8   Send        (U8 outb);
static BOOL SendBuf     (U8 *buf, U32 sz);
static BOOL RecBuf      (U8 *buf, U32 sz);
static BOOL BusSpeed    (U32 kbaud);
static BOOL SetSS       (U32 ss);
static U32  CheckMedia  (void);         /* Optional function for SD card check*/

/* SPI Device Driver Control Block */
SPI_DRV __DRV_ID = {
  Init,
  UnInit,
  Send,
  SendBuf,
  RecBuf,
  BusSpeed,
  SetSS,
  CheckMedia                            /* Can be NULL if not existing        */
};


/*--------------------------- Init -------------------------------------------*/

static BOOL Init (void) {
  /* Initialize and enable the SSI peripheral */

  SYSCTL_RCGC1_R |= (1 << 4);           /* Enable SSI0 peripheral clock       */

  /* Configure I/O pins */
  GPIO_PORTA_PCTL_R   |= 0x00110100;    /* PA2,4,5 -> Alt. function 1 (SSI0)  */
  GPIO_PORTA_PUR_R    |= 0x2c;          /* Enable pull-up resistors           */
  GPIO_PORTA_AFSEL_R  |= 0x34;          /* PA2,4,5 are controled by SSI       */
  GPIO_PORTA_AFSEL_R  &= ~0x8;          /* PA3 is GPIO pin                    */
  GPIO_PORTA_DEN_R    |= 0x3c;          /* Digital enable on PA2,3,4,5 pins   */
  GPIO_PORTA_DR4R_R   |= 0x2c;          /* Enable 4mA drive on PA2,3,5 pins   */
  GPIO_PORTA_DIR_R    |= 0x08;          /* PA3 is GPIO output                 */

  SetSS(1);                             /* Set chip select high               */

  /* Configure the SSI0 peripheral */
  SSI0_CR1_R &= ~(0x1F);                /* Master mode, SSI disabled          */
  SSI0_CR0_R |=   0x07;                 /* Clockrate = 1, Freescale Format    */
                                        /* Pol=0, Pha=0, 8 Bit                */
  SSI0_CR1_R |=   0x02;                 /* SSI Operation enabled              */

  return (__TRUE);
}


/*--------------------------- UnInit -----------------------------------------*/

static BOOL UnInit (void) {
  /* Return SSI interface to default state. */
  SYSCTL_RCGC1_R &= ~(1 << 4);          /* Disable SSI0 peripheral clock      */
  return (__TRUE);
}

/*--------------------------- Send -------------------------------------------*/

static U8 Send (U8 outb) {
  /* Write and Read a byte on SPI interface. */

  SSI0_DR_R = outb;
  while (!(SSI0_SR_R & SSI_SR_RNE));    /* Wait until Rx FIFO is empty`       */
  return (SSI0_DR_R);
}


/*--------------------------- SendBuf ----------------------------------------*/

static BOOL SendBuf (U8 *buf, U32 sz) {
  /* Send buffer to SPI interface. */
  U32 i;

  for (i = 0; i < sz; i++) {
    /* Wait if Tx FIFO is full. */
    while (!(SSI0_SR_R & SSI_SR_TNF));
    
    SSI0_DR_R = buf[i];

    /* Wait if Rx FIFO is empty */
    while (!(SSI0_SR_R & SSI_SR_RNE));
    SSI0_DR_R;
  }
  /* Wait until Tx finished, drain Rx FIFO */
  while (SSI0_SR_R & (SSI_SR_BSY | SSI_SR_RNE)) {
    SSI0_DR_R;
  }
  return (__TRUE);
}


/*--------------------------- RecBuf -----------------------------------------*/

static BOOL RecBuf (U8 *buf, U32 sz) {
  /* Receive SPI data to buffer. */
  U32 i;

  for (i = 0; i < sz; i++) {
    /* Wait if transmit FIFO full */
    while (!(SSI0_SR_R & SSI_SR_TNF));
    SSI0_DR_R = 0xFF;

    /* Wait if Rx FIFO is empty */
    while (!(SSI0_SR_R & SSI_SR_RNE));
    buf[i] = SSI0_DR_R;
  }
  return (__TRUE);
}

/*--------------------------- BusSpeed ---------------------------------------*/

static BOOL BusSpeed (U32 kbaud) {
  /* Set bus clock to required baud rate. */
  U32 clk, maxbr, dvsr, scr;

  /* Set SPI clock to required speed */
  clk = kbaud * 1000;

  if (clk > 25000000) {
    /* SSI clock cannot go beyond 25MHz */
    clk = 25000000;
  }

  maxbr = (__SSI_CLOCK + clk) / clk;
  dvsr  = 0;
  do {
    dvsr += 2;
    for (scr = 0; scr < 255; scr++) {
      if (maxbr <= (dvsr * (1 + scr))) {
        SSI0_CR1_R   = 0;               /* SSI disable                        */
        SSI0_CPSR_R  = dvsr;
        SSI0_CR0_R   = (scr << 8) | 7;
        SSI0_CR1_R  |= 2;               /* SSI enable                         */
        return (__TRUE);
      }
    }
  }
  while (dvsr < 255);
  return (__FALSE);
}


/*--------------------------- SetSS ------------------------------------------*/

static BOOL SetSS (U32 ss) {
  /* Enable/Disable SPI Chip Select (drive it high or low) */
  if (ss) {
    GPIO_PORTA_DATA_R |= 0x8;           /* SD card deselected                 */
  }
  else {
    GPIO_PORTA_DATA_R &= ~0x8;          /* SD card selected                   */
  }
  return (__TRUE);
}


/*--------------------------- CheckMedia -------------------------------------*/

static U32 CheckMedia (void) {
  /* Read CardDetect and WriteProtect SD card socket pins */
  U32 stat = 0;

#if 0 
  if (!(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_7))) {
    /* Card is inserted (CD=0). */
    stat |= M_INSERTED;
  } 
  if (GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_1)) {
    /* Write Protect switch is active (WP=1) */
    stat |= M_PROTECTED;
  }
#else
  /* When CD, WP signals are not connected */
  stat = M_INSERTED;
  return (stat);
#endif
}

/*-----------------------------------------------------------------------------
 * end of file
 *----------------------------------------------------------------------------*/
