/*----------------------------------------------------------------------------
 *      RL-ARM - FlashFS
 *----------------------------------------------------------------------------
 *      Name:    SPI_LPC214X.C
 *      Purpose: Serial Peripheral Interface Driver for NXP LPC214x
 *      Rev.:    V4.20
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2011 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <File_Config.h>
#include <LPC214x.H>                 /* LPC214x definitions                  */

/*----------------------------------------------------------------------------
  SPI Driver instance definition
   spi0_drv: First SPI driver
   spi1_drv: Second SPI driver
 *---------------------------------------------------------------------------*/

#define __DRV_ID  spi0_drv
#define __PCLK    48000000

/* SPI Driver Interface functions */
static BOOL Init (void);
static BOOL UnInit (void);
static U8   Send (U8 outb);
static BOOL SendBuf (U8 *buf, U32 sz);
static BOOL RecBuf (U8 *buf, U32 sz);
static BOOL BusSpeed (U32 kbaud);
static BOOL SetSS (U32 ss);
static U32  CheckMedia (void);        /* Optional function for SD card check */

/* SPI Device Driver Control Block */
SPI_DRV __DRV_ID = {
  Init,
  UnInit,
  Send,
  SendBuf,
  RecBuf,
  BusSpeed,
  SetSS,
  CheckMedia                          /* Can be NULL if not existing         */
};


/* SSPSR - bit definitions. */
#define TFE     0x01
#define TNF     0x02
#define RNE     0x04
#define RFF     0x08
#define BSY     0x10

/*--------------------------- Init ------------------------------------------*/

static BOOL Init (void) {
  /* Initialize and enable the SSP Interface module. */

  /* SSEL is GPIO, output set to high. */
  IODIR0 |= 1<<20;
  IOSET0  = 1<<20;
  /* SCK1, MISO1, MOSI1 are SSP pins. */
  PINSEL1 = (PINSEL1 & ~0x000003FC) | 0x000000A8;

  /* Enable SPI in Master Mode, CPOL=0, CPHA=0. */
  SSPCR0  = 0x0007;
  SSPCR1  = 0x0002;
  SSPCPSR = 0xFE;
  return (__TRUE);
}


/*--------------------------- UnInit ----------------------------------------*/

static BOOL UnInit (void) {
  /* Return SSP interface to default state. */

  IODIR0  &= ~(1<<20);
  PINSEL1 &= ~0x000003FC;
  SSPCR1  = 0x0000;
  SSPCR0  = 0x0000;
  SSPCPSR = 0x00;
  return (__TRUE);
}


/*--------------------------- Send ------------------------------------------*/

static U8 Send (U8 outb) {
  /* Send and Receive a byte on SPI interface. */

  SSPDR = outb;
  /* Wait if RNE cleared, Rx FIFO is empty. */
  while (!(SSPSR & RNE));
  return (SSPDR);
}


/*--------------------------- SendBuf ---------------------------------------*/

static BOOL SendBuf (U8 *buf, U32 sz) {
  /* Send buffer to SPI interface. */
  U32 i;

  for (i = 0; i < sz; i++) {
    SSPDR = buf[i];
    /* Wait if Tx FIFO is full. */
    while (!(SSPSR & TNF));
    SSPDR;
  }
  /* Wait until Tx finished, drain Rx FIFO. */
  while (SSPSR & (BSY | RNE)) {
    SSPDR;
  }
  return (__TRUE);
}


/*--------------------------- RecBuf ----------------------------------------*/

static BOOL RecBuf (U8 *buf, U32 sz) {
  /* Receive SPI data to buffer. */
  U32 i;

  for (i = 0; i < sz; i++) {
    SSPDR = 0xFF;
    /* Wait while Rx FIFO is empty. */
    while (!(SSPSR & RNE));
    buf[i] = SSPDR;
  }
  return (__TRUE);
}


/*--------------------------- BusSpeed --------------------------------------*/

static BOOL BusSpeed (U32 kbaud) {
  /* Set an SPI clock to required baud rate. */
  U32 div;

  div = (__PCLK/1000 + kbaud - 1) / kbaud;
  if (div == 0)   div = 0x02;
  if (div & 1)    div++;
  if (div > 0xFE) div = 0xFE;
  SSPCPSR = div;
  return (__TRUE);
}


/*--------------------------- SetSS -----------------------------------------*/

static BOOL SetSS (U32 ss) {
  /* Enable/Disable SPI Chip Select (drive it high or low). */

  if (ss) {
    /* SSEL is GPIO, output set to high. */
    IOSET0 = 1<<20;
  } 
  else {
    /* SSEL is GPIO, output set to low. */
    IOCLR0 = 1<<20;
  }
  return (__TRUE);
}


/*--------------------------- CheckMedia ------------------------------------*/

static U32 CheckMedia (void) {
  /* Read CardDetect and WriteProtect SD card socket pins. */
  U32 stat = 0;
 
#if 0
  if (!(IOPIN0 & 0x04)) {
    /* Card is inserted (CD=0). */
    stat |= M_INSERTED;
  } 
  if ((IOPIN0 & 0x20)) {
    /* Write Protect switch is active (WP=1). */
    stat |= M_PROTECTED;
  }
#else
  /* When CD,WP signals are not connected. */
  stat = M_INSERTED;
#endif
  return (stat);
}

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
