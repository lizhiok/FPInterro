/*----------------------------------------------------------------------------
 *      RL-ARM - FlashFS
 *----------------------------------------------------------------------------
 *      Name:    SPI_STR91x.c
 *      Purpose: Serial Peripheral Interface Driver for ST STR91x
 *      Rev.:    V4.20
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2011 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <File_Config.h>
#include <91x_lib.h>                  /* STR91x Library Definitions          */

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


/* SSP_SR - bit definitions. */
#define TFE     0x01
#define TNF     0x02
#define RNE     0x04
#define RFF     0x08
#define BSY     0x10

/*--------------------------- Init ------------------------------------------*/

static BOOL Init (void) {
  /* Initialize and enable the SSP Interface module. */

  /* SCLK, MISO1, MOSI1 are SSP pins. */
  SCU->GPIOOUT[5] &= 0x00FF;
  SCU->GPIOOUT[5] |= 0x4A00;
  SCU->GPIOIN[5]  |= 0x40;
  /* NSS is GPIO, output set to high. */
  GPIO5->DDR        |= 0x80;
  GPIO5->DR[0x80<<2] = 0x80;

  /* Enable SPI in Master Mode, CPOL=0, CPHA=0. */
  SSP0->CR0 = 0x0007;
  SSP0->CR1 = 0x0002;
  SSP0->PR  = 0xFE;
  return (__TRUE);
}


/*--------------------------- UnInit ----------------------------------------*/

static BOOL UnInit (void) {
  /* Reset SSP interface to default state. */

  SCU->GPIOOUT[5] &= 0x00FF;
  SCU->GPIOIN[5]  &= 0xBF;
  GPIO5->DDR      &= 0x7F;
  SSP0->CR1 = 0x0000;
  SSP0->CR0 = 0x0000;
  SSP0->PR  = 0x00;
  return (__TRUE);
}


/*--------------------------- Send ------------------------------------------*/

static U8 Send (U8 outb) {
  /* Send and Receive a byte on SPI interface. */

  SSP0->DR = outb;
  /* Wait if RNE cleared, Rx FIFO is empty. */
  while (!(SSP0->SR & RNE));
  return (SSP0->DR);
}


/*--------------------------- SendBuf ---------------------------------------*/

static BOOL SendBuf (U8 *buf, U32 sz) {
  /* Send buffer to SPI interface. */
  U32 i;

  for (i = 0; i < sz; i++) {
    SSP0->DR = buf[i];
    /* Wait if Tx FIFO is full. */
    while (!(SSP0->SR & TNF));
    SSP0->DR;
  }
  /* Wait until Tx finished, drain Rx FIFO. */
  while (SSP0->SR & (BSY | RNE)) {
    SSP0->DR;
  }
  return (__TRUE);
}


/*--------------------------- RecBuf ----------------------------------------*/

static BOOL RecBuf (U8 *buf, U32 sz) {
  /* Receive SPI data to buffer. */
  U32 i;

  for (i = 0; i < sz; i++) {
    SSP0->DR = 0xFF;
    /* Wait while Rx FIFO is empty. */
    while (!(SSP0->SR & RNE));
    buf[i] = SSP0->DR;
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
  SSP0->PR = div;
  return (__TRUE);
}


/*--------------------------- SetSS -----------------------------------------*/

static BOOL SetSS (U32 ss) {
  /* Enable/Disable SPI Chip Select (drive it high or low). */

  GPIO5->DR[0x80<<2] = ss ? 0x80 : 0x00;
  return (__TRUE);
}


/*--------------------------- CheckMedia ------------------------------------*/

static U32 CheckMedia (void) {
  /* Read CardDetect and WriteProtect SD card socket pins. */
  U32 stat = 0;
 
#if 0
  if (GPIO3->DR[0x01<<2] == 0) {
    /* Card is inserted (CD=0). */
    stat |= M_INSERTED;
  } 
  if (GPIO3->DR[0x02<<2] != 0) {
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
