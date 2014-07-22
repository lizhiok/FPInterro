/*----------------------------------------------------------------------------
 *      RL-ARM - FlashFS
 *----------------------------------------------------------------------------
 *      Name:    SPI_STR75x.c
 *      Purpose: Serial Peripheral Interface Driver for ST STR75x
 *      Rev.:    V4.20
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2011 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <File_Config.h>
#include <75x_lib.h>                  /* STR750 Library definitions          */

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

  /* SCLK0, MISO0, MOSI0 are SSP pins. */
  GPIO0->PC0 |=  0x000000F0;
  GPIO0->PC1 &= ~0x00000010;
  GPIO0->PC1 |=  0x000000E0;
  GPIO0->PC2 |=  0x000000F0;
  /* NSS is GPIO, output set to high. */
  GPIO0->PD  |= 0x00000010;

  /* Enable SPI in Master Mode, CPOL=0, CPHA=0. */
  SSP0->CR0 = 0x0007;
  SSP0->CR1 = 0x0002;
  SSP0->PR  = 0xFE;
  return (__TRUE);
}


/*--------------------------- UnInit ----------------------------------------*/

static BOOL UnInit (void) {
  /* Reset SSP interface to default state. */

  GPIO0->PC0 |=  0x000000F0;
  GPIO0->PC1 &= ~0x000000F0;
  GPIO0->PC2 &= ~0x000000F0;
  SSP0->CR0 = 0x0000;
  SSP0->CR1 = 0x0000;
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

  if (ss) {
    /* NSS is GPIO, output set to high. */
    GPIO0->PD |=  0x00000010;
  } 
  else {
    /* NSS is GPIO, output set to low. */
    GPIO0->PD &= ~0x00000010;
  }
  return (__TRUE);
}


/*--------------------------- CheckMedia ------------------------------------*/

static U32 CheckMedia (void) {
  /* Read CardDetect and WriteProtect SD card socket pins. */
  U32 stat = 0;
 
#if 0
  if ((GPIO1->PD & 0x01) == 0) {
    /* Card is inserted (CD=0). */
    stat |= M_INSERTED;
  } 
  if ((GPIO1->PD & 0x02) != 0) {
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
