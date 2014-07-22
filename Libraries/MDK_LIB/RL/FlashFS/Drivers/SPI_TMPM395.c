/*----------------------------------------------------------------------------
 *      RL-ARM - FlashFS
 *----------------------------------------------------------------------------
 *      Name:    SPI_TMPM395.c
 *      Purpose: Serial Peripheral Interface Driver for Toshiba TMPM395
 *      Rev.:    V4.20
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2011 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <File_Config.h>
#include "TMPM395.h"                          /* TMPM395 definitions         */

/*----------------------------------------------------------------------------
  SPI Driver instance definition
   spi0_drv: First SPI driver
   spi1_drv: Second SPI driver
 *---------------------------------------------------------------------------*/

#define __DRV_ID  spi0_drv
#define __PCLK    20000000

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

/* SSPxSR - bit definitions. */
#define TFE     0x01
#define TNF     0x02
#define RNE     0x04
#define RFF     0x08
#define BSY     0x10

/* SSP2 Pins */
#define SSP2_MISO       (1UL << 3)            /* PM.3 = MISO                 */
#define SSP2_MOSI       (1UL << 2)            /* PM.2 = MOSI                 */
#define SSP2_CLK        (1UL << 1)            /* PM.1 = CLK                  */
#define SSP2_NSS        (1UL << 0)            /* PM.0 = NSS                  */
#define SSP2_PINS       (SSP2_MISO | SSP2_MOSI | SSP2_CLK)


/*--------------------------- Init ------------------------------------------*/

static BOOL Init (void) {
  /* Initialize and enable the SSP Interface module. */

  /* SSEL is GPIO, output set to high. */
  TSB_PM->CR     |=  SSP2_NSS;                /* PM.0 is output              */
  TSB_PM->IE     &= ~SSP2_NSS;
  TSB_PM->DATA   |=  SSP2_NSS;                /* set PM.0 high (SSEL inact.) */
  TSB_PM->FR1    &= ~SSP2_NSS;                /* PM.0 SSEL (used as GPIO)    */

  /* SCK, MISO, MOSI are SSP pins. */
  TSB_PM->IE     |=  SSP2_MISO;               /* Enable  Input  PM.3         */
  TSB_PM->CR     &= ~SSP2_MISO;               /* Disable Output PM.3         */
  TSB_PM->IE     &= ~(SSP2_MOSI | SSP2_CLK);  /* Disable Input  PM.1..2      */
  TSB_PM->CR     |=  (SSP2_MOSI | SSP2_CLK);  /* Enable  Output PM.1..2      */
  TSB_PM->FR1    |=  SSP2_PINS;               /* PM.1..3 CLK, MOSI, MISO     */

  TSB_SSP2->CPSR  = 50;                       /* 20MHz / 50 = 400kBit        */
  TSB_SSP2->CR0   = 0x0007;                   /* 8Bit, CPOL=0, CPHA=0        */
  TSB_SSP2->CR1   = 0x0002;                   /* SSP0 enable, master         */

  return (__TRUE);
}


/*--------------------------- UnInit ----------------------------------------*/

static BOOL UnInit (void) {
  /* Return SSP interface to default state. */

  TSB_PM->CR     &= ~SSP2_NSS;
  TSB_PM->IE     &= ~SSP2_NSS;
  TSB_PM->DATA   &= ~SSP2_NSS;
  TSB_PM->FR1    &= ~SSP2_NSS;

  TSB_PM->IE     &= ~ SSP2_MISO;              
  TSB_PM->CR     &= ~SSP2_MISO;              
  TSB_PM->IE     &= ~(SSP2_MOSI | SSP2_CLK); 
  TSB_PM->CR     &= ~(SSP2_MOSI | SSP2_CLK); 
  TSB_PM->FR1    &= ~SSP2_PINS; 
  
  TSB_SSP2->CPSR  = 0;     
  TSB_SSP2->CR0   = 0; 
  TSB_SSP2->CR1   = 0;  

  return (__TRUE);
}


/*--------------------------- Send ------------------------------------------*/

static U8 Send (U8 outb) {
  /* Write and Read a byte on SPI interface. */

  TSB_SSP2->DR = outb;
  while (TSB_SSP2->SR & BSY);                 /* Wait for transfer to finish */
  return (TSB_SSP2->DR);                      /* Return received value       */
}


/*--------------------------- SendBuf ---------------------------------------*/

static BOOL SendBuf (U8 *buf, U32 sz) {
  /* Send buffer to SPI interface. */
  U32 i;

  for (i = 0; i < sz; i++) {
    TSB_SSP2->DR = buf[i];
    /* Wait if Tx FIFO is full. */
    while (!(TSB_SSP2->SR & TNF));
	TSB_SSP2->DR;
  }
  /* Wait until Tx finished, drain Rx FIFO. */
  while (TSB_SSP2->SR & (BSY | RNE)) {
    TSB_SSP2->DR;
  }		   
  return (__TRUE);
}


/*--------------------------- RecBuf ----------------------------------------*/

static BOOL RecBuf (U8 *buf, U32 sz) {
  /* Receive SPI data to buffer. */
  U32 i;

  for (i = 0; i < sz; i++) {
    TSB_SSP2->DR = 0xFF; 
    /* Wait while Rx FIFO is empty. */
    while (!(TSB_SSP2->SR & RNE));
    buf[i] = TSB_SSP2->DR;
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
  TSB_SSP2->CPSR = div;
  return (__TRUE);
}


/*--------------------------- SetSS -----------------------------------------*/

static BOOL SetSS (U32 ss) {
  /* Enable/Disable SPI Chip Select (drive it high or low). */

  if (ss) {
    /* SSEL is GPIO, set to high.  */
    TSB_PM->DATA |=  SSP2_NSS;                
  }
  else {
    /* SSEL is GPIO, set to low.   */
    TSB_PM->DATA &= ~SSP2_NSS;                
  }
  return (__TRUE);
}


/*--------------------------- CheckMedia ------------------------------------*/

static U32 CheckMedia (void) {
  /* Read CardDetect and WriteProtect SD card socket pins. */
  U32 stat = 0;
 
#if 0
  if (!(TSB_PM->DATA & (1 << 4))) {
    /* Card is inserted (CD=0). */
    stat |= M_INSERTED;
  } 
  if ((TSB_PM->DATA & (1 << 5))) {
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
