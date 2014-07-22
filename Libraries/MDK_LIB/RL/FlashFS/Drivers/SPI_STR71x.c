/*----------------------------------------------------------------------------
 *      RL-ARM - FlashFS
 *----------------------------------------------------------------------------
 *      Name:    SPI_STR71x.c
 *      Purpose: Serial Peripheral Interface Driver for ST STR71x
 *      Rev.:    V4.20
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2011 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <File_Config.h>
#include <71x_lib.h>                  /* STR710 definitions                  */

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


/* BSPI_CSR2 - bit definitions. */
#define TFNE    0x0200
#define TFF     0x0100
#define TUFL    0x0080
#define TFE     0x0040
#define ROFL    0x0020
#define RFF     0x0010
#define RFNE    0x0008
#define BERR    0x0004

/*--------------------------- Init ------------------------------------------*/

static BOOL Init (void) {
  /* Initialize and enable the SPI Interface module. */

  /* MISO1, MOSI1, SCLK1 are SPI pins */
  GPIO0->PC0 |=  0x0070;
  GPIO0->PC1 |=  0x0070;
  GPIO0->PC2 |=  0x0070;
  /* SPI SS is GPIO, output set to high */
  GPIO0->PD  |=  0x0080;
  GPIO0->PC0 |=  0x0080;
  GPIO0->PC1 &= ~0x0080;
  GPIO0->PC2 |=  0x0080;

  /* Enable SPI in Master Mode, CPOL=0, CPHA=0. */
  BSPI1->CLK  =  0xFE;
  BSPI1->CSR1 =  0x0003;
  BSPI1->CSR2 =  0x0001;
  return (__TRUE);
}


/*--------------------------- UnInit ----------------------------------------*/

static BOOL UnInit (void) {
  /* Reset SPI interface to default state. */

  GPIO0->PC0 |=  0x00F0;
  GPIO0->PC1 |=  0x00F0;
  GPIO0->PC2 &= ~0x00F0;
  BSPI1->CLK  =  0x00;
  BSPI1->CSR1 =  0x0000;
  BSPI1->CSR2 =  0x0000;
  return (__TRUE);
}


/*--------------------------- Send ------------------------------------------*/

static U8 Send (U8 outb) {
  /* Send and Receive a byte on SPI interface. */

  BSPI1->TXR = outb << 8;
  /* Wait if RFNE cleared, Rx FIFO is empty */
  while (!(BSPI1->CSR2 & RFNE));
  return (BSPI1->RXR >> 8);
}


/*--------------------------- SendBuf ---------------------------------------*/

static BOOL SendBuf (U8 *buf, U32 sz) {
  /* Send buffer to SPI interface. */
  U32 i;

  for (i = 0; i < sz; i++) {
    BSPI1->TXR = buf[i] << 8;
    /* Wait if Tx FIFO is full. */
    while (BSPI1->CSR2 & TFF);
    BSPI1->RXR;
  }
  /* Wait until Tx finished, drain Rx FIFO. */
  while (BSPI1->CSR2 & (TFNE | RFNE)) {
    BSPI1->RXR;
  }
  return (__TRUE);
}


/*--------------------------- RecBuf ----------------------------------------*/

static BOOL RecBuf (U8 *buf, U32 sz) {
  /* Receive SPI data to buffer. */
  U32 i;

  for (i = 0; i < sz; i++) {
    BSPI1->TXR = 0xFF << 8;
    /* Wait while Rx FIFO is empty. */
    while (!(BSPI1->CSR2 & RFNE));
    buf[i] = BSPI1->RXR >> 8;
  }
  return (__TRUE);
}


/*--------------------------- BusSpeed --------------------------------------*/

static BOOL BusSpeed (U32 kbaud) {
  /* Set an SPI clock to required baud rate. */
  U32 div;

  div = (__PCLK/1000 + kbaud - 1) / kbaud;
  if (div & 1)    div++;
  if (div < 0x06) div = 0x06;
  if (div > 0xFE) div = 0xFE;
  BSPI1->CLK = div;
  return (__TRUE);
}


/*--------------------------- SetSS -----------------------------------------*/

static BOOL SetSS (U32 ss) {
  /* Enable/Disable SPI Chip Select (drive it high or low). */

  if (ss) {
    /* NSS is GPIO, output set to high. */
    GPIO0->PD |=  0x0080;
  }
  else {
    /* NSS is GPIO, output set to low. */
    GPIO0->PD &= ~0x0080;
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
