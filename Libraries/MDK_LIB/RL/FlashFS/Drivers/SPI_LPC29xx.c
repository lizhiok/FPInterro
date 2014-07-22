/*----------------------------------------------------------------------------
 *      RL-ARM - FlashFS
 *----------------------------------------------------------------------------
 *      Name:    SPI_LPC29xx.c
 *      Purpose: Serial Peripheral Interface Driver for NXP LPC29xx
 *      Rev.:    V4.20
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2011 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <File_Config.h>
#include <LPC29xx.H>                 /* LPC29xx Definitions                  */

/*----------------------------------------------------------------------------
  SPI Driver instance definition
   spi0_drv: First SPI driver
   spi1_drv: Second SPI driver
 *---------------------------------------------------------------------------*/

#define __DRV_ID  spi0_drv
#define __CLKSPI  32000000

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
#define TFF     0x02
#define RFE     0x04
#define RFF     0x08
#define BSY     0x10

/*--------------------------- Init ------------------------------------------*/

static BOOL Init (void) {
  /* Initialize and enable the SSP Interface module. */

  /* SPI Clock Configuration */
  /* Configure SPI = FDIV1(CGU0) / 1 = 32 MHz */
  SPI_CLK_CONF = (6UL << 24) | (1 << 11) | (3 << 2);

  /* Initialize and enable the SSP Interface module. */
  /* P2.24 SPI chip Select, GPIO Output */
  SFSP2_24     = (0x03  << 2);
  GPIO2_DR    |= (0x01 << 24);           
	
  /* P3.13~P3.15 SPI SDO,SDI,SCK configured as SPI1 */
  SFSP3_13  = 0x01 | (0x03 << 2);
  SFSP3_14  = 0x01 | (0x03 << 2);
  SFSP3_15  = 0x01;

  /* P2.25 Card plugged, GPIO Input */
  /* 1 = NO Card, 0 = Card plugged. */
  SFSP2_25  =  (0x03 <<  2);
  GPIO2_DR &= ~(0x01 << 25);           
	
  SPI1_CONFIG &= ~(0x01 << 1);  /* Master Mode */
  
  SPI1_SLV_ENABLE = 0x01;
  SPI1_SLV0_SET1  = 0x200;
  SPI1_SLV0_SET2  = 0x007;

  SPI1_CONFIG |= 0x01;			    /* SPI Enable  */
  return (__TRUE);
}


/*--------------------------- UnInit ----------------------------------------*/

static BOOL UnInit (void) {
  /* Return SSP interface to default state. */

  SFSP2_24 = 0x00;
  SFSP2_25 = 0x00;
  SFSP3_13 = 0x00;
  SFSP3_14 = 0x00;
  SFSP3_15 = 0x00;

  SPI1_CONFIG     = 0x00010000;
  SPI1_SLV_ENABLE = 0x00;
  SPI1_SLV0_SET1  = 0x200;
  SPI1_SLV0_SET2  = 0x000;
  return (__TRUE);
}


/*--------------------------- Send ------------------------------------------*/

static U8 Send (U8 outb) {
  /* Send and Receive a byte on SPI interface. */

  SPI1_FIFO_DATA     = outb;
  SPI1_TX_FIFO_FLUSH = 0x00000001;

  /* Wait if BSY cleared. */
  while (SPI1_STAT & BSY);
  return (SPI1_FIFO_DATA);
}


/*--------------------------- SendBuf ---------------------------------------*/

static BOOL SendBuf (U8 *buf, U32 sz) {
  /* Send buffer to SPI interface. */
  U32 i;

  for (i = 0; i < sz; i++) {
    SPI1_FIFO_DATA = buf[i];
    /* Wait if Tx FIFO is full. */
    while (SPI1_STAT & TFF);
    SPI1_FIFO_DATA;
  }
  /* Wait until Tx finished, drain Rx FIFO. */
  while (SPI1_STAT & BSY) {
    SPI1_FIFO_DATA;
  }
  return (__TRUE);
}



/*--------------------------- RecBuf ----------------------------------------*/

static BOOL RecBuf (U8 *buf, U32 sz) {
  /* Receive SPI data to buffer. */
  U32 i;

  for (i = 0; i < sz; i++) {
    SPI1_FIFO_DATA = 0xFF;
    /* Wait while Rx FIFO is empty. */
    while (SPI1_STAT & RFE);
    buf[i] = SPI1_FIFO_DATA;
  }
  return (__TRUE);
}


/*--------------------------- BusSpeed --------------------------------------*/

static BOOL BusSpeed (U32 kbaud) {
  /* Set an SPI clock to required baud rate. */
  U32 div1;

  div1 = (__CLKSPI/2000 + kbaud - 1) / kbaud;
  if (div1 > 0)    div1--;
  if (div1 > 0xFF) div1 = 0xFF;
  SPI1_SLV0_SET1 = 0x200 | div1;
  return (__TRUE);
}


/*--------------------------- SetSS -----------------------------------------*/

static BOOL SetSS (U32 ss) {
  /* Enable/Disable SPI Chip Select (drive it high or low). */

  if (ss) {
    /* SSEL is GPIO, output set to high. */
    GPIO2_OR |=  (1 << 24);
  }
  else {
    /* SSEL is GPIO, output set to low. */
    GPIO2_OR &= ~(1 << 24);
  }
  return (__TRUE);
}


/*--------------------------- CheckMedia ------------------------------------*/

static U32 CheckMedia (void) {
  /* Read CardDetect and WriteProtect SD card socket pins. */
  U32 stat = 0;

  if (!(GPIO2_PINS & (1 << 25))) {
    /* Card is inserted (CD=0). */
    stat |= M_INSERTED;
  } 
  /* Write Protect pin does not exist on micro-SD socket. */
  return (stat);
}

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
