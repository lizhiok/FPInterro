/*----------------------------------------------------------------------------
 *      RL-ARM - FlashFS
 *----------------------------------------------------------------------------
 *      Name:    SPI_SAM7X.c
 *      Purpose: Serial Peripheral Interface Driver for Atmel AT91SAM7X
 *      Rev.:    V4.20
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2011 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <File_Config.h>
#include <AT91SAM7X256.H>               /* AT91SAM7X256 definitions          */

/*----------------------------------------------------------------------------
  SPI Driver instance definition
   spi0_drv: First SPI driver
   spi1_drv: Second SPI driver
 *---------------------------------------------------------------------------*/

#define __DRV_ID  spi0_drv
#define __MCK     46080000

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


/*----------------------------------------------------------------------------
 *      User configuration part
 *---------------------------------------------------------------------------*/

//-------- <<< Use Configuration Wizard in Context Menu >>> -----------------

//<h>SPI Configuration for SAM7X
//  <o>Configured Drive  <0=> SPI Flash <1=> Memory Card
//  <i>SPI functions are used for this drive
//</h>
#define CFG_DRIVE  1

//------------- <<< end of configuration section >>> -----------------------

#if CFG_DRIVE == 0
 #define SPI_CS    0
 #define SPI_SS    AT91C_PA12_SPI0_NPCS0         /* SPI DATA Flash       */
#else
 #define SPI_CS    1
 #define SPI_SS    AT91C_PA13_SPI0_NPCS1         /* SPI SD Card          */
#endif
#define SPI_PCS    (~(1<<SPI_CS) & 0x0F)         /* set PCS according CS */


/*--------------------------- Init ------------------------------------------*/

static BOOL Init (void) {
  /* Initialize and enable the SSP Interface module. */

  /* Enable Clocks */
  AT91C_BASE_PMC->PMC_PCER        = (1 << AT91C_ID_PIOA) |
                                    (1 << AT91C_ID_SPI0) ;

  /* SPI0_MISO, SPI0_MOSI, SPI0_SPCK are SPI0 pins */
  AT91C_BASE_PIOA->PIO_ASR        = 
  AT91C_BASE_PIOA->PIO_PDR        = AT91C_PA16_SPI0_MISO |
                                    AT91C_PA17_SPI0_MOSI |
                                    AT91C_PA18_SPI0_SPCK ;

  /* SPIO_NPCS1 is GPIO, output set to high. */
  AT91C_BASE_PIOA->PIO_PER        = 
  AT91C_BASE_PIOA->PIO_OER        = 
  AT91C_BASE_PIOA->PIO_SODR       = SPI_SS;

  /* Initialize SPI0 Controller */
  AT91C_BASE_SPI0->SPI_CR         = AT91C_SPI_SWRST;
  AT91C_BASE_SPI0->SPI_MR         = AT91C_SPI_MSTR | AT91C_SPI_MODFDIS;

  /* Set DLYBCT = 0(10ns), DLYBS = 15(300ns), SBCR = 3(16MHz SPI clock) */
  AT91C_BASE_SPI0->SPI_CSR[SPI_CS] = AT91C_SPI_CPOL | AT91C_SPI_BITS_8 |
                                     (0 << 24) | (15 << 16) | (3 << 8) ;
  /* Use NPCSx as chip select. */
  AT91C_BASE_SPI0->SPI_MR        &= 0xFFF0FFFF;
  AT91C_BASE_SPI0->SPI_MR        |= ((SPI_PCS<<16) & AT91C_SPI_PCS);
  AT91C_BASE_SPI0->SPI_CR         = AT91C_SPI_SPIEN;

  return (__TRUE);
}


/*--------------------------- UnInit ----------------------------------------*/

static BOOL UnInit (void) {
  /* Return SSP interface to default state. */

  AT91C_BASE_PIOA->PIO_PER         = AT91C_PA16_SPI0_MISO |
                                     AT91C_PA17_SPI0_MOSI |
                                     AT91C_PA18_SPI0_SPCK ;

  AT91C_BASE_PIOA->PIO_PER         =
  AT91C_BASE_PIOA->PIO_ODR         =
  AT91C_BASE_PIOA->PIO_CODR        = SPI_SS;

  AT91C_BASE_SPI0->SPI_CR          = 0;
  AT91C_BASE_SPI0->SPI_MR          = 0;
  AT91C_BASE_SPI0->SPI_CSR[SPI_CS] = 0;

  return (__TRUE);
}


/*--------------------------- Send ------------------------------------------*/

static U8 Send (U8 outb) {
  /* Write and Read a byte on SPI interface. */

  /* Wait until TDRE is set. */
  AT91C_BASE_SPI0->SPI_TDR = outb;

  /* Wait until RDRF is set. */
  while (!(AT91C_BASE_SPI0->SPI_SR & AT91C_SPI_RDRF));
  return ((U8)AT91C_BASE_SPI0->SPI_RDR);
}


/*--------------------------- SendBuf ---------------------------------------*/

static BOOL SendBuf (U8 *buf, U32 sz) {
  /* Send buffer to SPI interface. */
  U32 i;

  for (i = 0; i < sz; i++) {
    AT91C_BASE_SPI0->SPI_TDR = buf[i];
    /* Wait if Tx FIFO is full. */
    while (!(AT91C_BASE_SPI0->SPI_SR & AT91C_SPI_TDRE));
    AT91C_BASE_SPI0->SPI_RDR;
  }
  while (!(AT91C_BASE_SPI0->SPI_SR & AT91C_SPI_RDRF));
  AT91C_BASE_SPI0->SPI_RDR;
  return (__TRUE);
}


/*--------------------------- RecBuf ----------------------------------------*/

static BOOL RecBuf (U8 *buf, U32 sz) {
  /* Receive SPI data to buffer. */
  U32 i;

  for (i = 0; i < sz; i++) {
    AT91C_BASE_SPI0->SPI_TDR = 0xFF; 
    /* Wait while Rx FIFO is empty. */
    while (!(AT91C_BASE_SPI0->SPI_SR & AT91C_SPI_RDRF));
    buf[i] = (U8)AT91C_BASE_SPI0->SPI_RDR;
  }
  return (__TRUE);
}


/*--------------------------- BusSpeed --------------------------------------*/

static BOOL BusSpeed (U32 kbaud) {
  /* Set an SPI clock to required baud rate. */	  
  U32 scbr;

  scbr = (__MCK/1000 + kbaud - 1) / kbaud;
  if (scbr == 0)  scbr = 1;
  if (scbr > 255) scbr = 255;
  AT91C_BASE_SPI0->SPI_CSR[SPI_CS] = (AT91C_BASE_SPI0->SPI_CSR[SPI_CS] & ~0xFF00) |
                                     (scbr << 8);
  return (__TRUE);
}


/*--------------------------- SetSS -----------------------------------------*/

static BOOL SetSS (U32 ss) {
  /* Enable/Disable SPI Chip Select (drive it high or low). */

  if (ss) {
    AT91C_BASE_PIOA->PIO_SODR = SPI_SS;
  }
  else {
    AT91C_BASE_PIOA->PIO_CODR = SPI_SS;
  }
  return (__TRUE);
}


/*--------------------------- CheckMedia ------------------------------------*/

static U32 CheckMedia (void) {
  /* Read CardDetect and WriteProtect SD card socket pins. */
  U32 stat = 0;
 
#if 0 
  if (!(AT91C_BASE_PIOA->PIO_ODSR & (1 << 29))) {
    /* Card is inserted (CD=0). */
    stat |= M_INSERTED;
  } 
  if ((AT91C_BASE_PIOA->PIO_ODSR & (1 << 28))) {
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
