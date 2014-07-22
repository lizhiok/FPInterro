/*----------------------------------------------------------------------------
 *      RL-ARM - FlashFS
 *----------------------------------------------------------------------------
 *      Name:    SPI_EFM32_Gxxx.c
 *      Purpose: Serial Peripheral Interface Driver for EFM32 Gxxx device
 *      Rev.:    V4.20
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2011 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <File_Config.h>
#include <EFM32.H>
#include "efm32_cmu.h"
#include "efm32_gpio.h"

/*----------------------------------------------------------------------------
  SPI Driver instance definition
   spi0_drv: First SPI driver
   spi1_drv: Second SPI driver
 *---------------------------------------------------------------------------*/

#define __DRV_ID    spi0_drv

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


/*--------------------------- Init ------------------------------------------*/

static BOOL Init (void) {
  /* Initialize and enable the SSP Interface module. */
  USART_TypeDef *spi;

  /* Enabling clock to USART 0 */
  CMU_ClockEnable(cmuClock_USART0, true);
  CMU_ClockEnable(cmuClock_GPIO, true);

  /* Setup SPI at USART0#2 */
  spi = (USART_TypeDef *)USART0;

  /* Setting baudrate */
  spi->CLKDIV  =  4424;
  
  /* Configure SPI */
  /* Using synchronous (SPI) mode*/
  spi->CTRL = USART_CTRL_SYNC | USART_CTRL_MSBF;
  /* Clearing old transfers/receptions, and disabling interrupts */
  spi->CMD = USART_CMD_CLEARRX | USART_CMD_CLEARTX;
  spi->IEN = 0;
  /* Enabling pins and setting location, SPI CS not enable */
  spi->ROUTE = USART_ROUTE_TXPEN | USART_ROUTE_RXPEN | USART_ROUTE_CLKPEN | (2 << 8);
  /* Enabling TX and RX */
  spi->CMD = USART_CMD_TXEN | USART_CMD_RXEN;

  /* Set to master and without controlling the CS line */
  spi->CMD = USART_CMD_MASTEREN;

  /* Clear previous interrupts */
  spi->IFC = _USART_IFC_MASK;

  /* IO configuration (USART 0, Location #2) */
  GPIO_PinModeSet(gpioPortC, 11, gpioModePushPull, 0); /* MOSI */
  GPIO_PinModeSet(gpioPortC, 10, gpioModePushPull, 0); /* MISO */
  GPIO_PinModeSet(gpioPortC, 8,  gpioModePushPull, 0); /* CS */
  GPIO_PinModeSet(gpioPortC, 9,  gpioModePushPull, 0); /* Clock */

  return (__TRUE);
}


/*--------------------------- UnInit ----------------------------------------*/

static BOOL UnInit (void) {
  /* Return SSP interface to default state. */
  USART_TypeDef *spi;

  spi = (USART_TypeDef *)USART0;
  spi->CLKDIV  = 0;
  spi->CTRL    = 0;
  spi->CMD     = 0;
  spi->IEN     = 0;
  spi->ROUTE   = 0;

  CMU_ClockEnable(cmuClock_USART0, false);  

  GPIO_PinModeSet(gpioPortC, 11, gpioModeDisabled, 0);             /* MOSI  */
  GPIO_PinModeSet(gpioPortC, 10, gpioModeDisabled, 0);             /* MISO  */
  GPIO_PinModeSet(gpioPortC, 8,  gpioModeDisabled, 0);             /* CS    */
  GPIO_PinModeSet(gpioPortC, 9,  gpioModeDisabled, 0);             /* Clock */

  return (__TRUE);
}


/*--------------------------- Send ------------------------------------------*/

static U8 Send (U8 outb) {
  /* Write and Read a byte on SPI interface. */
  USART_TypeDef *spi = USART0;
  while (!(spi->STATUS & USART_STATUS_TXBL)) ;
  spi->CMD = USART_CMD_CLEARRX | USART_CMD_CLEARTX;

  spi->TXDATA = outb;
  /* Wait till Rx buffer has valid data. */
  while (!(spi->STATUS & USART_STATUS_RXDATAV));
  return ((U8)spi->RXDATA);
}


/*--------------------------- SendBuf ---------------------------------------*/

static BOOL SendBuf (U8 *buf, U32 sz) {
  /* Send buffer to SPI interface. */
  U32 i;
  USART_TypeDef *spi = USART0;

  for (i = 0; i < sz; i++) {
    while (!(spi->STATUS & USART_STATUS_TXBL)) ;
    spi->CMD = USART_CMD_CLEARRX | USART_CMD_CLEARTX;

    spi->TXDATA = buf[i];
    /* Wait till Tx buffer is empty. */
    while (!(spi->STATUS & USART_STATUS_TXBL));
    spi->RXDATA;
  }
  /* drain Rx FIFO. */
  while (spi->STATUS & USART_STATUS_RXDATAV) {
    spi->RXDATA;
  }
  return (__TRUE);
}


/*--------------------------- RecBuf ----------------------------------------*/

static BOOL RecBuf (U8 *buf, U32 sz) {
  /* Receive SPI data to buffer. */
  U32 i;
  USART_TypeDef *spi = USART0;

  for (i = 0; i < sz; i++) {
    while (!(spi->STATUS & USART_STATUS_TXBL)) ;
    spi->CMD = USART_CMD_CLEARRX | USART_CMD_CLEARTX;

    spi->TXDATA = 0xFF; 
    /* Wait till Rx buffer has valid data. */
    while (!(spi->STATUS & USART_STATUS_RXDATAV));
    buf[i] = spi->RXDATA;
  }
  return (__TRUE);
}


/*--------------------------- BusSpeed --------------------------------------*/

static BOOL BusSpeed (U32 kbaud) {
  /* Set an SPI clock to required baud rate. */
  U32  hfperclk,clkdiv;

  /* br = hfperclk / (2 x (1 + clkdiv/256)) */
  hfperclk = SystemHFClockGet() / 1000;
  clkdiv   = ((hfperclk / 2) + kbaud - 1) / kbaud;
  if (clkdiv != 0)     clkdiv--;
  if (clkdiv > 0x1FFF) clkdiv = 0x1FFF;
  USART0->CLKDIV = clkdiv << 8;
  return (__TRUE);
}


/*--------------------------- SetSS -----------------------------------------*/

static BOOL SetSS (U32 ss) {
  /* Enable/Disable SPI Chip Select (drive it high or low). */

  if (ss) {
    /* SPI CS set to high.  */
    GPIO->P[2].DOUTSET = 0x100;  }
  else {
    /* SPI CS set to low.   */
    GPIO->P[2].DOUTCLR = 0x100;  }
  return (__TRUE);
}


/*--------------------------- CheckMedia ------------------------------------*/

static U32 CheckMedia (void) {
  /* Read CardDetect and WriteProtect SD card socket pins. */
  U32 stat = 0;
 
  if (!(GPIO_PinOutGet(gpioPortA, 1))) {
    /* Card is inserted (CD=0). */
    stat |= M_INSERTED;
  } 
  if (GPIO_PinOutGet(gpioPortA, 2)) {
    /* Write Protect switch is active (WP=1). */
    stat |= M_PROTECTED;
  }
  return (stat);
}

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
