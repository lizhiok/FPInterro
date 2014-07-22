/*----------------------------------------------------------------------------
 *      RL-ARM - FlashFS
 *----------------------------------------------------------------------------
 *      Name:    SPI_LM3S37x8.c
 *      Purpose: Serial Peripheral Interface Driver for Luminary LM3S37x8
 *      Rev.:    V4.20
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2011 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <File_Config.h>
#include <LM3Sxxxx.H>

/*----------------------------------------------------------------------------
  SPI Driver instance definition
   spi0_drv: First SPI driver
   spi1_drv: Second SPI driver
 *---------------------------------------------------------------------------*/

#define __DRV_ID  spi0_drv

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

#ifdef SSI1
 #define SSIx_PERI      SYSCTL_PERIPH_SSI1
 #define SSIx_BASE      SSI1_BASE
#else
 #define SSIx_PERI      SYSCTL_PERIPH_SSI0
 #define SSIx_BASE      SSI0_BASE
#endif

/* Peripheral definitions for EK-LM3S37x8 board */
#define SSI_PORT_PERI   SYSCTL_PERIPH_GPIOA
#define CS_PORT_PERI    SYSCTL_PERIPH_GPIOA
#define SSI_PORT        GPIO_PORTA_BASE
#define CS_PORT         GPIO_PORTA_BASE

#define SSI_CS          GPIO_PIN_3
#define SSI_CLK         GPIO_PIN_2
#define SSI_RX          GPIO_PIN_4
#define SSI_TX          GPIO_PIN_5
#define SSI_PINS        (SSI_TX | SSI_RX | SSI_CLK)


/*--------------------------- Init ------------------------------------------*/

static BOOL Init (void) {
  /* Initialize and enable the SSP Interface module. */

  /* Enable the SSI peripherals. */
  SysCtlPeripheralEnable(SSIx_PERI);
  SysCtlPeripheralEnable(SSI_PORT_PERI);
  SysCtlPeripheralEnable(CS_PORT_PERI);

  /* Configure the appropriate pins to be SSI instead of GPIO */
  GPIODirModeSet(SSI_PORT, SSI_PINS, GPIO_DIR_MODE_HW);
  GPIODirModeSet(CS_PORT,  SSI_CS,   GPIO_DIR_MODE_OUT);
  GPIOPadConfigSet(SSI_PORT, SSI_PINS, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPU);
  GPIOPadConfigSet(CS_PORT,  SSI_CS,   GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPU);

  /* Deassert the SSIx chip select */
  GPIOPinWrite(CS_PORT, SSI_CS, SSI_CS);

  /* Configure the SSIx port */
  SSIConfig(SSIx_BASE, SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 400000, 8);
  SSIEnable(SSIx_BASE);

  return (__TRUE);
}


/*--------------------------- UnInit ----------------------------------------*/

static BOOL UnInit (void) {
  /* Return SSP interface to default state. */

  SysCtlPeripheralDisable(SSIx_PERI);
  SysCtlPeripheralDisable(SSI_PORT_PERI);
  SysCtlPeripheralDisable(CS_PORT_PERI);

  GPIOPinWrite(CS_PORT, SSI_CS, !SSI_CS);

  GPIODirModeSet(SSI_PORT, SSI_PINS, GPIO_DIR_MODE_IN);
  GPIODirModeSet(CS_PORT,  SSI_CS,   GPIO_DIR_MODE_IN);
  GPIOPadConfigSet(SSI_PORT, SSI_PINS, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
  GPIOPadConfigSet(CS_PORT,  SSI_CS,   GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);

  SSIDisable(SSIx_BASE);
  HWREG(SSIx_BASE + SSI_O_CR0)   = 0;
  HWREG(SSIx_BASE + SSI_O_CR1)   = 0;
  HWREG(SSIx_BASE + SSI_O_CPSR)  = 0;

  return (__TRUE);
}

/*--------------------------- Send ------------------------------------------*/

static U8 Send (U8 outb) {
  /* Write and Read a byte on SPI interface. */

  HWREG(SSI0_BASE + SSI_O_DR) = outb;
  while(!(HWREG(SSIx_BASE + SSI_O_SR) & SSI_SR_RNE));
  return (HWREG(SSIx_BASE + SSI_O_DR));
}


/*--------------------------- SendBuf ---------------------------------------*/

static BOOL SendBuf (U8 *buf, U32 sz) {
  /* Send buffer to SPI interface. */
  U32 i;

  for (i = 0; i < sz; i++) {
    HWREG(SSI0_BASE + SSI_O_DR) = buf[i];
    /* Wait if Tx FIFO is full. */
    while(!(HWREG(SSIx_BASE + SSI_O_SR) & SSI_SR_TNF));
    HWREG(SSIx_BASE + SSI_O_DR);
  }
  /* Wait until Tx finished, drain Rx FIFO. */
  while (HWREG(SSIx_BASE + SSI_O_SR) & (SSI_SR_BSY | SSI_SR_RNE)) {
    HWREG(SSIx_BASE + SSI_O_DR);
  }
  return (__TRUE);
}


/*--------------------------- RecBuf ----------------------------------------*/

static BOOL RecBuf (U8 *buf, U32 sz) {
  /* Receive SPI data to buffer. */
  U32 i;

  for (i = 0; i < sz; i++) {
    HWREG(SSI0_BASE + SSI_O_DR) = 0xFF; 
    /* Wait while Rx FIFO is empty. */
    while (!(HWREG(SSIx_BASE + SSI_O_SR) & SSI_SR_RNE));
    buf[i] = HWREG(SSIx_BASE + SSI_O_DR);
  }
  return (__TRUE);
}

/*--------------------------- BusSpeed --------------------------------------*/

static BOOL BusSpeed (U32 kbaud) {
  /* Set an SPI clock to required baud rate. */
  U32 baud = kbaud * 1000;

  if (baud > 12500000) {
    /* Maximum allowed clock is 12.5MHz. */
    baud = 12500000;
  }
  SSIDisable(SSIx_BASE);
  SSIConfig (SSIx_BASE, SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, baud, 8);
  SSIEnable (SSIx_BASE);
  return (__TRUE);
}


/*--------------------------- SetSS -----------------------------------------*/

static BOOL SetSS (U32 ss) {
  /* Enable/Disable SPI Chip Select (drive it high or low). */
  
  HWREG(CS_PORT + (GPIO_O_DATA + (SSI_CS << 2))) = ss ? SSI_CS : 0;

  return (__TRUE);
}


/*--------------------------- CheckMedia ------------------------------------*/

static U32 CheckMedia (void) {
  /* Read CardDetect and WriteProtect SD card socket pins. */
  U32 stat = 0;
 
#if 0 
  if (!(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_7))) {
    /* Card is inserted (CD=0). */
    stat |= M_INSERTED;
  } 
  if (GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_1)) {
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
