/*----------------------------------------------------------------------------
 *      RL-ARM - FlashFS
 *----------------------------------------------------------------------------
 *      Name:    SDIO_STM32F103.c
 *      Purpose: SD/SDIO MMC Interface Driver for ST STM32F103
 *      Rev.:    V4.51
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2011 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <File_Config.h>
#include <STM32f10x.h>               /* STM32F10x Definitions                */
#include "SDIO_STM32F103.h"

/*----------------------------------------------------------------------------
  Memory Card FAT Driver instance definition
   mci0_drv: First SD/MMC drive [M0:]
   mci1_drv: Second SD/MMC drive [M1:]
 *---------------------------------------------------------------------------*/

#define __DRV_ID  mci0_drv
#define __SDIOCLK 72000000
#define __CPUCLK  72000000

/* MCI Driver Interface functions */
static BOOL Init (void);
static BOOL UnInit (void);
static void Delay (U32 us);
static BOOL BusMode (U32 mode);
static BOOL BusWidth (U32 width);
static BOOL BusSpeed (U32 kbaud);
static BOOL Command (U8 cmd, U32 arg, U32 resp_type, U32 *rp);
static BOOL ReadBlock (U32 bl, U8 *buf, U32 cnt);
static BOOL WriteBlock (U32 bl, U8 *buf, U32 cnt);
static U32  CheckMedia (void);        /* Optional function for SD card check */

/* MCI Device Driver Control Block */
MCI_DRV __DRV_ID = {
  Init,
  UnInit,
  Delay,
  BusMode,
  BusWidth,
  BusSpeed,
  Command,
  ReadBlock,
  WriteBlock,
  NULL,
  CheckMedia                          /* Can be NULL if not existing         */
};


/* Wait time in for loop cycles */
#define DMA_TOUT  10000000

/* Local Function Prototypes */
static void DmaStart (U32 mode, U8 *buf, U32 cnt);

/*--------------------------- Init ------------------------------------------*/

static BOOL Init (void) {
  /* Initialize SDIO interface. */

  /* Power Up the SDIO and DMA controller. */
  RCC->AHBENR  |= 0x00000402;          /* AHB clock enable SDIO, DMA2       */

  /* Configure SDIO Pins */
  RCC->APB2ENR |= 0x000000B0;          /* APB2 clock ena. GPIOC,GPIOD,GPIOF */

  /* PC.08, PC.09, PC.10, PC.11, PC.12 pin: D0, D1, D2, D3, CLK pin */ 
  GPIOC->CRH  &= ~0x000FFFFF;
  GPIOC->CRH  |=  0x000BBBBB;          /* Alternate Function PushPull 50MHz */

  /* Configure PD.02 CMD line */
  GPIOD->CRL  &= ~0x00000F00;
  GPIOD->CRL  |=  0x00000B00;          /* Alternate Function PushPull 50MHz */

  /* Configure PF.11 Card Detect input */
  GPIOF->CRH  &= ~0x0000F000;
  GPIOF->CRH  |=  0x00008000;          /* Input with pull-up/pull-down      */
  GPIOF->ODR  |=  0x00000800;          /* Enable pull up                    */

  /* Clear all pending interrupts. */
  SDIO->CMD    = 0;
  SDIO->DCTRL  = 0;
  SDIO->ICR    = 0x00C007FF;

  /* Power up, switch on VCC for the Flash Card. */
  SDIO->POWER  = 0x03;

  /* Success, SDIO initialized. */
  return (__TRUE);
}


/*--------------------------- UnInit ----------------------------------------*/

static BOOL UnInit (void) {
  /* Reset the SDIO peripheral to default state. */

  /* Power down, switch off VCC for the Flash Card. */
  SDIO->POWER  = 0x00;

  /* Clear all pending interrupts. */
  SDIO->CMD    = 0;
  SDIO->DCTRL  = 0;
  SDIO->ICR    = 0x00C007FF;

  /* Disable MCI pins. */
  GPIOC->CRH  &= ~0x000FFFFF;
  GPIOC->CRH  |=  0x00044444;
  GPIOD->CRL  &= ~0x00000F00;
  GPIOD->CRL  |=  0x00000400;

  /* Power Down the SDIO controller. */
  RCC->AHBENR &= ~0x00000400;

  return (__TRUE);
}


/*--------------------------- Delay -----------------------------------------*/

static void Delay (U32 us) {
  /* Approximate delay in micro seconds. */
  U32 i;

  for (i = WAIT_CNT(__CPUCLK, us); i; i--);
}


/*--------------------------- BusMode ---------------------------------------*/

static BOOL BusMode (U32 mode) {
  /* Set SDIO Bus mode to Open Drain or Push Pull. */

  switch (mode) {
    case BUS_OPEN_DRAIN:
    case BUS_PUSH_PULL:
      /* Not configurable. */
      return (__TRUE);
  }
  return (__FALSE);
}


/*--------------------------- BusWidth --------------------------------------*/

static BOOL BusWidth (U32 width) {
  /* Set SDIO Bus width. */

  switch (width) {
    case 1:
      SDIO->CLKCR &= ~0x1800;
      return (__TRUE);

    case 4:
      SDIO->CLKCR |=  0x0800;
      return (__TRUE);
  }
  return (__FALSE);
}


/*--------------------------- BusSpeed --------------------------------------*/

static BOOL BusSpeed (U32 kbaud) {
  /* Set SDIO clock speed to desired value. */
  U32 div;

  /* baud = SDIOCLK / (div + 2) */
  div = (__SDIOCLK/1000 + kbaud - 1) / kbaud;
  if (div < 2)    div  = 0;
  else            div -= 2;
  if (div > 0xFF) div  = 0xFF;
  SDIO->CLKCR = (SDIO->CLKCR & ~0xFF) | 0x300 | div;
  return (__TRUE);
}


/*--------------------------- Command ---------------------------------------*/

static BOOL Command (U8 cmd, U32 arg, U32 resp_type, U32 *rp) {
  /* Send a Command to Flash card and get a Response. */
  U32 cmdval,stat;

  cmd   &= 0x3F;
  cmdval = 0x400 | cmd;
  switch (resp_type) {
    case RESP_SHORT:
      cmdval |= 0x40;
      break;
    case RESP_LONG:
      cmdval |= 0xC0;
      break;
  }
  /* Send the command. */
  SDIO->ARG = arg;
  SDIO->CMD = cmdval;

  if (resp_type == RESP_NONE) {
    /* Wait until command finished. */
    while (SDIO->STA & SDIO_STA_CMDACT);
    SDIO->ICR = 0x00C007FF; 
    return (__TRUE);
  }

  for (;;) {
    stat = SDIO->STA;
    if (stat & SDIO_STA_CTIMEOUT) {
      SDIO->ICR = stat & SDIO_STA_CLEAR_MASK;
      return (__FALSE);
    }
    if (stat & SDIO_STA_CCRCFAIL) {
      SDIO->ICR = stat & SDIO_STA_CLEAR_MASK;
      if ((cmd == SEND_OP_COND)      ||
          (cmd == SEND_APP_OP_COND)  ||
          (cmd == STOP_TRANS)) {
        SDIO->CMD = 0;
        break;
      }
      return (__FALSE);
    }
    if (stat & SDIO_STA_CMDREND) {
      SDIO->ICR = stat & SDIO_STA_CLEAR_MASK;
      break;
    }
  }
  if ((SDIO->RESPCMD & 0x3F) != cmd) {
    if ((SDIO->RESPCMD & 0x3F) != 0x3F) {
      return (__FALSE);
    }
  }
  /* Read MCI response registers */
  rp[0] = SDIO->RESP1;
  if (resp_type == RESP_LONG) {
    rp[1] = SDIO->RESP2;
    rp[2] = SDIO->RESP3;
    rp[3] = SDIO->RESP4;
  }
  return (__TRUE);
}


/*--------------------------- ReadBlock -------------------------------------*/

static BOOL ReadBlock (U32 bl, U8 *buf, U32 cnt) {
  /* Read one or more 512 byte blocks from Flash Card. */
  U32 i;

  /* Set SDIO Transfer registers. */
  SDIO->DTIMER  = DATA_RD_TOUT_VALUE;
  SDIO->DLEN    = cnt * 512;

  /* Start DMA Peripheral to Memory transfer. */
  DmaStart (DMA_READ, buf, cnt);
  SDIO->DCTRL = 0x9B;

  for (i = DMA_TOUT; i; i--) {
    if (DMA2->ISR & (1<<13)) {
       DMA2->IFCR = (0xF<<12);               /* clear all channel IRQs    */
      /* Data transfer finished. */
      return (__TRUE);
    }
  }
  /* DMA Transfer timeout. */
  return (__FALSE);
}


/*--------------------------- WriteBlock ------------------------------------*/

static BOOL WriteBlock (U32 bl, U8 *buf, U32 cnt) {
  /* Write a cnt number of 512 byte blocks to Flash Card. */
  U32 i,j;

  for (j = 0; j < cnt; buf += 512, j++) {
    /* Set SDIO Transfer registers. */
    SDIO->DTIMER  = DATA_WR_TOUT_VALUE;
    SDIO->DLEN    = 512;

    /* Start DMA Memory to Peripheral transfer. */
    DmaStart (DMA_WRITE, buf, 1);
    SDIO->DCTRL = 0x99;

    for (i = DMA_TOUT; i; i--) {
      if (DMA2->ISR & (1<<13)) {
          DMA2->IFCR = (0xF<<12);             /* clear all channel IRQs    */
        /* Data transfer finished. */
        break;
      }
    }

    if (i == 0) {
      /* DMA Data Transfer timeout. */
      return (__FALSE);
    }

    if (cnt == 1) {
      return (__TRUE);
    }

    /* Wait until Data Block sent to Card. */
    while (SDIO->STA != (SDIO_STA_DATAEND | SDIO_STA_DBCKEND)) {
      if (SDIO->STA & (SDIO_STA_DCRCFAIL | SDIO_STA_DTIMEOUT)) {
        /* Error while Data Block sending occured. */
        return (__FALSE);
      }
    }
  }
  return (__TRUE);
}


/*--------------------------- DmaStart --------------------------------------*/

static void DmaStart (U32 mode, U8 *buf, U32 cnt) {
  /* Configure DMA controller Ch4 for read or write. */

  DMA2_Channel4->CCR &= ~(1 << 0);             /* disable channel4          */

  DMA2_Channel4->CPAR = (U32)&(SDIO->FIFO);
  DMA2_Channel4->CMAR = (U32)buf;
  /* The burst size set to 8, transfer size cnt*512 bytes. */
  DMA2_Channel4->CNDTR = ((cnt*512) >> 2);     /* trans. size cnt*512 bytes */

  if (mode == DMA_READ) {
    /* Transfer from SDIO-FIFO to memory. */
    DMA2_Channel4->CCR = (2<<12) |             /* Channel Priority High     */
                         (2<<10) |             /* Memory     size  32-bits  */
                         (2<< 8) |             /* Peripheral size  32-bits  */
                         (1<< 7) |             /* Memory increment enabled  */
                         (0<< 4);              /* Read from peripheral      */
  }
  else {
    /* Transfer from memory to SDIO-FIFO. */
    DMA2_Channel4->CCR = (2<<12) |             /* Channel Priority High     */
                         (2<<10) |             /* Memory     size  32-bits  */
                         (2<< 8) |             /* Peripheral size  32-bits  */
                         (1<< 7) |             /* Memory increment enabled  */
                         (1<< 4);              /* Read from memory          */
  }
  /* Enable DMA channels, little endian */
  DMA2->IFCR = (0xF<<12);                      /* clear all channel IRQs    */
  DMA2_Channel4->CCR |= (1 << 0);              /* enable channel4           */
}


/*--------------------------- CheckMedia ------------------------------------*/

static U32 CheckMedia (void) {
  /* Read CardDetect and WriteProtect SD card socket pins. */
  U32 stat = 0;
 
  if (!(GPIOF->IDR & (1 << 11))) {
    /* Card is inserted (CD=0). */
    stat |= M_INSERTED;
  }

#if 0 
  if ((GPIOA->IDR  & 0x20)) {
    /* Write Protect switch is active (WP=1). */
    stat |= M_PROTECTED;
  }
#endif

  return (stat);
}

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
