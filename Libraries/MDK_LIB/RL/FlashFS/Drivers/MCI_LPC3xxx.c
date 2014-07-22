/*----------------------------------------------------------------------------
 *      RL-ARM - FlashFS
 *----------------------------------------------------------------------------
 *      Name:    MCI_LPC3xxx.c
 *      Purpose: Multimedia Card Interface Driver for NXP LPC3xxx
 *      Rev.:    V4.51
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2011 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <File_Config.h>
#include <LPC325x.h>
#include "MCI_LPC3xxx.h"

/*----------------------------------------------------------------------------
  Memory Card FAT Driver instance definition
   mci0_drv: First SD/MMC drive [M0:]
   mci1_drv: Second SD/MMC drive [M1:]
 *---------------------------------------------------------------------------*/

#define __DRV_ID  mci0_drv
#define __MCLK    69330000
#define __CPUCLK  208000000

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

/* Following macro definitions can be used to select the Evaluation Board:

  _LPC3180_   - phyCORE-LPC3180 used
  _LPC3250_   - phyCORE-LPC3250 used  (default)                              */

#ifdef _LPC3180_
 /* Pins: CD=GPI_05, WP=GPI_02 */
 #define CD_STATE  (P3_INP_STATE & 0x0020)
 #define WP_STATE  (P3_INP_STATE & 0x0004)
#else
 /* Pins: CD=GPIO_01, WP=GPIO_00 */
 #define CD_STATE  (P3_INP_STATE & 0x0800)
 #define WP_STATE  (P3_INP_STATE & 0x0400)
#endif

/* Wait time in for loop cycles */
#define DMA_TOUT  10000000

/* Local Function Prototypes */
static void DmaStart (U32 mode, U8 *buf);
static void DmaStop (void);


/*--------------------------- Init ------------------------------------------*/

static BOOL Init (void) {
  /* Initialize MCI interface. */

  /* Enable PIN MUX for MCI, set MSSDCLK clock.       */
  /* HCLKPLL = 208MHz, MSSDCLK = 69.33MHz (208MHz/3). */
  MS_CTRL  = 0x0223;

  /* Clear all pending interrupts. */
  MCI_COMMAND   = 0;
  MCI_DATA_CTRL = 0;
  MCI_CLEAR     = 0x7FF;

  /* Power up, switch on VCC for the Flash Card. */
  MCI_POWER  = 0x02;
  Delay (10000);

  /* Power on the Flash Card. */
  MCI_POWER |= 0x01;

  return (__TRUE);
}


/*--------------------------- UnInit ----------------------------------------*/

static BOOL UnInit (void) {
  /* Reset the MCI peripheral to default state. */

  /* Power down, switch off VCC for the Flash Card. */
  MCI_POWER = 0x00;

  /* Clear all pending interrupts. */
  MCI_COMMAND   = 0;
  MCI_DATA_CTRL = 0;
  MCI_CLEAR     = 0x7FF;

  /* Disable PIN MUX for MCI. */
  MS_CTRL = 0x0000;
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
  /* Set MCI Bus mode to Open Drain or Push Pull. */

  switch (mode) {
    case BUS_OPEN_DRAIN:
      MCI_POWER |= 0x40;
      break;

    case BUS_PUSH_PULL:
      MCI_POWER &= ~0x40;
      break;

    default:
      return (__FALSE);
  }
  return (__TRUE);
}


/*--------------------------- BusWidth --------------------------------------*/

static BOOL BusWidth (U32 width) {
  /* Set MCI Bus width. */

  switch (width) {
    case 1:
      MCI_CLOCK &= ~0x0800;
      break;

    case 4:
      MCI_CLOCK |=  0x0800;
      break;

    default:
      return (__FALSE);
  }
  return (__TRUE);
}


/*--------------------------- BusSpeed --------------------------------------*/

static BOOL BusSpeed (U32 kbaud) {
  /* Set a MCI clock speed to desired value. */
  U32 div;

  /* baud = MCLK / (2 x (div + 1)) */
  div = (__MCLK/2000 + kbaud - 1) / kbaud;
  if (div > 0)    div--;
  if (div > 0xFF) div = 0xFF;
  MCI_CLOCK = (MCI_CLOCK & ~0xFF) | 0x300 | div;
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
  MCI_ARGUMENT = arg;
  MCI_COMMAND  = cmdval;

  if (resp_type == RESP_NONE) {
    /* Wait until command finished. */
    while (MCI_STATUS & MCI_CMD_ACTIVE);
    MCI_CLEAR = 0x7FF;
    return (__TRUE);
  }

  for (;;) {
    stat = MCI_STATUS;
    if (stat & MCI_CMD_TIMEOUT) {
      MCI_CLEAR = stat & MCI_CLEAR_MASK;
      return (__FALSE);
    }
    if (stat & MCI_CMD_CRC_FAIL) {
      MCI_CLEAR = stat & MCI_CLEAR_MASK;
      if ((cmd == SEND_OP_COND)      ||
          (cmd == SEND_APP_OP_COND)  ||
          (cmd == STOP_TRANS)) {
        MCI_COMMAND = 0;
        break;
      }
      return (__FALSE);
    }
    if (stat & MCI_CMD_RESP_END) {
      MCI_CLEAR = stat & MCI_CLEAR_MASK;
      break;
    }
  }
  /* Read MCI response registers */
  rp[0] = MCI_RESP0;
  if (resp_type == RESP_LONG) {
    rp[1] = MCI_RESP1;
    rp[2] = MCI_RESP2;
    rp[3] = MCI_RESP3;
  }
  return (__TRUE);
}


/*--------------------------- ReadBlock -------------------------------------*/

static BOOL ReadBlock (U32 bl, U8 *buf, U32 cnt) {
  /* Read one or more 512 byte blocks from Flash Card. */
  U32 i;

  /* Make sure the Rx FIFO is empty. */
  while (MCI_STATUS & MCI_RX_DATA_AVAIL) {
    MCI_FIFO;
  }

  /* Set MCI Transfer registers. */
  MCI_DATA_TMR  = DATA_RD_TOUT_VALUE;
  MCI_DATA_LEN  = cnt * 512;

  /* Start DMA Peripheral to Memory transfer. */
  DmaStart (DMA_READ, buf);
  MCI_DATA_CTRL = 0x9B;

  for (i = DMA_TOUT; i; i--) {
    if (GPDMA_RAW_INT_TCSTAT & 0x01) {
      /* Data transfer finished. */
      break;
    }
    if (MCI_STATUS & MCI_DATA_TIMEOUT) {
      /* Data read timeout. */
       DmaStop ();
       return (__FALSE);
    }
  }
  DmaStop ();
  return (__TRUE);
}


/*--------------------------- WriteBlock ------------------------------------*/

static BOOL WriteBlock (U32 bl, U8 *buf, U32 cnt) {
  /* Write a cnt number of 512 byte blocks to Flash Card. */
  U32 i,j;

  for (j = 0; j < cnt; buf += 512, j++) {
    /* Set MCI Transfer registers. */
    MCI_DATA_TMR  = DATA_WR_TOUT_VALUE;
    MCI_DATA_LEN  = 512;

    /* Start DMA Memory to Peripheral transfer. */
    DmaStart (DMA_WRITE, buf);
    MCI_DATA_CTRL = 0x99;

    for (i = DMA_TOUT; i; i--) {
      if (GPDMA_RAW_INT_TCSTAT & 0x01) {
        /* Data transfer finished. */
        break;
      }
      if (MCI_STATUS & MCI_DATA_TIMEOUT) {
        /* Data write timeout, write request failed. */
        DmaStop ();
        return (__FALSE);
      }
    }
    if (i == 0) {
      /* DMA Data Transfer timeout. */
      return (__FALSE);
    }
    if (cnt == 1) {
      break;
    }

    /* Wait until Data Block sent to Card. */
    while (MCI_STATUS != (MCI_DATA_END | MCI_DATA_BLK_END)) {
      if (MCI_STATUS & (MCI_DATA_CRC_FAIL | MCI_DATA_TIMEOUT)) {
        /* Error while Data Block sending occured. */
        return (__FALSE);
      }
    }
    /* Wait 2 SD clocks */
    for (i = WAIT_2SD_CLK(__CPUCLK); i; i--);
  }
  return (__TRUE);
}


/*--------------------------- DmaStart --------------------------------------*/

static void DmaStart (U32 mode, U8 *buf) {
  /* Configure DMA controller Ch0 for read or write. */

  if (mode == DMA_READ) {
    /* Transfer from MCI-FIFO to memory. */
    GPDMA_CH0_SRC  = (U32)&MCI_FIFO;
    GPDMA_CH0_DEST = (U32)buf;
    GPDMA_CH0_LLI  = 0;
    /* The burst size set to 8, data width 32-bit. */
    GPDMA_CH0_CTRL = (2 << 12) | (2 << 15) | (2 << 18) | (2 << 21) |
                     (1 << 24) | (1 << 27) | (1u << 31);
    GPDMA_SYNC    |= 0x0020;
    GPDMA_CH0_CFG  = 0x10001 | (0x04 << 1) | (0x06 << 11);
  }
  else {
    /* Transfer from memory to MCI-FIFO. */
    GPDMA_CH0_SRC  = (U32)buf;
    GPDMA_CH0_DEST = (U32)&MCI_FIFO;
    GPDMA_CH0_LLI  = 0;
    /* The burst size set to 8, data width 32-bit. */
    GPDMA_CH0_CTRL = (2 << 12) | (2 << 15) | (2 << 18) | (2 << 21) |
                     (1 << 26) | (1 << 24) | (1u << 31);
    GPDMA_SYNC    |= 0x0020;
    GPDMA_CH0_CFG  = 0x10001 | (0x04 << 6) | (0x05 << 11);
  }
  /* Enable DMA channels, little endian */
  GPDMA_INT_TCCLR = 0x01;
  GPDMA_CONFIG    = 0x01;
}


/*--------------------------- DmaStop ---------------------------------------*/

static void DmaStop (void) {
  /* Stop DMA controller. */

  GPDMA_INT_TCCLR = 0x01;
  GPDMA_CONFIG    = 0x00;
}


/*--------------------------- CheckMedia ------------------------------------*/

static U32 CheckMedia (void) {
  /* Read CardDetect and WriteProtect SD card socket pins. */
  U32 stat = 0;
 
  if (!CD_STATE) {
    /* Card is inserted (CD=0). */
    stat |= M_INSERTED;
  } 
  if (WP_STATE) {
    /* Write Protect switch is active (WP=1). */
    stat |= M_PROTECTED;
  }
  return (stat);
}

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
