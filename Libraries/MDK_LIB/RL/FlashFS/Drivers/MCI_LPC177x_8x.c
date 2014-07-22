/*-----------------------------------------------------------------------------
 *      RL-ARM - FlashFS
 *-----------------------------------------------------------------------------
 *      Name:    MCI_LPC177x_8x.c
 *      Purpose: Multimedia Card Interface Driver for NXP LPC178x/7x
 *      Rev.:    V4.54
 *-----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2012 KEIL - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include <File_Config.h>
#include <LPC177x_8x.h>                 /* LPC177x/8x definitions             */
#include "MCI_LPC177x_8x.h"

/*-----------------------------------------------------------------------------
  Memory Card FAT Driver instance definition
   mci0_drv: First SD/MMC drive [M0:]
   mci1_drv: Second SD/MMC drive [M1:]
 *----------------------------------------------------------------------------*/

#define __DRV_ID  mci0_drv
#define __MCLK    60000000
#define __CPUCLK 120000000

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

/* Local Functions */
static void DmaStart (U32 mode, U8 *buf);

/*--------------------------- Init -------------------------------------------*/

static BOOL Init (void) {
  /* Initialize MCI interface. */

  /* Power Up the MCI and GPDMA controller. */
  LPC_SC->PCONP |=  (1 << 28) | (1 << 29);

  /* Configure MCI pins (fast mode) */
  LPC_IOCON->P1_2  = (1 << 9) | 2;      /* MCICLK */
  LPC_IOCON->P1_3  = (1 << 9) | 2;      /* MCICMD */
  LPC_IOCON->P1_5  = (1 << 9) | 2;      /* MCIPWR */
  LPC_IOCON->P1_6  = (1 << 9) | 2;      /* MCIDAT0 */
  LPC_IOCON->P1_7  = (1 << 9) | 2;      /* MCIDAT1 */
  LPC_IOCON->P1_11 = (1 << 9) | 2;      /* MCIDAT2 */
  LPC_IOCON->P1_12 = (1 << 9) | 2;      /* MCIDAT3 */

  LPC_IOCON->P0_7  = (1 << 7)|(2 << 3); /* MMC_CD */
  LPC_GPIO0->DIR &= ~(1 << 7);

  /* Clear all pending interrupts. */
  LPC_MCI->COMMAND  = 0;
  LPC_MCI->DATACTRL = 0;
  LPC_MCI->CLEAR    = 0x7FF;
  
  /* Set MCIPWR pin as active low (EA LPC1788 board) */
  LPC_SC->SCS &= ~(1 << 3);

  /* Power up, switch on VCC for the Flash Card. */
  LPC_MCI->POWER  = 0x02;
  Delay (10000);

  /* Power on the Flash Card. */
  LPC_MCI->POWER |= 0x01;

  return (__TRUE);
}


/*--------------------------- UnInit -----------------------------------------*/

static BOOL UnInit (void) {
  /* Reset the MCI peripheral to default state. */

  /* Power down, switch off VCC for the Flash Card. */
  LPC_MCI->POWER = 0x00;

  /* Clear all pending interrupts. */
  LPC_MCI->COMMAND   = 0;
  LPC_MCI->DATACTRL = 0;
  LPC_MCI->CLEAR     = 0x7FF;

  /* Set MCI pins to reset state */
  LPC_IOCON->P1_2  = (1 << 5)|(2 << 3); /* MCICLK  */
  LPC_IOCON->P1_3  = (1 << 5)|(2 << 3); /* MCICMD  */
  LPC_IOCON->P1_5  = (1 << 5)|(2 << 3); /* MCIPWR  */
  LPC_IOCON->P1_6  = (1 << 5)|(2 << 3); /* MCIDAT0 */
  LPC_IOCON->P1_7  = (1 << 5)|(2 << 3); /* MCIDAT1 */
  LPC_IOCON->P1_11 = (1 << 5)|(2 << 3); /* MCIDAT2 */
  LPC_IOCON->P1_12 = (1 << 5)|(2 << 3); /* MCIDAT3 */
  
  LPC_IOCON->P0_7 = (1 << 7) | (1 << 5);

  /* Power Down the MCI controller. */
  LPC_SC->PCONP   &= ~(1 << 28);
  return (__TRUE);
}


/*--------------------------- Delay ------------------------------------------*/

static void Delay (U32 us) {
  /* Approximate delay in micro seconds. */
  U32 i;

  for (i = WAIT_CNT(__CPUCLK, us); i; i--);
}


/*--------------------------- BusMode ----------------------------------------*/

static BOOL BusMode (U32 mode) {
  /* Set MCI Bus mode to Open Drain or Push Pull. */

  switch (mode) {
    case BUS_OPEN_DRAIN:
      LPC_MCI->POWER |=  0x40;          /* Open drain active                  */
      return (__TRUE);

    case BUS_PUSH_PULL:
      LPC_MCI->POWER &= ~0x40;          /* Push-pull active                   */
      return (__TRUE);
  }
  return (__FALSE);
}


/*--------------------------- BusWidth ---------------------------------------*/

static BOOL BusWidth (U32 width) {
  /* Set MCI Bus width. */

  switch (width) {
    case 1:
      LPC_MCI->CLOCK &= ~(1 << 11);     /* Standard bus mode (1-bit)          */
      return (__TRUE);

    case 4:
      LPC_MCI->CLOCK |=  (1 << 11);     /* Wide bus mode (4-bit)              */
      return (__TRUE);
  }
  return (__FALSE);
}


/*--------------------------- BusSpeed ---------------------------------------*/

static BOOL BusSpeed (U32 kbaud) {
  /* Set a MCI clock speed to desired value. */
  U32 div;

  /* baud = MCLK / (2 x (div + 1)) */
  div = (__MCLK/2000 + kbaud - 1) / kbaud;
  if (div > 0)    div--;
  if (div > 0xFF) div = 0xFF;
  LPC_MCI->CLOCK = (LPC_MCI->CLOCK & ~0xFF) | 0x100 | div;
  return (__TRUE);
}


/*--------------------------- Command ----------------------------------------*/

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
  LPC_MCI->ARGUMENT = arg;
  LPC_MCI->COMMAND  = cmdval;

  if (resp_type == RESP_NONE) {
    /* Wait until command finished. */
    while (LPC_MCI->STATUS & MCI_CMD_ACTIVE);
    LPC_MCI->CLEAR = 0x7FF;
    return (__TRUE);
  }

  for (;;) {
    stat = LPC_MCI->STATUS;
    if (stat & MCI_CMD_TIMEOUT) {
      LPC_MCI->CLEAR = stat & MCI_CLEAR_MASK;
      return (__FALSE);
    }
    if (stat & MCI_CMD_CRC_FAIL) {
      LPC_MCI->CLEAR = stat & MCI_CLEAR_MASK;
      if ((cmd == SEND_OP_COND)      ||
          (cmd == SEND_APP_OP_COND)  ||
          (cmd == STOP_TRANS)) {
        LPC_MCI->COMMAND = 0;
        break;
      }
      return (__FALSE);
    }
    if (stat & MCI_CMD_RESP_END) {
      LPC_MCI->CLEAR = stat & MCI_CLEAR_MASK;
      break;
    }
  }
  if ((LPC_MCI->RESP_CMD & 0x3F) != cmd) {
    if ((LPC_MCI->RESP_CMD & 0x3F) != 0x3F) {
      return (__FALSE);
    }
  }
  /* Read MCI response registers */
  rp[0] = LPC_MCI->RESP0;
  if (resp_type == RESP_LONG) {
    rp[1] = LPC_MCI->RESP1;
    rp[2] = LPC_MCI->RESP2;
    rp[3] = LPC_MCI->RESP3;
  }
  return (__TRUE);
}


/*--------------------------- ReadBlock --------------------------------------*/

static BOOL ReadBlock (U32 bl, U8 *buf, U32 cnt) {
  /* Read one or more 512 byte blocks from Flash Card. */
  U32 i;

  /* Set MCI Transfer registers. */
  LPC_MCI->DATATMR  = DATA_RD_TOUT_VALUE;
  LPC_MCI->DATALEN  = cnt * 512;

  /* Start DMA Peripheral to Memory transfer. */
  DmaStart (DMA_READ, buf);
  LPC_MCI->DATACTRL = 0x9B;

  for (i = DMA_TOUT; i; i--) {
    if (LPC_GPDMA->RawIntTCStat & 0x01) {
      /* Data transfer finished. */
      return (__TRUE);
    }
  }
  /* DMA Transfer timeout. */
  return (__FALSE);
}


/*--------------------------- WriteBlock -------------------------------------*/

static BOOL WriteBlock (U32 bl, U8 *buf, U32 cnt) {
  /* Write a cnt number of 512 byte blocks to Flash Card. */
  U32 i,j;

  for (j = 0; j < cnt; buf += 512, j++) {
    /* Set MCI Transfer registers. */
    LPC_MCI->DATATMR  = DATA_WR_TOUT_VALUE;
    LPC_MCI->DATALEN  = 512;

    /* Start DMA Memory to Peripheral transfer. */
    DmaStart (DMA_WRITE, buf);
    LPC_MCI->DATACTRL = 0x99;

    for (i = DMA_TOUT; i; i--) {
      if (LPC_GPDMA->RawIntTCStat & 0x01) {
        /* Data transfer finished. */
        break;
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
    while (LPC_MCI->STATUS != (MCI_DATA_END | MCI_DATA_BLK_END)) {
      if (LPC_MCI->STATUS & (MCI_DATA_CRC_FAIL | MCI_DATA_TIMEOUT)) {
        /* Error while Data Block sending occured. */
        return (__FALSE);
      }
    }
    /* Wait 2 SD clocks */
    for (i = WAIT_2SD_CLK(__CPUCLK); i; i--);
  }
  return (__TRUE);
}


/*--------------------------- DmaStart ---------------------------------------*/

static void DmaStart (U32 mode, U8 *buf) {
  /* Configure DMA for read or write. */

  if (mode == DMA_READ) {
    /* Transfer from MCI-FIFO to memory. */
    LPC_GPDMACH0->CSrcAddr  = (U32)&LPC_MCI->FIFO;
    LPC_GPDMACH0->CDestAddr = (U32)buf;
    /* The burst size set to 8, transfer size 512 bytes. */
    LPC_GPDMACH0->CControl = (1U  << 31) |    /* Terminal count interrupt en  */
                             (1U  << 27) |    /* Destination address increment*/
                             (2U  << 21) |    /* Destination transfer width   */
                             (2U  << 18) |    /* Source transfer width        */
                             (2U  << 15) |    /* Destination burst size       */
                             (2U  << 12) |    /* Source burst size            */
                             (512 >>  2) ;    /* Transfer size                */
    
    LPC_GPDMACH0->CConfig  = (0x06 << 11)|    /* Transfer type                */
                             (0x00 <<  6)|    /* Destination peripheral       */
                             (0x01 <<  1)|    /* Source peripheral            */
                             (0x01 <<  0);    /* Channel enable               */
  }
  else {
    /* Transfer from memory to MCI-FIFO. */
    LPC_GPDMACH0->CSrcAddr  = (U32)buf;
    LPC_GPDMACH0->CDestAddr = (U32)&LPC_MCI->FIFO;
    /* The burst size set to 8, transfer size 512 bytes. */
    LPC_GPDMACH0->CControl = (1U  << 31) |    /* Terminal count interrupt en  */
                             (1U  << 26) |    /* Source address increment     */
                             (2U  << 21) |    /* Destination transfer width   */
                             (2U  << 18) |    /* Source transfer width        */
                             (2U  << 15) |    /* Destination burst size       */
                             (2U  << 12) |    /* Source burst size            */
                             (512 >>  2) ;    /* Transfer size                */
    
    LPC_GPDMACH0->CConfig  = (0x05 << 11)|    /* Transfer type                */
                             (0x01 <<  6)|    /* Destination peripheral       */
                             (0x00 <<  1)|    /* Source peripheral            */
                             (0x01 <<  0);    /* Channel enable               */
  }
  /* Enable DMA channels, little endian */
  LPC_GPDMA->IntTCClear = 0x01;
  LPC_GPDMA->Config     = 0x01;
}

/*--------------------------- CheckMedia -------------------------------------*/

static U32 CheckMedia (void) {
  /* Read CardDetect and WriteProtect SD card socket pins. */
  U32 stat = 0;
#if 0
  if (!(LPC_GPIO0->PIN & SD_CD_PIN)) {
    /* Card is inserted (CD = 0) */
    stat |= M_INSERTED;
  }
  if ((LPC_GPIO0->PIN & SD_WP_PIN)) {
    /* Write Protect switch is active (WP=1). */
    stat |= M_PROTECTED;
  }
#else
  /* CD and WP control is not implemented */
  stat |= M_INSERTED;
#endif
  return (stat);
}

/*-----------------------------------------------------------------------------
 * End of file
 *----------------------------------------------------------------------------*/
