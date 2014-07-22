/*-----------------------------------------------------------------------------
 *      RL-ARM - FlashFS
 *-----------------------------------------------------------------------------
 *      Name:    SDIO_LPC43xx.c
 *      Purpose: SD/SDIO MMC Interface Driver for NXP LPC43xx
 *      Rev.:    V4.50
 *-----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2012 KEIL - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
  Note:  define 'LPC4330_XPLORER' in your project settings to use this file
         for the NGX LPC4330 Xplorer board.
 *----------------------------------------------------------------------------*/

#include <File_Config.h>
#include <LPC43xx.h>                    /* LPC43xx Definitions                */
#include "SDIO_LPC43xx.h"

/*-----------------------------------------------------------------------------
  Memory Card FAT Driver instance definition
   mci0_drv: First SD/MMC drive [M0:]
   mci1_drv: Second SD/MMC drive [M1:]
 *----------------------------------------------------------------------------*/

#define __DRV_ID   mci0_drv
#define __SDIOCLK 180000000
#define __CPUCLK  180000000

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
static BOOL SetDma (U32 mode, U8 *buf, U32 cnt);
static U32  CheckMedia (void);         /* Optional function for SD card check */

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
  SetDma,
  CheckMedia                            /* Can be NULL if not existing        */
};


/* Wait time in for loop cycles */
#define DMA_TOUT  10000000

static __align(4) SDIO_DESC DMADesc;

/* Clock Control Unit register bits */
#define CCU_CLK_CFG_RUN   (1 << 0)
#define CCU_CLK_CFG_AUTO  (1 << 1)
#define CCU_CLK_STAT_RUN  (1 << 0)

/*--------------------------- Init -------------------------------------------*/

static BOOL Init (void) {
  /* Initialize SD/MMC interface. */

  /* reset SDIO, DMA Peripheral */
  LPC_RGU->RESET_CTRL0 = ((1 << 19) |
                          (1 << 20)  );
  __NOP(); __NOP(); __NOP();

  /* Connect SDIO base clock to PLL1                                          */
  LPC_CGU->BASE_SDIO_CLK  = (0x01 << 11) | (0x09 << 24);

  /* Enable GPIO register interface clock                                     */
  LPC_CCU1->CLK_M4_GPIO_CFG |= CCU_CLK_CFG_AUTO | CCU_CLK_CFG_RUN;
  while (!(LPC_CCU1->CLK_M4_GPIO_STAT & CCU_CLK_STAT_RUN));

  /* Enable SDIO clocks                                                       */
  LPC_CCU1->CLK_M4_SDIO_CFG |= CCU_CLK_CFG_AUTO | CCU_CLK_CFG_RUN;
  while (!(LPC_CCU1->CLK_M4_SDIO_CFG & CCU_CLK_STAT_RUN));

  LPC_CCU2->CLK_SDIO_CFG |= CCU_CLK_CFG_AUTO | CCU_CLK_CFG_RUN;
  while (!(LPC_CCU2->CLK_SDIO_CFG & CCU_CLK_STAT_RUN));

  /* Configure SDIO port pins */
#ifdef LPC4330_XPLORER                           /* NGX LPC4330-Xplorer board */
  LPC_SCU->SFSCLK_2 =                                             0x4; /* CLK2  = SD_CLK            */
  LPC_SCU->SFSP1_9  = (1 << 7) | (1 << 6) | (1 << 5) | (1 << 4) | 0x7; /* P1.9  = SD_DAT0           */
  LPC_SCU->SFSP1_10 = (1 << 7) | (1 << 6) | (1 << 5) | (1 << 4) | 0x7; /* P1.10 = SD_DAT1           */
  LPC_SCU->SFSP1_11 = (1 << 7) | (1 << 6) | (1 << 5) | (1 << 4) | 0x7; /* P1.11 = SD_DAT2           */
  LPC_SCU->SFSP1_12 = (1 << 7) | (1 << 6) | (1 << 5) | (1 << 4) | 0x7; /* P1.12 = SD_DAT3           */
  LPC_SCU->SFSP1_6  = (1 << 7) | (1 << 6) | (1 << 5) | (1 << 4) | 0x7; /* P1.6  = SD_CMD            */
#else                                            /* Keil MCB4300 board        */
  LPC_SCU->SFSPC_0  = (1 << 7) | (1 << 6) | (1 << 5) | (1 << 4) | 0x7; /* PC.0  = SD_CLK            */
  LPC_SCU->SFSPC_4  = (1 << 7) | (1 << 6) | (1 << 5) | (1 << 4) | 0x7; /* PC.4  = SD_DAT0           */
  LPC_SCU->SFSPC_5  = (1 << 7) | (1 << 6) | (1 << 5) | (1 << 4) | 0x7; /* PC.5  = SD_DAT1           */
  LPC_SCU->SFSPC_6  = (1 << 7) | (1 << 6) | (1 << 5) | (1 << 4) | 0x7; /* PC.6  = SD_DAT2           */
  LPC_SCU->SFSPC_7  = (1 << 7) | (1 << 6) | (1 << 5) | (1 << 4) | 0x7; /* PC.7  = SD_DAT3           */
  LPC_SCU->SFSPC_8  = (1 << 7) | (1 << 6) | (1 << 5) | (1 << 4) | 0x7; /* PC.8  = SD_CD             */
  LPC_SCU->SFSPC_10 = (1 << 7) | (1 << 6) | (1 << 5) | (1 << 4) | 0x7; /* PC.10 = SD_CMD            */
#endif

  /* Reset SDMMC interface and wait until reset complete */
  LPC_SDMMC->CTRL |= SDIO_CTRL_DMA_RST | SDIO_CTRL_FIFO_RST |SDIO_CTRL_CTRL_RST;
  while (LPC_SDMMC->CTRL & (SDIO_CTRL_DMA_RST | SDIO_CTRL_FIFO_RST |SDIO_CTRL_CTRL_RST));

  /* Power enable */
  LPC_SDMMC->PWREN |= SDIO_PWREN_EN;

  /* Set FIFO Threshold watermark */
  LPC_SDMMC->FIFOTH = (1 << 28) | (15 << 16) | 16;

  /* Set Bus Mode */
  LPC_SDMMC->BMOD = (1 << 8) | SDIO_BMOD_DE;

  /* Set descriptor address */
  LPC_SDMMC->DBADDR = (U32)&DMADesc;

  /* Clear Interrupt Flags */
  LPC_SDMMC->RINTSTS  = SDIO_RINTSTS_MSK;
  LPC_SDMMC->IDSTS   |= SDIO_IDSTS_MSK;

  /* Enable DMA interrupts */
  LPC_SDMMC->IDINTEN |= 0x37;

  /* Enable internal DMA */
  LPC_SDMMC->CTRL |= SDIO_CTRL_USE_IDMA | SDIO_CTRL_DMA_EN;

  /* Success, SDIO initialized. */
  return (__TRUE);
}


/*--------------------------- UnInit -----------------------------------------*/

static BOOL UnInit (void) {

  /* Set SDIO port pins to their reset values */
#ifdef LPC4330_XPLORER                           /* NGX LPC4330-Xplorer board */
  LPC_SCU->SFSCLK_2 = 0;                /* CLK2  = Function reserved          */
  LPC_SCU->SFSP1_9  = 0;                /* P1.9  = Function reserved          */
  LPC_SCU->SFSP1_10 = 0;                /* P1.10 = Function reserved          */
  LPC_SCU->SFSP1_11 = 0;                /* P1.11 = Function reserved          */
  LPC_SCU->SFSP1_12 = 0;                /* P1.12 = Function reserved          */
  LPC_SCU->SFSP1_6  = 0;                /* P1.6  = Function reserved          */
#else                                            /* Keil MCB4300 board        */
  LPC_SCU->SFSPC_0  = 0;                /* PC.0  = Function reserved          */
  LPC_SCU->SFSPC_4  = 0;                /* PC.4  = Function reserved          */
  LPC_SCU->SFSPC_5  = 0;                /* PC.5  = Function reserved          */
  LPC_SCU->SFSPC_6  = 0;                /* PC.6  = Function reserved          */
  LPC_SCU->SFSPC_7  = 0;                /* PC.7  = Function reserved          */
  LPC_SCU->SFSPC_8  = 0;                /* PC.8  = Function reserved          */
  LPC_SCU->SFSPC_10 = 0;                /* PC.10 = Function reserved          */
#endif

  /* Disable SDIO interface clock */
  LPC_CCU1->CLK_M4_SDIO_CFG = 0;
  LPC_CCU2->CLK_SDIO_CFG    = 0;
  LPC_CGU->BASE_SDIO_CLK    = 1;
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
  /* Set SDIO Bus mode to Open Drain or Push Pull. */

  switch (mode) {
    case BUS_OPEN_DRAIN:
    case BUS_PUSH_PULL:
      /* Not configurable. */
      return (__TRUE);

    default:
      return (__FALSE);
  }
}


/*--------------------------- BusWidth ---------------------------------------*/

static BOOL BusWidth (U32 width) {
  /* Set SDIO Bus width. */
  switch (width) {
    case 1:
      LPC_SDMMC->CTYPE &= ~1;
      return (__TRUE);

    case 4:
      LPC_SDMMC->CTYPE |=  1;
      return (__TRUE);

    default:
      return (__FALSE);
  }
}


/*--------------------------- BusSpeed ---------------------------------------*/

static BOOL BusSpeed (U32 kbaud) {
  /* Set SDIO clock speed to desired value. */
  U32 div;

  /* baud = SDIOCLK / (div + 2) */
  div = (__SDIOCLK/1000 + kbaud - 1) / kbaud;
  div >>= 1;
  if (div > 0xFF) div  = 0xFF;
  LPC_SDMMC->CLKENA = 0;                /* Disable clock                      */
  LPC_SDMMC->CLKSRC = 0;                /* Clock source is clock divider 0    */
  LPC_SDMMC->CLKDIV = div;              /* Set clock divider                  */
  LPC_SDMMC->CLKENA = 1;                /* Don't stop clock when card in idle */

  /* Send "update clock registers" command and wait until finished */
  LPC_SDMMC->CMD = SDIO_CMD_CLK_UPD | SDIO_CMD_WAIT_PRV | SDIO_CMD_START;
  while (LPC_SDMMC->CMD & SDIO_CMD_START);
  return (__TRUE);
}


/*--------------------------- Command ----------------------------------------*/

static BOOL Command (U8 cmd, U32 arg, U32 resp_type, U32 *rp) {
  /* Send a Command to Flash card and get a Response. */
  U32 cmdval, ints;

  /* Clear interrupt status */
  LPC_SDMMC->RINTSTS = SDIO_RINTSTS_MSK;

  /* Set command register value */
  cmdval = (cmd & 0x3F) | SDIO_CMD_RESP_CRC | SDIO_CMD_WAIT_PRV | SDIO_CMD_START;
  switch (resp_type) {
    case RESP_SHORT:
      cmdval |= SDIO_CMD_RESP_EXP;
      break;
    case RESP_LONG:
      cmdval |= SDIO_CMD_RESP_EXP | SDIO_CMD_RESP_LEN;
      break;
  }

  if (cmd == READ_BLOCK  || cmd == READ_MULT_BLOCK ||
      cmd == WRITE_BLOCK || cmd == WRITE_MULT_BLOCK) {
    /* Set data expected and read/write bits */
    cmdval |= SDIO_CMD_DATA_EXP | SDIO_CMD_WAIT_PRV;
    if (cmd == WRITE_BLOCK || cmd == WRITE_MULT_BLOCK) {
      cmdval |= SDIO_CMD_READ_WRITE;
    }
  }

  if ((cmd == SEND_OP_COND) || (cmd == SEND_APP_OP_COND)  || (cmd == STOP_TRANS)) {
    /* Disable response CRC check */
    cmdval &= ~SDIO_CMD_RESP_CRC;
  }

  /* Send the command */
  LPC_SDMMC->CMDARG = arg;
  LPC_SDMMC->CMD    = cmdval;

  if (resp_type == RESP_NONE) {
    /* Wait until command finished */
    while (!(LPC_SDMMC->RINTSTS & SDIO_RINTSTS_CDONE));
    return (__TRUE);
  }

  for (;;) {
    ints = LPC_SDMMC->RINTSTS;

    if (ints & (SDIO_RINTSTS_RE | SDIO_RINTSTS_RCRC | SDIO_RINTSTS_RTO)) {
      /* Response error, CRC error, response timeout */
      return (__FALSE);
    }

    if (ints & SDIO_RINTSTS_HLE) {
      /* Hardware locked write */
      LPC_SDMMC->CMD = cmdval;
    }

    if (ints & SDIO_RINTSTS_CDONE) {
      /* Command done */
      if (cmdval & SDIO_CMD_DATA_EXP) {
        if (ints & (1 << 3)) {
          break;
        }
      }
      else break;
    }
  }
  /* Read MCI response registers */
  rp[0] = LPC_SDMMC->RESP0;
  if (resp_type == RESP_LONG) {
    rp[0] = LPC_SDMMC->RESP3;
    rp[1] = LPC_SDMMC->RESP2;
    rp[2] = LPC_SDMMC->RESP1;
    rp[3] = LPC_SDMMC->RESP0;
  }
  return (__TRUE);
}


/*--------------------------- ReadBlock --------------------------------------*/

static BOOL ReadBlock (U32 bl, U8 *buf, U32 cnt) {
  /* Read one or more 512 byte blocks from Flash Card. */
  U32 i, idsts;

  /* Wait until DMA completes the operation */
  for (i = DMA_TOUT; i; i--) {
    idsts = LPC_SDMMC->IDSTS;
    if (idsts & SDIO_IDSTS_FBE) {
      /* Fatal Bus Error */
      break;
    }
    if (idsts & SDIO_IDSTS_CES) {
      /* Card error summary */
      break;
    }
    if (idsts & SDIO_IDSTS_RI) {
      /* Data reception finished */
      LPC_SDMMC->IDSTS |= SDIO_IDSTS_MSK;
      return (__TRUE);
    }
  }
  /* Clear Interrupt flags */
  LPC_SDMMC->IDSTS = SDIO_IDSTS_MSK;
  /* DMA Data Transfer timeout. */
  return (__FALSE);
}


/*--------------------------- WriteBlock -------------------------------------*/

static BOOL WriteBlock (U32 bl, U8 *buf, U32 cnt) {
  /* Write a cnt number of 512 byte blocks to Flash Card. */
  U32 i, idsts;

  /* Wait until DMA completes the operation */
  for (i = DMA_TOUT; i; i--) {
    idsts = LPC_SDMMC->IDSTS;
    if (idsts & SDIO_IDSTS_FBE) {
      /* Fatal Bus Error */
      break;
    }
    if (idsts & SDIO_IDSTS_CES) {
      /* Card error summary */
      break;
    }
    if (idsts & SDIO_IDSTS_TI) {
      /* Data transmission finished */
      LPC_SDMMC->IDSTS = SDIO_IDSTS_MSK;
      return (__TRUE);
    }
  }
  /* Clear Interrupt flags */
  LPC_SDMMC->IDSTS = SDIO_IDSTS_MSK;
  /* DMA Data Transfer timeout. */
  return (__FALSE);
}

/*--------------------------- DmaStart ---------------------------------------*/

static BOOL SetDma (U32 mode, U8 *buf, U32 cnt) {
  /* Configure DMA Descriptor for read or write */

  LPC_SDMMC->BYTCNT = cnt * 512;

  DMADesc.BufSize  = cnt * 512;
  DMADesc.BufAddr  = (U32)buf;
  DMADesc.NextDesc = 0;
  DMADesc.CtrlStat = DESC_LD | DESC_FS | DESC_OWN;
  return __TRUE;
}


/*--------------------------- CheckMedia -------------------------------------*/

static U32 CheckMedia (void) {
  /* Read CardDetect and WriteProtect SD card socket pins. */
  U32 stat = 0;

#ifdef LPC4330_XPLORER                           /* NGX LPC4330-Xplorer board */
  stat |= M_INSERTED;                         /* CARD_DETECT is not connected */

#else                                            /* Keil MCB4300 board        */
  if (!(LPC_SDMMC->CDETECT & 1)) {
    /* Card is inserted (CD=0). */
    stat |= M_INSERTED;
  }
#endif

#if 0
  if (LPC_SDMMC->WRTPRT & 1) {
    /* Card write protect is on */
    stat |= M_PROTECTED;
  }
#endif
  return (stat);
}

/*-----------------------------------------------------------------------------
 * End of file
 *----------------------------------------------------------------------------*/
