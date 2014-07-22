/*-----------------------------------------------------------------------------
 *      RL-ARM - FlashFS
 *-----------------------------------------------------------------------------
 *      Name:    SDMMC_XMC4500.c
 *      Purpose: SD/MMC Interface Driver for Infineon XMC4500
 *      Rev.:    V4.54
 *-----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2012 KEIL - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include <File_Config.h>
#include <XMC4500.h>                    /* XMC4500 definitions                */
#include "SDMMC_XMC4500.h"

/*-----------------------------------------------------------------------------
  Memory Card FAT Driver instance definition
   mci0_drv: First SD/MMC drive [M0:]
   mci1_drv: Second SD/MMC drive [M1:]
 *----------------------------------------------------------------------------*/

#define __DRV_ID  mci0_drv
#define __MCLK    48000000
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
static BOOL SetDma (U32 mode, U8 *buf, U32 cnt);
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
  SetDma,
  CheckMedia                          /* Can be NULL if not existing         */
};


/* Wait time in for loop cycles */
#define DMA_TOUT  10000000
#define HW_TOUT   10000000

volatile uint32_t Isr_Status;

/*--------------------------- SDMMC0_0_IRQHandler ----------------------------*/

void SDMMC0_0_IRQHandler (void) {
  uint32_t norm, err;

  norm = SDMMC->INT_STATUS_NORM;
  err  = SDMMC->INT_STATUS_ERR;

  if (norm & SDMMC_INT_STATUS_NORM_ERR_INT_Msk) {
    if (err & SDMMC_INT_STATUS_ERR_CMD_TIMEOUT_ERR_Msk) {
      SDMMC->INT_STATUS_ERR = SDMMC_INT_STATUS_ERR_CMD_TIMEOUT_ERR_Msk;
    }

    if (err & SDMMC_INT_STATUS_ERR_CMD_CRC_ERR_Msk) {
      SDMMC->INT_STATUS_ERR = SDMMC_INT_STATUS_ERR_CMD_CRC_ERR_Msk;
    }

    if (err & SDMMC_INT_STATUS_ERR_CMD_END_BIT_ERR_Msk) {
      SDMMC->INT_STATUS_ERR = SDMMC_INT_STATUS_ERR_CMD_END_BIT_ERR_Msk;
    }

    if (err & SDMMC_INT_STATUS_ERR_CMD_IND_ERR_Msk) {
      SDMMC->INT_STATUS_ERR = SDMMC_INT_STATUS_ERR_CMD_IND_ERR_Msk;
    }

    if (err & SDMMC_INT_STATUS_ERR_DATA_TIMEOUT_ERR_Msk) {
      SDMMC->INT_STATUS_ERR = SDMMC_INT_STATUS_ERR_DATA_TIMEOUT_ERR_Msk;
    }

    if (err & SDMMC_INT_STATUS_ERR_DATA_CRC_ERR_Msk) {
      SDMMC->INT_STATUS_ERR = SDMMC_INT_STATUS_ERR_DATA_CRC_ERR_Msk;
    }

    if (err & SDMMC_INT_STATUS_ERR_DATA_END_BIT_ERR_Msk) {
      SDMMC->INT_STATUS_ERR = SDMMC_INT_STATUS_ERR_DATA_END_BIT_ERR_Msk;
    }

    if (err & SDMMC_INT_STATUS_ERR_ACMD_ERR_Msk) {
      SDMMC->INT_STATUS_ERR = SDMMC_INT_STATUS_ERR_ACMD_ERR_Msk;
    }
  }

  if (norm & SDMMC_INT_STATUS_NORM_BUFF_READ_READY_Msk) {
    SDMMC->INT_STATUS_NORM = SDMMC_INT_STATUS_NORM_BUFF_READ_READY_Msk;
    Isr_Status |= SDMMC_STAT_BUF_RDY;
  }

  if (norm & SDMMC_INT_STATUS_NORM_BUFF_WRITE_READY_Msk) {
    SDMMC->INT_STATUS_NORM = SDMMC_INT_STATUS_NORM_BUFF_WRITE_READY_Msk;
    Isr_Status |= SDMMC_STAT_BUF_RDY;
  }

  if (norm & SDMMC_INT_STATUS_NORM_TX_COMPLETE_Msk) {
    SDMMC->INT_STATUS_NORM = SDMMC_INT_STATUS_NORM_TX_COMPLETE_Msk;
    Isr_Status |= SDMMC_STAT_TX_DONE;
  }

  if (norm & SDMMC_INT_STATUS_NORM_CMD_COMPLETE_Msk) {
    SDMMC->INT_STATUS_NORM = SDMMC_INT_STATUS_NORM_CMD_COMPLETE_Msk;
    Isr_Status |= SDMMC_STAT_CMD_OK;
  }
}


/*--------------------------- Init -------------------------------------------*/

static BOOL Init (void) {
  /* Initialize MCI interface. */
  uint32_t i;

  /* Dereset SD/MMC peripheral and enable clock */
  SCU_RESET->PRCLR1 = SCU_RESET_PRCLR1_MMCIRS_Msk;
  SCU_CLK->CLKSET   = SCU_CLK_CLKSET_MMCCEN_Msk;
  
  /* Configure SD/MMC pins */
  PORT1->IOCR4 &= ~(PORT1_IOCR4_PC7_Msk | PORT1_IOCR4_PC6_Msk);
  PORT1->IOCR4 |=  (0x10U << PORT1_IOCR4_PC7_Pos) | (0x10U << PORT1_IOCR4_PC6_Pos);
  PORT1->HWSEL &= ~(0xF << 12);
  PORT1->HWSEL |=  (0x5 << 12);         /* P1.6 and P1.7 -> HW0               */
  
  PORT3->IOCR4 &= ~(PORT3_IOCR4_PC6_Msk | PORT3_IOCR4_PC5_Msk);
  PORT3->IOCR4 |=  (0x10U << PORT3_IOCR4_PC6_Pos) | (0x10U << PORT3_IOCR4_PC5_Pos);
  PORT3->HWSEL &= ~(0xF << 10);
  PORT3->HWSEL |=  (0x5 << 10);         /* P3.5 and P3.6 -> HW0               */
  
  PORT4->IOCR0 &= ~(PORT4_IOCR0_PC1_Msk | PORT4_IOCR0_PC0_Msk);
  PORT4->IOCR0 |=  (0x10U << PORT4_IOCR0_PC1_Pos) | (0x10U << PORT4_IOCR0_PC0_Pos);
  PORT4->HWSEL &= ~(0xF <<  0);
  PORT4->HWSEL |=  (0x5 <<  0);         /* P4.0 and P4.1 -> HW0               */
  
  /* Perform software reset */
  SDMMC->SW_RESET = SDMMC_SW_RESET_SW_RST_ALL_Msk;
  for (i = HW_TOUT; i; i--) {
    if (!(SDMMC->SW_RESET & SDMMC_SW_RESET_SW_RST_ALL_Msk)) {
      break;
    }
  }
  if (i == 0) return (__FALSE);

  /* Set data imeout: TMCLK * 2^27 */
  SDMMC->TIMEOUT_CTRL |= 0xE;

  /* Power up, switch on VCC for the Flash Card. */
  SDMMC->POWER_CTRL |=  (7 << SDMMC_POWER_CTRL_SD_BUS_VOLTAGE_SEL_Pos)  |
                         SDMMC_POWER_CTRL_SD_BUS_POWER_Msk              ;

  /* Select and enable SDMMC interrupt flags */
  SDMMC->EN_INT_STATUS_NORM = SDMMC_EN_INT_STATUS_NORM_MSK;
  SDMMC->EN_INT_STATUS_ERR  = SDMMC_EN_INT_STATUS_ERR_MSK;
  
  /* Select and enable SDMMC interupt signal lines */
  SDMMC->EN_INT_SIGNAL_NORM = SDMMC_EN_INT_SIGNAL_NORM_MSK;
  SDMMC->EN_INT_SIGNAL_ERR  = SDMMC_EN_INT_SIGNAL_ERR_MSK;

  /* Enable SDMMC interrupt in NVIC */
  NVIC_EnableIRQ (SDMMC0_0_IRQn);

   /* Dereset, enable and configure GPDMA */
   SCU_RESET->PRCLR2 = SCU_RESET_PRCLR2_DMA0RS_Msk;
   GPDMA0->DMACFGREG = GPDMA1_DMACFGREG_DMA_EN_Msk;
  return (__TRUE);
}


/*--------------------------- UnInit -----------------------------------------*/

static BOOL UnInit (void) {
  /* Reset the SD/MMC peripheral to default state. */
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

  PORT3->IOCR4 &= ~PORT3_IOCR4_PC5_Msk;

  switch (mode) {
    case BUS_OPEN_DRAIN:                /* Configure CMD line as open drain   */
      PORT3->IOCR4 |=  (0x18U << PORT3_IOCR4_PC5_Pos);
      return (__TRUE);

    case BUS_PUSH_PULL:                 /* Configure CMD line as push pull    */
      PORT3->IOCR4 |=  (0x10U << PORT3_IOCR4_PC5_Pos);
      return (__TRUE);
  }
  return (__FALSE);
}


/*--------------------------- BusWidth ---------------------------------------*/

static BOOL BusWidth (U32 width) {
  /* Set MCI Bus width. */

  switch (width) {
    case 1:                             /* Standard bus mode (1-bit)          */
      SDMMC->HOST_CTRL &= ~SDMMC_HOST_CTRL_DATA_TX_WIDTH_Msk;
      return (__TRUE);

    case 4:                             /* Wide bus mode (4-bit)              */
      SDMMC->HOST_CTRL |=  SDMMC_HOST_CTRL_DATA_TX_WIDTH_Msk;
      return (__TRUE);
  }
  return (__FALSE);
}


/*--------------------------- BusSpeed ---------------------------------------*/

static BOOL BusSpeed (U32 kbaud) {
  /* Set a MCI clock speed to desired value. */
  U32 i, div, clk;

  /* Disable (internal) SDMMC peripheral and (external) SD Clock */
  SDMMC->CLOCK_CTRL &= ~(SDMMC_CLOCK_CTRL_SDCLOCK_EN_Msk        |
                         SDMMC_CLOCK_CTRL_INTERNAL_CLOCK_EN_Msk);

  if (kbaud >= 25000) {                 /* SD clock will be above 25MHz       */
    SDMMC->HOST_CTRL |=  SDMMC_HOST_CTRL_HIGH_SPEED_EN_Msk;
  }
  else {                                /* SD clock will be up to 25MHz       */
    SDMMC->HOST_CTRL &= ~SDMMC_HOST_CTRL_HIGH_SPEED_EN_Msk;
  }
  
  for (div = 1; div <= 0x80; div <<= 1) {
    clk = (__MCLK / 1000) >> div;
    if (clk <= kbaud) {
      break;
    }
  }
  SDMMC->CLOCK_CTRL &= ~SDMMC_CLOCK_CTRL_SDCLK_FREQ_SEL_Msk;
  SDMMC->CLOCK_CTRL |= div << SDMMC_CLOCK_CTRL_SDCLK_FREQ_SEL_Pos;
  SDMMC->CLOCK_CTRL |= SDMMC_CLOCK_CTRL_INTERNAL_CLOCK_EN_Msk;
  
  for (i = 10000; i; i--) {
    if (SDMMC->CLOCK_CTRL & SDMMC_CLOCK_CTRL_INTERNAL_CLOCK_STABLE_Msk) {
      /* Internal clock is stable, enable SD clock */
      SDMMC->CLOCK_CTRL |= SDMMC_CLOCK_CTRL_SDCLOCK_EN_Msk;
      break;
    }
  }
  return ((i == 0) ? (__FALSE) : (__TRUE));
}


/*--------------------------- Command ----------------------------------------*/

static BOOL Command (U8 cmd, U32 arg, U32 resp_type, U32 *rp) {
  /* Send a Command to Flash card and get a Response. */
  uint32_t i, resp, cmdval;

  cmd   &= 0x3F;
  cmdval = (cmd << SDMMC_COMMAND_CMD_IND_Pos)   ; /* Set command index          */

  if (cmd == STOP_TRANS) return (__TRUE);
  if (cmd == READ_BLOCK  || cmd == READ_MULT_BLOCK  ||
      cmd == WRITE_BLOCK || cmd == WRITE_MULT_BLOCK ) {
      cmdval |= SDMMC_COMMAND_DATA_PRESENT_SELECT_Msk |
                SDMMC_COMMAND_CMD_IND_CHECK_EN_Msk    | /* Command Index Check Enable */
                SDMMC_COMMAND_CMD_CRC_CHECK_EN_Msk    ; /* Command CRC Check Enable   */
  }

  /* Set response type */
  switch (resp_type) {
    case RESP_SHORT:
      resp = SDMMC_RESP_48;             /* Response length 48                 */
      break;
    case RESP_LONG:
      resp = SDMMC_RESP_136;            /* Response length 136                */
      break;
    default:
      resp = SDMMC_NO_RESP;
      break;
  }
  cmdval |= resp;

  while (SDMMC->PRESENT_STATE & SDMMC_PRESENT_STATE_COMMAND_INHIBIT_CMD_Msk);
  while (SDMMC->PRESENT_STATE & SDMMC_PRESENT_STATE_COMMAND_INHIBIT_DAT_Msk);

  Isr_Status = 0;

  /* Send the command. */
  SDMMC->ARGUMENT1 = arg;
  SDMMC->COMMAND   = cmdval;

  for (i = 30000; i; i--) {
    if (Isr_Status & SDMMC_STAT_CMD_OK) { 
      break;
    }
  }
  if (i == 0)
    return (__FALSE);

  if (resp_type != RESP_NONE) {
    /* Read SDMMC response registers */    
    if (resp_type == RESP_LONG) {
      rp[0] = (SDMMC->RESPONSE6 << 8) | (SDMMC->RESPONSE4 >> 24);
      rp[1] = (SDMMC->RESPONSE4 << 8) | (SDMMC->RESPONSE2 >> 24);
      rp[2] = (SDMMC->RESPONSE2 << 8) | (SDMMC->RESPONSE0 >> 24);
      rp[3] =  SDMMC->RESPONSE0 << 8;
    }
    else {
      rp[0] = SDMMC->RESPONSE0;
    }
  }
  return (__TRUE);
}


/*--------------------------- ReadBlock --------------------------------------*/

static BOOL ReadBlock (U32 bl, U8 *buf, U32 cnt) {
  /* Read one or more 512 byte blocks from Flash Card. */
  U32 i;

  while (cnt--) {
    /* Wait until FIFO ready for reading */
    while ((Isr_Status & SDMMC_STAT_BUF_RDY) == 0);
    Isr_Status &= ~SDMMC_STAT_BUF_RDY;

    for (i = 0; i < 512/4; i++) {
      *(U32 *)buf = SDMMC->DATA_BUFFER;
      buf += 4;
    }
  }
  /* Wait until transfer complete */
  while ((Isr_Status & SDMMC_STAT_TX_DONE) == 0);

  return (__TRUE);
}


/*--------------------------- WriteBlock -------------------------------------*/

static BOOL WriteBlock (U32 bl, U8 *buf, U32 cnt) {
  /* Write a cnt number of 512 byte blocks to Flash Card. */
  U32 i;

  while (cnt--) {
    while ((Isr_Status & SDMMC_STAT_BUF_RDY) == 0);
    Isr_Status &= ~SDMMC_STAT_BUF_RDY;

    /* Enable channel and start DMA transfer. */
    GPDMA0->CHENREG   = 0x101;
    
    for (i = DMA_TOUT; i; i--) {
      /* Check DMA Transfer complete interrupt status */
      if (GPDMA0->RAWTFR & 1) {
        GPDMA0->CLEARTFR = 1;
        break;                          /* Data transfer finished.            */
      }
    }
    if (i == 0) {                       /* DMA Transfer timeout.              */
      return (__FALSE);
    }
  }
  /* Wait until transfer complete */
  while ((Isr_Status & SDMMC_STAT_TX_DONE) == 0);
  return (__TRUE);
}


/*--------------------------- DmaStart ---------------------------------------*/

static BOOL SetDma (U32 mode, U8 *buf, U32 cnt) {
  /* Configure DMA for read or write. */

  if (mode == DMA_READ) {
    /* Data transfer direction: Card to Host */
    SDMMC->TRANSFER_MODE |=  SDMMC_TRANSFER_MODE_TX_DIR_SELECT_Msk;
  }
  else {
    /* Data transfer direction: Host to Card */
    SDMMC->TRANSFER_MODE &= ~SDMMC_TRANSFER_MODE_TX_DIR_SELECT_Msk;
    
    GPDMA0_CH0->CTLH = 512/4;

    /* Configure GPDMA for memory to peripheral transfer */
    GPDMA0_CH0->SAR = (uint32_t)buf;
    GPDMA0_CH0->DAR = (uint32_t)&SDMMC->DATA_BUFFER;

    GPDMA0_CH0->CTLL =  0 << 14 | /* Source Burst Transaction Len: 8              */
                        0 << 11 | /* Dest Burst Transaction Len: 8                */
                        2 << 7  | /* Destination Address Increment: Don't change  */
                        2 << 4  | /* Source Transfer Width: 32                    */
                        2 << 1  ; /* Destination Transfer Width: 32               */
  }

  SDMMC->BLOCK_SIZE     = 0x200;
  SDMMC->TRANSFER_MODE &= ~SDMMC_TRANSFER_MODE_MULTI_BLOCK_SELECT_Msk;
  SDMMC->TRANSFER_MODE &= ~SDMMC_TRANSFER_MODE_ACMD_EN_Msk;
  SDMMC->TRANSFER_MODE &= ~SDMMC_TRANSFER_MODE_BLOCK_COUNT_EN_Msk;

  if (cnt > 1) {
    /* Multiple block data transfer */
    SDMMC->TRANSFER_MODE |= SDMMC_TRANSFER_MODE_MULTI_BLOCK_SELECT_Msk  |
                            1 << SDMMC_TRANSFER_MODE_ACMD_EN_Pos        |
                            SDMMC_TRANSFER_MODE_BLOCK_COUNT_EN_Msk      ;
    SDMMC->BLOCK_COUNT    = cnt;
  }
  return (__TRUE);
}

/*--------------------------- CheckMedia -------------------------------------*/

static U32 CheckMedia (void) {
  /* Read CardDetect and WriteProtect SD card socket pins. */

  /* CD and WP control is not implemented */
  return (M_INSERTED);
}

/*-----------------------------------------------------------------------------
 * End of file
 *----------------------------------------------------------------------------*/
