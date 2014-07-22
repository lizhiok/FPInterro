/*----------------------------------------------------------------------------
 *      RL-ARM - FlashFS
 *----------------------------------------------------------------------------
 *      Name:    MCI_SAM3X.c
 *      Purpose: Multimedia Card Interface Driver for Atmel AT91SAM3X
 *      Rev.:    V5.00
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <File_Config.h>
#include <sam3xa.h>
#include "MCI_SAM3X.h"

/*----------------------------------------------------------------------------
  Memory Card FAT Driver instance definition
   mci0_drv: First SD/MMC drive [M0:]
   mci1_drv: Second SD/MMC drive [M1:]
 *---------------------------------------------------------------------------*/

#define __DRV_ID  mci0_drv
#define __HSMCICLK  84000000            /* HSMCI peripheral clock frequency   */
#define __CPUCLK    84000000            /* CPU clock frequency                */

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
#define HSMCI_TOUT          100000
#define DMA_TOUT          10000000      /* ~0.5s @ 84MHz                      */
#define WAIT_CNT(ck,us)   ((ck/4000000)*us)

/* DMA channel used for MCI */
#define DMA_CH       3

/* Local Variables */
static U8  CmdBusMode;                /* Open Drain mode = 1, Push Pull = 0  */


/*--------------------------- Init ------------------------------------------*/

static BOOL Init (void) {
  /* Initialize MCI interface. */
  U32 i, msk;

  CmdBusMode = 0;                       /* Cmd line set to open drain mode    */

  /* Enable PIOA, MCI and DMA clock. */
  PMC->PMC_WPMR  = PMC_WPDIS_KEY;        /* Disable PMC write protect        */
  PMC->PMC_PCER0 = (1 << ID_PIOA) | (1 << ID_PIOE) | (1 << ID_HSMCI);
  PMC->PMC_PCER1 = (1 << (ID_DMAC - 32));
  PMC->PMC_WPMR  = PMC_WPEN_KEY;         /* Enable PWM write protect         */

  /* PIO Write Protect Disable */
  PIOA->PIO_WPMR = PIO_WPDIS_KEY;
  PIOB->PIO_WPMR = PIO_WPDIS_KEY;
  PIOE->PIO_WPMR = PIO_WPDIS_KEY;

  /* Enable MCI Pins. */
  msk = PIO_PA19A_MCCK  | PIO_PA20A_MCCDA | PIO_PA21A_MCDA0 |
        PIO_PA22A_MCDA1 | PIO_PA23A_MCDA2 | PIO_PA24A_MCDA3;
  
  PIOA->PIO_IDR   = msk;
  PIOA->PIO_PDR   = msk;
  PIOA->PIO_PUER  = msk; 
  PIOA->PIO_PUDR  = PIO_PA19A_MCCK;

  PIOA->PIO_ABSR &= PIO_PA19A_MCCK  | PIO_PA20A_MCCDA | PIO_PA21A_MCDA0 |
                    PIO_PA22A_MCDA1 | PIO_PA23A_MCDA2 | PIO_PA24A_MCDA3;

 /* Enable CD pin. */
  PIOE->PIO_IDR  = PIO_PE6;
  PIOE->PIO_ODR  = PIO_PE6;
  PIOE->PIO_PER  = PIO_PE6;

  /* Initialize DMA channel */
  DMAC->DMAC_EN     = DMAC_EN_ENABLE;
  DMAC->DMAC_CHDR   = (1       << DMA_CH);
  DMAC->DMAC_EBCIDR = (0x10101 << DMA_CH);

  /* Disable write protection */
  HSMCI->HSMCI_WPMR = HSMCI_WPDIS_KEY;

  /* Initialize HSMCI peripheral */
  HSMCI->HSMCI_CR   = HSMCI_CR_SWRST;
  HSMCI->HSMCI_CR   = HSMCI_CR_PWSDIS | HSMCI_CR_MCIDIS;
  HSMCI->HSMCI_IDR  = 0xFCFFF1FF;
  HSMCI->HSMCI_DTOR = HSMCI_DTOR_DTOMUL_1048576   | HSMCI_DTOR_DTOCYC(15);
  HSMCI->HSMCI_CSTOR= HSMCI_CSTOR_CSTOMUL_1048576 | HSMCI_CSTOR_CSTOCYC (1);
  HSMCI->HSMCI_CFG  = HSMCI_CFG_LSYNC | HSMCI_CFG_HSMODE | HSMCI_CFG_FERRCTRL | HSMCI_CFG_FIFOMODE;
  HSMCI->HSMCI_MR   = HSMCI_MR_WRPROOF   | HSMCI_MR_RDPROOF      |
                      HSMCI_MR_PWSDIV(7) | HSMCI_MR_CLKDIV (0xFF);
  HSMCI->HSMCI_DMA  = 0;

  /* Enable write protection */
  HSMCI->HSMCI_WPMR = HSMCI_WPEN_KEY;
  
  /* Enable HSMCI */
  HSMCI->HSMCI_CR   = HSMCI_CR_PWSEN | HSMCI_CR_MCIEN;

  /* Send 74 clock cycles for memory card initialization sequence */
  HSMCI->HSMCI_CMDR = HSMCI_CMDR_SPCMD_INIT;
  for (i = HSMCI_TOUT; i; i--) {
    if (HSMCI->HSMCI_SR & HSMCI_SR_CMDRDY) {
      return (__TRUE);
    }
  }
  return (__FALSE);
}


/*--------------------------- UnInit ----------------------------------------*/

static BOOL UnInit (void) {
  /* Reset the MCI peripheral to default state. */

  /* Disable MCI pins. */
  PIOA->PIO_PUDR  = PIO_PA19A_MCCK  | PIO_PA20A_MCCDA | PIO_PA21A_MCDA0 |
                    PIO_PA22A_MCDA1 | PIO_PA23A_MCDA2 | PIO_PA24A_MCDA3;
  HSMCI->HSMCI_CR = HSMCI_CR_PWSEN | HSMCI_CR_MCIDIS;

  /* Power Down the MCI controller. */
  PMC->PMC_PCDR0  = (1 << ID_HSMCI);
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
      CmdBusMode = 1;
      return (__TRUE);

    case BUS_PUSH_PULL:
      CmdBusMode = 0;
      return (__TRUE);
  }
  return (__FALSE);
}


/*--------------------------- BusWidth --------------------------------------*/

static BOOL BusWidth (U32 width) {
  /* Set MCI Bus width. */
  BOOL rtv = __TRUE;

  /* Disable SDCR register write protection */
  HSMCI->HSMCI_WPMR = HSMCI_WPDIS_KEY;
  
  /* Select SlotA bus width */
  HSMCI->HSMCI_SDCR &= ~(HSMCI_SDCR_SDCBUS_Msk | HSMCI_SDCR_SDCSEL_Msk);
  switch (width) {
    case 1: /* 1-bit mode is already selected */      break;
    case 4: HSMCI->HSMCI_SDCR |= HSMCI_SDCR_SDCBUS_4; break;
    
    default:
      rtv = __FALSE;
      break;
  }
  
  /* Enable SDCR register write protection */
  HSMCI->HSMCI_WPMR = HSMCI_WPEN_KEY;
  return (rtv);
}


/*--------------------------- BusSpeed --------------------------------------*/

static BOOL BusSpeed (U32 kbaud) {
  /* Set a MCI clock speed to desired value. */
  U32 div;
  
  if (kbaud > 10000)
    kbaud = 10000;

  /* baud = __HSMCICLK / (2 x (div + 1)) */
  div = (__HSMCICLK/2000 + kbaud - 1) / kbaud;
  if (div > 0)    div--;
  if (div > 0xFF) div = 0xFF;
  
  /* Disable SDCR register write protection */
  HSMCI->HSMCI_WPMR = HSMCI_WPDIS_KEY;
  /* Set clock divider */
  HSMCI->HSMCI_MR  = (HSMCI->HSMCI_MR & ~HSMCI_MR_CLKDIV_Msk) | div;
  /* Enable SDCR register write protection */
  HSMCI->HSMCI_WPMR = HSMCI_WPEN_KEY;
  return (__TRUE);
}


/*--------------------------- Command ---------------------------------------*/

static BOOL Command (U8 cmd, U32 arg, U32 resp_type, U32 *rp) {
  /* Send a Command to Flash card and get a Response. */
  U32 i,cmdval,stat;

  cmd   &= HSMCI_CMDR_CMDNB_Msk;
  cmdval = HSMCI_CMDR_MAXLAT | ((CmdBusMode & 1) << 11) | cmd;
  switch (cmd) {
    case READ_MULT_BLOCK:
      cmdval |= HSMCI_CMDR_TRTYP_MULTIPLE;
    case READ_BLOCK:
      cmdval |= HSMCI_CMDR_TRTYP_BLOCK | HSMCI_CMDR_TRCMD_START_DATA | HSMCI_CMDR_TRDIR_READ;
      break;

    case WRITE_MULT_BLOCK:
      cmdval |= HSMCI_CMDR_TRTYP_MULTIPLE;
    case WRITE_BLOCK:
      cmdval |= HSMCI_CMDR_TRTYP_BLOCK | HSMCI_CMDR_TRCMD_START_DATA | HSMCI_CMDR_TRDIR_WRITE;
      break;

    case STOP_TRANS:
      cmdval |= HSMCI_CMDR_TRCMD_STOP_DATA;
      break;
  }
  switch (resp_type) {
    case RESP_SHORT:
      cmdval |= HSMCI_CMDR_RSPTYP_48_BIT;
      break;
    case RESP_LONG:
      cmdval |= HSMCI_CMDR_RSPTYP_136_BIT;
      break;
  }

  HSMCI->HSMCI_ARGR = arg;
  HSMCI->HSMCI_CMDR = cmdval;

  if (resp_type == RESP_NONE) {
    /* Wait until command finished. */
    while (!(HSMCI->HSMCI_SR & HSMCI_SR_CMDRDY));
    return (__TRUE);
  }

  for (i = DMA_TOUT; i; i--) {
    stat = HSMCI->HSMCI_SR;
    if (stat & HSMCI_SR_RTOE) {
      return (__FALSE);
    }
    if (stat & HSMCI_SR_RCRCE) {
      if ((cmd == SEND_OP_COND)      ||
          (cmd == SEND_APP_OP_COND)  ||
          (cmd == STOP_TRANS)) {
        break;
      }
      return (__FALSE);
    }
    if (stat & HSMCI_SR_CMDRDY) {
      break;
    }
  }
  /* Read MCI response registers */
  rp[0] = HSMCI->HSMCI_RSPR[0];
  if (resp_type == RESP_LONG) {
    rp[1] = HSMCI->HSMCI_RSPR[1];
    rp[2] = HSMCI->HSMCI_RSPR[2];
    rp[3] = HSMCI->HSMCI_RSPR[3];
  }
  return (__TRUE);
}


/*--------------------------- ReadBlock -------------------------------------*/

static BOOL ReadBlock (U32 bl, U8 *buf, U32 cnt) {
  /* Read one or more 512 byte blocks from Flash Card. */
  U32 i, sz, *bp;

  if (cnt !=  1) {
    bp = (U32 *)buf;
    sz = cnt * 512 / 4;
    
    for (i = 0; i < sz; i++) {
      while ((HSMCI->HSMCI_SR & HSMCI_SR_RXRDY) == 0);
      bp[i] = HSMCI->HSMCI_RDR;
    }
    while ((HSMCI->HSMCI_SR & HSMCI_SR_XFRDONE) == 0);
    return __TRUE;
  }
  else {
    /* Wait until transfer finished */
    for (i = DMA_TOUT; i; i--) {
      if (HSMCI->HSMCI_SR & HSMCI_SR_XFRDONE) {
        /* Data transfer finished. */
        return (__TRUE);
      }
    }
    /* DMA Transfer timeout. */
    return (__FALSE);
  }
}


/*--------------------------- WriteBlock ------------------------------------*/
  
static BOOL WriteBlock (U32 bl, U8 *buf, U32 cnt) {
  /* Write a cnt number of 512 byte blocks to Flash Card. */
  U32 i, sz, *bp;

  if (cnt != 1) {
    bp = (U32 *)buf;
    sz = cnt * 512 / 4;
  
    for (i = 0; i < sz; i++) {
      while ((HSMCI->HSMCI_SR & HSMCI_SR_TXRDY) == 0);
      HSMCI->HSMCI_TDR = bp[i];
    }
    while ((HSMCI->HSMCI_SR & HSMCI_SR_XFRDONE) == 0);
    return __TRUE;
  }
  else {
    /* Wait until transfer finished */
    for (i = DMA_TOUT; i; i--) {
      if (HSMCI->HSMCI_SR & HSMCI_SR_XFRDONE) {
        /* Data transfer finished. */
        return (__TRUE);
      }
    }
    /* DMA Transfer timeout. */
    return (__FALSE);
  }
}


/*--------------------------- SetDma ----------------------------------------*/

static BOOL SetDma (U32 mode, U8 *buf, U32 cnt) {
  /* Configure DMA for read or write. */
  
  /* Disable write protection */
  HSMCI->HSMCI_WPMR = HSMCI_WPDIS_KEY;
  /* Set block len gth and count of blocks */
  HSMCI->HSMCI_BLKR = HSMCI_BLKR_BLKLEN (512) | cnt;
  
  if (cnt != 1) {
    /* Disable DMA */
    HSMCI->HSMCI_DMA = 0;
    DMAC->DMAC_CHDR = (1 << DMA_CH);
  }
  else {
    HSMCI->HSMCI_DMA  = HSMCI_DMA_ROPT | HSMCI_DMA_DMAEN;
    HSMCI->HSMCI_BLKR = HSMCI_BLKR_BLKLEN (512) | cnt;
    
    /* Disable DMA channel */
    DMAC->DMAC_CHDR = (1 << DMA_CH);
    while (DMAC->DMAC_CHSR & (1 << DMA_CH));

    DMAC->DMAC_EBCISR;
    DMAC->DMAC_CH_NUM[DMA_CH].DMAC_DSCR  = 0;
    DMAC->DMAC_CH_NUM[DMA_CH].DMAC_CTRLA = DMAC_CTRLA_DST_WIDTH_WORD |
                                           DMAC_CTRLA_SRC_WIDTH_WORD |
                                           DMAC_CTRLA_BTSIZE ((cnt * 512) / 4);
    if (mode == DMA_READ) {
      /* Transfer data from card to memory. */
      DMAC->DMAC_CH_NUM[DMA_CH].DMAC_SADDR = (U32)&HSMCI->HSMCI_RDR;
      DMAC->DMAC_CH_NUM[DMA_CH].DMAC_DADDR = (U32)buf;
      DMAC->DMAC_CH_NUM[DMA_CH].DMAC_CTRLB = DMAC_CTRLB_DST_INCR_INCREMENTING  |
                                             DMAC_CTRLB_SRC_INCR_FIXED         |
                                             DMAC_CTRLB_FC_PER2MEM_DMA_FC      |
                                             DMAC_CTRLB_DST_DSCR_FETCH_DISABLE |
                                             DMAC_CTRLB_SRC_DSCR_FETCH_DISABLE ;
      DMAC->DMAC_CH_NUM[DMA_CH].DMAC_CFG   = DMAC_CFG_AHB_PROT (1)|
                                             DMAC_CFG_SOD_ENABLE  |
                                             DMAC_CFG_SRC_H2SEL_HW;
                                             
    }
    else {
      /* Transfer data from memory to card. */
      DMAC->DMAC_CH_NUM[DMA_CH].DMAC_SADDR = (U32)buf;
      DMAC->DMAC_CH_NUM[DMA_CH].DMAC_DADDR = (U32)&HSMCI->HSMCI_TDR;
      DMAC->DMAC_CH_NUM[DMA_CH].DMAC_CTRLB = DMAC_CTRLB_DST_INCR_FIXED         |
                                             DMAC_CTRLB_SRC_INCR_INCREMENTING  |
                                             DMAC_CTRLB_FC_MEM2PER_DMA_FC      |
                                             DMAC_CTRLB_DST_DSCR_FETCH_DISABLE |
                                             DMAC_CTRLB_SRC_DSCR_FETCH_DISABLE ;
      DMAC->DMAC_CH_NUM[DMA_CH].DMAC_CFG   = DMAC_CFG_AHB_PROT (1)|
                                             DMAC_CFG_SOD_ENABLE  |
                                             DMAC_CFG_DST_H2SEL_HW;
    }
    /* Enable DMA channel */
    DMAC->DMAC_CHER = (1 << DMA_CH);
  }
  return (__TRUE);
}


/*--------------------------- CheckMedia ------------------------------------*/

static U32 CheckMedia (void) {
  /* Read CardDetect and WriteProtect SD card socket pins. */
  U32 stat = 0;

  if (!(PIOE->PIO_PDSR & PIO_PE6)) {
    /* Card is inserted (CD=0). */
    stat |= M_INSERTED;
  } 
#if 0
  if (pPIOC->PIO_PDSR & AT91C_PIO_PC10) {
    /* Write Protect switch is active (WP=1). */
    stat |= M_PROTECTED;
  }
#endif
  return (stat);
}

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
