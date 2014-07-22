/*----------------------------------------------------------------------------
 *      RL-ARM - FlashFS
 *----------------------------------------------------------------------------
 *      Name:    MCI_SAM3U.c
 *      Purpose: Multimedia Card Interface Driver for Atmel AT91SAM3U
 *      Rev.:    V4.20
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2011 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <File_Config.h>
#include "MCI_SAM3U.h"
#include <SAM3U.H>

/*----------------------------------------------------------------------------
  Memory Card FAT Driver instance definition
   mci0_drv: First SD/MMC drive [M0:]
   mci1_drv: Second SD/MMC drive [M1:]
 *---------------------------------------------------------------------------*/

#define __DRV_ID  mci0_drv
#define __MCICLK  84000000
#define __CPUCLK  84000000

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
#define DMA_TOUT          20000000
#define WAIT_CNT(ck,us)   ((ck/4000000)*us)

/* DMA channel used for MCI */
#define _CHAN       0

/* Local Variables */
static U8  od_mode;                   /* Open Drain mode = 1, Push Pull = 0  */

/* Local Functions */
static void DmaStart (void);
static void DmaStop (void);


/*--------------------------- Init ------------------------------------------*/

static BOOL Init (void) {
  U32 val;
  /* Initialize MCI interface. */

  /* Enable PIOA, MCI and DMA clock. */
  PMC->PMC_WPMR  = PMC_WPDIS_KEY;                   /* Disable write protect  */
  PMC->PMC_PCER0 = (1 << ID_PIOA) | (1 << ID_HSMCI) | (1 << ID_DMAC);
  PMC->PMC_WPMR  = PMC_WPEN_KEY;                    /* Enable write protect   */

  /* Enable MCI Pins. */
  val = PIO_PA3A_MCCK  | PIO_PA4A_MCCDA | PIO_PA5A_MCDA0 |
        PIO_PA6A_MCDA1 | PIO_PA7A_MCDA2 | PIO_PA8A_MCDA3; 
  PIOA->PIO_IDR  = val;
  PIOA->PIO_PDR  = val;
  PIOA->PIO_PUER = val;  
  PIOA->PIO_PUDR =  PIO_PA3A_MCCK;

  PIOA->PIO_ABSR &= PIO_PA3A_MCCK  | PIO_PA4A_MCCDA | PIO_PA5A_MCDA0 |
                    PIO_PA6A_MCDA1 | PIO_PA7A_MCDA2 | PIO_PA8A_MCDA3;


 /* Enable CD pin. */
  PIOA->PIO_IDR = PIO_PA25;
  PIOA->PIO_ODR = PIO_PA25; 
  PIOA->PIO_PER = PIO_PA25;

  /* Initialize DMA channel 0 */
  DMAC->DMAC_CHDR   = (1 << _CHAN);
  DMAC->DMAC_EBCIDR = 0x000F0F0F;
  DMAC->DMAC_EN     = DMAC_EN_ENABLE;

  /* Initialize MCI. */
  HSMCI->HSMCI_CR   = HSMCI_CR_SWRST;
  HSMCI->HSMCI_CR   = HSMCI_CR_PWSDIS   | HSMCI_CR_MCIDIS;
  HSMCI->HSMCI_IDR  = 0xFFFFFFFF;
  HSMCI->HSMCI_DTOR = HSMCI_DTOR_DTOMUL_Msk | HSMCI_DTOR_DTOCYC_Msk;
  HSMCI->HSMCI_DMA  = 0;
  HSMCI->HSMCI_CFG  = (0x1 << 0) | (0x0 << 4);
  HSMCI->HSMCI_SDCR = 0x00000000;

  /* Enable MCI. */
  od_mode           = 0;
  HSMCI->HSMCI_MR   = (512 << 16)          | 
                      HSMCI_MR_WRPROOF     | HSMCI_MR_RDPROOF |
                      HSMCI_MR_PWSDIV_Msk  | 0xFF;
  HSMCI->HSMCI_CR   = HSMCI_CR_MCIEN;

  HSMCI->HSMCI_CMDR = MCI_SPCMD_INIT | HSMCI_CMDR_OPDCMD;
  while (!(HSMCI->HSMCI_SR & HSMCI_SR_CMDRDY));

  return (__TRUE);
}


/*--------------------------- UnInit ----------------------------------------*/

static BOOL UnInit (void) {
  /* Reset the MCI peripheral to default state. */

  /* Disable MCI pins. */
  PIOA->PIO_PUDR  = PIO_PA3A_MCCK  | PIO_PA4A_MCCDA | PIO_PA5A_MCDA0 |
                    PIO_PA6A_MCDA1 | PIO_PA7A_MCDA2 | PIO_PA8A_MCDA3;
  HSMCI->HSMCI_CR = HSMCI_CR_PWSEN | HSMCI_CR_MCIDIS;

  /* Power Down the MCI controller. */
  PMC->PMC_WPMR  = PMC_WPDIS_KEY;                   /* Disable write protect  */
  PMC->PMC_PCDR0  = (1 << ID_HSMCI);
  PMC->PMC_WPMR  = PMC_WPEN_KEY;                    /* Enable write protect   */
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
      od_mode = 1;
      return (__TRUE);

    case BUS_PUSH_PULL:
      od_mode = 0;
      return (__TRUE);
  }
  return (__FALSE);
}


/*--------------------------- BusWidth --------------------------------------*/

static BOOL BusWidth (U32 width) {
  /* Set MCI Bus width. */

  switch (width) {
    case 1:
      HSMCI->HSMCI_SDCR &= ~HSMCI_SDCR_SDCBUS_Msk;
      return (__TRUE);

    case 4:
      HSMCI->HSMCI_SDCR |= (0x2 << 6);
      return (__TRUE);
  }
  return (__FALSE);
}


/*--------------------------- BusSpeed --------------------------------------*/

static BOOL BusSpeed (U32 kbaud) {
  /* Set a MCI clock speed to desired value. */
  U32 div;

  /* baud = MCICLK / (2 x (div + 1)) */
  div = (__MCICLK/2000 + kbaud - 1) / kbaud;
  if (div > 0)    div--;
  if (div > 0xFF) div = 0xFF;
  HSMCI->HSMCI_MR  = (HSMCI->HSMCI_MR & ~HSMCI_MR_CLKDIV_Msk) | div;
  return (__TRUE);
}


/*--------------------------- Command ---------------------------------------*/

static BOOL Command (U8 cmd, U32 arg, U32 resp_type, U32 *rp) {
  /* Send a Command to Flash card and get a Response. */
  U32 i,cmdval,stat;

  cmd   &= HSMCI_CMDR_CMDNB_Msk;
  cmdval = HSMCI_CMDR_MAXLAT | ((od_mode & 1) << 11) | cmd;
  switch (cmd) {
    case READ_BLOCK:
      cmdval |= MCI_TRCMD_START | MCI_TRDIR_READ;
      DmaStart ();
      break;
    case READ_MULT_BLOCK:
      cmdval |= MCI_TRCMD_START | MCI_TRDIR_READ | MCI_TRTYP_MULTIPLE;
      DmaStart ();
      break;
    case WRITE_BLOCK:
      cmdval |= MCI_TRCMD_START;
      DmaStart ();
      break;
    case WRITE_MULT_BLOCK:
      cmdval |= MCI_TRCMD_START | MCI_TRTYP_MULTIPLE;
      DmaStart ();
      break;
    case STOP_TRANS:
      cmdval |= MCI_TRCMD_STOP;
      break;
  }
  switch (resp_type) {
    case RESP_SHORT:
      cmdval |= MCI_RSPTYP_48;
      break;
    case RESP_LONG:
      cmdval |= MCI_RSPTYP_136;
      break;
  }

  HSMCI->HSMCI_CR   = HSMCI_CR_MCIEN;
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
  U32 i;

  for (i = DMA_TOUT; i; i--) {
    if (HSMCI->HSMCI_SR & HSMCI_SR_DMADONE) {
      /* Data transfer finished. */
      DmaStop ();
      return (__TRUE);
    }
  }
  /* DMA Transfer timeout. */
  DmaStop ();
  return (__FALSE);
}


/*--------------------------- WriteBlock ------------------------------------*/
  
static BOOL WriteBlock (U32 bl, U8 *buf, U32 cnt) {
  /* Write a cnt number of 512 byte blocks to Flash Card. */
  U32 i;

  for (i = DMA_TOUT; i; i--) {
    if (HSMCI->HSMCI_SR & HSMCI_SR_DMADONE) {
      /* Data transfer finished. */
      DmaStop ();
      return (__TRUE);
    }
  }
  /* DMA Transfer timeout. */
  DmaStop ();
  return (__FALSE);
}


/*--------------------------- SetDma ----------------------------------------*/

static BOOL SetDma (U32 mode, U8 *buf, U32 cnt) {
  /* Configure DMA for read or write. */

  HSMCI->HSMCI_CR   = HSMCI_CR_MCIDIS;
  DMAC->DMAC_CHDR   = (1 << _CHAN);
  HSMCI->HSMCI_BLKR = (512 << 16) | cnt;
  DMAC->DMAC_CH_NUM[_CHAN].DMAC_CTRLA = (0x2 << 28) | (0x2 << 24) |
                                        (512 >> 2) * cnt;
  if (mode == DMA_READ) {
    /* Transfer data from card to memory. */
    DMAC->DMAC_CH_NUM[_CHAN].DMAC_SADDR = (U32)HSMCI_FIFO_ADR;
    DMAC->DMAC_CH_NUM[_CHAN].DMAC_DADDR = (U32)buf;
    DMAC->DMAC_CH_NUM[_CHAN].DMAC_CTRLB = (0x1 << 16) | (0x1 << 20) |
                                          (0x2 << 21) | (0x2 << 24);
    DMAC->DMAC_CH_NUM[_CHAN].DMAC_CFG   = (0x1 <<  9);
  }
  else {
    /* Transfer data from memory to card. */
    DMAC->DMAC_CH_NUM[_CHAN].DMAC_SADDR = (U32)buf;
    DMAC->DMAC_CH_NUM[_CHAN].DMAC_DADDR = (U32)HSMCI_FIFO_ADR;
    DMAC->DMAC_CH_NUM[_CHAN].DMAC_CTRLB = (0x1 << 16) | (0x1 << 20) |
                                          (0x1 << 21) | (0x2 << 28);
    DMAC->DMAC_CH_NUM[_CHAN].DMAC_CFG   = (0x1 << 13);
  }
  return (__TRUE);
}


/*--------------------------- DmaStart --------------------------------------*/

static void DmaStart (void) {
  /* Start DMA for read or write. */

  HSMCI->HSMCI_DMA |= HSMCI_DMA_DMAEN;
  DMAC->DMAC_CHER   = (1 << _CHAN);
}


/*--------------------------- DmaStop ---------------------------------------*/

static void DmaStop (void) {
  /* Stop the DMA. */

  DMAC->DMAC_CHDR   = (1 << _CHAN);
  HSMCI->HSMCI_DMA &= ~HSMCI_DMA_DMAEN;
}

/*--------------------------- CheckMedia ------------------------------------*/

static U32 CheckMedia (void) {
  /* Read CardDetect and WriteProtect SD card socket pins. */
  U32 stat = 0;

  if (!(PIOA->PIO_PDSR & PIO_PA25)) {
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
