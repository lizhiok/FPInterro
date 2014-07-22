/*----------------------------------------------------------------------------
 *      RL-ARM - FlashFS
 *----------------------------------------------------------------------------
 *      Name:    MCI_SAM4S.c
 *      Purpose: Multimedia Card Interface Driver for Atmel AT91SAM4S
 *      Rev.:    V4.60
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2011 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <File_Config.h>
#include "SAM4S.H"
#include "MCI_SAM4S.h"

/*----------------------------------------------------------------------------
  Memory Card FAT Driver instance definition
   mci0_drv: First SD/MMC drive [M0:]
   mci1_drv: Second SD/MMC drive [M1:]
 *---------------------------------------------------------------------------*/

#define __DRV_ID    mci0_drv
#define __HSMCICLK  96000000            /* HSMCI peripheral clock frequency   */
#define __CPUCLK    96000000            /* CPU clock frequency                */

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

/* Local Variables */
static U8  CmdBusMode;                /* Open Drain mode = 1, Push Pull = 0  */


/*--------------------------- Init ------------------------------------------*/

static BOOL Init (void) {
  /* Initialize MCI interface. */
  uint32_t i;
  
  CmdBusMode = 0;                       /* Cmd line set to open drain mode    */

  /* Enable PIOA, MCI and DMA clock. */
  PMC->PMC_WPMR  = PMC_WPDIS_KEY;       /* Disable PMC write protection       */
  PMC->PMC_PCER0 = PMC_PCER0_PID11 |    /* Parallel I/O Controller A clock    */
                   PMC_PCER0_PID18 ;    /* Multimedia Card Interface clock    */
  PMC->PMC_WPMR  = PMC_WPEN_KEY;        /* Enable PMC write protection        */
  
  /* Configure I/O pins */
  PIOA->PIO_WPMR = PIO_WPDIS_KEY;       /* Disable PIO write protection       */
  
  PIOA->PIO_IDR  = PIO_IDR_PIN_MSK;
  PIOA->PIO_PDR  = PIO_PDR_PIN_MSK;
  PIOA->PIO_PUER = PIO_PUER_PIN_MSK;

  /* Assign memory card I/O lines to peripheral C (HSMCI) */
  PIOA->PIO_ABCDSR[0] &= ~PIO_ABCDSR_PIN_MSK;
  PIOA->PIO_ABCDSR[1] |=  PIO_ABCDSR_PIN_MSK;
  
  PIOA->PIO_WPMR = PIO_WPEN_KEY;        /* Enable PIO write protection        */
  
  /* Disable HSMCI write protection */
  HSMCI->HSMCI_WPMR = HSMCI_WPDIS_KEY;

  /* Initialize HSMCI peripheral */
  HSMCI->HSMCI_CR   = HSMCI_CR_SWRST;
  HSMCI->HSMCI_CR   = HSMCI_CR_PWSDIS | HSMCI_CR_MCIDIS;
  HSMCI->HSMCI_IDR  = 0xFCFFF1FF;
  HSMCI->HSMCI_DTOR = HSMCI_DTOR_DTOMUL_1048576   | HSMCI_DTOR_DTOCYC(15);
  HSMCI->HSMCI_CSTOR= HSMCI_CSTOR_CSTOMUL_1048576 | HSMCI_CSTOR_CSTOCYC (1);
  HSMCI->HSMCI_CFG  = HSMCI_CFG_HSMODE | HSMCI_CFG_FERRCTRL | HSMCI_CFG_FIFOMODE;
  HSMCI->HSMCI_MR   = HSMCI_MR_PDCMODE | HSMCI_MR_WRPROOF   |
                      HSMCI_MR_RDPROOF | HSMCI_MR_PWSDIV (7);

  /* Enable HSMCI write protection */
  HSMCI->HSMCI_WPMR = HSMCI_WPEN_KEY;
  
  /* Enable HSMCI */
  HSMCI->HSMCI_CR   = HSMCI_CR_MCIEN;

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
  /* Reset the HSMCI peripheral to default state. */

  /* Return memory card I/O pins control to PIO */
  PIOA->PIO_PER = PIO_PER_PIN_MSK;

  /* Power Down the MCI controller. */
  PMC->PMC_PCDR0  = PMC_PCDR0_PID18;
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

  /* Disable HSMCI write protection */
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
  
  /* Enable HSMCI write protection */
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
  
  /* Disable HSMCI write protection */
  HSMCI->HSMCI_WPMR = HSMCI_WPDIS_KEY;
  /* Set clock divider */
  HSMCI->HSMCI_MR  = (HSMCI->HSMCI_MR & ~HSMCI_MR_CLKDIV_Msk) | div;
  /* Enable HSMCI write protection */
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
  if (resp_type != RESP_SHORT) {
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

  PDC_HSMCI->PERIPH_PTCR = PERIPH_PTCR_RXTEN;
  for (i = DMA_TOUT; i; i--) {
    if (HSMCI->HSMCI_SR & HSMCI_SR_ENDRX) {
      return (__TRUE);
    }
  }
  /* DMA Transfer timeout. */
  return (__FALSE);
}


/*--------------------------- WriteBlock ------------------------------------*/
  
static BOOL WriteBlock (U32 bl, U8 *buf, U32 cnt) {
  /* Write a cnt number of 512 byte blocks to Flash Card. */
  U32 i;

  PDC_HSMCI->PERIPH_PTCR = PERIPH_PTCR_TXTEN;
  for (i = DMA_TOUT; i; i--) {
    if (HSMCI->HSMCI_SR & HSMCI_SR_BLKE) {
      return (__TRUE);
    }
  }
  /* DMA Transfer timeout. */
  return (__FALSE);
}


/*--------------------------- SetDma ----------------------------------------*/

static BOOL SetDma (U32 mode, U8 *buf, U32 cnt) {
  /* Configure DMA for read or write. */

  HSMCI->HSMCI_BLKR = HSMCI_BLKR_BLKLEN (512) | cnt;

  if (mode == DMA_READ) {
    /* Transfer data from card to memory. */
    PDC_HSMCI->PERIPH_PTCR = PERIPH_PTCR_RXTDIS;
    PDC_HSMCI->PERIPH_RPR  = (uint32_t)buf;
    PDC_HSMCI->PERIPH_RCR  = (cnt * 512) / 4;
    PDC_HSMCI->PERIPH_RNCR = 0;
  }
  else {
    /* Transfer data from memory to card. */
    PDC_HSMCI->PERIPH_PTCR = PERIPH_PTCR_TXTDIS;
    PDC_HSMCI->PERIPH_TPR  = (uint32_t)buf;
    PDC_HSMCI->PERIPH_TCR  = (cnt * 512) / 4;
    PDC_HSMCI->PERIPH_TNCR = 0;
  }
  return (__TRUE);
}


/*--------------------------- CheckMedia ------------------------------------*/

static U32 CheckMedia (void) {
  /* Read CardDetect and WriteProtect SD card socket pins. */
  U32 stat = 0;

  if (!(PIOA->PIO_PDSR & PIO_PDSR_P6)) {
    /* Card is inserted (CD=0). */
    stat |= M_INSERTED;
  } 
#if 0
  if (PIOA->PIO_PDSR & PIO_PDSR_P7) {
    /* Write Protect switch is active (WP=1). */
    stat |= M_PROTECTED;
  }
#endif
  return (stat);
}

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
