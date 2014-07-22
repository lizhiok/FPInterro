/*----------------------------------------------------------------------------
 *      RL-ARM - FlashFS
 *----------------------------------------------------------------------------
 *      Name:    MCI_SAM9260.c
 *      Purpose: Multimedia Card Interface Driver for Atmel AT91SAM9260
 *      Rev.:    V4.54
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2011 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <File_Config.h>
#include <AT91SAM9260.H>

/*----------------------------------------------------------------------------
  Memory Card FAT Driver instance definition
   mci0_drv: First SD/MMC drive [M0:]
   mci1_drv: Second SD/MMC drive [M1:]
 *---------------------------------------------------------------------------*/

#define __DRV_ID  mci0_drv
#define __MCICLK  96109714
#define __CPUCLK  96109714

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
#define WAIT_CNT(ck,us)   ((ck/5000000)*us)

/* Absolute IO access macros */
#define pPMC      AT91C_BASE_PMC
#define pPIOA     AT91C_BASE_PIOA
#define pMCI      AT91C_BASE_MCI

/* Local Variables */
static U8  od_mode;                   /* Open Drain mode = 1, Push Pull = 0  */

/* Local Functions */
static void DmaStop (void);

/*--------------------------- Init ------------------------------------------*/

static BOOL Init (void) {
  /* Initialize MCI interface. */

  /* Enable PIOA and MCI clock. */
  pPMC->PMC_PCER   = (1 << AT91C_ID_PIOA) | (1 << AT91C_ID_MCI);

  /* Enable MCI Pins. */
  pPIOA->PIO_IDR   =
  pPIOA->PIO_PDR   = 
  pPIOA->PIO_PPUER = AT91C_PA0_MCDB0 | AT91C_PA1_MCCDB | AT91C_PA3_MCDB3 |
                     AT91C_PA4_MCDB2 | AT91C_PA5_MCDB1 | AT91C_PA8_MCCK;
  pPIOA->PIO_PPUDR = AT91C_PA8_MCCK;
  pPIOA->PIO_ASR   = AT91C_PA8_MCCK;
  pPIOA->PIO_BSR   = AT91C_PA0_MCDB0 | AT91C_PA1_MCCDB | AT91C_PA3_MCDB3 |
                     AT91C_PA4_MCDB2 | AT91C_PA5_MCDB1;

  /* Initialize MCI, clear pending interrupts. */
  pMCI->MCI_CR     = AT91C_MCI_SWRST;
  pMCI->MCI_CR     = AT91C_MCI_PWSDIS | AT91C_MCI_MCIDIS;
  pMCI->MCI_IDR    = 0xFFFFFFFF;
  pMCI->MCI_DTOR   = AT91C_MCI_DTOMUL | AT91C_MCI_DTOCYC;
  pMCI->MCI_SDCR   = 0x00000001;

  /* Enable MCI. */
  od_mode          = 0;
  pMCI->MCI_MR     = (512 << 16)       | AT91C_MCI_PDCMODE | 
                     AT91C_MCI_WRPROOF | AT91C_MCI_RDPROOF |
                     AT91C_MCI_PWSDIV  | 0xFF;

  pMCI->MCI_CR     = AT91C_MCI_MCIEN;
  pMCI->MCI_CMDR   = AT91C_MCI_SPCMD_INIT | AT91C_MCI_OPDCMD;
  while (!(pMCI->MCI_SR & AT91C_MCI_CMDRDY));

  return (__TRUE);
}


/*--------------------------- UnInit ----------------------------------------*/

static BOOL UnInit (void) {
  /* Reset the MCI peripheral to default state. */

  /* Disable MCI pins. */
  pPIOA->PIO_PPUDR = 
  pPIOA->PIO_ASR   = AT91C_PA0_MCDB0 | AT91C_PA1_MCCDB | AT91C_PA3_MCDB3 |
                     AT91C_PA4_MCDB2 | AT91C_PA5_MCDB1 | AT91C_PA8_MCCK;
  pMCI->MCI_CR     = AT91C_MCI_PWSEN | AT91C_MCI_MCIDIS;

  /* Power Down the MCI controller. */
  pPMC->PMC_PCDR   = (1 << AT91C_ID_MCI);
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
      pMCI->MCI_SDCR &= ~AT91C_MCI_SCDBUS;
      return (__TRUE);

    case 4:
      pMCI->MCI_SDCR |=  AT91C_MCI_SCDBUS;
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
  pMCI->MCI_MR  = (pMCI->MCI_MR & ~AT91C_MCI_CLKDIV) | div;
  return (__TRUE);
}


/*--------------------------- Command ---------------------------------------*/

static BOOL Command (U8 cmd, U32 arg, U32 resp_type, U32 *rp) {
  /* Send a Command to Flash card and get a Response. */
  U32 i,cmdval,stat;

  cmd    &= AT91C_MCI_CMDNB;
  cmdval  = AT91C_MCI_MAXLAT | ((od_mode & 1) << 11) | cmd;
  switch (cmd) {
    case READ_BLOCK:
      cmdval |= AT91C_MCI_TRCMD_START | AT91C_MCI_TRDIR;
      break;
    case READ_MULT_BLOCK:
      cmdval |= AT91C_MCI_TRCMD_START | AT91C_MCI_TRDIR | AT91C_MCI_TRTYP_MULTIPLE;
      break;
    case WRITE_BLOCK:
      cmdval |= AT91C_MCI_TRCMD_START;
      break;
    case WRITE_MULT_BLOCK:
      cmdval |= AT91C_MCI_TRCMD_START | AT91C_MCI_TRTYP_MULTIPLE;
      break;
    case STOP_TRANS:
      cmdval |= AT91C_MCI_TRCMD_STOP;
      break;
  }
  switch (resp_type) {
    case RESP_SHORT:
      cmdval |= AT91C_MCI_RSPTYP_48;
      break;
    case RESP_LONG:
      cmdval |= AT91C_MCI_RSPTYP_136;
      break;
  }
  /* Send the command. */
  pMCI->MCI_CR   = AT91C_MCI_MCIEN;
  pMCI->MCI_ARGR = arg;
  pMCI->MCI_CMDR = cmdval;

  if (resp_type == RESP_NONE) {
    /* Wait until command finished. */
    while (!(pMCI->MCI_SR & AT91C_MCI_CMDRDY));
    return (__TRUE);
  }

  for (i = DMA_TOUT; i; i--) {
    stat = pMCI->MCI_SR;
    if (stat & AT91C_MCI_RTOE) {
      return (__FALSE);
    }
    if (stat & AT91C_MCI_RCRCE) {
      if ((cmd == SEND_OP_COND)      ||
          (cmd == SEND_APP_OP_COND)  ||
          (cmd == STOP_TRANS)) {
        break;
      }
      return (__FALSE);
    }
    if (stat & AT91C_MCI_CMDRDY) {
      break;
    }
  }
  if (i == 0) return (__FALSE);

  /* Read MCI response registers */
  rp[0] = pMCI->MCI_RSPR[0];
  if (resp_type == RESP_LONG) {
    rp[1] = pMCI->MCI_RSPR[1];
    rp[2] = pMCI->MCI_RSPR[2];
    rp[3] = pMCI->MCI_RSPR[3];
  }
  return (__TRUE);
}


/*--------------------------- ReadBlock -------------------------------------*/

static BOOL ReadBlock (U32 bl, U8 *buf, U32 cnt) {
  /* Read one or more 512 byte blocks from Flash Card. */
  U32 i;
  
  /* Start DMA Peripheral to Memory transfer. */
  pMCI->MCI_PTCR = AT91C_PDC_RXTEN;

  for (i = DMA_TOUT; i; i--) {
    if (pMCI->MCI_SR & AT91C_MCI_ENDRX) {
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
  U32 i,stat;
  
   /* Start DMA Memory to Peripheral transfer. */
   pMCI->MCI_PTCR = AT91C_PDC_TXTEN;

  for (i = DMA_TOUT; i; i--) {
    stat = pMCI->MCI_SR;
    if (cnt > 1) {
      if (stat & AT91C_MCI_BLKE) {
        /* Multiple block Data transfer finished. */
        DmaStop ();
        return (__TRUE);
      }
    }
    else {
      if (stat & AT91C_MCI_TXBUFE) {
        /* Single block Data transfer finished. */
        DmaStop ();
        return (__TRUE);
      }
    }
  }
  /* DMA Transfer timeout. */
  DmaStop ();
  return (__FALSE);
}


/*--------------------------- SetDma ----------------------------------------*/

static BOOL SetDma (U32 mode, U8 *buf, U32 cnt) {
  /* Configure DMA for read or write. */

  pMCI->MCI_CR    = AT91C_MCI_MCIDIS;
  pMCI->MCI_PTCR  = AT91C_PDC_TXTDIS  | AT91C_PDC_RXTDIS;
  pMCI->MCI_BLKR  = (512 << 16) | cnt;
  if (mode == DMA_READ) {
    /* Transfer data from card to memory. */
    pMCI->MCI_RPR  = (U32)buf;
    pMCI->MCI_RCR  = (512 >> 2) * cnt;
  }
  else {
    /* Transfer data from memory to card. */
    pMCI->MCI_TPR  = (U32)buf;
    pMCI->MCI_TCR  = (512 >> 2) * cnt;
  }
  return (__TRUE);
}


/*--------------------------- DmaStop ---------------------------------------*/

static void DmaStop (void) {
  /* Stop the DMA. */
  pMCI->MCI_PTCR = AT91C_PDC_TXTDIS | AT91C_PDC_RXTDIS;
}


/*--------------------------- CheckMedia ------------------------------------*/

static U32 CheckMedia (void) {
  /* Read CardDetect and WriteProtect SD card socket pins. */
  U32 stat = 0;

#if 0
  if (!(pPIOA->PIO_PDSR & AT91C_PIO_PA10)) {
    /* Card is inserted (CD=0). */
    stat |= M_INSERTED;
  } 
  if (pPIOA->PIO_PDSR & AT91C_PIO_PA11) {
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
