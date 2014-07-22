/*----------------------------------------------------------------------------
 *      RL-ARM - FlashFS
 *----------------------------------------------------------------------------
 *      Name:    NAND_SAM3X.c
 *      Purpose: NAND Flash Interface Driver for Atmel ATSAM3X
 *      Rev.:    V4.20
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <File_Config.h>
#include <string.h>
#include <sam3xa.h>
#include "NAND_SAM3x.h"

//-------- <<< Use Configuration Wizard in Context Menu >>> -----------------
// <o0>Chip select used for NAND <0x00=> NCS0
//                               <0x01=> NCS1
//                               <0x02=> NCS2
//                               <0x03=> NCS3
//                               <0x04=> NCS4
//                               <0x05=> NCS5
//                               <0x06=> NCS6
//                               <0x07=> NCS7
//
//
// <o1>Device bus width <0x00=>  8-bit
//                      <0x01=> 16-bit
// <e2>Enable DMA transfer
//   <o3>Selected DMA Channel <0-3:0>
// </e2>

#define NAND_HW_CS_0  0x00
#define NAND_HW_BUS   0x00
#define NAND_DMA_EN      1
#define NAND_DMA_CH      0

//------------- <<< end of configuration section >>> -----------------------
#define NAND_HW_ECC   0x00
#define NAND_CHIP_CNT    1

/* SMC data bus */
#define SMC_DATA8         *((U8  *)(0x60000000 + 0x1000000 * NAND_HW_CS_0))
#define SMC_DATA16        *((U16 *)(0x60000000 + 0x1000000 * NAND_HW_CS_0))


static const U8 CsCfg[NAND_CHIP_CNT] = {
  NAND_HW_CS_0, /* Chip select for instance 0 */
};


/*----------------------------------------------------------------------------
  NAND Device Driver Control Block
 *----------------------------------------------------------------------------*/
const NAND_DRV nand0_drv = {
  Init,
  UnInit,
  PageRead,
  PageWrite,
  BlockErase,
};


/*----------------------------------------------------------------------------
  Setup Page Data Layout
 *----------------------------------------------------------------------------*/
void SetPageLayout (NAND_DRV_CFG *cfg) {
  /* Define spare area layout */
  cfg->PgLay->Pos_LSN = 0;
  cfg->PgLay->Pos_COR = 4;
  cfg->PgLay->Pos_BBM = 5;
  cfg->PgLay->Pos_ECC = 6;

  /* Define page organization */
  cfg->PgLay->SectInc  =  512;
  cfg->PgLay->SpareOfs =  512 * (cfg->PageSize / 512);
  cfg->PgLay->SpareInc =   16;
}

/*----------------------------------------------------------------------------
        Enable Automatic Chip Select
 *----------------------------------------------------------------------------*/
static void EnableAutoCS (U32 cs) {
  switch (cs) {
    case SAM3X_NAND_CS0:
      /* Config for PA6 == NCS0 */
      PIOA->PIO_PDR   =  PIO_PA6;
      PIOA->PIO_ABSR |=  PIO_PA6;
      break;
    case SAM3X_NAND_CS1:
      /* Config for PA7 == NCS1 */
      PIOA->PIO_PDR   =  PIO_PA7;
      PIOA->PIO_ABSR |=  PIO_PA7;
      break;
    case SAM3X_NAND_CS2:
      /* Config for PB24 == NCS2 */
      PIOB->PIO_PDR   =  PIO_PB24;
      PIOB->PIO_ABSR |=  PIO_PB24;
      break;
    case SAM3X_NAND_CS3:
      /* Config for PB27 == NCS3 */
      PIOB->PIO_PDR   =  PIO_PB27;
      PIOB->PIO_ABSR &= ~PIO_PB27;
      break;
    case SAM3X_NAND_CS4:
      /* Config for PE5 == NCS4 */
      PIOE->PIO_PDR   =  PIO_PE5;
      PIOE->PIO_ABSR &= ~PIO_PE5;
      break;
    case SAM3X_NAND_CS5:
      /* Config for PE6 == NCS5 */
      PIOE->PIO_PDR   =  PIO_PE6;
      PIOE->PIO_ABSR &= ~PIO_PE6;
      break;
    case SAM3X_NAND_CS6:
      /* Config for PE18 == NCS6 */
      PIOE->PIO_PDR   =  PIO_PE18;
      PIOE->PIO_ABSR |=  PIO_PE18;
      break;
    case SAM3X_NAND_CS7:
      /* Config for PE27 == NCS7 */
      PIOE->PIO_PDR   =  PIO_PE27;
      PIOE->PIO_ABSR &= ~PIO_PE27;
      break;
  }
}


/*-----------------------------------------------------------------------------
 *      Read page
 *  row  = Page address
 *  *buf = Pointer to data buffer
 *  *cfg = Pointer to configuration structure
 *
 *  Return: RTV_NOERR         - Page read successful
 *          ERR_NAND_HW_TOUT  - Hardware transfer timeout
 *          ERR_NAND_DMA_TOUT - DMA transfer timeout
 *          ERR_ECC_COR       - ECC corrected the data within page
 *          ERR_ECC_UNCOR     - ECC was not able to correct the data
 *----------------------------------------------------------------------------*/
static U32 PageRead (U32 row, U8 *buf, NAND_DRV_CFG *cfg) {
  U32 i, cs, cyc, ecc, cmdAddr;

  /* Set address */
  cyc = cfg->AddrCycles;
  if (cyc == 5) {
    SMC->SMC_ADDR = 0;
  }

  cs = CsCfg[cfg->DrvInst];

  /* Set and execute read command */
  cmdAddr = NFC_CMD_BASE | (NAND_CMD_READ1ST <<  2)| /* Read 1st Cycle      */
                           (NAND_CMD_READ2ND << 10)| /* Read 2nd Cycle      */
                           (1 << 18)               | /* Execute CMD2        */ 
                           (cyc              << 19)| /* Set address cycles  */
                           (cs               << 22)| /* Set chip select     */
                           (1 << 25)               ; /* Auto read after cmd */

  SMC_WRITE_CMD (cmdAddr, row << 8);
  
  /* Wait until transfer done: Tout ~10msec @ 100MHz */
  for (i = NAND_TIMEOUT; i; i--) {
    if (SMC->SMC_SR & NFC_XFR_DONE) break;
  }
  if (i == 0) { return ERR_NAND_HW_TOUT; }

  ecc = ECC_NOERR;

  if (NAND_HW_ECC) {
    /* Hardware ECC */
  }

  /* Transfer data */
  if (NAND_DMA_EN) {
    /* Use DMA */
    DMAC->DMAC_CH_NUM[NAND_DMA_CH].DMAC_SADDR = NFC_SRAM_BASE_ADDR;
    DMAC->DMAC_CH_NUM[NAND_DMA_CH].DMAC_DADDR = (U32)buf;
    DMAC->DMAC_CH_NUM[NAND_DMA_CH].DMAC_CTRLA = cfg->PageSize;
    DMAC->DMAC_CHER |= (1 << NAND_DMA_CH);

    /* Wait until transfer done: Tout ~10msec @ 100MHz */
    for (i = NAND_TIMEOUT; i; i--) {
      if (!(DMAC->DMAC_CHSR & (1 << NAND_DMA_CH))) break;
    }
    if (i == 0) { return ERR_NAND_DMA_TOUT; }
  }
  else {
    /* Read Data */
    memcpy (buf, (U8 *)NFC_SRAM_BASE_ADDR, cfg->PageSize);
  }
  
  if (ecc & ECC_UNCORRECTED) {
    return ERR_ECC_UNCOR;
  }
  if (ecc & ECC_CORRECTED) {
    return ERR_ECC_COR;
  }

  return RTV_NOERR;
}

/*-----------------------------------------------------------------------------
 *      Write page
 *  row  = Page address
 *  *buf = Pointer to data buffer
 *  *cfg = Pointer to configuration structure
 *
 *  Return: RTV_NOERR         - Page write successful
 *          ERR_NAND_PROG     - Page write failed
 *          ERR_NAND_HW_TOUT  - Hardware transfer timeout
 *          ERR_NAND_DMA_TOUT - DMA transfer timeout 
 *----------------------------------------------------------------------------*/
static U32 PageWrite (U32 row, U8 *buf, NAND_DRV_CFG *cfg) {
  U8 status, *nfcBuf;
  U32 i, cs, spSz, adrCyc, cmdAddr;

  /* Transfer data */
  if (NAND_DMA_EN) {
    /* Use DMA */
    DMAC->DMAC_CH_NUM[NAND_DMA_CH].DMAC_SADDR = (U32)buf;
    DMAC->DMAC_CH_NUM[NAND_DMA_CH].DMAC_DADDR = NFC_SRAM_BASE_ADDR;
    DMAC->DMAC_CH_NUM[NAND_DMA_CH].DMAC_CTRLA = cfg->PageSize;
    DMAC->DMAC_CHER |= (1 << NAND_DMA_CH);
  
    /* Wait until transfer done: Tout ~10msec @ 100MHz */
    for (i = NAND_TIMEOUT; i; i--) {
      if (!(DMAC->DMAC_CHSR & (1 << NAND_DMA_CH))) break;
    }
    if (i == 0) { return ERR_NAND_DMA_TOUT; }
  }
  else {    
    /* Write data to NFC SRAM */
    memcpy ((U8 *)NFC_SRAM_BASE_ADDR, buf, cfg->PageSize);
  }

  /* Set address */
  adrCyc = cfg->AddrCycles;

  if (adrCyc == 5) {
    SMC->SMC_ADDR = 0;
  }
  
  cs = CsCfg[cfg->DrvInst];

  /* Set and execute write command */
  cmdAddr = NFC_CMD_BASE | (NAND_CMD_PROG1ST <<  2)| /* Pg Program 1st Cycle */
                           (adrCyc           << 19)| /* Set address cycles   */
                           (cs               << 22)| /* Set chip select      */
                           (1 << 25)               | /* Auto write after cmd */
                           (1 << 26)               ; /* NFC writes data      */
  SMC_WRITE_CMD (cmdAddr, row << 8);
  
  /* Wait until transfer done: Tout ~10msec @ 100MHz */
  for (i = NAND_TIMEOUT; i; i--) {
    if (SMC->SMC_SR & NFC_XFR_DONE) break;
  }
  if (i == 0) { return ERR_NAND_HW_TOUT; }

  if (NAND_HW_ECC) {
    /* Hardware ECC */  
  }

  /* Write spare area */
  nfcBuf = (U8 *)(NFC_SRAM_BASE_ADDR + cfg->PgLay->SpareOfs);

  spSz = (cfg->PageSize / 512) * 16;

  if (NAND_HW_BUS) {
    for (i = 0; i < spSz; i += 2) {
      SMC_DATA16 = ((U16)nfcBuf[i + 1] << 8) | nfcBuf[i];
    }
  }
  else {
    for (i = 0; i < spSz; i++) {
      SMC_DATA8 = nfcBuf[i];
    }
  }

  /* Send Program2 command */
  cmdAddr = NFC_CMD_BASE | (NAND_CMD_PROG2ND << 2) | (cs << 22);
  SMC_WRITE_CMD (cmdAddr, 0);

  /* Wait Ready/Busy: Tout ~10msec @ 100MHz */
  for (i = NAND_TIMEOUT; i; i--) {
    if (SMC->SMC_SR & NFC_RB_RISE) break;
  }
  if (i == 0) { return ERR_NAND_HW_TOUT; }

  /* Check status */
  cmdAddr = NFC_CMD_BASE | (NAND_CMD_STATUS << 2) | (cs << 22);
  SMC_WRITE_CMD (cmdAddr, 0);
  status = SMC_DATA8;

  if (status & NAND_STAT_FAIL) {
    return ERR_NAND_PROG;
  }
  return RTV_NOERR;
}

/*-----------------------------------------------------------------------------
 *      Erase block
 *  row  = Block address
 *  *cfg = Pointer to configuration structure
 *
 *  Return: RTV_NOERR         - Block erase successful
 *          ERR_NAND_ERASE    - Block erase failed
 *          ERR_NAND_HW_TOUT  - Hardware transfer timeout
 *----------------------------------------------------------------------------*/
static U32 BlockErase (U32 row, NAND_DRV_CFG *cfg) {
  U8 status;
  U32 i, cs, cmdAddr;

  cs = CsCfg[cfg->DrvInst];

  /* Set and execute erase command */
  cmdAddr = NFC_CMD_BASE | (NAND_CMD_ERASE1ST <<  2) | /* Erase 1st Cycle    */
                           (NAND_CMD_ERASE2ND << 10) | /* Erase 2md Cycle    */
                           (1 << 18)                 | /* Execute CMD2       */
                           (3 << 19)                 | /* Set address cycles */
                           (cs                << 22) ; /* Set chip select    */

  SMC_WRITE_CMD (cmdAddr, row);

  /* Wait Ready/Busy: Tout ~10msec @ 100MHz */
  for (i = NAND_TIMEOUT; i; i--) {
    if (SMC->SMC_SR & NFC_RB_RISE) break;
  }
  if (i == 0) { return ERR_NAND_HW_TOUT; }
  
  /* Check status */
  cmdAddr = NFC_CMD_BASE | (NAND_CMD_STATUS << 2) | (cs << 22);
  SMC_WRITE_CMD (cmdAddr, 0);
  status = SMC_DATA8;

  if (status & NAND_STAT_FAIL) {
    return ERR_NAND_ERASE; 
  }
  return RTV_NOERR;
}

/*-----------------------------------------------------------------------------
 *      Uninitialise NAND flash driver
 *  *cfg = Pointer to configuration structure
 *
 *  Return: RTV_NOERR         - UnInit successful
 *----------------------------------------------------------------------------*/
static U32 UnInit (NAND_DRV_CFG *cfg) {
  /* PIO Write Protect Disable */
  PIOB->PIO_WPMR = PIO_WPDIS_KEY;

  /* Release NFC peripheral pins to PIO */
  PIOC->PIO_PER   =  0x001BFFFC;                    // D0..D15, NANDOE, NANDWE
  PIOD->PIO_PER   =  0x00000300;                    // NANDALE, NANDCLE
  PIOA->PIO_PER   =  0x00000004;                    // NANDRDY
  PIOA->PIO_ABSR &= ~0x00000004;

  return RTV_NOERR;
}

/*-----------------------------------------------------------------------------
 *      Initialise NAND flash driver
 *
 *  *cfg = Pointer to configuration structure
 *
 *  Return: RTV_NOERR         - NAND Flash Initialisation successful
 *          ERR_NAND_HW_TOUT  - NAND Flash Reset Command failed
 *          ERR_NAND_DMA_TOUT - DMA Configuration failed
 *----------------------------------------------------------------------------*/
static U32 Init (NAND_DRV_CFG *cfg) {
  U8 pgSz, status;
  U32 i, cs, inst, cmdAddr;

  inst = cfg->DrvInst;
  if (inst >= NAND_CHIP_CNT) return ERR_INVALID_PARAM;
  
  /* Set hardware parameters */
  pgSz = 0x00;
  switch (cfg->PageSize) {
    case 528:
      break;
    case 2112:
      pgSz = 0x02;
      break;
    case 4224:
      pgSz = 0x03;
      break;
    default:
      return ERR_NAND_UNSUPPORTED;
  }

  /* Setup Page Data Layout */
  SetPageLayout (cfg);

  /* Configure Power Management Controller: Enable SMC and PIO clocks */
  PMC->PMC_WPMR  = PMC_WPDIS_KEY;                   /* Disable write protect  */
  PMC->PMC_PCER0 = (1 << ID_SMC) | (1 << ID_PIOA) | (1 << ID_PIOC)
                                 | (1 << ID_PIOD) | (1 << ID_PIOE);
  PMC->PMC_WPMR  = PMC_WPEN_KEY;                    /* Enable write protect   */

  /* PIO Write Protect Disable */
  PIOB->PIO_WPMR = PIO_WPDIS_KEY;

  /* Configure PIO Controller: Assign SMC to peripheral function */
  PIOA->PIO_PDR   =  0x00000004;                    // NANDRDY
  PIOA->PIO_ABSR |=  0x00000004;
  PIOC->PIO_PDR   =  0x001BFFFC;                    // D0..D15, NANDOE, NANDWE
  PIOC->PIO_ABSR &= ~0x001BFFFC;
  PIOD->PIO_PDR   =  0x00000300;                    // NANDALE, NANDCLE
  PIOD->PIO_ABSR &= ~0x00000300;

  /* Disable SMC Write Protection */
  SMC->SMC_WPCR = SMC_WPDIS_KEY;

  /* Enable NAND Flash Controller */
  SMC->SMC_CTRL = 1;

  /* Set NFC Configuration Register */
  SMC->SMC_CFG = (pgSz & 0x03) | (1UL <<  9) | // Set page size, Read Spare Area
                                 (0   << 12) | // Rising edge is detected 
                                 (1UL << 16) | // Set Data Timeout Cycle Number
                                 (7UL << 20) ; // Set Data Timeout Multiplier
  /* Select SRAM Bank 0 */
  SMC->SMC_BANK = 0;  

  /* Enable Auto Chip Select Control */
  cs = CsCfg[inst];

  EnableAutoCS(cs);

  /* Configure timing and signals */
  SMC->SMC_CS_NUMBER[cs].SMC_SETUP =   (0 <<  0) | // Write Enable Setup Length
                                       (1 <<  8) | // CS Setup length in Write Access
                                       (0 << 16) | // Read Enable Setup Length
                                       (1 << 24) ; // CS Setup length in Read Access
  
  SMC->SMC_CS_NUMBER[cs].SMC_PULSE =   (3 <<  0) | // Write Enable Pulse Length
                                       (4 <<  8) | // CS Pulse length in Write Access
                                       (3 << 16) | // Read Enable Pulse Length
                                       (4 << 24) ; // CS Pulse length in Read Access
  
  SMC->SMC_CS_NUMBER[cs].SMC_CYCLE =   (4 <<  0) | // Total Write Cycle Length
                                       (7 << 16) ; // Total Read Cycle Length
  
  SMC->SMC_CS_NUMBER[cs].SMC_TIMINGS = (1 <<  0) | // CLE to RE Low Delay
                                       (7 <<  4) | // ALE to Data Start
                                       (1 <<  8) | // ALE to RE Low Delay
                                       (1 << 16) | // Ready to RE Low Delay
                                       (7 << 24) | // WE High to RE Busy
                             ((cs & 0x03) << 28) | // R/B Line Selection
                                      (1U << 31) ; // NAND Flash Selection

  SMC->SMC_CS_NUMBER[cs].SMC_MODE =    (1 <<  0) | // read is controlled by NRD signal
                                       (1 <<  1) ; // write is controlled by NWE signal

  if (NAND_HW_BUS == NAND_BUS_W16) {
  SMC->SMC_CS_NUMBER[cs].SMC_MODE |=   (0 <<  8) | // NCS, NWE, NRD
                                       (1 << 12) ; // 16-bit bus width
  }

  if (NAND_HW_ECC) {
    /* Configure Hardware ECC module */
    if (NAND_HW_BUS == NAND_BUS_W16) {
      /* 1 bit correction for a page - only mode for 16 bit devices */
      SMC->SMC_ECC_MD = pgSz & 0x03;
    }
    else {
      /* 1 bit correction per 256 bytes of data for 8 bit devices */
      SMC->SMC_ECC_MD = pgSz | (2 << 4);
    }

    SMC->SMC_ECC_CTRL = 3;
  }

  /* Configure DMA if enabled */
  if (NAND_DMA_EN) { 
    /* Disable DMA Write Protection */
    PMC->PMC_WPMR  = PMC_WPDIS_KEY;
    /* Enable DMA peripheral clock */    
    PMC->PMC_PCER1  = (1 << (ID_DMAC - 32));

    /* Read channel status, clear and disable pending interrupts */
    DMAC->DMAC_CHSR;
    DMAC->DMAC_EBCISR;
    DMAC->DMAC_CHDR |= 1 << (8 + NAND_DMA_EN);
    for (i = NAND_TIMEOUT; i; i--) {
      if (!(DMAC->DMAC_CHSR & (1 << NAND_DMA_CH))) break;
    }
    if (i == 0) { return ERR_NAND_DMA_TOUT; }
    DMAC->DMAC_EBCIDR |= 0x010101 << NAND_DMA_CH;
  
    /* Enable DMA */
    DMAC->DMAC_EN = 1;

    /* Channel configuration */
    DMAC->DMAC_CH_NUM[NAND_DMA_CH].DMAC_CTRLB = (1UL << 16) | (1UL << 20);
    DMAC->DMAC_CH_NUM[NAND_DMA_CH].DMAC_CFG   = (1UL << 16) |  /* Stop on Done En. */
                                                (1UL << 24) ;  /* AHB Protection   */
  }

  /* Disable Scrambling */
  SMC->SMC_OCMS = 0x00;

  /* Reset flash chip - set and execute reset command */
  cmdAddr = NFC_CMD_BASE | (NAND_CMD_RESET << 2) | (cs << 22);
  SMC_WRITE_CMD (cmdAddr, 0);
  
  /* Wait Ready/Busy: Tout ~10msec @ 100MHz */
  for (i = NAND_TIMEOUT; i; i--) {
    if (SMC->SMC_SR & NFC_RB_RISE) break;
  }
  if (i == 0) { return ERR_NAND_HW_TOUT; }

  for (i = NAND_TIMEOUT; i; i--) {
    /* Check status */
    cmdAddr = NFC_CMD_BASE | (NAND_CMD_STATUS  <<  2) | (cs << 22);
    SMC_WRITE_CMD (cmdAddr, 0);
    status = SMC_DATA8;
    if (status & NAND_STAT_RDY) {
      return RTV_NOERR; 
    }
  }
  return ERR_NAND_HW_TOUT;
}
/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
