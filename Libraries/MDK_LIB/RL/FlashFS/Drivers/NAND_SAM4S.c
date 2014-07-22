/*----------------------------------------------------------------------------
 *      RL-ARM - FlashFS
 *----------------------------------------------------------------------------
 *      Name:    NAND_SAM4S.c
 *      Purpose: NAND Flash Interface Driver for Atmel ATSAM4S
 *      Rev.:    V4.60
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2011 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <File_Config.h>
#include <string.h>
#include <SAM4S.H>                    /* SAM4S definitions                   */
#include "NAND_SAM4S.h"

//-------- <<< Use Configuration Wizard in Context Menu >>> -----------------
// <o0>Chip select used for NAND <0x00=> NCS0
//                               <0x01=> NCS1
//                               <0x02=> NCS2
//                               <0x03=> NCS3
//
//
// <o1>Device bus width <0x00=>  8-bit
//                      <0x01=> 16-bit

#define NAND_HW_CS    0x00
#define NAND_HW_BUS   0x00


//------------- <<< end of configuration section >>> -----------------------

/* SMC data bus */
#define SMC_NAND_BASE     (0x60000000 + 0x1000000 * NAND_HW_CS)
#define SMC_ALE_ADDR       0x00200000
#define SMC_CLE_ADDR       0x00400000

#if NAND_HW_BUS == NAND_BUS_W8
 #define SMC_DATA *((volatile U8  *)(SMC_NAND_BASE))
 #define SMC_ADDR *((volatile U8  *)(SMC_NAND_BASE + SMC_ALE_ADDR))
 #define SMC_CMD  *((volatile U8  *)(SMC_NAND_BASE + SMC_CLE_ADDR))
#else
 #define SMC_DATA *((volatile U16 *)(SMC_NAND_BASE))
 #define SMC_ADDR *((volatile U16 *)(SMC_NAND_BASE + SMC_ALE_ADDR))
 #define SMC_CMD  *((volatile U16 *)(SMC_NAND_BASE + SMC_CLE_ADDR))
#endif


/*----------------------------------------------------------------------------
  Function prototypes
 *----------------------------------------------------------------------------*/
static U32 Init       (NAND_DRV_CFG *cfg);
static U32 UnInit     (NAND_DRV_CFG *cfg);
static U32 PageRead   (U32 row, U8 *buf, NAND_DRV_CFG *cfg);
static U32 PageWrite  (U32 row, U8 *buf, NAND_DRV_CFG *cfg);
static U32 BlockErase (U32 row, NAND_DRV_CFG *cfg);


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


/*------------------------------------------------------------------------------
  Write command
 *----------------------------------------------------------------------------*/
static void WrCmd (U32 cmd) {
  U32 i;
  SMC_CMD = (U8)cmd;
  for (i = 10; i; i--);                 /* Write enable high to busy delay    */ 
}


/*------------------------------------------------------------------------------
  Set page address
 *----------------------------------------------------------------------------*/
static void SetPgAddr (U32 numCyc, U32 row, U32 pgSz) {
  U32 i;

  SMC_ADDR = 0;
  numCyc--;
  if (pgSz > 528) {
    SMC_ADDR = 0;
    numCyc--;
  }

  i = 0;
  while (numCyc--) {
    SMC_ADDR = (U8)(row >> i);
    i += 8;
  }
}


/*------------------------------------------------------------------------------
  Set block address
 *----------------------------------------------------------------------------*/
__inline void SetBlAddr (U32 numCyc, U32 row, U32 pgSz) {
  U32 i;

  numCyc = (pgSz > 528) ? (numCyc - 2) : (numCyc - 1);

  i = 0;
  while (numCyc--) {
    SMC_ADDR = (U8)(row >> i);
    i += 8;
  }
}


/*------------------------------------------------------------------------------
  Wait until NAND Ready (waiting for low to high R/B signal transition)

  Return: NAND_BUSY  - NAND is busy, timeout expired
          NAND_READY - NAND is ready for the next command
 *----------------------------------------------------------------------------*/
static U32 WaitReady (void) {
  U32 i;

  for (i = NAND_TIMEOUT; i; i--) {
    /* NAND is ready if R/B signal goes high */
    if (PIOC->PIO_PDSR & PIO_PDSR_P18) {
      return NAND_READY;
    }
  }
  /* Timeout expired */
  return NAND_BUSY;
}


/*------------------------------------------------------------------------------
  Check status register if specified flag is set

  flag = NAND status flag

  Return: NAND_FLAG_SET  - NAND flag is set
          NAND_FLAG_CLR  - NAND flag is cleared
          NAND_FLAG_TOUT - Timeout expired, NAND busy          
 *----------------------------------------------------------------------------*/
static U32 ChkStatus (U32 flag) {
  /* Read status */
  WrCmd (NAND_CMD_STATUS);
  return (SMC_DATA & flag) ? NAND_FLAG_SET : NAND_FLAG_CLR; 
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
 *----------------------------------------------------------------------------*/
static U32 PageRead (U32 row, U8 *buf, NAND_DRV_CFG *cfg) {
  U32 i;

  /* Write command 1 */
  WrCmd (NAND_CMD_READ1ST);

  /* Set address */
  SetPgAddr (cfg->AddrCycles, row, cfg->PageSize);

  /* Write command 2 */
  WrCmd (NAND_CMD_READ2ND);

  /* Wait until NAND ready */
  if (WaitReady() == NAND_BUSY) {
    return ERR_NAND_HW_TOUT;
  }

  /* Transfer data from the NAND chip */
  if (NAND_HW_BUS == NAND_BUS_W8) {
    /* 8-bit bus width */
    for (i = 0; i < cfg->PageSize; i += 8) {
      buf[i]   = SMC_DATA;
      buf[i+1] = SMC_DATA;
      buf[i+2] = SMC_DATA;
      buf[i+3] = SMC_DATA;
      buf[i+4] = SMC_DATA;
      buf[i+5] = SMC_DATA;
      buf[i+6] = SMC_DATA;
      buf[i+7] = SMC_DATA;
    }
  }
  else {
    /* 16-bit bus width */
    for (i = 0; i < cfg->PageSize; i += 2) {
      buf[i] = SMC_DATA;
    }
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
  U32 i;

  /* Write command 1 */
  WrCmd (NAND_CMD_PROG1ST);

  /* Set address */
  SetPgAddr (cfg->AddrCycles, row, cfg->PageSize);

  /* Transfer data to the NAND chip */ 
  if (NAND_HW_BUS == NAND_BUS_W8) {
    /* 8-bit bus width */
    for (i = 0; i < cfg->PageSize; i += 8) {
      SMC_DATA = buf[i];
      SMC_DATA = buf[i+1];
      SMC_DATA = buf[i+2];
      SMC_DATA = buf[i+3];
      SMC_DATA = buf[i+4];
      SMC_DATA = buf[i+5];
      SMC_DATA = buf[i+6];
      SMC_DATA = buf[i+7];
    }
  }
  else {
    /* 16-bit bus width */
    for (i = 0; i < cfg->PageSize; i += 2) {
      SMC_DATA = ((U16)buf[i + 1] << 8) | buf[i];
    }
  }

  /* Write command 2 */
  WrCmd (NAND_CMD_PROG2ND);

  /* Wait until NAND ready */
  if (WaitReady() == NAND_BUSY) {
    return ERR_NAND_HW_TOUT;
  }

  /* Check status */
  switch (ChkStatus (NAND_STAT_FAIL)) {
    case NAND_FLAG_TOUT:
      return ERR_NAND_HW_TOUT;
    case NAND_FLAG_SET:
      return ERR_NAND_PROG;
    default:
      return RTV_NOERR;
  }
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
  /* Write command 1 */
  WrCmd (NAND_CMD_ERASE1ST);

  /* Set address */
  SetBlAddr (cfg->AddrCycles, row, cfg->PageSize);

  /* Write command 2 */
  WrCmd (NAND_CMD_ERASE2ND);

  /* Wait until NAND ready */
  if (WaitReady() ==  NAND_BUSY) {
    return ERR_NAND_HW_TOUT;
  }

  /* Check status */
  switch (ChkStatus (NAND_STAT_FAIL)) {
    case NAND_FLAG_TOUT:
      return ERR_NAND_HW_TOUT;
    case NAND_FLAG_SET:
      return ERR_NAND_ERASE;
    default:
      return RTV_NOERR;
  }
}

/*-----------------------------------------------------------------------------
 *      Uninitialise NAND flash driver
 *  *cfg = Pointer to configuration structure
 *
 *  Return: RTV_NOERR         - UnInit successful
 *----------------------------------------------------------------------------*/
static U32 UnInit (NAND_DRV_CFG *cfg) {
  /* PIO Write Protect Disable */
  PIOC->PIO_WPMR = PIO_WPDIS_KEY;
  
  /* Release pins to PIO control */
  PIOC->PIO_PER    = SMC_PIO_PDR_PIN_MSK;
  PIOC->PIO_PUER   = PIO_PUER_P18;

  /* Release chip select pin to PIO control */
#if   NAND_HW_CS == SMC_NCS0
  /* Chip enable 0: PC14 - NCS0 [nCE] */
  PIOC->PIO_PER    = PIO_PER_P14;
#elif NAND_HW_CS == SMC_NCS1
  /* Chip enable 1: PC15 - NCS1 [nCE] */
  PIOC->PIO_PER    = PIO_PER_P15;
#elif NAND_HW_CS == SMC_NCS2
  /* Chip enable 2: PA22 - NCS2 [nCE] */
  PIOA->PIO_PER    = PIO_PER_P22;
#elif NAND_HW_CS == SMC_NCS3
  /* Chip enable 3: PC12 - NCS3 [nCE] */
  PIOC->PIO_PER    = PIO_PER_P12;
#endif

  /* PIO Write Protect Enable */
  PIOC->PIO_WPMR = PIO_WPEN_KEY;
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
  U32 cs;

  /* Configure Power Management Controller: Enable SMC and PIO clocks */
  PMC->PMC_WPMR  = PMC_WPDIS_KEY;                   /* Disable write protect  */
  PMC->PMC_PCER0 = (1 << ID_SMC) | (1 << ID_PIOA) | (1 << ID_PIOC);
  PMC->PMC_WPMR  = PMC_WPEN_KEY;                    /* Enable write protect   */

  /* PIO Write Protect Disable */
  PIOC->PIO_WPMR = PIO_WPDIS_KEY;
  
  /* Configure SMC I/O pins */
  PIOC->PIO_ABCDSR[0] &= ~SMC_PIO_ABCDSR_PIN_MSK;
  PIOC->PIO_ABCDSR[1] &= ~SMC_PIO_ABCDSR_PIN_MSK;

  PIOC->PIO_PDR    = SMC_PIO_PDR_PIN_MSK;

  /* Configure ready/busy pin (PC18) */
  PIOC->PIO_PER    = PIO_PER_P18;
  PIOC->PIO_ODR    = PIO_ODR_P18;
  PIOC->PIO_PUDR   = PIO_PUDR_P18;

  /* Configure chip select pin */
#if   NAND_HW_CS == SMC_NCS0
  /* Chip enable 0: PC14 - NCS0 [nCE] */
  PIOC->PIO_PDR    = PIO_PDR_P14;
  PIOC->PIO_PUER   = PIO_PUER_P14;
#elif NAND_HW_CS == SMC_NCS1
  /* Chip enable 1: PC15 - NCS1 [nCE] */
  PIOC->PIO_PDR    = PIO_PDR_P15;
  PIOC->PIO_PUER   = PIO_PUER_P15;
#elif NAND_HW_CS == SMC_NCS2
  /* Chip enable 2: PA22 - NCS2 [nCE] */
  PIOA->PIO_PDR    = PIO_PDR_P22;
  PIOA->PIO_PUER   = PIO_PUER_P22;
#elif NAND_HW_CS == SMC_NCS3
  /* Chip enable 3: PC12 - NCS3 [nCE] */
  PIOC->PIO_PDR    = PIO_PDR_P12;
  PIOC->PIO_PUER   = PIO_PUER_P12;
#endif

  /* PIO Write Protect Enable */
  PIOC->PIO_WPMR = PIO_WPEN_KEY;

  /* Assign Chip Select to a NAND Flash in the Matrix */
  MATRIX->CCFG_SMCNFCS = 1 << NAND_HW_CS;

  /* Disable SMC Write Protection */
  SMC->SMC_WPMR = SMC_WPDIS_KEY;

  /* Configure timing and signals */
  cs = NAND_HW_CS;

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

  SMC->SMC_CS_NUMBER[cs].SMC_MODE =    (1 <<  0) | // read is controlled by NRD signal
                                       (1 <<  1) ; // write is controlled by NWE signal

  /* Disable Scrambling */
  SMC->SMC_OCMS = 0x00;

  /* Reset NAND chip */
  WrCmd (NAND_CMD_RESET);
  
  if (WaitReady() == NAND_BUSY) {
    /* Reset failed */
    return ERR_NAND_HW_TOUT;
  }
  /* Check status */
  if (ChkStatus (NAND_STAT_RDY) != NAND_FLAG_SET) {
    return ERR_NAND_HW_TOUT;
  }
  /* NAND is ready */
  return RTV_NOERR;
}
/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
