/*------------------------------------------------------------------------------
 *      RL-ARM - FlashFS
 *------------------------------------------------------------------------------
 *      Name:    NAND_LPC24xx.c
 *      Purpose: 8bit NAND Flash Interface Driver using EMC on NXP LPC24xx
 *      Rev.:    V4.54
 *------------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2012 KEIL - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include <File_Config.h>
#include <string.h>
#include <LPC24xx.h>
#include "NAND_LPC24xx.h"

//-------- <<< Use Configuration Wizard in Context Menu >>> -----------------
// <o0>Chip select used for NAND <0x00=> CS0
//                               <0x01=> CS1
//                               <0x02=> CS2
//                               <0x03=> CS3

#define NAND0_HW_CS  0x01

//------------- <<< end of configuration section >>> -----------------------

/* EMC data bus */
#define EMC_NAND_BASE  (0x80000000 + 0x1000000 * NAND0_HW_CS)
#define EMC_ALE_ADDR    0x00080000
#define EMC_CLE_ADDR    0x00100000

#define EMC_DATA8 *((volatile U8 *)(EMC_NAND_BASE))
#define EMC_ADDR8 *((volatile U8 *)(EMC_NAND_BASE + EMC_ALE_ADDR))
#define EMC_CMD8  *((volatile U8 *)(EMC_NAND_BASE + EMC_CLE_ADDR))


#define PIN_RB (1 << 12) /* MCB2400 */


/*----------------------------------------------------------------------------
 *      NAND driver prototypes
 *---------------------------------------------------------------------------*/
static U32 Init         (NAND_DRV_CFG *cfg);
static U32 UnInit       (NAND_DRV_CFG *cfg);
static U32 PageRead     (U32 row, U8 *buf, NAND_DRV_CFG *cfg);
static U32 PageWrite    (U32 row, U8 *buf, NAND_DRV_CFG *cfg);
static U32 BlockErase   (U32 row, NAND_DRV_CFG *cfg);


/*------------------------------------------------------------------------------
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
  EMC_CMD8 = (U8)cmd;
  for (i = 10; i; i--);                 /* Write enable high to busy delay    */ 
}


/*------------------------------------------------------------------------------
  Set page address
 *----------------------------------------------------------------------------*/
static void SetPgAddr (U32 numCyc, U32 row, U32 pgSz) {
  U32 i;

  EMC_ADDR8 = 0;
  numCyc--;
  if (pgSz > 528) {
    EMC_ADDR8 = 0;
    numCyc--;
  }

  i = 0;
  while (numCyc--) {
    EMC_ADDR8 = (U8)(row >> i);
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
    EMC_ADDR8 = (U8)(row >> i);
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
    if (FIO2PIN & PIN_RB) {
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
  return (EMC_DATA8 & flag) ? NAND_FLAG_SET : NAND_FLAG_CLR; 
}


/*------------------------------------------------------------------------------
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
  U32 i, sz;

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

  /* Read page from NAND chip */
  sz = cfg->PageSize;
  for (i = 0; i < sz; i += 8) {
    buf[i]   = EMC_DATA8;
    buf[i+1] = EMC_DATA8;
    buf[i+2] = EMC_DATA8;
    buf[i+3] = EMC_DATA8;
    buf[i+4] = EMC_DATA8;
    buf[i+5] = EMC_DATA8;
    buf[i+6] = EMC_DATA8;
    buf[i+7] = EMC_DATA8;
  }

  return RTV_NOERR;
}


/*------------------------------------------------------------------------------
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
  U32 i, sz;

  /* Write command 1 */
  WrCmd (NAND_CMD_PROG1ST);

  /* Set address */
  SetPgAddr (cfg->AddrCycles, row, cfg->PageSize);

  /* Write data to NAND chip */
  sz = cfg->PageSize;
  for (i = 0; i < sz; i += 8) {
    EMC_DATA8 = buf[i];
    EMC_DATA8 = buf[i+1];
    EMC_DATA8 = buf[i+2];
    EMC_DATA8 = buf[i+3];
    EMC_DATA8 = buf[i+4];
    EMC_DATA8 = buf[i+5];
    EMC_DATA8 = buf[i+6];
    EMC_DATA8 = buf[i+7];
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


/*------------------------------------------------------------------------------
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


/*------------------------------------------------------------------------------
 *      Uninitialise NAND flash driver
 *  *cfg = Pointer to configuration structure
 *
 *  Return: RTV_NOERR         - UnInit successful
 *----------------------------------------------------------------------------*/
static U32 UnInit (NAND_DRV_CFG *cfg) {

  /* Set NAND peripheral lines to reset state */

  /* Data Lines: P3.0 .. P3.7 - D0 .. D7 */
  PINSEL6 &= ~0xFFFF;

  /* Address latch: P4.19 - A19  [ALE]  */
  /* Command latch: P4.20 - A20  [CLE]  */
  /* Read enable:   P4.24 - OE   [nRE]  */
  /* Write enable:  P4.25 - WE   [nWE]  */
  PINSEL9 &= ~0x000F03C0;
 
  /* Ready/Busy */
  PINSEL4 &= ~(3UL << 24);  /* MCB 2400: P2.12 is GPIO  */
  
#if   NAND0_HW_CS == EMC_NAND_CS0
  /* Chip enable 0: P4.30 - CS0 [nCE] */
  PINSEL9 &= ~(3UL << 28);
#elif NAND0_HW_CS == EMC_NAND_CS1
  /* Chip enable 1: P4.31 - CS1 [nCE] */
  PINSEL9 &= ~(3UL << 30);
#elif NAND0_HW_CS == EMC_NAND_CS2
  /* Chip enable 2: P2.14 - CS2 [nCE] */
  PINSEL4 &= ~(3UL << 28);
#elif NAND0_HW_CS == EMC_NAND_CS3
  /* Chip enable 3: P2.15 - CS3 [nCE] */
  PINSEL4 &= ~(3UL << 30);
#endif

  return RTV_NOERR;
}


/*------------------------------------------------------------------------------
 *      Initialise NAND flash driver
 *
 *  *cfg = Pointer to configuration structure
 *
 *  Return: RTV_NOERR         - NAND Flash Initialisation successful
 *          ERR_NAND_HW_TOUT  - NAND Flash Reset Command failed
 *          ERR_NAND_DMA_TOUT - DMA Configuration failed
 *----------------------------------------------------------------------------*/
static U32 Init (NAND_DRV_CFG *cfg) {

  /* Power On External Memory Controler */
  PCONP |= (1 << 11);

  /* Init NAND peripheral lines */

  /* Data Lines: P3.0 .. P3.7 - D0 .. D7 */
  PINSEL6 = (PINSEL6 & ~0xFFFF) | 0x5555;

  /* Address latch: P4.19 - A19  [ALE]  */
  /* Command latch: P4.20 - A20  [CLE]  */
  /* Read enable:   P4.24 - OE   [nRE]  */
  /* Write enable:  P4.25 - WE   [nWE]  */
  PINSEL9 = (PINSEL9 & ~0x000F03C0) | 0x00050140;

  /* Ready/Busy pin */
  PINSEL4 &= ~(3UL << 24);              /* MCB2400: P2.12 is GPIO  */
  FIO2DIR &= ~PIN_RB;                   /* Ready/Busy pin is input */

#if   NAND0_HW_CS == EMC_NAND_CS0
  /* Chip enable 0: P4.30 - CS0 [nCE] */
  PINSEL9 = (PINSEL9 & ~(3UL << 28)) | (1UL << 28);
#elif NAND0_HW_CS == EMC_NAND_CS1
  /* Chip enable 1: P4.31 - CS1 [nCE] */
  PINSEL9 = (PINSEL9 & ~(3UL << 30)) | (1UL << 30);
#elif NAND0_HW_CS == EMC_NAND_CS2
  /* Chip enable 2: P2.14 - CS2 [nCE] */
  PINSEL4 = (PINSEL4 & ~(3UL << 28)) | (1UL << 28);
#elif NAND0_HW_CS == EMC_NAND_CS3
  /* Chip enable 3: P2.15 - CS3 [nCE] */
  PINSEL4 = (PINSEL4 & ~(3UL << 30)) | (1UL << 30);
#endif

  /* Init External Memory Controller peripheral */

  /* Enable EMC, Normal memory map */
  EMC_CTRL = 1;

  /* Delays are configured for CPU clock 72 MHz */
  EMC_STA_CFG     (NAND0_HW_CS) = (1 << 7); /* BLSn[3:0] are Low for Read/Write   */
  EMC_STA_WAITWEN (NAND0_HW_CS) = 2;        /* Delay from CS to WE                */
  EMC_STA_WAITOEN (NAND0_HW_CS) = 2;        /* Delay from CS or addr change to RE */
  EMC_STA_WAITRD  (NAND0_HW_CS) = 6;        /* Delay from CS to read access       */
  EMC_STA_WAITPAGE(NAND0_HW_CS) = 2;        /* Page Mode Read Delay               */
  EMC_STA_WAITWR  (NAND0_HW_CS) = 6;        /* Delay from CS to write access      */
  EMC_STA_WAITTURN(NAND0_HW_CS) = 2;        /* Turn Round Delay                   */

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
/*------------------------------------------------------------------------------
 * end of file
 *----------------------------------------------------------------------------*/
