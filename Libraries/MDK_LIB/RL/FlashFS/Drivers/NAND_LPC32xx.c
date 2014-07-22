/*----------------------------------------------------------------------------
 *      RL-ARM - FlashFS
 *----------------------------------------------------------------------------
 *      Name:    NAND_LPC32xx.c
 *      Purpose: NAND Flash Interface Driver for NXP LPC32xx
 *      Rev.:    V4.20
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2011 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <File_Config.h>
#include <LPC325x.h>
#include "NAND_LPC32xx.h"

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

/*-----------------------------------------------------------------------------
 *      Wait for status flag
 *  flag = Status flag to be checked in MLC NAND Status Register (MLC_ISR)
 * 
 * Return: 0 if timeout expired
 *         1 if flag set
 *----------------------------------------------------------------------------*/
static U32 StatusFlag (U32 flag) {
  U32 i;

  for (i = NAND_TIMEOUT; i; i--) {
    if (MLC_ISR & flag) {
      break;                                /* Break if flag is set           */
    }
  }
  if (i == 0) {
    return (0);                             /* Timeout                        */
  }
  return (1);                               /* Flag set                       */
}

/*-----------------------------------------------------------------------------
 *      Initialise NAND flash driver
 *
 *  *cfg = Pointer to configuration structure
 *
 *  Return: RTV_NOERR             - NAND Flash Initialisation successful
 *          ERR_NAND_HW_TOUT      - NAND Flash Reset Command failed
 *          ERR_NAND_UNSUPPORTED  - Page size invalid
 *----------------------------------------------------------------------------*/
static U32 Init (NAND_DRV_CFG *cfg) {
  U32 pgSz, cyc;

  pgSz = cfg->PageSize;
  switch (pgSz) {
    case 528:
    case 2112:
      break;
    default:
      return ERR_NAND_UNSUPPORTED;
  }

  FLASHCLK_CTRL = 0x00000022;               /* Setup NAND Flash Clock Control */
  MLC_CEH       = 0;                        /* Force nCE assert               */
  MLC_CMD       = NAND_CMD_RESET;           /* Reset NAND Flash               */

  if (!StatusFlag (NAND_CHIP_BUSY)) {       /* Wait while NAND busy           */
    return ERR_NAND_HW_TOUT; 
  }

  cyc = cfg->AddrCycles;                    /* Get address cycles             */

  MLC_LOCK_PR = 0xA25E;                     /* Unlock MLC_ICR register        */
  MLC_ICR     = ((cyc  ==    4) << 1) | 
                ((pgSz == 2112) << 2);

  MLC_LOCK_PR  = 0xA25E;                    /* Unlock MLC_TIME register       */
  MLC_TIME_REG = (3 << 24) | (11 << 19) | (4 << 16) | (2 << 12) |
                 (4 << 8)  | ( 3 <<  4) | (4 <<  0);

  P3_OUTP_SET |= (1 << 19);                   /* Disable write protect        */
  return RTV_NOERR;
}

/*-----------------------------------------------------------------------------
 *      Uninitialise NAND flash driver
 *  *cfg = Pointer to configuration structure
 *
 *  Return: RTV_NOERR         - UnInit successful
 *----------------------------------------------------------------------------*/
static U32 UnInit(NAND_DRV_CFG *cfg) {
  return RTV_NOERR;
}

/*-----------------------------------------------------------------------------
 *      Set Row Address
 *  addr = NAND row + col address
 *  cyc  = number of adress cycles
 *    Return Value:         NAND_ERROR
 *----------------------------------------------------------------------------*/
static void SetAddr (U32 addr, U32 cyc) {
  if (cyc == 5) {  
    MLC_ADDR = 0;
  }
  MLC_ADDR = (addr >>  0) & 0xFF;
  MLC_ADDR = (addr >>  8) & 0xFF;
  MLC_ADDR = (addr >> 16) & 0xFF;
  MLC_ADDR = (addr >> 24) & 0xFF;
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
static U32 BlockErase(U32 row, NAND_DRV_CFG *cfg) {

  if (!StatusFlag (NAND_CON_READY)) {       /* Wait for controller ready    */
    return ERR_NAND_HW_TOUT; 
  }
  MLC_CMD  = NAND_CMD_ERASE1ST;             /* Erase command 1              */
  SetAddr (row, cfg->AddrCycles);           /* Set address                  */
  MLC_CMD  = NAND_CMD_ERASE2ND;             /* Erase command 2              */

  if (!StatusFlag (NAND_CON_READY)) {       /* Wait for controller ready    */
    return ERR_NAND_HW_TOUT; 
  }

  if (!StatusFlag (NAND_CHIP_BUSY)) {       /* Wait while NAND busy         */
    return ERR_NAND_HW_TOUT; 
  }

  MLC_CMD = NAND_CMD_STATUS;                /* Write Read Status command    */
  if ((U8)MLC_DATAX(0) & NAND_STAT_FAIL) {  /* Check if command successful  */
    return ERR_NAND_ERASE;                  /* Block Erase Failed           */
  }
  return RTV_NOERR;
}

/*-----------------------------------------------------------------------------
 *      Read page
 *  row  = Page address
 *  *buf = Pointer to data buffer
 *  *cfg = Pointer to configuration structure
 *
 *  Return: RTV_NOERR         - Page read successful
 *          ERR_NAND_HW_TOUT  - Hardware transfer timeout
 *          ERR_ECC_COR       - ECC corrected the data within page
 *          ERR_ECC_UNCOR     - ECC was not able to correct the data
 *----------------------------------------------------------------------------*/
static U32 PageRead(U32 row, U8 *buf, NAND_DRV_CFG *cfg) {
  U32 i, sec, ecc;
  U32 *p = (U32 *)buf;
                                                                          
  MLC_CMD = NAND_CMD_READ1ST;               /* Read command (1st cycle)     */
  if (cfg->PageSize > 528) {
    MLC_CMD = NAND_CMD_READ2ND;             /* Read command (2nd cycle)     */
  }
  SetAddr (row << 8, cfg->AddrCycles);      /* Set address                  */
  
  ecc = ECC_NOERR;
  
  for (sec = 0; sec < cfg->SectorsPerPage; sec++) {
    MLC_ECC_AUTO_DEC_REG = 0x00;            /* Auto Decode                  */
    if (!StatusFlag (NAND_CON_READY)) {     /* Wait for controller ready    */
      return ERR_NAND_HW_TOUT; 
    }
    if (!StatusFlag (NAND_ECC_READY)) {     /* Wait for ECC ready           */
      return ERR_NAND_HW_TOUT; 
    }
    
    if (MLC_ISR & NAND_ERR_DET) {           /* Check for decode error       */
      ecc |= ECC_CORRECTED;
    }
    if (MLC_ISR & NAND_DEC_FAIL) {
      ecc |= ECC_UNCORRECTED;
    }
    for (i = 0; i < (528 >> 2); i++) {
      *p++ = MLC_BUFFX (i);                 /* Read main + spare area       */
    }
  }
  if (ecc & ECC_UNCORRECTED) {
    /* ECC was not able to correct the data within page */
    return ERR_ECC_UNCOR;
  }
  if (ecc & ECC_CORRECTED) {
    /* ECC corrected the data within page*/
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
 *----------------------------------------------------------------------------*/
static U32 PageWrite(U32 row, U8 *buf, NAND_DRV_CFG *cfg) {
  U32 i, sec;

  U32 *p = (U32 *)buf;

  if (!StatusFlag (NAND_CON_READY)) {       /* Wait for controller ready    */
    return ERR_NAND_HW_TOUT; 
  }

  MLC_CMD = NAND_CMD_PROG1ST;               /* Programm command 1           */
  SetAddr (row << 8, cfg->AddrCycles);      /* Set address                  */

  for (sec = 0; sec < cfg->SectorsPerPage; sec++) {
    MLC_ECC_ENC_REG = 0;                    /* Start Encode Cycle           */
    for (i = 0; i < (528 >> 2); i++)        /* Write main + spare area      */
      MLC_BUFFX(i) = *p++;
    MLC_ECC_AUTO_ENC_REG = 0;               /* Auto encode                  */
    if (!StatusFlag (NAND_CON_READY)) {     /* Wait for controller ready    */
      return ERR_NAND_HW_TOUT; 
    }
  }
  MLC_CMD = NAND_CMD_PROG2ND;               /* Programm command 2           */

  if (!StatusFlag (NAND_CON_READY)) {       /* Wait for controller ready    */
    return ERR_NAND_HW_TOUT; 
  }
  if (!StatusFlag (NAND_CHIP_BUSY)) {       /* Wait while NAND busy         */
    return ERR_NAND_HW_TOUT; 
  }

  MLC_CMD = NAND_CMD_STATUS;                /* Send status command          */
  if ((U8)MLC_DATAX(0) & NAND_STAT_FAIL) {
    return ERR_NAND_PROG;                   /* Programming failed           */
  }
  return RTV_NOERR;
}
