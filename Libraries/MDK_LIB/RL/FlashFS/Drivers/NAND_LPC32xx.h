/*----------------------------------------------------------------------------
 *      RL-ARM - FlashFS
 *----------------------------------------------------------------------------
 *      Name:    NAND_LPC32xx.h 
 *      Purpose: NAND Flash Interface Driver for NXP LPC32xx Defs
 *      Rev.:    V4.20
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2011 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#ifndef __NAND_LPC32XX_H_
#define __NAND_LPC32XX_H_

/*-----------------------------------------------------------------------------
 *      NAND Controller Buffer Definition
 *----------------------------------------------------------------------------*/
#define MLC_BUFFX(x)           (*(volatile unsigned long  *)(0x200A8000+(x<<2)))
#define MLC_DATAX(x)           (*(volatile unsigned long  *)(0x200B0000+(x<<2)))

/*----------------------------------------------------------------------------
 *      Various NAND Driver Defines
 *---------------------------------------------------------------------------*/
#define NAND_TIMEOUT   5000000

#define NAND_CHIP_BUSY 0x01
#define NAND_CON_READY 0x02
#define NAND_ECC_READY 0x04
#define NAND_ERR_DET   0x08
#define NAND_DEC_FAIL  0x40

/*-----------------------------------------------------------------------------
 *      NAND driver prototypes
 *----------------------------------------------------------------------------*/
U32 Init         (NAND_DRV_CFG *cfg);
U32 UnInit       (NAND_DRV_CFG *cfg);
U32 PageRead     (U32 row, U8 *buf, NAND_DRV_CFG *cfg);
U32 PageWrite    (U32 row, U8 *buf, NAND_DRV_CFG *cfg);
U32 BlockErase   (U32 row, NAND_DRV_CFG *cfg);

#endif /* __NAND_LPC32XX_H_ */

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
