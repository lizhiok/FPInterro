/*----------------------------------------------------------------------------
 *      RL-ARM - FlashFS
 *----------------------------------------------------------------------------
 *      Name:    NAND_LPC24xx.h
 *      Purpose: 8bit NAND Flash Interface Driver using EMC on NXP LPC24xx Definitions
 *      Rev.:    V4.54
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2012 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#ifndef __NAND_LPC24XX_H
#define __NAND_LPC24XX_H

/*----------------------------------------------------------------------------
 *      LPC24XX NAND Driver Defines
 *---------------------------------------------------------------------------*/

/* Hardware timeout */
#define NAND_TIMEOUT      150000       /* ~10ms @ 100MHz should be enough    */

/* Bus width */
#define NAND_BUS_W8         0x00
#define NAND_BUS_W16        0x01

/* Chip select */
#define EMC_NAND_CS0        0x00
#define EMC_NAND_CS1        0x01
#define EMC_NAND_CS2        0x02
#define EMC_NAND_CS3        0x03

/* Ready/Busy */
#define NAND_BUSY           0x00
#define NAND_READY          0x01

/* Flag Set/Cleared */
#define NAND_FLAG_CLR       0x00
#define NAND_FLAG_SET       0x01
#define NAND_FLAG_TOUT      0x02

/* EMC Static Memory Bank Configuration Registers Macros */
#define EMC_STACFG_BASE       (EMC_BASE_ADDR + 0x200)

#define EMC_STA_CFG(cs)       *((volatile U32 *)(EMC_STACFG_BASE + 0x20*(cs) + 0x00))
#define EMC_STA_WAITWEN(cs)   *((volatile U32 *)(EMC_STACFG_BASE + 0x20*(cs) + 0x04))
#define EMC_STA_WAITOEN(cs)   *((volatile U32 *)(EMC_STACFG_BASE + 0x20*(cs) + 0x08))
#define EMC_STA_WAITRD(cs)    *((volatile U32 *)(EMC_STACFG_BASE + 0x20*(cs) + 0x0C))
#define EMC_STA_WAITPAGE(cs)  *((volatile U32 *)(EMC_STACFG_BASE + 0x20*(cs) + 0x10))
#define EMC_STA_WAITWR(cs)    *((volatile U32 *)(EMC_STACFG_BASE + 0x20*(cs) + 0x14))
#define EMC_STA_WAITTURN(cs)  *((volatile U32 *)(EMC_STACFG_BASE + 0x20*(cs) + 0x18))

#endif /* __NAND_LPC24XX_H_ */

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
