/*----------------------------------------------------------------------------
 *      RL-ARM - FlashFS
 *----------------------------------------------------------------------------
 *      Name:    NAND_SAM3X.h 
 *      Purpose: NAND Flash Interface Driver for Atmel ATSAM3X Defs
 *      Rev.:    V4.20
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2011 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#ifndef __NAND_SAM3X_H_
#define __NAND_SAM3X_H_

/*----------------------------------------------------------------------------
 *      SAM3X Peripheral Write Protection Keys
 *---------------------------------------------------------------------------*/

/* PMC Write Protect Mode Keys */
#define PMC_WPEN_KEY        0x504D4301
#define PMC_WPDIS_KEY       0x504D4300

/* PIO Write Protect Mode Keys */
#define PIO_WPEN_KEY        0x50494F01
#define PIO_WPDIS_KEY       0x50494F00

/* SMC Write Protection Control Keys */
#define SMC_WPEN_KEY        0x534D4301
#define SMC_WPDIS_KEY       0x534D4300

/*----------------------------------------------------------------------------
 *      SAM3X NAND Driver Defines
 *---------------------------------------------------------------------------*/
/* Command address */
#define SMC_WRITE_CMD(cmdAddr, nandAddr) \
{ *((volatile unsigned long *) cmdAddr) = (unsigned long) nandAddr; }

/* NFC Command Registers Base Address */ 
#define NFC_CMD_BASE        0x68000000

/* NFC SRAM Base Address */
#define NFC_SRAM_BASE_ADDR  0x20100000

/* Bus width */
#define NAND_BUS_W8         0x00
#define NAND_BUS_W16        0x01

/* Chip select */
#define SAM3X_NAND_CS0      0x00
#define SAM3X_NAND_CS1      0x01
#define SAM3X_NAND_CS2      0x02
#define SAM3X_NAND_CS3      0x03
#define SAM3X_NAND_CS4      0x04
#define SAM3X_NAND_CS5      0x05
#define SAM3X_NAND_CS6      0x06
#define SAM3X_NAND_CS7      0x07

/* NFC flags */
#define NFC_XFR_DONE        0x00010000
#define NFC_RB_RISE         0x00000010

/* Hardware timeout */
#define NAND_TIMEOUT        150000     /* ~10ms @ 100MHz should be enough    */

/*----------------------------------------------------------------------------
 *      NAND driver prototypes
 *---------------------------------------------------------------------------*/
U32 Init         (NAND_DRV_CFG *cfg);
U32 UnInit       (NAND_DRV_CFG *cfg);
U32 PageRead     (U32 row, U8 *buf, NAND_DRV_CFG *cfg);
U32 PageWrite    (U32 row, U8 *buf, NAND_DRV_CFG *cfg);
U32 BlockErase   (U32 row, NAND_DRV_CFG *cfg);

#endif /* __NAND_SAM3X_H_ */

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
