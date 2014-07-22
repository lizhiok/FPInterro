/*----------------------------------------------------------------------------
 *      RL-ARM - FlashFS
 *----------------------------------------------------------------------------
 *      Name:    FSMC_STM32F2xx.h 
 *      Purpose: Flexible static memory controller Driver for ST STM32F2xx
 *      Rev.:    V4.22
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2011 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#ifndef __FSMC_STM32F2XX_H
#define __FSMC_STM32F2XX_H

#include <RTL.h>

/*----------------------------------------------------------------------------
 *      STM32F2xx FSMC Properties
 *---------------------------------------------------------------------------*/

/* AHB Access Macro */
#define FSMC_AHB_W8(base,  addr)  (*(volatile U8  *)((base) + (addr)))
#define FSMC_AHB_W16(base, addr)  (*(volatile U16 *)((base) + (addr)))

/* Bank Base Address */
#define FSMC_BANK1_BASE 0x60000000UL
#define FSMC_BANK2_BASE 0x70000000UL
#define FSMC_BANK3_BASE 0x80000000UL
#define FSMC_BANK4_BASE 0x90000000UL

#define FSMC_BANK3_CMD  0x010000UL
#define FSMC_BANK3_ADR  0x020000UL


/* NOR/PSRAM Chip Select */
#define FSMC_NE1      0                    /* Bank 1, Chip Select 1 (NE1)   */
#define FSMC_NE2      1                    /* Bank 1, Chip Select 2 (NE2)   */
#define FSMC_NE3      2                    /* Bank 1, Chip Select 3 (NE3)   */
#define FSMC_NE4      3                    /* Bank 1, Chip Select 4 (NE4)   */

/* NAND Chip Select */
#define FSMC_NCE2     4                    /* Bank 2 Chip Select (NCE2)     */
#define FSMC_NCE3     5                    /* Bank 3 Chip Select (NCE3)     */

/* PC Card Chip Select */
#define FSMC_NCE4_1   6                    /* Bank 4, Chip Select 1 (NCE4_1)*/
#define FSMC_NCE4_2   7                    /* Bank 4, Chip Select 2 (NCE4_2)*/

/* Invalid Chip Select */
#define FSMC_CS_INV   8

/* FSMC Function prototypes */
U32 FSMC_Init   (U32 cs);
U32 FSMC_UnInit (U32 cs);

/*----------------------------------------------------------------------------
 *      NAND Driver Specific Defines
 *---------------------------------------------------------------------------*/
#define NAND_TIMEOUT    200000

#define NAND_DATA_W8  FSMC_AHB_W8(FSMC_BANK3_BASE, 0)
#define NAND_ADR_W8   FSMC_AHB_W8(FSMC_BANK3_BASE, FSMC_BANK3_ADR)
#define NAND_CMD_W8   FSMC_AHB_W8(FSMC_BANK3_BASE, FSMC_BANK3_CMD)

/* Flag Set/Cleared */
#define NAND_FLAG_CLR       0x00
#define NAND_FLAG_SET       0x01
#define NAND_FLAG_TOUT      0x02

/*----------------------------------------------------------------------------
 *      NAND Device Specific Defines
 *---------------------------------------------------------------------------*/
/* Timing configured for HCLK @ 120MHz */
#define NAND_CO_HIZ_TIME      (0x02 << 24) 
#define NAND_CO_HOLD_TIME     (0x02 << 16)
#define NAND_CO_WAIT_TIME     (0x02 <<  8)
#define NAND_CO_SETUP_TIME    (0x01 <<  0)

#define NAND_AT_HIZ_TIME      (0x02 << 24)
#define NAND_AT_HOLD_TIME     (0x02 << 16)
#define NAND_AT_WAIT_TIME     (0x02 <<  8)
#define NAND_AT_SETUP_TIME    (0x01 <<  0)


/*----------------------------------------------------------------------------
 *      OneNAND Driver Specific Defines
 *---------------------------------------------------------------------------*/
#define TIMEOUT_OneNAND 200000

/* OneNAND Flag Masks */
#define ON_FL_PROG    ((1 << 15) | (1 << 12))
#define ON_FL_LOAD    ((1 << 15) | (1 << 13))
#define ON_FL_ERASE   ((1 << 15) | (1 << 11))

/*----------------------------------------------------------------------------
 *      OneNAND Device Specific Defines
 *---------------------------------------------------------------------------*/

/* OneNAND Page Layout Definition */
#define ON_POS_LSN      2
#define ON_POS_COR      1
#define ON_POS_BBM      0
#define ON_POS_ECC      8

#define ON_SECT_INC   512
#define ON_SPARE_OFS 2048
#define ON_SPARE_INC   16

/* Data/Spare buffer */
#define ON_DBUF_ADDR    0x00000400UL       /* DataRAM 0 User Area Address   */
#define ON_SBUF_ADDR    0x00010020UL       /* DataRAM 0 Spare Area Address  */

/* OneNAND Commands */
#define ON_CMD_LDM_BUF  (U16)0x00          /* Load main area into buffer    */
#define ON_CMD_LDS_BUF  (U16)0x13          /* Load spare area into buffer   */
#define ON_CMD_PRG_BUF  (U16)0x80          /* Program from buffer to NAND   */
#define ON_CMD_UNLOCK   (U16)0x23          /* Unlock NAND array a block     */
#define ON_CMD_ERASE    (U16)0x94          /* Block erase                   */
#define ON_CMD_RST_COR  (U16)0xF0          /* Reset NAND Flash Core         */
#define ON_CMD_RST_HOT  (U16)0xF3          /* Reset OneNAND == Hot reset    */

/* OneNAND Register Adress Map */
#define ON_REG_MAN_ID   0x1E000UL          /* Manufacturer ID               */
#define ON_REG_DEV_ID   0x1E002UL          /* Device ID                     */
#define ON_REG_DBF_SZ   0x1E006UL          /* Data buffer size              */
#define ON_REG_BBF_SZ   0x1E008UL          /* Boot buffer size              */
#define ON_REG_DBF_AM   0x1E00AUL          /* Data/Boot buffer amount       */
#define ON_REG_TECH     0x1E00CUL          /* Info about technology         */
#define ON_REG_SA_1     0x1E200UL          /* NAND Flash Block Address      */
#define ON_REG_SA_8     0x1E20EUL          /* NAND Page & Sector Address    */
#define ON_REG_ST_BUF   0x1E400UL          /* Buffer number for page data   */
#define ON_REG_CMD      0x1E440UL          /* Ctrl/Mem operation commands   */
#define ON_REG_CFG_1    0x1E442UL          /* System Configuration 1        */
#define ON_REG_CFG_2    0x1E444UL          /* System Configuration 2        */
#define ON_REG_STAT     0x1E480UL          /* Controller Status/Results     */
#define ON_REG_INT      0x1E482UL          /* Command Competion ISR Stat    */
#define ON_REG_SBA      0x1E498UL          /* Start Block Address           */
#define ON_REG_WPS      0x1E49CUL          /* Write Protection Status       */
#define ON_REG_ECC_ST   0x1FE00UL          /* ECC Status of sector          */

#endif /* __FSMC_STM32F2XX_H */

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
