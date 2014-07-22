/*----------------------------------------------------------------------------
 *      RL-ARM - FlashFS
 *----------------------------------------------------------------------------
 *      Name:    NAND_MKxx.h
 *      Purpose: NAND Flash Interface Driver Definitions for Freescale Kinetis
 *      Rev.:    V4.60
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2012 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#ifndef __NAND_MKxx_H
#define __NAND_MKxx_H

/*----------------------------------------------------------------------------
 *      Kinetis NAND Driver Defines
 *---------------------------------------------------------------------------*/

/* Page layout definition */
#define MK_POS_LSN             0
#define MK_POS_COR             4
#define MK_POS_BBM             5
#define MK_POS_ECC             6

#define MK_SECT_INC          512
#define MK_SPARE_OFS        2048
#define MK_SPARE_INC           6

/* Hardware timeout */
#define NAND_TIMEOUT      300000        /* ~10ms @ 120MHz should be enough    */

/* ECC result position */
#define MK_ECC_AD           2116        /* Byte pos: MK_ECC_AD[11:3] + 4      */

/* Bus width */
#define NAND_BUS_W8         0x00
#define NAND_BUS_W16        0x01

/* Ready/Busy */
#define NAND_BUSY           0x00
#define NAND_READY          0x01

/* Flag Set/Cleared */
#define NAND_FLAG_CLR       0x00
#define NAND_FLAG_SET       0x01
#define NAND_FLAG_TOUT      0x02

/* NFC chip select definitions */
#define NAND_CS0            0x00
#define NAND_CS1            0x01

/* NFC command codes */
#define CODE_READ_ID        (0x1 <<  2) /* Read NAND ID register              */
#define CODE_READ_STATUS    (0x1 <<  3) /* Read NAND status                   */
#define CODE_SEND_CMD3      (0x1 <<  4) /* Send command byte 3 CMD1[BYTE3]    */
#define CODE_DATA_FROM_NAND (0x1 <<  5) /* Transfer data from NAND            */
#define CODE_WAIT_RB        (0x1 <<  6) /* Wait for R/B handshake             */
#define CODE_SEND_CMD2      (0x1 <<  7) /* Send command byte 2 CMD1[BYTE2]    */
#define CODE_DATA_TO_NAND   (0x1 <<  8) /* Transfer data to NAND              */
#define CODE_SEND_ROW_ADDR  (0x7 <<  9) /* Send row address 1, 2 and 3        */
#define CODE_SEND_COL_ADDR  (0x3 << 12) /* Send column address 1 and 2        */
#define CODE_SEND_CMD1      (0x1 << 14) /* Send command byte 1 CMD2[BYTE1]    */
#define CODE_DATA_TO_SRAM   (0x1 << 15) /* Read data from memory to SRAM      */

/* NFC command masks */
#define NFC_CMD_RESET  (CODE_SEND_CMD1      | \
                        CODE_WAIT_RB        | \
                        CODE_SEND_CMD3      | \
                        CODE_READ_STATUS    )

#define NFC_CMD_ERASE  (CODE_SEND_CMD1      | \
                        CODE_SEND_ROW_ADDR  | \
                        CODE_SEND_CMD2      | \
                        CODE_WAIT_RB        | \
                        CODE_SEND_CMD3      | \
                        CODE_READ_STATUS    )

#define NFC_CMD_READ   (CODE_SEND_CMD1      | \
                        CODE_SEND_COL_ADDR  | \
                        CODE_SEND_ROW_ADDR  | \
                        CODE_SEND_CMD2      | \
                        CODE_WAIT_RB        | \
                        CODE_DATA_FROM_NAND )

#define NFC_CMD_WRITE  (CODE_SEND_CMD1      | \
                        CODE_SEND_COL_ADDR  | \
                        CODE_SEND_ROW_ADDR  | \
                        CODE_DATA_TO_NAND   | \
                        CODE_SEND_CMD2      | \
                        CODE_WAIT_RB        | \
                        CODE_SEND_CMD3      | \
                        CODE_READ_STATUS    )

#endif /* __NAND_MKxx_H */

/*----------------------------------------------------------------------------
 * End of file
 *---------------------------------------------------------------------------*/
