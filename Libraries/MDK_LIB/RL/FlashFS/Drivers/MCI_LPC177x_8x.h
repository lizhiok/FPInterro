/*-----------------------------------------------------------------------------
 *      RL-ARM - FlashFS
 *-----------------------------------------------------------------------------
 *      Name:    MCI_LPC177x_8x.h
 *      Purpose: Multimedia Card Interface Driver for NXP LPC178x/7x Definitions
 *      Rev.:    V4.54
 *-----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2012 KEIL - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#ifndef __MCI_LPC177X_8X_H
#define __MCI_LPC177X_8X_H

/* SD Card communication speed */
#define SD_CLK              25000000

/* Wait timeouts, in multiples of 6 byte send over MCI (for 1 bit mode)      */
#define DATA_RD_TOUT_VALUE  (100*(SD_CLK/1000)) /* ~100ms at 25MHz SD clock  */
#define DATA_WR_TOUT_VALUE  (500*(SD_CLK/1000)) /* ~500ms at 25MHz SD clock  */
#define WAIT_2SD_CLK(ck)    (ck/(SD_CLK*2)+1)   /* ~2 SD clocks wait time    */
#define WAIT_CNT(ck,us)     ((ck/5000000)*us)   /* delay in for loop cycles  */

/* MCI Status register bit information */
#define MCI_CMD_CRC_FAIL    0x00000001
#define MCI_DATA_CRC_FAIL   0x00000002
#define MCI_CMD_TIMEOUT     0x00000004
#define MCI_DATA_TIMEOUT    0x00000008
#define MCI_TX_UNDERRUN     0x00000010
#define MCI_RX_OVERRUN      0x00000020
#define MCI_CMD_RESP_END    0x00000040
#define MCI_CMD_SENT        0x00000080
#define MCI_DATA_END        0x00000100
#define MCI_START_BIT_ERR   0x00000200
#define MCI_DATA_BLK_END    0x00000400
#define MCI_CMD_ACTIVE      0x00000800
#define MCI_TX_ACTIVE       0x00001000
#define MCI_RX_ACTIVE       0x00002000
#define MCI_TX_HALF_EMPTY   0x00004000
#define MCI_RX_HALF_FULL    0x00008000
#define MCI_TX_FIFO_FULL    0x00010000
#define MCI_RX_FIFO_FULL    0x00020000
#define MCI_TX_FIFO_EMPTY   0x00040000
#define MCI_RX_FIFO_EMPTY   0x00080000
#define MCI_TX_DATA_AVAIL   0x00100000
#define MCI_RX_DATA_AVAIL   0x00200000

#define MCI_CLEAR_MASK      0x000007FF

/* Card detect and Write protect pin definition */
#define SD_CD_PIN 0
#define SD_WP_PIN 0

#endif

/*-----------------------------------------------------------------------------
 * End of file
 *----------------------------------------------------------------------------*/
