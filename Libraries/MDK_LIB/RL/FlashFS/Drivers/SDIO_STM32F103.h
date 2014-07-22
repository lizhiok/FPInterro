/*----------------------------------------------------------------------------
 *      RL-ARM - FlashFS
 *----------------------------------------------------------------------------
 *      Name:    SDIO_STM32F103.h
 *      Purpose: SD/SDIO MMC Interface Driver for ST STM32F103 Definitions
 *      Rev.:    V4.20
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2011 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#ifndef __SDIO_STM32X_H
#define __SDIO_STM32X_H

/* SD Card communication speed */
#define SD_CLK              24000000

/* Wait timeouts, in multiples of 6 byte send over MCI (for 1 bit mode)      */
#define WR_TOUT             100000              /* ~ 200 ms at MCI clk 24MHz */
#define RD_STOP_TOUT        100                 /* ~ 200 us at MCI clk 24MHz */
#define DATA_RD_TOUT_VALUE  (100*(SD_CLK/1000)) /* ~100ms at 24MHz SD clock  */
#define DATA_WR_TOUT_VALUE  (200*(SD_CLK/1000)) /* ~200ms at 24MHz SD clock  */
#define WAIT_2SD_CLK(ck)    (ck/(SD_CLK*2)+1)   /* ~2 SD clocks wait time    */
#define WAIT_CNT(ck,us)     ((ck/5000000)*us)   /* delay in for loop cycles  */

/* SDIO Status register bit information */
//#define SDIO_STA_CCRCFAIL       0x00000001  /* Command response received (CRC check failed)  */
//#define SDIO_STA_DCRCFAIL       0x00000002  /* Data block sent/received (CRC check failed)   */
//#define SDIO_STA_CTIMEOUT       0x00000004  /* Command response timeout.                     */
//#define SDIO_STA_DTIMEOUT       0x00000008  /* Data timeout                                  */
//#define SDIO_STA_TXUNDERR       0x00000010  /* Transmit FIFO underrun error                  */
//#define SDIO_STA_RXOVERR        0x00000020  /* Received FIFO overrun error                   */
//#define SDIO_STA_CMDREND        0x00000040  /* Command response received (CRC check passed)  */
//#define SDIO_STA_CMDSENT        0x00000080  /* Command sent (no response required)           */
//#define SDIO_STA_DATAEND        0x00000100  /* Data end (data counter, SDIDCOUNT, is zero)   */
//#define SDIO_STA_STBITERR       0x00000200  /* Start bit not detected                        */
//#define SDIO_STA_DBCKEND        0x00000400  /* Data block sent/received (CRC check passed)   */
//#define SDIO_STA_CMDACT         0x00000800  /* Command transfer in progress                  */
//#define SDIO_STA_TXACT          0x00001000  /* Data transmit in progress                     */
//#define SDIO_STA_RXACT          0x00002000  /* Data receive in progress                      */
//#define SDIO_STA_TXFIFOHE       0x00004000  /* Transmit FIFO Half Empty                      */
//#define SDIO_STA_RXFIFOHF       0x00008000  /* Receive FIFO Half Full                        */
//#define SDIO_STA_TXFIFOF        0x00010000  /* Transmit FIFO full                            */
//#define SDIO_STA_RXFIFOF        0x00020000  /* Receive FIFO full                             */
//#define SDIO_STA_TXFIFOE        0x00040000  /* Transmit FIFO empty                           */
//#define SDIO_STA_RXFIFOE        0x00080000  /* Receive FIFO empty                            */
//#define SDIO_STA_TXDAVL         0x00100000  /* Data available in transmit FIFO               */
//#define SDIO_STA_RXDAVL         0x00200000  /* Data available in receive FIFO                */
//#define SDIO_STA_SDIOIT         0x00400000  /*  SDIO interrupt received                      */
//#define SDIO_STA_CEATAEND       0x00800000  /* CE-ATA cmd completion sig. received for CMD61 */

#define SDIO_STA_CLEAR_MASK     0x00C007FF

#endif

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
