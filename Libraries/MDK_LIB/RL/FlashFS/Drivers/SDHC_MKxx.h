/*-----------------------------------------------------------------------------
 *      RL-ARM - FlashFS
 *-----------------------------------------------------------------------------
 *      Name:    SDHC_MKxx.h
 *      Purpose: SD/SDIO MMC Interface Driver Definitions for Freescale MKxx
 *      Rev.:    V4.54
 *-----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2012 KEIL - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#ifndef __SDHC_MKXX_H
#define __SDHC_MKXX_H

/* SD Card communication speed */
#define SD_CLK              24000000

/* Wait timeouts, in multiples of 6 byte send over MCI (for 1 bit mode)       */
#define WR_TOUT             100000              /* ~ 200 ms at MCI clk 24MHz  */
#define RD_STOP_TOUT        100                 /* ~ 200 us at MCI clk 24MHz  */
#define DATA_RD_TOUT_VALUE  (100*(SD_CLK/1000)) /* ~100ms at 24MHz SD clock   */
#define DATA_WR_TOUT_VALUE  (200*(SD_CLK/1000)) /* ~200ms at 24MHz SD clock   */
#define WAIT_2SD_CLK(ck)    (ck/(SD_CLK*2)+1)   /* ~2 SD clocks wait time     */
#define WAIT_CNT(ck,us)     ((ck/5000000)*us)   /* delay in for loop cycles   */

#define SDHC_IRQSTAT_ALL   0x117F01FF
#define SDHC_IRQSTAT_CMD   0x017F01FE

#endif /* __SDHC_MKXX_H */

/*-----------------------------------------------------------------------------
 * end of file
 *----------------------------------------------------------------------------*/
