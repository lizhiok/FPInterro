/*-----------------------------------------------------------------------------
 *      RL-ARM - FlashFS
 *-----------------------------------------------------------------------------
 *      Name:    SDIO_LPC18xx.h
 *      Purpose: SD/SDIO MMC Interface Driver for NXP LPC18xx Definitions
 *      Rev.:    V4.50
 *-----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2012 KEIL - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#ifndef __SDIO_LPC18XX_H
#define __SDIO_LPC18XX_H

/* SD Card communication speed */
#define SD_CLK              24000000

/* Wait timeouts, in multiples of 6 byte send over MCI (for 1 bit mode)       */
#define WR_TOUT             100000              /* ~ 200 ms at MCI clk 24MHz  */
#define RD_STOP_TOUT        100                 /* ~ 200 us at MCI clk 24MHz  */
#define DATA_RD_TOUT_VALUE  (100*(SD_CLK/1000)) /* ~100ms at 24MHz SD clock   */
#define DATA_WR_TOUT_VALUE  (200*(SD_CLK/1000)) /* ~200ms at 24MHz SD clock   */
#define WAIT_2SD_CLK(ck)    (ck/(SD_CLK*2)+1)   /* ~2 SD clocks wait time     */
#define WAIT_CNT(ck,us)     ((ck/5000000)*us)   /* delay in for loop cycles   */

/* Descriptor bit definitions */
#define DESC_DIC (1U <<  1)
#define DESC_LD  (1U <<  2)
#define DESC_FS  (1U <<  3)
#define DESC_CH  (1U <<  4)
#define DESC_ER  (1U <<  5)
#define DESC_CES (1U << 30)
#define DESC_OWN (1U << 31)

typedef struct {
  U32 CtrlStat;
  U32 BufSize;
  U32 BufAddr;
  U32 NextDesc;
} SDIO_DESC;


/* SDIO Register Interface Definitions */
#define SDIO_PWREN_EN       (1U <<  0)  /* PWREN:Power Enable bit             */

#define SDIO_CTRL_CTRL_RST  (1U <<  0)  /* CTRL:Controller Reset              */
#define SDIO_CTRL_FIFO_RST  (1U <<  1)  /* CTRL:FIFO Reset                    */
#define SDIO_CTRL_DMA_RST   (1U <<  2)  /* CTRL:DMA Reset                     */
#define SDIO_CTRL_DMA_EN    (1U <<  5)  /* CTRL:DMA Enable                    */
#define SDIO_CTRL_USE_IDMA  (1U << 25)  /* CTRL:Use Internal DMAC             */

#define SDIO_CMD_RESP_EXP   (1U <<  6)  /* CMD:Response Expect                */
#define SDIO_CMD_RESP_LEN   (1U <<  7)  /* CMD:Response Length                */
#define SDIO_CMD_RESP_CRC   (1U <<  8)  /* CMD:Check Response CRC             */
#define SDIO_CMD_DATA_EXP   (1U <<  9)  /* CMD:Data Expected                  */
#define SDIO_CMD_READ_WRITE (1U << 10)  /* CMD:Read Write                     */
#define SDIO_CMD_WAIT_PRV   (1U << 13)  /* CMD:Wait prvdata complete          */
#define SDIO_CMD_SEND_INIT  (1U << 15)  /* CMD:Send Initialization            */
#define SDIO_CMD_CLK_UPD    (1U << 21)  /* CMD:Update clock registers only    */
#define SDIO_CMD_START      (1U << 31)  /* CMD:Start Cmd                      */

#define SDIO_RINTSTS_RE     (1U <<  1)  /* RINTSTS:Response error             */
#define SDIO_RINTSTS_CDONE  (1U <<  2)  /* RINTSTS:Command done               */
#define SDIO_RINTSTS_RCRC   (1U <<  6)  /* RINTSTS:Response CRC Error         */
#define SDIO_RINTSTS_RTO    (1U <<  8)  /* RINTSTS:Response timeout           */
#define SDIO_RINTSTS_DRTO   (1U <<  9)  /* RINTSTS:Data read timeout          */
#define SDIO_RINTSTS_HLE    (1U << 12)  /* RINTSTS:Hardware locked write      */

#define SDIO_IDSTS_TI       (1U <<  0)  /* IDSTS:Transmit Interrupt           */
#define SDIO_IDSTS_RI       (1U <<  1)  /* IDSTS:Receive Interrupt            */
#define SDIO_IDSTS_FBE      (1U <<  2)  /* IDSTS:Fatal Bus Error Interrupt    */
#define SDIO_IDSTS_DU       (1U <<  4)  /* IDSTS:Descriptor Unavailable       */
#define SDIO_IDSTS_CES      (1U <<  5)  /* IDSTS:Card Error Summary           */

#define SDIO_BMOD_DE        (1U <<  7)  /* BMOD:SD/MMC DMA Enable             */

/* SDIO Register Masks */
#define SDIO_RINTSTS_MSK    0x0001FFFF; /* RINTSTS: Interrupt flags mask      */
#define SDIO_IDSTS_MSK      0x00000337; /* IDSTS:Interrupt flags mask         */

#endif /* __SDIO_LPC18XX_H */

/*-----------------------------------------------------------------------------
 * End of file
 *----------------------------------------------------------------------------*/
