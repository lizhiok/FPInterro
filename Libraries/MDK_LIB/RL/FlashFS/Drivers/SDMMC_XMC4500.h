/*-----------------------------------------------------------------------------
 *      RL-ARM - FlashFS
 *-----------------------------------------------------------------------------
 *      Name:    SDMMC_XMC4500.h
 *      Purpose: SD/MMC Interface Driver for Infineon XMC4500 Definitions
 *      Rev.:    V4.54
 *-----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2012 KEIL - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#ifndef __SDMMC_XMC4500_H
#define __SDMMC_XMC4500_H

#define WAIT_CNT(ck,us)     ((ck/5000000)*us)   /* delay in for loop cycles  */

/* SDMMC interrupt masks */
#define SDMMC_EN_INT_STATUS_NORM_MSK (SDMMC_EN_INT_STATUS_NORM_CARD_REMOVAL_EN_Msk      | \
                                      SDMMC_EN_INT_STATUS_NORM_CARD_INS_EN_Msk          | \
                                      SDMMC_EN_INT_STATUS_NORM_BUFF_READ_READY_EN_Msk   | \
                                      SDMMC_EN_INT_STATUS_NORM_BUFF_WRITE_READY_EN_Msk  | \
                                      SDMMC_EN_INT_STATUS_NORM_TX_COMPLETE_EN_Msk       | \
                                      SDMMC_EN_INT_STATUS_NORM_CMD_COMPLETE_EN_Msk      )

#define SDMMC_EN_INT_STATUS_ERR_MSK  (SDMMC_EN_INT_STATUS_ERR_CEATA_ERR_EN_Msk          | \
                                      SDMMC_EN_INT_STATUS_ERR_ACMD_ERR_EN_Msk           | \
                                      SDMMC_EN_INT_STATUS_ERR_CURRENT_LIMIT_ERR_EN_Msk  | \
                                      SDMMC_EN_INT_STATUS_ERR_DATA_END_BIT_ERR_EN_Msk   | \
                                      SDMMC_EN_INT_STATUS_ERR_DATA_CRC_ERR_EN_Msk       | \
                                      SDMMC_EN_INT_STATUS_ERR_DATA_TIMEOUT_ERR_EN_Msk   | \
                                      SDMMC_EN_INT_STATUS_ERR_CMD_IND_ERR_EN_Msk        | \
                                      SDMMC_EN_INT_STATUS_ERR_CMD_END_BIT_ERR_EN_Msk    | \
                                      SDMMC_EN_INT_STATUS_ERR_CMD_CRC_ERR_EN_Msk        | \
                                      SDMMC_EN_INT_STATUS_ERR_CMD_TIMEOUT_ERR_EN_Msk    )

#define SDMMC_EN_INT_SIGNAL_NORM_MSK (SDMMC_EN_INT_SIGNAL_NORM_BUFF_READ_READY_EN_Msk   | \
                                      SDMMC_EN_INT_SIGNAL_NORM_BUFF_WRITE_READY_EN_Msk  | \
                                      SDMMC_EN_INT_SIGNAL_NORM_TX_COMPLETE_EN_Msk       | \
                                      SDMMC_EN_INT_SIGNAL_NORM_CMD_COMPLETE_EN_Msk      )

#define SDMMC_EN_INT_SIGNAL_ERR_MSK  (SDMMC_EN_INT_SIGNAL_ERR_CEATA_ERR_EN_Msk          | \
                                      SDMMC_EN_INT_SIGNAL_ERR_ACMD_ERR_EN_Msk           | \
                                      SDMMC_EN_INT_SIGNAL_ERR_CURRENT_LIMIT_ERR_EN_Msk  | \
                                      SDMMC_EN_INT_SIGNAL_ERR_DATA_END_BIT_ERR_EN_Msk   | \
                                      SDMMC_EN_INT_SIGNAL_ERR_DATA_CRC_ERR_EN_Msk       | \
                                      SDMMC_EN_INT_SIGNAL_ERR_DATA_TIMEOUT_ERR_EN_Msk   | \
                                      SDMMC_EN_INT_SIGNAL_ERR_CMD_IND_ERR_EN_Msk        | \
                                      SDMMC_EN_INT_SIGNAL_ERR_CMD_END_BIT_ERR_EN_Msk    | \
                                      SDMMC_EN_INT_SIGNAL_ERR_CMD_CRC_ERR_EN_Msk        | \
                                      SDMMC_EN_INT_SIGNAL_ERR_CMD_TIMEOUT_ERR_EN_Msk    )

/* Response types */
#define SDMMC_NO_RESP       0x00
#define SDMMC_RESP_136      0x01
#define SDMMC_RESP_48       0x02
#define SDMMC_RESP_48b      0x03

/* IRQ Handler Status Flags */
#define SDMMC_STAT_CMD_OK   (1 << 0)
#define SDMMC_STAT_TX_DONE  (1 << 1)
#define SDMMC_STAT_BUF_RDY  (1 << 2)

#endif /* __SDMMC_XMC4500_H */

/*-----------------------------------------------------------------------------
 * End of file
 *----------------------------------------------------------------------------*/
