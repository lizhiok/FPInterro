/*-----------------------------------------------------------------------------
 *      RL-ARM - TCPnet
 *-----------------------------------------------------------------------------
 *      Name:    ENET_MKxx.H
 *      Purpose: Freescale MKxx Ethernet Controller Driver definitions
 *      Rev.:    V4.70
 *-----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#ifndef __ENET_MKXX_H
#define __ENET_MKXX_H

/* ETH Memory Buffer configuration. */
#define NUM_RX_BUF          4           /* 0x1800 for Rx (4*1536=6K)         */
#define NUM_TX_BUF          2           /* 0x0600 for Tx (2*1536=3K)         */
#define ETH_BUF_SIZE        1536        /* ETH Receive/Transmit buffer size  */

/* uDMA Descriptors. */
typedef volatile struct {
  uint16_t RBD[16];
} RX_Desc;

typedef volatile struct {
  uint16_t TBD[16];
} TX_Desc;

/* Receive Buffer Descriptor Field Definitions */
#define DESC_RX_E           (1 << 15)   /* Empty                              */
#define DESC_RX_RO1         (1 << 14)   /* Receive software ownership         */
#define DESC_RX_W           (1 << 13)   /* Wrap                               */
#define DESC_RX_RO2         (1 << 12)   /* Receive software ownership         */
#define DESC_RX_L           (1 << 11)   /* Last in frame                      */
#define DESC_RX_M           (1 <<  8)   /* Miss                               */
#define DESC_RX_BC          (1 <<  7)   /* DA is broadcast                    */
#define DESC_RX_MC          (1 <<  6)   /* DA is multicast                    */
#define DESC_RX_LG          (1 <<  5)   /* Rx frame length violation          */
#define DESC_RX_NO          (1 <<  4)   /* Non-octet aligned frame            */
#define DESC_RX_CR          (1 <<  2)   /* Receive CRC or frame error         */
#define DESC_RX_OV          (1 <<  1)   /* Overrun                            */
#define DESC_RX_TR          (1 <<  0)   /* Receive frame is truncated         */

#define DESC_RX_INT         (1 <<  7)   /* Generate RXB/RXF interrupt         */
#define DESC_RX_BDU         (1 << 15)   /* Last descriptor update done        */

/* Transmit Buffer Descriptor Field Definitions */
#define DESC_TX_R           (1 << 15)   /* Ready                              */
#define DESC_TX_TO1         (1 << 14)   /* Transmit software ownership        */
#define DESC_TX_W           (1 << 13)   /* Wrap                               */
#define DESC_TX_TO2         (1 << 12)   /* Transmit software ownership        */
#define DESC_TX_L           (1 << 11)   /* Last in frame                      */
#define DESC_TX_TC          (1 << 10)   /* Transmit CRC                       */
#define DESC_TX_ABC         (1 <<  9)   /* Append bad CRC                     */

/* MII Management Time out values */
#define MII_WR_TOUT         0x00050000  /* MII Write timeout count            */
#define MII_RD_TOUT         0x00050000  /* MII Read timeout count             */

/* KSZ8041NL PHY Registers */
#define PHY_REG_BCTRL       0x00        /* Basic Control Register             */
#define PHY_REG_BSTAT       0x01        /* Basic Status Register              */
#define PHY_REG_ID1         0x02        /* PHY Identifier 1                   */
#define PHY_REG_ID2         0x03        /* PHY Identifier 2                   */
#define PHY_REG_ANA         0x04        /* Auto-Negotiation Advertisement     */
#define PHY_REG_ANLPA       0x05        /* Auto-Neg. Link Partner Abitily     */
#define PHY_REG_ANE         0x06        /* Auto-Neg. Expansion                */
#define PHY_REG_ANNP        0x07        /* Auto-Neg. Next Page TX             */
#define PHY_REG_LPNPA       0x08        /* Link Partner Next Page Ability     */
#define PHY_REG_RXERC       0x15        /* RXER Counter                       */
#define PHY_REG_ICS         0x1B        /* Interrupt Control/Status           */
#define PHY_REG_PC1         0x1E        /* PHY Control 1                      */
#define PHY_REG_PC2         0x1F        /* PHY Control 2                      */

/* Duplex and speed modes */
#define PHY_CON_10M         0x0001
#define PHY_CON_100M        0x0002
#define PHY_CON_HD          0x0004
#define PHY_CON_FD          0x0008

#define PHY_FULLD_100M      0x2100      /* Full Duplex 100Mbit                */
#define PHY_HALFD_100M      0x2000      /* Half Duplex 100Mbit                */
#define PHY_FULLD_10M       0x0100      /* Full Duplex 10Mbit                 */
#define PHY_HALFD_10M       0x0000      /* Half Duplex 10MBit                 */
#define PHY_AUTO_NEG        0x1000      /* Select Auto Negotiation            */

#define PHY_DEF_ADDR        0x00        /* Default PHY device address         */
#define PHY_ID_DP83848C     0x20005C90  /* DP83848C PHY Identifier            */
#define PHY_ID_ST802RT1     0x02038460  /* ST802RT1x PHY Identifier           */
#define PHY_ID_KSZ8041      0x00221510  /* KSZ8041x PHY Identifier            */

#endif /* __ENET_MKXX_H */

/*-----------------------------------------------------------------------------
 * End of file
 *----------------------------------------------------------------------------*/

