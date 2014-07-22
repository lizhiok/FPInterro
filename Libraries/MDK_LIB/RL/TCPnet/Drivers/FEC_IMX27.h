/*----------------------------------------------------------------------------
 *      RL-ARM - TCPnet
 *----------------------------------------------------------------------------
 *      Name:    FEC_IMX27.H
 *      Purpose: Freescale i.MX27 Fast Ethernet Controller Driver definitions
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#ifndef __FEC_IMX27_H
#define __FEC_IMX27_H

/* FEC Memory Buffer configuration. */
#define NUM_RX_BUF          32          /* Receive Buffer 32*256 = 8K        */
#define ETH_RX_BUF_SIZE     256         /* FEC Receive buffer size.          */
#define ETH_RX_BUF_NUM      6           /* Max. Buffers used for a packet    */

#define NUM_TX_BUF          3           /* Transmit Buffer 3*1536 = 4.5K     */
#define ETH_TX_BUF_SIZE     1536

#define ETH_MAX_FLEN        1536        /* Max. Ethernet Frame Size          */

typedef struct {
  U16 Len;
  U16 Stat;
  U32 Buf;
} RX_Desc;

typedef struct {
  U16 Len;
  U16 Ctrl;
  U32 Buf;
} TX_Desc;

/* Ethernet Interrupt Event/Mask Register */
#define INT_HBERR       0x80000000      /* Heartbeat Error                   */
#define INT_BABR        0x40000000      /* Babbling Receive Error            */
#define INT_BABT        0x20000000      /* Babbling Transmit Error           */
#define INT_GRA         0x10000000      /* Graceful Stop Complete            */
#define INT_TXF         0x08000000      /* Transmit Frame Interrupt          */
#define INT_TXB         0x04000000      /* Transmit Buffer Interrupt         */
#define INT_RXF         0x02000000      /* Receive Frame Interrupt           */
#define INT_RXB         0x01000000      /* Receive Buffer Interrupt          */
#define INT_MII         0x00800000      /* MII Interrupt                     */
#define INT_EBERR       0x00400000      /* Ethernet Bus Error                */
#define INT_LC          0x00200000      /* Late Collision                    */
#define INT_RL          0x00100000      /* Collision Retry Limit             */
#define INT_UN          0x00080000      /* Transmit FIFO Underrun            */

/* Ethernet Control Register */
#define ECR_ETHER_EN    0x00000002      /* Ethernet Controller Enable        */
#define ECR_RESET       0x00000001      /* Ethernet Controller Reset         */

/* MII Management Frame Register */
#define MMFR_MII_ST     0x40000000      /* Start of Frame Delimiter Value    */
#define MMFR_MII_WR     0x10000000      /* Write MII Frame                   */
#define MMFR_MII_RD     0x20000000      /* Read MII Frame                    */
#define MMFR_MII_PA     0x0F800000      /* PHY Address Mask                  */
#define MMFR_MII_RA     0x007C0000      /* Register Address Mask             */
#define MMFR_MII_TA     0x00020000      /* Turn Around Value                 */
#define MMFR_MII_DATA   0x0000FFFF      /* Management Frame Data             */

#define MMFR_RA_SHIFT   18              /* MII Register Address Bits         */
#define MMFR_PA_SHIFT   23              /* MII PHY Address Bits              */

/* MII Speed Control Register */
#define MSCR_DIS_PRE    0x00000080      /* Disable MII Preamble              */
#define MSCR_MII_SPEED  0x0000007E      /* MII Speed Mask                    */

#define MII_WR_TOUT     0x00300000      /* MII Write timeout count           */
#define MII_RD_TOUT     0x00300000      /* MII Read timeout count            */

/* MIB Control Register */
#define MIBC_MIB_DIS    0x80000000      /* MIB Block Disable                 */
#define MIBC_MIB_IDLE   0x40000000      /* MIB Block Status Idle             */

/* Receive Control Register */
#define RCR_MAX_FL      0x07FF0000      /* Maximum Frame Length Mask         */
#define RCR_FCE         0x00000020      /* Flow Control Enable               */
#define RCR_BC_REJ      0x00000010      /* Broadcast Frame Reject            */
#define RCR_PROM        0x00000008      /* Promiscuous Mode                  */
#define RCR_MII_MODE    0x00000004      /* Media Independent Interface Mode  */
#define RCR_DRT         0x00000002      /* Disable Receive on Transmit       */
#define RCR_LOOP        0x00000001      /* Internal Loopback                 */

/* Transmit Control Register */
#define TCR_RFC_PAUSE   0x00000010      /* Receive Frame Control Pause       */
#define TCR_TFC_PAUSE   0x00000008      /* Transmit Frame Control Pause      */
#define TCR_FDEN        0x00000004      /* Full Duplex Enable                */
#define TCR_HBC         0x00000002      /* Heartbit Control                  */
#define TCR_GTS         0x00000001      /* Graceful Transmit Stop            */

/* Receive Buffer Descriptor Status */
#define RBD_EMPTY       0x8000          /* Buffer Empty                      */
#define RBD_WRAP        0x2000          /* Wrap, last BD in Ring             */
#define RBD_LAST        0x0800          /* Last Buffer in Frame              */
#define RBD_MISS        0x0100          /* Miss bit for promiscuous mode     */
#define RBD_BC          0x0080          /* Broadcast Frame received          */
#define RBD_MC          0x0040          /* Multicast Frame received (not BC) */
#define RBD_LG          0x0020          /* Frame Length Violation            */
#define RBD_NO          0x0010          /* Non-octed Aligned Frame           */
#define RBD_CR          0x0004          /* Receive CRC Error                 */
#define RBD_OV          0x0002          /* Receive FIFO Overrun              */
#define RBD_TR          0x0001          /* Truncated Frame                   */

#define RBD_ERR         (RBD_LG | RBD_NO | RBD_CR | RBD_OV | RBD_TR)

/* Transmit Buffer Descriptor Control */
#define TBD_READY       0x8000          /* Buffer is ready                   */
#define TBD_WRAP        0x2000          /* Wrap, last BD in Ring             */
#define TBD_LAST        0x0800          /* Last Buffer in Frame              */
#define TBD_TC          0x0400          /* Transmit CRC                      */
#define TBD_ABC         0x0200          /* Append bad CRC                    */


/* KSZ8001 PHY Registers */
#define PHY_REG_BCR     0x00            /* Basic Control Register            */
#define PHY_REG_BSR     0x01            /* Basic Status Register             */
#define PHY_REG_IDR1    0x02            /* PHY Identifier 1                  */
#define PHY_REG_IDR2    0x03            /* PHY Identifier 2                  */
#define PHY_REG_ANAR    0x04            /* Auto-Negotiation Advertisement    */
#define PHY_REG_ANLPAR  0x05            /* Auto-Neg. Link Partner Abitily    */
#define PHY_REG_ANER    0x06            /* Auto-Neg. Expansion Register      */
#define PHY_REG_ANNPR   0x07            /* Auto-Neg. Next Page TX            */
#define PHY_REG_LPNPA   0x08            /* Link Partner Next Page Abitity    */

/* PHY Extended Registers */
#define PHY_REG_RECR    0x15            /* Receive Error Counter             */
#define PHY_REG_ICSR    0x1B            /* Interrupt Control/Status Register */
#define PHY_REG_LCSR    0x1D            /* Link Control/Status Register      */
#define PHY_REG_PCR     0x1E            /* PHY Control Register              */
#define PHY_REG_100TPCR 0x1F            /* 100 BASE-TX PHY Control Register  */

#define PHY_FULLD_100M  0x2100          /* Full Duplex 100Mbit               */
#define PHY_HALFD_100M  0x2000          /* Half Duplex 100Mbit               */
#define PHY_FULLD_10M   0x0100          /* Full Duplex 10Mbit                */
#define PHY_HALFD_10M   0x0000          /* Half Duplex 10MBit                */
#define PHY_AUTO_NEG    0x1200          /* Select Auto Negotiation           */

#define KSZ8001_DEF_ADR 0x0001          /* Default PHY device address        */
#define KSZ8001_ID      0x00221610      /* PHY Identifier                    */

#endif

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/

