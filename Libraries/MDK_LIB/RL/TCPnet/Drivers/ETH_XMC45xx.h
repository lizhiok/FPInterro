/*-----------------------------------------------------------------------------
 *      RL-ARM - TCPnet
 *-----------------------------------------------------------------------------
 *      Name:    ETH_XMC45xx.H
 *      Purpose: Infineon XMC4500 Ethernet Controller Driver definitions
 *      Rev.:    V4.71
 *-----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#ifndef __ETH_XMC45XX_H
#define __ETH_XMC45XX_H

/* ETH Memory Buffer configuration. */
#define NUM_RX_BUF          32          /* Receive Buffer 32*256 = 8K        */
#define ETH_RX_BUF_SIZE     256         /* ETH Receive buffer size.          */
#define ETH_RX_BUF_NUM      6           /* Max. Buffers used for a packet    */

#define NUM_TX_BUF          3           /* Transmit Buffer 3*1536 = 4.5K     */
#define ETH_TX_BUF_SIZE     1536

#define ETH_MAX_FLEN        1536        /* Max. Ethernet Frame Size          */

/* DMA Descriptors */
typedef struct {
  U32 volatile Stat;                    /* Packet Status                     */
  U32 Ctrl;                             /* Packet Control                    */
  U32 Addr;                             /* Buffer Address                    */
  U32 Next;                             /* Next Descriptor Address           */
} RX_Desc;

typedef struct {
  U32 volatile CtrlStat;                /* Packet Control/Status             */
  U32 Size;                             /* Packet Size                       */
  U32 Addr;                             /* Buffer Address                    */
  U32 Next;                             /* Next Descriptor Address           */
} TX_Desc;

/* RX Descriptor Status (RDES0) */
#define RDES0_OWN           0x80000000  /* Ownership Bit (1= EMAC)           */
#define RDES0_AFM           0x40000000  /* Destination Address Filter Fail   */
#define RDES0_FL            0x3FFF0000  /* Frame Length                      */
#define RDES0_ES            0x00008000  /* Error Summary                     */
#define RDES0_DE            0x00004000  /* Descriptor Error                  */
#define RDES0_SAF           0x00002000  /* Source Address Filter Fail        */
#define RDES0_LE            0x00001000  /* Frame Length Error                */
#define RDES0_OE            0x00000800  /* Overflow Error                    */
#define RDES0_VLAN          0x00000400  /* VLAN Tag                          */
#define RDES0_FS            0x00000200  /* First Descriptor                  */
#define RDES0_LS            0x00000100  /* Last Descriptor                   */
#define RDES0_CEGF          0x00000080  /* Giant Frame (> 1518 bytes)        */
#define RDES0_LC            0x00000040  /* Late Collision                    */
#define RDES0_FT            0x00000020  /* Frame Type Ethernet               */
#define RDES0_RWT           0x00000010  /* Receive Watchdog Timeout          */
#define RDES0_RE            0x00000008  /* Receive Error on MII              */
#define RDES0_DBE           0x00000004  /* Dribbling Bit Error               */
#define RDES0_CE            0x00000002  /* CRC Error                         */
#define RDES0_PCE           0x00000001  /* Payload Checksum Error            */

/* RX Descriptor Control (RDES1) */
#define RDES1_DIC           0x80000000  /* Disable Interrupt on Completition */
#define RDES1_RBS2          0x1FFF0000  /* Receive Buffer 2 Size mask        */
#define RDES1_RER           0x00008000  /* Receive End of Ring               */
#define RDES1_RCH           0x00004000  /* Second Address Chained            */
#define RDES1_RBS1          0x00001FFF  /* Receive Buffer 1 Size mask        */

/* TX Descriptor Status (TDES0) */
#define TDES0_OWN           0x80000000  /* Ownership Bit (1= EMAC)           */
#define TDES0_IC            0x40000000  /* Interrupt on Completition         */
#define TDES0_LS            0x20000000  /* Last Segment                      */
#define TDES0_FS            0x10000000  /* First Segment                     */
#define TDES0_DC            0x08000000  /* Disable CRC                       */
#define TDES0_DP            0x04000000  /* Disable Padding                   */
#define TDES0_TTSE          0x02000000  /* Transmit Time Stamp Enable        */
#define TDES0_CIC           0x00C00000  /* Checksum Insertion Control        */
#define TDES0_TER           0x00200000  /* Transmit End of Ring              */
#define TDES0_TCH           0x00100000  /* Second Address Chained            */
#define TDES0_TTSS          0x00020000  /* Tx Time Stamp Status              */
#define TDES0_IHE           0x00010000  /* IP Header Error                   */
#define TDES0_ES            0x00008000  /* Error Summary                     */
#define TDES0_JT            0x00004000  /* Jabber Timeout                    */
#define TDES0_FF            0x00002000  /* Frame Flushed                     */
#define TDES0_IPE           0x00001000  /* IP Payload Checksum Error         */
#define TDES0_LCA           0x00000800  /* Loss of Carrier                   */
#define TDES0_NC            0x00000400  /* No Carrier                        */
#define TDES0_LC            0x00000200  /* Late Collision                    */
#define TDES0_EC            0x00000100  /* Excessive Collisions              */
#define TDES0_VF            0x00000080  /* VLAN Frame                        */
#define TDES0_CC            0x00000078  /* Collision Count                   */
#define TDES0_ED            0x00000004  /* Excessive Deferral                */
#define TDES0_UF            0x00000002  /* Underflow Error                   */
#define TDES0_DB            0x00000001  /* Deferred Bit                      */

/* TX Descriptor Control (TDES1) */
#define TDES1_TBS2          0x1FFF0000  /* Buffer 2 Size                     */
#define TDES1_TBS1          0x00001FFF  /* Buffer 1 Size                     */

/* GMII Management Time out values */
#define GMII_WR_TOUT        0x00050000  /* MII Write timeout count           */
#define GMII_RD_TOUT        0x00050000  /* MII Read timeout count            */

/* GMII Management Address Register */
#define GMII_READ           0x00000005  /* MII Read  (MDC = Eclk / 62)       */
#define GMII_WRITE          0x00000007  /* MII Write                         */

/* DP83848C PHY Registers */
#define PHY_REG_BMCR        0x00        /* Basic Mode Control Register       */
#define PHY_REG_BMSR        0x01        /* Basic Mode Status Register        */
#define PHY_REG_IDR1        0x02        /* PHY Identifier 1                  */
#define PHY_REG_IDR2        0x03        /* PHY Identifier 2                  */
#define PHY_REG_ANAR        0x04        /* Auto-Negotiation Advertisement    */
#define PHY_REG_ANLPAR      0x05        /* Auto-Neg. Link Partner Abitily    */
#define PHY_REG_ANER        0x06        /* Auto-Neg. Expansion Register      */
#define PHY_REG_ANNPTR      0x07        /* Auto-Neg. Next Page TX            */

/* PHY Extended Registers */
#define PHY_REG_STS         0x10        /* Status Register                   */
#define PHY_REG_MICR        0x11        /* MII Interrupt Control Register    */
#define PHY_REG_MISR        0x12        /* MII Interrupt Status Register     */
#define PHY_REG_FCSCR       0x14        /* False Carrier Sense Counter       */
#define PHY_REG_RECR        0x15        /* Receive Error Counter             */
#define PHY_REG_PCSR        0x16        /* PCS Sublayer Config. and Status   */
#define PHY_REG_RBR         0x17        /* RMII and Bypass Register          */
#define PHY_REG_LEDCR       0x18        /* LED Direct Control Register       */
#define PHY_REG_PHYCR       0x19        /* PHY Control Register              */
#define PHY_REG_10BTSCR     0x1A        /* 10Base-T Status/Control Register  */
#define PHY_REG_CDCTRL1     0x1B        /* CD Test Control and BIST Extens.  */
#define PHY_REG_EDCR        0x1D        /* Energy Detect Control Register    */

#define PHY_FULLD_100M      0x2100      /* Full Duplex 100Mbit               */
#define PHY_HALFD_100M      0x2000      /* Half Duplex 100Mbit               */
#define PHY_FULLD_10M       0x0100      /* Full Duplex 10Mbit                */
#define PHY_HALFD_10M       0x0000      /* Half Duplex 10MBit                */
#define PHY_AUTO_NEG        0x3000      /* Select Auto Negotiation           */

#define DP83848C_DEF_ADR    0x0001      /* Default PHY device address        */
#define DP83848C_ID         0x20005C90  /* PHY Identifier                    */

#endif /*  __ETH_XMC45XX_H */

/*-----------------------------------------------------------------------------
 * End of file
 *----------------------------------------------------------------------------*/

