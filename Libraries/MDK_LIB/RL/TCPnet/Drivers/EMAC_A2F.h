/*----------------------------------------------------------------------------
 *      RL-ARM - TCPnet
 *----------------------------------------------------------------------------
 *      Name:    EMAC_A2F.h
 *      Purpose: Actel SmartFusion Ethernet MAC Controller Driver definitions
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#ifndef __EMAC_A2F_H
#define __EMAC_A2F_H

/* EMAC Memory Buffer configuration. */
#define NUM_RX_BUF          24          /* Receive Buffer 24*256 = 6.0kB     */
#define ETH_RX_BUF_SIZE     256         /* FEC Receive buffer size.          */
#define ETH_RX_BUF_NUM      6           /* Max. Buffers used for a packet    */

#define NUM_TX_BUF          3           /* Transmit Buffer 3*1536 = 4.6kB    */
#define ETH_TX_BUF_SIZE     1536

#define ETH_MAX_FLEN        1536        /* Max. Ethernet Frame Size          */

typedef struct {                        /* DMA Descriptor struct             */
  volatile U32 Stat;                    /* Packet Status                     */
           U32 Ctrl;                    /* Packet Control                    */
           U32 Buf1;                    /* Buffer_1 Address                  */
           U32 Buf2;                    /* Buffer_2/Next Descriptor Address  */
} DMA_Desc;

/* MAC Bus Mode Register (CSR0) */
#define CSR0_SWR            0x00000001  /* Software Reset                    */
#define CSR0_BAR            0x00000002  /* Bus Arbitration Scheme            */
#define CSR0_DSL            0x0000007C  /* Descriptor Skip Length mask       */
#define CSR0_BLE            0x00000080  /* Big/Little Endian data buffer     */
#define CSR0_PBL            0x00003F00  /* Programmable Burst Length         */
#define CSR0_TAP            0x000E0000  /* Transmit Automatic Polling        */
#define CSR0_DBO            0x00100000  /* Descriptor Byte Ordering Mode     */
#define CSR0_SPD            0x00200000  /* Clock Frequency Selection         */

/* Status and Control Register (CSR5) */
#define CSR5_TI             0x00000001  /* Transmit Interrupt                */
#define CSR5_TPS            0x00000002  /* Transmit Process Stopped          */
#define CSR5_TU             0x00000004  /* Transmit Buffer Unavailable       */
#define CSR5_UNF            0x00000020  /* Transmit Underflow                */
#define CSR5_RI             0x00000040  /* Receive Interrupt                 */
#define CSR5_RU             0x00000080  /* Receive Buffer Unavailable        */
#define CSR5_RPS            0x00000100  /* Receive Process Stopped           */
#define CSR5_ETI            0x00000400  /* Early Transmit Interrupt          */
#define CSR5_GTE            0x00000800  /* General-purpose Timer Expiration  */
#define CSR5_ERI            0x00004000  /* Early Receive Interrupt           */
#define CSR5_AIS            0x00008000  /* Abnormal Interrupt Summary        */
#define CSR5_NIS            0x00010000  /* Normal Interrupt Summary          */
#define CSR5_RS             0x000E0000  /* Receive Process State             */
#define CSR5_TS             0x00700000  /* Transmit Process State            */

/* Operation Mode Register (CSR6) */
#define CSR6_HP             0x00000001  /* Hash/Perfect Receive Filtering    */
#define CSR6_SR             0x00000002  /* Start/Stop Receive Command        */
#define CSR6_HO             0x00000004  /* Hash-Only Filtering Mode          */
#define CSR6_PB             0x00000008  /* Pass Bad Frames                   */
#define CSR6_IF             0x00000010  /* Inverse Filtering                 */
#define CSR6_PR             0x00000040  /* Promiscuous Mode                  */
#define CSR6_PM             0x00000080  /* Pass all Multicast                */
#define CSR6_FD             0x00000200  /* Full Duplex Mode                  */
#define CSR6_ST             0x00002000  /* Start/Stop Transmission Command   */
#define CSR6_TR             0x0000C000  /* Treshold Control Bits             */
#define CSR6_SF             0x00200000  /* Store and Forward                 */
#define CSR6_TTM            0x00400000  /* Transmit Treshold Mode            */
#define CSR6_RA             0x40000000  /* Receive All                       */

/* Interrupt Enable Register (CSR7) */
#define CSR7_TIE            0x00000001  /* Transmit Interrupt Enable         */
#define CSR7_TSE            0x00000002  /* Transmit Stopped Enable           */
#define CSR7_TUE            0x00000004  /* Transmit Buffer Unavailable Enable*/
#define CSR7_UNE            0x00000020  /* Transmit Underflow Enable         */
#define CSR7_RIE            0x00000040  /* Receive Interrupt Enable          */
#define CSR7_RUE            0x00000080  /* Receive Buffer Unavailable Enable */
#define CSR7_RSE            0x00000100  /* Receive Stopped Enable            */
#define CSR7_ETE            0x00000400  /* Early Transmit Interrupt Enable   */
#define CSR7_GTE            0x00000800  /* GP Timer Overflow Enable          */
#define CSR7_ERE            0x00004000  /* Early Receive Interrupt Enable    */
#define CSR7_AIE            0x00008000  /* Abnormal Interrupt Summary Enable */
#define CSR7_NIE            0x00010000  /* Normal Interrupt Summary Enable   */

/* Missed Frames and Overflow Counter Register (CSR8) */
#define CSR8_MFC            0x0000FFFF  /* Missed Frame Counter              */
#define CSR8_MFO            0x00010000  /* Missed Frame Overflow             */
#define CSR8_FOC            0x0FFE0000  /* FIFO Overflow Counter             */
#define CSR8_OCO            0x10000000  /* FIFO Overflow Counter Overflow    */

/* RMII Management Interface Register (CSR9) */
#define CSR9_MDC            0x00010000  /* RMII Management Clock             */
#define CSR9_MDO            0x00020000  /* RMII Management Write Data        */
#define CSR9_MDEN           0x00040000  /* RMII Management Operation Mode    */
#define CSR9_MDI            0x00080000  /* RMII Management Data in Signal    */

#define RMII_WR_TOUT        0x00050000  /* RMII Write timeout count          */
#define RMII_RD_TOUT        0x00050000  /* RMII Read timeout count           */

/* General-Purpose Timer and Interrupt Migitation Control Register (CSR11) */
#define CSR11_TIM           0x0000FFFF  /* Timer Value                       */
#define CSR11_CON           0x00010000  /* Continuous Mode                   */
#define CSR11_NRP           0x000E0000  /* Number of Received Packets        */
#define CSR11_RT            0x00F00000  /* Receive Timer                     */
#define CSR11_RT            0x00F00000  /* Receive Timer                     */
#define CSR11_NTP           0x07000000  /* Number of Transmit Packets        */
#define CSR11_TT            0x78000000  /* Transmit Timer                    */
#define CSR11_CS            0x80000000  /* Cycle Size                        */

/* RX Descriptor Status (RDES0) */
#define RDES0_ZERO          0x00000001  /* Zero for legal length packets     */
#define RDES0_CE            0x00000002  /* CRC Error                         */
#define RDES0_DB            0x00000004  /* Dribbling bit                     */
#define RDES0_RE            0x00000010  /* RMII Error                        */
#define RDES0_FT            0x00000020  /* Large Frame Type (>1500 bytes)    */
#define RDES0_CS            0x00000040  /* Collision Seen                    */
#define RDES0_TL            0x00000080  /* Frame Too Long (>1518 bytes)      */
#define RDES0_LS            0x00000100  /* Last Descriptor                   */
#define RDES0_FS            0x00000200  /* First Descriptor                  */
#define RDES0_MF            0x00000400  /* Multicast Frame                   */
#define RDES0_RF            0x00000800  /* Runt Frame                        */
#define RDES0_DE            0x00004000  /* Descriptor Error                  */
#define RDES0_ES            0x00008000  /* Error Summary                     */
#define RDES0_FL            0x3FFF0000  /* Frame Length                      */
#define RDES0_FF            0x40000000  /* Filtering Fail                    */
#define RDES0_OWN           0x80000000  /* Ownership Bit (1= EMAC)           */

/* RX Descriptor Control and Count (RDES1) */
#define RDES1_RBS1          0x000007FF  /* Buffer 1 Size                     */
#define RDES1_RBS2          0x003FF800  /* Buffer 2 Size                     */
#define RDES1_RCH           0x01000000  /* Second Address Chained            */
#define RDES1_RER           0x02000000  /* Receive End of Ring               */

/* TX Descriptor Status (TDES0) */
#define TDES0_DE            0x00000001  /* Deferred                          */
#define TDES0_UF            0x00000002  /* Underflow Error                   */
#define TDES0_CC            0x00000078  /* Collision Count                   */
#define TDES0_EC            0x00000100  /* Excessive Collisions              */
#define TDES0_LC            0x00000200  /* Late Collision                    */
#define TDES0_NC            0x00000400  /* No Carrier                        */
#define TDES0_LO            0x00000800  /* Loss of Carrier                   */
#define TDES0_ES            0x00008000  /* Error Summary                     */
#define TDES0_OWN           0x80000000  /* Ownership Bit (1= EMAC)           */

/* TX Descriptor Control (TDES1) */
#define TDES1_TBS1          0x000007FF  /* Buffer 1 Size                     */
#define TDES1_TBS2          0x003FF800  /* Buffer 2 Size                     */
#define TDES1_FT0           0x00400000  /* Filtering Type Bit 0              */
#define TDES1_DPD           0x00800000  /* Disable Padding                   */
#define TDES1_TCH           0x01000000  /* Second Address Chained            */
#define TDES1_TER           0x02000000  /* Transmit End of Ring              */
#define TDES1_AC            0x04000000  /* Add SRC Disable                   */
#define TDES1_SET           0x08000000  /* Setup Packet                      */
#define TDES1_FT1           0x10000000  /* Filtering Type Bit 1              */
#define TDES1_FS            0x20000000  /* First Descriptor                  */
#define TDES1_LS            0x40000000  /* Last Descriptor                   */
#define TDES1_IC            0x80000000  /* Interrupt on Completition         */

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

#define DP83848C_DEF_ADR    0x0100      /* Default PHY device address        */
#define DP83848C_ID         0x20005C90  /* PHY Identifier                    */

#endif

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
