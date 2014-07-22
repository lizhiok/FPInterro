/*----------------------------------------------------------------------------
 *      RL-ARM - TCPnet
 *----------------------------------------------------------------------------
 *      Name:    STR9_ENET.H
 *      Purpose: ST STR912 ENET Ethernet Controller Driver definitions
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#ifndef __STR9_ENET_H
#define __STR9_ENET_H

/* ENET Memory Buffer configuration. */
#define NUM_RX_BUF          4           /* 0x1800 for Rx (4*1536=6K)         */
#define NUM_TX_BUF          3           /* 0x0900 for Tx (3*1536=4.5K)       */
#define ETH_BUF_SIZE        1536        /* ENET Receive/Transmit buffer size */

/* DMA Descriptor. */
typedef struct {
  U32 Ctrl;
  U32 Addr;
  U32 Next;
  U32 Stat;
} DMA_Desc;

/* DMA Status and Control Register */
#define SCR_TX_FIFO_SIZE    0xF0000000  /* Transmit FIFO size                */
#define SCR_TX_IO_WIDTH     0x0C000000  /* Transmit bus data width           */
#define SCR_TX_CH_STAT      0x03000000  /* Transmit channel information      */
#define SCR_RX_FIFO_SIZE    0x00F00000  /* Receive FIFO size                 */
#define SCR_RX_IO_WIDTH     0x000C0000  /* Receive bus data width            */
#define SCR_RX_CH_STAT      0x00030000  /* Receive channel information       */
#define SCR_REVISION        0x0000FF00  /* DMA Hardware revision number      */
#define SCR_TX_BURST_SIZE   0x000000C0  /* Transmit max. burst size          */
#define SCR_RX_BURST_SIZE   0x00000030  /* Receive max. burst size           */
#define SCR_LOOPB           0x00000002  /* Loopback mode                     */
#define SCR_SRESET          0x00000001  /* MAC DMA Software reset            */

#define SCR_RX_BURST_DEF    0x00000000  /* Default value RX mode burst size  */
#define SCR_TX_BURST_DEF    0x000000C0  /* Default value TX mode burst size  */

/* DMA Interrupt Enable and Interrupt Status Registers */
#define INT_TX_CURR_DONE    0x80000000  /* TX DMA completed current transfer */
#define INT_MAC_8023        0x10000000  /* MAC 802.3 interrupt               */
#define INT_TX_MERR         0x02000000  /* Master error during TX            */
#define INT_TX_DONE         0x00800000  /* TX master DMA completed           */
#define INT_TX_NEXT         0x00400000  /* Invalid TX descriptor fetched     */
#define INT_TX_TOUT         0x00080000  /* Transmit timeout                  */
#define INT_TX_ENTRY        0x00040000  /* TX DMA trigger entry error        */
#define INT_TX_FULL         0x00020000  /* TX FIFO is full                   */
#define INT_TX_EMPTY        0x00010000  /* TX FIFO is empty                  */
#define INT_RX_CURR_DONE    0x00008000  /* RX DMA completed current transfer */
#define INT_RX_MERR         0x00000200  /* Master error during RX            */
#define INT_RX_DONE         0x00000080  /* RX master DMA completed           */
#define INT_RX_NEXT         0x00000040  /* Invalid RX descriptor fetched     */
#define INT_RX_PKT_LOST     0x00000020  /* RX packet lost because DMA busy   */
#define INT_RX_TOUT         0x00000008  /* Receive timeout                   */
#define INT_RX_ENTRY        0x00000004  /* RX DMA trigger entry error        */
#define INT_RX_FULL         0x00000002  /* RX FIFO is full                   */
#define INT_RX_EMPTY        0x00000001  /* RX FIFO is empty                  */

/* DMA Clock Configuration Register */
#define CCR_SEL_CLK         0x0000000C  /* HCLK, PCLK clock relations        */
#define CCR_SEL_CLK_DEF     0x00000004  /* HCLK = 2*PCLK                     */

/* DMA RX Start Register  */
#define RXSTR_DFETCH_DLY    0x00FFFF00  /* Descriptor fetch delay            */
#define RXSTR_COLL_SEEN     0x00000080  /* Late Collision Seen control bit   */
#define RXSTR_RUNT_FRAME    0x00000040  /* Discard Runt frames               */
#define RXSTR_FILTER_FAIL   0x00000020  /* Discard MAC filtered frames       */
#define RXSTR_START_FETCH   0x00000004  /* Start RX DMA fetching descriptors */
#define RXSTR_DMA_EN        0x00000001  /* DMA enable                        */

#define RXSTR_DFETCH_DEF    0x00010000  /* Default value for DFETCH delay    */

/* DMA RX Control Register */
#define RXCR_ADDR_WRAP      0xFFC00000  /* DMA address counter wrap location */
#define RXCR_ENTRY_TRIG     0x003E0000  /* Entry trigger count               */
#define RXCR_DLY_EN         0x00008000  /* DMA trigger delay enable          */
#define RXCR_NEXT_EN        0x00004000  /* Next Descriptor Fetch mode enable */
#define RXCR_CONT_EN        0x00001000  /* Continuous Mode Enable            */
#define RXCR_DMA_XFERCNT    0x00000FFF  /* DMA transfer count                */

/* DMA RX Base Address Register */
#define RXSAR_RX_ADDR       0xFFFFFFFC  /* Address for master DMA transfer   */
#define RXSAR_FIX_ADDR      0x00000002  /* Use Fixed address                 */
#define RXSAR_WRAP_EN       0x00000001  /* Wrap Enable of DMA trans. address */

/* DMA RX Next Descriptor Address Register */
#define RXNDAR_DESC_ADDR    0xFFFFFFFC  /* RX DMA Next Descriptor pointer    */
#define RXNDAR_NPOL_EN      0x00000001  /* Next Descriptor Polling Enable    */

/* DMA RX Status Register */
#define RXSR_ENTRIES        0x3F000000  /* RX FIFO entry count               */
#define RXSR_DMA_PTR        0x001F0000  /* DMA RX FIFO pointer               */
#define RXSR_IO_PTR         0x00001F00  /* IO RX FIFO pointer                */
#define RXSR_DELAY_T        0x00000008  /* RX FIFO delay timeout             */
#define RXSR_ENTRY_T        0x00000004  /* RX FIFO entry treshhold flag      */
#define RXSR_FULL           0x00000002  /* RX FIFO full flag                 */
#define RXSR_EMPTY          0x00000001  /* RX FIFO empty flag                */

/* DMA TX Start Register  */
#define TXSTR_DFETCH_DLY    0x00FFFF00  /* Descriptor fetch delay            */
#define TXSTR_UNDER_RUN     0x00000020  /* Underrun enabled                  */
#define TXSTR_START_FETCH   0x00000004  /* Start TX DMA fetching descriptors */
#define TXSTR_DMA_EN        0x00000001  /* DMA enable                        */

#define TXSTR_DFETCH_DEF    0x00010000  /* Default value for DFETCH delay    */

/* DMA TX Control Register */
#define TXCR_ADDR_WRAP      0xFFC00000  /* DMA address counter wrap location */
#define TXCR_ENTRY_TRIG     0x003E0000  /* Entry trigger count               */
#define TXCR_DLY_EN         0x00008000  /* DMA trigger delay enable          */
#define TXCR_NEXT_EN        0x00004000  /* Next Descriptor Fetch mode enable */
#define TXCR_CONT_EN        0x00001000  /* Continuous Mode Enable            */
#define TXCR_DMA_XFERCNT    0x00000FFF  /* DMA transfer count                */

/* DMA Base Address Register */
#define TXSAR_RX_ADDR       0xFFFFFFFC  /* Address for master DMA transfer   */
#define TXSAR_FIX_ADDR      0x00000002  /* Use Fixed address                 */
#define TXSAR_WRAP_EN       0x00000001  /* Wrap Enable of DMA trans. address */

/* DMA Next Descriptor Address Register */
#define TXNDAR_DESC_ADDR    0xFFFFFFFC  /* RX DMA Next Descriptor pointer    */
#define TXNDAR_NPOL_EN      0x00000001  /* Next Descriptor Polling Enable    */

/* DMA TX Status Register */
#define TXSR_ENTRIES        0x3F000000  /* TX FIFO entry count               */
#define TXSR_DMA_PTR        0x001F0000  /* DMA TX FIFO pointer               */
#define TXSR_IO_PTR         0x00001F00  /* IO TX FIFO pointer                */
#define TXSR_DELAY_T        0x00000008  /* TX FIFO delay timeout             */
#define TXSR_ENTRY_T        0x00000004  /* TX FIFO entry treshhold flag      */
#define TXSR_FULL           0x00000002  /* TX FIFO full flag                 */
#define TXSR_EMPTY          0x00000001  /* TX FIFO empty flag                */

/* DMA Descriptor TX/RX Packet Control */
#define DMA_CTRL_FLEN       0x00000FFF  /* Frame length (max 4096)           */
#define DMA_CTRL_NEXT_EN    0x00004000  /* Next Descriptor Fetch enable      */

/* DMA Descriptor TX/RX Packet Address */
#define DMA_ADDR_MASK       0xFFFFFFFC  /* DMA Start Address Mask (32 bit)   */
#define DMA_ADDR_FIX_ADDR   0x00000002  /* Disable incrementing of DMA_ADDR  */
#define DMA_ADDR_WRAP_EN    0x00000001  /* Wrap Enable of DMA transf.address */

/* DMA Descriptor TX/RX Next Address */
#define DMA_NEXT_ADDR_MASK  0xFFFFFFFC  /* Next descriptor starting address  */
#define DMA_NEXT_POL_EN     0x00000001  /* Next Descriptor Polling Enable    */

/* DMA Descriptor TX Packet Status */
#define DMA_TX_PKT_RTRY     0x80000000  /* Packet Retry                      */
#define DMA_TX_BCNT         0x7FFC0000  /* Byte Counter                      */
#define DMA_TX_VALID        0x00010000  /* Packet Valid indicator            */
#define DMA_TX_CCNT         0x00003C00  /* Collision Count Mask              */
#define DMA_TX_LCOLLO       0x00000200  /* Late Collision Observed           */
#define DMA_TX_DEFER        0x00000100  /* Deferred                          */
#define DMA_TX_URUN         0x00000080  /* Under Run                         */
#define DMA_TX_ECOLL        0x00000040  /* Excessive Collisions              */
#define DMA_TX_LCOLL        0x00000020  /* Late Collision                    */
#define DMA_TX_EXCD         0x00000010  /* Excessive Deferral                */
#define DMA_TX_LOC          0x00000008  /* Loss of Carrier                   */
#define DMA_TX_NOC          0x00000004  /* No Carrier                        */
#define DMA_TX_FA           0x00000001  /* Frame Aborted                     */

/* DMA Descriptor RX Packet Status */
#define DMA_RX_FA           0x80000000  /* Frame Aborted                     */
#define DMA_RX_PKT_FILT     0x40000000  /* Packet Filter                     */
#define DMA_RX_FLT_FAIL     0x20000000  /* Filtering Fail                    */
#define DMA_RX_BCAST        0x10000000  /* BroadCast Frame                   */
#define DMA_RX_MCAST        0x08000000  /* Multicast Frame                   */
#define DMA_RX_UCTRL_FR     0x04000000  /* Unsupported Control Frame         */
#define DMA_RX_CTL_FR       0x02000000  /* Control Frame                     */
#define DMA_RX_LEN_ERR      0x01000000  /* Length Error                      */
#define DMA_RX_VLAN1_FR     0x00400000  /* One-Level VLAN Frame              */
#define DMA_RX_VLAN2_FR     0x00800000  /* Two-Level VLAN Frame              */
#define DMA_RX_CRC_ERR      0x00200000  /* CRC Error                         */
#define DMA_RX_EBIT         0x00100000  /* Extra Bits                        */
#define DMA_RX_MII_ERR      0x00080000  /* MII Error                         */
#define DMA_RX_FTYPE        0x00040000  /* Frame Type                        */
#define DMA_RX_LCOLL        0x00020000  /* Late Collision Seen               */
#define DMA_RX_VALID        0x00010000  /* Valid bit indicator               */
#define DMA_RX_RUNT_FR      0x00008000  /* Runt Frame                        */
#define DMA_RX_WDTO         0x00004000  /* Watchdog Timeout                  */
#define DMA_RX_FCI          0x00002000  /* False Carrier Indication          */
#define DMA_RX_OLEN_FR      0x00001000  /* Over Lenght Frame                 */
#define DMA_RX_FLEN         0x000007FF  /* Frame Length (max 2047)           */

#define DMA_RX_ERROR_MASK   (DMA_RX_FA      | DMA_RX_LEN_ERR | \
                             DMA_RX_CRC_ERR | DMA_RX_MII_ERR | DMA_RX_LCOLL  | \
                             DMA_RX_VALID   | DMA_RX_RUNT_FR | DMA_RX_OLEN_FR)
/* MAC Control Register */
#define MCR_RA              0x80000000  /* Receive All                       */
#define MCR_EN              0x40000000  /* Endianity, 0= Little endian mode  */
#define MCR_PS              0x03000000  /* Prescaler Bits                    */
#define MCR_DRO             0x00800000  /* Disable Receive Own frames        */
#define MCR_LM              0x00600000  /* Loopback Mode                     */
#define MCR_FDM             0x00100000  /* Full Duplex Mode                  */
#define MCR_AFM             0x000E0000  /* Address Filtering Mode            */
#define MCR_PWF             0x00010000  /* Pass Wrong Frame                  */
#define MCR_VFM             0x00008000  /* VLAN Filtering Mode               */
#define MCR_ELC             0x00001000  /* Enable Late Collision             */
#define MCR_DBF             0x00000800  /* Disable Broadcast Frame 0= enabled*/
#define MCR_DPR             0x00000400  /* Disable Packet Retry              */
#define MCR_RVFF            0x00000200  /* VCI Rx Frame filtering            */
#define MCR_APR             0x00000100  /* Automatic Pad Removal             */
#define MCR_BL              0x000000C0  /* Back-off Limit                    */
#define MCR_DCE             0x00000020  /* Deferral Check Enable             */
#define MCR_RVBE            0x00000010  /* Reception VCI Burst Enable        */
#define MCR_TE              0x00000008  /* Transmission Enable               */
#define MCR_RE              0x00000004  /* Reception Enable                  */
#define MCR_RCFA            0x00000001  /* Reverse Control Frame Address     */

#define MCR_PS_DEF          0x01000000  /* Prescaler = 2, hclk > 50MHz       */
#define VLAN_ETPID          0x00008100  /* Ethernet Tag Protocol Identifier  */
#define MCR_PFM             0x00000000  /* Perfect MAC filtering mode        */
#define MCR_PFM_MCAST       0x000A0000  /* Perfect MAC filtering & Multicast */

/* MAC MII Address Register */
#define MIIA_PADDR          0x0000F800  /* PHY Physical Address              */
#define MIIA_RADDR          0x000007C0  /* PHY Register Address              */
#define MIIA_PR             0x00000004  /* Preamble Removal Bit              */
#define MIIA_WR             0x00000002  /* Write/Not read bit                */
#define MIIA_BUSY           0x00000001  /* Busy bit, start read/write operat.*/

#define MII_WR_TOUT         0x00050000  /* MII Write timeout count           */
#define MII_RD_TOUT         0x00050000  /* MII Read timeout count            */

/* MAC MII Control Frame Register */
#define MCF_PTIME           0xFFFF0000  /* Pause Time for tx of ctrl frame   */
#define MCF_PCF             0x00000004  /* Pass Control Frame                */
#define MCF_FCE             0x00000002  /* Flow Control Enable               */
#define MCF_FCB             0x00000001  /* Flow Control Busy                 */

/* MAC Transmission Status Register */
#define MTS_PR              0x80000000  /* Packet Retry                      */
#define MTS_BC              0x7FFC0000  /* Byte Count                        */
#define MTS_CC              0x00003C00  /* Collision Count                   */
#define MTS_LCO             0x00000200  /* Late Collision Observed           */
#define MTS_DEF             0x00000100  /* Transmission Deferred             */
#define MTS_UR              0x00000080  /* Under run                         */
#define MTS_EC              0x00000040  /* Excessive collision               */
#define MTS_LC              0x00000020  /* Late collision                    */
#define MTS_ED              0x00000010  /* Excessive deferral                */
#define MTS_LOC             0x00000008  /* Loss of Carrier                   */
#define MTS_NC              0x00000004  /* No Carrier                        */
#define MTS_FA              0x00000001  /* Frame Aborted                     */

/* MAC Reception Status Register */
#define MRS_FA              0x80000000  /* Frame Aborted                     */
#define MRS_PF              0x40000000  /* Packet filter                     */
#define MRS_FF              0x20000000  /* Filtering fail                    */
#define MRS_BF              0x10000000  /* Broadcast Frame                   */
#define MRS_MCF             0x08000000  /* Multicast Frame                   */
#define MRS_UCF             0x04000000  /* Unsupported Control Frame         */
#define MRS_CF              0x02000000  /* Control Frame                     */
#define MRS_LE              0x01000000  /* Length Error                      */
#define MRS_VL2             0x00800000  /* Vlan2 Tag                         */
#define MRS_VL1             0x00400000  /* Vlan1 Tag                         */
#define MRS_CE              0x00200000  /* CRC Error                         */
#define MRS_EB              0x00100000  /* Extra Bit                         */
#define MRS_ME              0x00080000  /* MII Error                         */
#define MRS_FT              0x00040000  /* Frame Type                        */
#define MRS_LC              0x00020000  /* Late Collision                    */
#define MRS_OL              0x00010000  /* Over Length                       */
#define MRS_RF              0x00008000  /* Runt Frame                        */
#define MRS_WT              0x00004000  /* Watchdog Time-out                 */
#define MRS_FCI             0x00002000  /* False Carrier Indication          */
#define MRS_FL              0x000007FF  /* Frame Length                      */

/* STE100P PHY Registers */
#define PHY_REG_XCR         0           /* XCVR Control Register             */
#define PHY_REG_XSR         1           /* XCVR Status Register              */
#define PHY_REG_PID1        2           /* PHY Identifier 1                  */
#define PHY_REG_PID2        3           /* PHY Identifier 2                  */
#define PHY_REG_ANA         4           /* Auto-Negotiation Advertisement    */
#define PHY_REG_ANLPA       5           /* Auto-Neg. Link Partner Abitily    */
#define PHY_REG_ANE         6           /* Auto-Neg. Expansion Register      */
#define PHY_REG_XCIIS       17          /* XCVR Config. and Interrupt Status */
#define PHY_REG_XIE         18          /* XCVR Interrupt Enable Register    */
#define PHY_REG_100CTR      19          /* 100Base-TX PHY Control/Status     */
#define PHY_REG_XMC         20          /* XCVE Mode Control Register        */

#define PHY_FULLD_100M      0x2100      /* Full Duplex 100Mbit               */
#define PHY_HALFD_100M      0x2000      /* Half Duplex 100Mbit               */
#define PHY_FULLD_10M       0x0100      /* Full Duplex 10Mbit                */
#define PHY_HALFD_10M       0x0000      /* Half Duplex 10MBit                */
#define PHY_AUTO_NEG        0x3000      /* Select Auto Negotiation           */

#define STE100P_DEF_ADR     0x0000      /* Default PHY device address        */
#define STE100P_ID          0x1C040010  /* PHY Identifier                    */

#endif

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/

