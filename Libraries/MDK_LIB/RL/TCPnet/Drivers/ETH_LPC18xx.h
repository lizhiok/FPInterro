/*-----------------------------------------------------------------------------
 *      RL-ARM - TCPnet
 *-----------------------------------------------------------------------------
 *      Name:    ETH_LPC18XX.H
 *      Purpose: NXP LPC18XX Ethernet Controller Driver definitions
 *      Rev.:    V4.70
 *-----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#ifndef __ETH_LPC18XX_H
#define __ETH_LPC18XX_H

/* ETH Memory Buffer configuration. */
#define NUM_RX_BUF          4           /* 0x1800 for Rx (4*1536=6K)          */
#define NUM_TX_BUF          4           /* 0x0600 for Tx (2*1536=3K)          */
#define ETH_BUF_SIZE        1536        /* ETH Receive/Transmit buffer size   */

#define CSR_CLK_RANGE       0x0003      /* CSR Clock range 100-150MHz         */

/* DMA Descriptors. */
typedef struct {
  U32 volatile Stat;
  U32 Ctrl;
  U32 Addr;
  U32 Next;
  U32 Ext;
  U32 Rsvd;
  U32 RTSL;
  U32 RTSH;
} RX_Desc;

typedef struct {
  U32 volatile CtrlStat;
  U32 Size;
  U32 Addr;
  U32 Next;
  U32 Rsvd1;
  U32 Rsvd2;
  U32 RTSL;
  U32 RTSH;
} TX_Desc;

/* DMA Descriptor RX Packet Status */
#define DMA_RX_OWN          0x80000000  /* Own bit 1=DMA, 0=CPU               */
#define DMA_RX_AFM          0x40000000  /* Destination address filter fail    */
#define DMA_RX_FL           0x3FFF0000  /* Frame length mask                  */
#define DMA_RX_ES           0x00008000  /* Error summary                      */
#define DMA_RX_DE           0x00004000  /* Descriptor error                   */
#define DMA_RX_SAF          0x00002000  /* Source address filter fail         */
#define DMA_RX_LE           0x00001000  /* Length error                       */
#define DMA_RX_OE           0x00000800  /* Overflow error                     */
#define DMA_RX_VLAN         0x00000400  /* VLAN tag                           */
#define DMA_RX_FS           0x00000200  /* First descriptor                   */
#define DMA_RX_LS           0x00000100  /* Last descriptor                    */
#define DMA_RX_TSA          0x00000080  /* Timestamp Avail/IP checksum error  */
#define DMA_RX_LC           0x00000040  /* late collision                     */
#define DMA_RX_FT           0x00000020  /* Frame type                         */
#define DMA_RX_RWT          0x00000010  /* Receive watchdog timeout           */
#define DMA_RX_RE           0x00000008  /* Receive error                      */
#define DMA_RX_DRE          0x00000004  /* Dribble bit error                  */
#define DMA_RX_CE           0x00000002  /* CRC error                          */
#define DMA_RX_ESA          0x00000001  /* Extended Status/Rx MAC Address     */

/* DMA Descriptor RX Packet Control */
#define DMA_RX_RBS2         0x1FFF0000  /* Receive Buffer 2 size              */
#define DMA_RX_RER          0x00008000  /* Receive End of Ring                */
#define DMA_RX_RCH          0x00004000  /* Second Address Chained             */
#define DMA_RX_RBS1         0x00003FFF  /* Receive Buffer 1 size              */

/* DMA Descriptor TX Packet Control/Status */
#define DMA_TX_OWN          0x80000000  /* Own bit 1=DMA, 0=CPU               */
#define DMA_TX_IC           0x40000000  /* Interrupt on completition          */
#define DMA_TX_LS           0x20000000  /* Last segment                       */
#define DMA_TX_FS           0x10000000  /* First segment                      */
#define DMA_TX_DC           0x08000000  /* Disable CRC                        */
#define DMA_TX_DP           0x04000000  /* Disable pad                        */
#define DMA_TX_TTSE         0x02000000  /* Transmit time stamp enable         */
#define DMA_TX_CIC          0x00C00000  /* Checksum insertion control         */
#define DMA_TX_TER          0x00200000  /* Transmit end of ring               */
#define DMA_TX_TCH          0x00100000  /* Second address chained             */
#define DMA_TX_TTSS         0x00020000  /* Transmit time stamp status         */
#define DMA_TX_IHE          0x00010000  /* IP header error status             */
#define DMA_TX_ES           0x00008000  /* Error summary                      */
#define DMA_TX_JT           0x00004000  /* Jabber timeout                     */
#define DMA_TX_FF           0x00002000  /* Frame flushed                      */
#define DMA_TX_IPE          0x00001000  /* IP payload error                   */
#define DMA_TX_LC           0x00000800  /* Loss of carrier                    */
#define DMA_TX_NC           0x00000400  /* No carrier                         */
#define DMA_TX_LCOL         0x00000200  /* Late collision                     */
#define DMA_TX_EC           0x00000100  /* Excessive collision                */
#define DMA_TX_VF           0x00000080  /* VLAN frame                         */
#define DMA_TX_CC           0x00000078  /* Collision count                    */
#define DMA_TX_ED           0x00000004  /* Excessive deferral                 */
#define DMA_TX_UF           0x00000002  /* Underflow error                    */
#define DMA_TX_DB           0x00000001  /* Deferred bit                       */

#define DMA_RX_ERROR_MASK   (DMA_RX_ES | DMA_RX_LE | DMA_RX_RWT | \
                             DMA_RX_RE | DMA_RX_CE)
#define DMA_RX_SEG_MASK     (DMA_RX_FS | DMA_RX_LS)


/* MAC Configuration Register */
#define MCR_WD              0x00800000  /* Watchdog disable                   */
#define MCR_JD              0x00400000  /* Jabber disable                     */
#define MCR_JE              0x00100000  /* Jumbo Frame Enable                 */
#define MCR_IFG             0x000E0000  /* Interframe gap mask                */
#define MCR_DCRS            0x00010000  /* Disable carrier sense during tx    */
#define MCR_PS              0x00008000  /* Port Select                        */
#define MCR_FES             0x00004000  /* Speed                              */
#define MCR_DO              0x00002000  /* Disable Receive Own                */
#define MCR_LM              0x00001000  /* Loopback mode                      */
#define MCR_DM              0x00000800  /* Duplex mode                        */
#define MCR_IPC             0x00000400  /* Checksum offload                   */
#define MCR_DR              0x00000200  /* Disable Retry                      */
#define MCR_ACS             0x00000080  /* Automatic pad / CRC stripping      */
#define MCR_BL              0x00000060  /* Back-off limit mask                */
#define MCR_DF              0x00000010  /* Deferral check                     */
#define MCR_TE              0x00000008  /* Transmitter enable                 */
#define MCR_RE              0x00000004  /* Receiver enable                    */

/* MAC Frame Filter Register */
#define MFFR_RA             0x80000000  /* Receive all                        */
#define MFFR_SAF            0x00000200  /* Source address filter              */
#define MFFR_SAIF           0x00000100  /* Source address inverse filtering   */
#define MFFR_PCF            0x000000C0  /* Pass control frames mask           */
#define MFFR_DBF            0x00000020  /* Disable Broadcase Frames           */
#define MFFR_PAM            0x00000010  /* Pass all multicast                 */
#define MFFR_DAIF           0x00000008  /* Dest. address inverse filtering    */
#define MFFR_PM             0x00000001  /* Promiscuous mode                   */

/* MAC MII Address Register */
#define MMAR_GB             0x00000001  /* MII busy                           */
#define MMAR_MW             0x00000002  /* MII write                          */
#define MMAR_CR             0x0000001C  /* Clock range                        */
#define MMAR_GR             0x000007C0  /* MII register address mask          */
#define MMAR_PA             0x0000F800  /* PHY address mask                   */

/* MAC MII Data Register */
#define MMDR_GD             0x0000FFFF  /* MII 16-bit rw data                 */

/* MAC Flow Control Register */
#define MFCR_PT             0xFFFF0000  /* Pause time mask                    */
#define MFCR_DZQP           0x00000080  /* Disable Zero-Quanta Pause          */
#define MFCR_PLT            0x00000030  /* Pause low threshold                */
#define MFCR_UP             0x00000008  /* Unicaste pause frame detect        */
#define MFCR_RFE            0x00000004  /* Receive flow control enable        */
#define MFCR_TFE            0x00000002  /* Transmit flow control enable       */
#define MFCR_FCB            0x00000001  /* Flow Ctrl Busy/Backpressure Act.   */

/* DMA Status Register */
#define DSR_TSTS            0x20000000  /* Timestamp trigger status           */
#define DSR_PMTS            0x10000000  /* PMT status                         */
#define DSR_MMCS            0x08000000  /* MMC status                         */
#define DSR_EBS             0x03800000  /* Error bits status mask             */
#define DSR_TPS             0x00700000  /* Transmit process state             */
#define DSR_RPS             0x000E0000  /* Receive process state              */

#define DSR_NIS             0x00010000  /* Normal interrupt summary           */
#define DSR_AIE             0x00008000  /* Abnormal interrupt summary         */
#define DSR_ERI             0x00004000  /* Early receive interrupt            */
#define DSR_FBI             0x00002000  /* Fatal bus error interrupt          */
#define DSR_ETI             0x00000400  /* Early transmit interrupt           */
#define DSR_RWT             0x00000200  /* Receive watchdog timeout status    */
#define DSR_RPSS            0x00000100  /* Receive process stopped status     */
#define DSR_RU              0x00000080  /* Receive buffer unavailable status  */
#define DSR_RI              0x00000040  /* Receive interrupt                  */
#define DSR_UNF             0x00000020  /* Transmit underflow status          */
#define DSR_OVF             0x00000010  /* Receive overflow status            */
#define DSR_TJT             0x00000008  /* Transmit jabber timeout            */
#define DSR_TU              0x00000004  /* Transmit buffer unavailable        */
#define DSR_TPSS            0x00000002  /* Transmit process stopped status    */
#define DSR_TI              0x00000001  /* Transmit interrupt                 */

/* DMA Bus Mode Register */
#define DBMR_TXPR           0x08000000  /* DMA Tx priority                    */
#define DBMR_MB             0x04000000  /* Mixed burst                        */
#define DBMR_AAL            0x02000000  /* Address-aligned beats              */
#define DBMR_PBL8x          0x01000000  /* 8 x PBL mode                       */
#define DBMR_USP            0x00800000  /* Use separate PBL                   */
#define DBMR_RPBL           0x007E0000  /* Rx DMA PBL mask                    */
#define DBMR_FB             0x00010000  /* Fixed burst                        */
#define DBMR_PR             0x0000C000  /* Rx-to-Tx priority ratio            */
#define DBMR_PBL            0x00003F00  /* Programmable burst length mask     */
#define DBMR_ATDS           0x00000080  /* Alternate (Enh.) descriptor size   */
#define DBMR_DSL            0x0000007C  /* Descriptor skip length             */
#define DBMR_DA             0x00000002  /* DMA arbitration                    */
#define DBMR_SWR            0x00000001  /* Software reset                     */

/* DMA Operation Mode Register */
#define DOMR_DTCEFD         0x04000000  /* Dropping of TCP/IP chksum err dis. */
#define DOMR_RSF            0x02000000  /* Receive store and forward          */
#define DOMR_DFRF           0x01000000  /* Disable flushing of received frms  */
#define DOMR_TSF            0x00200000  /* Transmit storea and forward        */
#define DOMR_FTF            0x00100000  /* Flush transmit FIFO                */
#define DOMR_TTC            0x0001C000  /* Transmit treshold control mask     */
#define DOMR_ST             0x00002000  /* Start/stop transmission            */
#define DOMR_FEF            0x00000080  /* Forward error frames               */
#define DOMR_FUGF           0x00000040  /* Forward undersized good frames     */
#define DOMR_RTC            0x00000018  /* Receive threshold control mask     */
#define DOMR_OSF            0x00000004  /* Operate on second frame            */
#define DOMR_SR             0x00000002  /* Start/stop receive                 */

/* DMA Interrupt Enable Register */
#define INT_NISE            0x00010000  /* Normal interrupt summary           */
#define INT_AISE            0x00008000  /* Abnormal interrupt summary         */
#define INT_ERIE            0x00004000  /* Early receive interrupt            */
#define INT_FBEIE           0x00002000  /* Fatal bus error interrupt          */
#define INT_ETIE            0x00000400  /* Early transmit interrupt           */
#define INT_RWTIE           0x00000200  /* Receive watchdog timeout interrupt */
#define INT_RPSIE           0x00000100  /* Receive process stopped intterrupt */
#define INT_RBUIE           0x00000080  /* Receive buffer unavailable inter.  */
#define INT_RIE             0x00000040  /* Receive interrupt                  */
#define INT_TUIE            0x00000020  /* Transmit underflow interrupt       */
#define INT_ROIE            0x00000010  /* Receive overflow interrupt         */
#define INT_TJTIE           0x00000008  /* Transmit jabber timeout interrupt  */
#define INT_TBUIE           0x00000004  /* Transmit buffer unavailable inter. */
#define INT_TPSIE           0x00000002  /* Transmit process stopped interrupt */
#define INT_TIE             0x00000001  /* Transmit interrupt                 */

/* Wait for operation to finish timeout */
#define TIMEOUT             0x10000

/* MII Management Command Register */
#define MII_WR_TOUT         0x00050000  /* MII Write timeout count            */
#define MII_RD_TOUT         0x00050000  /* MII Read timeout count             */

/* MII Management Address Register */
#define MADR_PHY_ADR        0x00001F00  /* PHY Address Mask                   */

/* DP83848C PHY */
#define DP83848C_DEF_ADR    0x01        /* Default PHY device address         */
#define DP83848C_ID         0x20005C90  /* PHY Identifier                     */

/* DP83848C PHY Registers */
#define PHY_REG_BMCR        0x00        /* Basic Mode Control Register        */
#define PHY_REG_BMSR        0x01        /* Basic Mode Status Register         */
#define PHY_REG_IDR1        0x02        /* PHY Identifier 1                   */
#define PHY_REG_IDR2        0x03        /* PHY Identifier 2                   */
#define PHY_REG_ANAR        0x04        /* Auto-Negotiation Advertisement     */
#define PHY_REG_ANLPAR      0x05        /* Auto-Neg. Link Partner Abitily     */
#define PHY_REG_ANER        0x06        /* Auto-Neg. Expansion Register       */
#define PHY_REG_ANNPTR      0x07        /* Auto-Neg. Next Page TX             */

/* PHY Extended Registers */
#define PHY_REG_STS         0x10        /* Status Register                    */
#define PHY_REG_MICR        0x11        /* MII Interrupt Control Register     */
#define PHY_REG_MISR        0x12        /* MII Interrupt Status Register      */
#define PHY_REG_FCSCR       0x14        /* False Carrier Sense Counter        */
#define PHY_REG_RECR        0x15        /* Receive Error Counter              */
#define PHY_REG_PCSR        0x16        /* PCS Sublayer Config. and Status    */
#define PHY_REG_RBR         0x17        /* RMII and Bypass Register           */
#define PHY_REG_LEDCR       0x18        /* LED Direct Control Register        */
#define PHY_REG_PHYCR       0x19        /* PHY Control Register               */
#define PHY_REG_10BTSCR     0x1A        /* 10Base-T Status/Control Register   */
#define PHY_REG_CDCTRL1     0x1B        /* CD Test Control and BIST Extens.   */
#define PHY_REG_EDCR        0x1D        /* Energy Detect Control Register     */

/* PHY Control and Status bits  */
#define PHY_FULLD_100M      0x2100      /* Full Duplex 100Mbit                */
#define PHY_HALFD_100M      0x2000      /* Half Duplex 100Mbit                */
#define PHY_FULLD_10M       0x0100      /* Full Duplex 10Mbit                 */
#define PHY_HALFD_10M       0x0000      /* Half Duplex 10MBit                 */
#define PHY_AUTO_NEG        0x1000      /* Select Auto Negotiation            */
#define PHY_AUTO_NEG_DONE   0x0020      /* AutoNegotiation Complete (BMSR reg)*/
#define PHY_BMCR_RESET      0x8000      /* Reset bit (BMCR reg)               */
#define LINK_VALID_STS      0x0001      /* Link Valid Status (REG_STS reg)    */
#define FULL_DUP_STS        0x0004      /* Full Duplex Status (REG_STS reg)   */
#define SPEED_10M_STS       0x0002      /* 10Mbps Status (REG_STS reg)        */

/*  Misc    */
#define ETHERNET_RST  (1 << 22)         /* Reset Unit ETH reset bit           */

#endif /* __ETH_LPC18XX_H */

/*------------------------------------------------------------------------------
 * end of file
 *----------------------------------------------------------------------------*/
