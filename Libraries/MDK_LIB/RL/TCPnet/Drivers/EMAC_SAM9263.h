/*----------------------------------------------------------------------------
 *      RL-ARM - TCPnet
 *----------------------------------------------------------------------------
 *      Name:    EMAC_SAM9263.H
 *      Purpose: Atmel AT91SAM9263 Ethernet Controller Driver definitions
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#ifndef __EMAC_SAM9263_H
#define __EMAC_SAM9263_H

#include <AT91SAM9263.H>                      /* AT91SAM9263 definitions     */

/* EMAC Memory Buffer configuration. */
#define NUM_RX_BUF          64          /* 0x2000 for Rx (64*128=8K)         */
#define ETH_RX_BUF_SIZE     128         /* EMAC Receive buffer size.         */
#define ETH_RX_BUF_NUM      (1536/ETH_RX_BUF_SIZE)

#define NUM_TX_BUF          1           /* 0x0600 for Tx                     */
#define ETH_TX_BUF_SIZE     1536        /* EMAC Transmit buffer size         */

#define AT91C_PHY_ADDR      0

/* Absolute IO access macros */
#define pEMAC   AT91C_BASE_MACB
#define pPIOC   AT91C_BASE_PIOC
#define pPIOE   AT91C_BASE_PIOE
#define pRSTC   AT91C_BASE_RSTC
#define pAIC    AT91C_BASE_AIC
#define pPMC    AT91C_BASE_PMC

typedef struct {
  U32 addr;
  U32 stat;
} Buf_Desc;

/* Receive status defintion */
#define RD_BROADCAST_ADDR   (1U << 31)  /* Broadcat address detected         */
#define RD_MULTICAST_HASH   (1U << 30)  /* MultiCast hash match              */
#define RD_UNICAST_HASH     (1U << 29)  /* UniCast hash match                */
#define RD_EXTERNAL_ADDR    (1U << 28)  /* External Address match            */
#define RD_SA1_ADDR         (1U << 26)  /* Specific address 1 match          */
#define RD_SA2_ADDR         (1U << 25)  /* Specific address 2 match          */
#define RD_SA3_ADDR         (1U << 24)  /* Specific address 3 match          */
#define RD_SA4_ADDR         (1U << 23)  /* Specific address 4 match          */
#define RD_TYPE_ID          (1U << 22)  /* Type ID match                     */
#define RD_VLAN_TAG         (1U << 21)  /* VLAN tag detected                 */
#define RD_PRIORITY_TAG     (1U << 20)  /* PRIORITY tag detected             */
#define RD_VLAN_PRIORITY    (7U << 17)  /* PRIORITY Mask                     */
#define RD_CFI_IND          (1U << 16)  /* CFI indicator                     */
#define RD_EOF              (1U << 15)  /* EOF                               */
#define RD_SOF              (1U << 14)  /* SOF                               */
#define RD_RBF_OFFSET       (3U << 12)  /* Receive Buffer Offset Mask        */
#define RD_LENGTH_MASK      0x07FF      /* Length of frame mask              */

/* Transmit Status definition */
#define TD_TRANSMIT_OK      (1U << 31)  /* Transmit OK                       */
#define TD_TRANSMIT_WRAP    (1U << 30)  /* Wrap bit: mark the last descriptor*/
#define TD_TRANSMIT_ERR     (1U << 29)  /* RLE:transmit error                */
#define TD_TRANSMIT_UND     (1U << 28)  /* Transmit Underrun                 */
#define TD_BUF_EX           (1U << 27)  /* Buffers exhausted in mid frame    */
#define TD_TRANSMIT_NO_CRC  (1U << 16)  /* No CRC will be appended to frame  */
#define TD_LAST_BUF         (1U << 15)  /* Last buffer in TX frame           */
#define TD_LENGTH_MASK      0x07FF      /* Length of frame mask              */

#define AT91C_OWNERSHIP_BIT 0x00000001  /* Buffer owned by software          */

/* DM9161 PHY registers. */
#define PHY_REG_BMCR        0x00        /* Basic mode control register       */
#define PHY_REG_BMSR        0x01        /* Basic mode status register        */
#define PHY_REG_PHYID1      0x02        /* PHY ID identifier #1              */
#define PHY_REG_PHYID2      0x03        /* PHY ID identifier #2              */
#define PHY_REG_ANAR        0x04        /* AutoNegotiation Advertisement reg.*/
#define PHY_REG_ANLPAR      0x05        /* AutoNeg.Link partner ability reg  */
#define PHY_REG_ANER        0x06        /* AutoNeg. Expansion register       */
#define PHY_REG_DSCR        0x10        /* DAVICOM Specified Config. reg     */
#define PHY_REG_DSCSR       0x11        /* DAVICOM Spec. Config/Status reg   */
#define PHY_REG_10BTCSR     0x12        /* 10BASET Configuration/Status reg  */
#define PHY_REG_PWDOR       0x13        /* Power Down Control Register       */
#define PHY_REG_SCR         0x14        /* Specified Config register         */
#define PHY_REG_INTR        0x15        /* Interrupt register                */
#define PHY_REG_RECR        0x16        /* Receive Error Counter register    */
#define PHY_REG_DISCR       0x17        /* Disconnect Counter register       */
#define PHY_REG_RLSR        0x18        /* Hardware Reset Latch State reg.   */

/* Basic mode control register. */
#define BMCR_RESV           0x007f      /* Unused...                         */
#define BMCR_CTST           0x0080      /* Collision test                    */
#define BMCR_FULLDPLX       0x0100      /* Full duplex                       */
#define BMCR_ANRESTART      0x0200      /* Auto negotiation restart          */
#define BMCR_ISOLATE        0x0400      /* Disconnect DM9161 from MII        */
#define BMCR_PDOWN          0x0800      /* Powerdown the DM9161              */
#define BMCR_ANENABLE       0x1000      /* Enable auto negotiation           */
#define BMCR_SPEED100       0x2000      /* Select 100Mbps                    */
#define BMCR_LOOPBACK       0x4000      /* TXD loopback bits                 */
#define BMCR_RESET          0x8000      /* Reset the DM9161                  */

#define PHY_FULLD_100M      0x2100      /* Full Duplex 100Mbit               */
#define PHY_HALFD_100M      0x2000      /* Half Duplex 100Mbit               */
#define PHY_FULLD_10M       0x0100      /* Full Duplex 10Mbit                */
#define PHY_HALFD_10M       0x0000      /* Half Duplex 10MBit                */
#define PHY_AUTO_NEG        0x3100      /* Select Auto Negotiation           */

/* Basic mode status register. */
#define BMSR_ERCAP          0x0001      /* Ext-reg capability                */
#define BMSR_JCD            0x0002      /* Jabber detected                   */
#define BMSR_LINKST         0x0004      /* Link status                       */
#define BMSR_ANEGCAPABLE    0x0008      /* Able to do auto-negotiation       */
#define BMSR_RFAULT         0x0010      /* Remote fault detected             */
#define BMSR_ANEGCOMPLETE   0x0020      /* Auto-negotiation complete         */
#define BMSR_MIIPRESUP      0x0040      /* MII Frame Preamble Suppression    */
#define BMSR_RESV           0x0780      /* Unused...                         */
#define BMSR_10HALF         0x0800      /* Can do 10mbps, half-duplex        */
#define BMSR_10FULL         0x1000      /* Can do 10mbps, full-duplex        */
#define BMSR_100HALF        0x2000      /* Can do 100mbps, half-duplex       */
#define BMSR_100FULL        0x4000      /* Can do 100mbps, full-duplex       */
#define BMSR_100BASE4       0x8000      /* Can do 100mbps, 4k packets        */

/* Advertisement control register. */
#define ANAR_SLCT           0x001f      /* Selector bits                     */
#define ANAR_CSMA           0x0001      /* Only selector supported           */
#define ANAR_10HALF         0x0020      /* Try for 10mbps half-duplex        */
#define ANAR_10FULL         0x0040      /* Try for 10mbps full-duplex        */
#define ANAR_100HALF        0x0080      /* Try for 100mbps half-duplex       */
#define ANAR_100FULL        0x0100      /* Try for 100mbps full-duplex       */
#define ANAR_100BASE4       0x0200      /* Try for 100mbps 4k packets        */
#define ANAR_FCS            0x0400      /* Try for Flow Control Support      */
#define ANAR_RESV           0x1800      /* Unused...                         */
#define ANAR_RFAULT         0x2000      /* Say we can detect faults          */
#define ANAR_LPACK          0x4000      /* Ack link partners response        */
#define ANAR_NPAGE          0x8000      /* Next page bit                     */

#define ANAR_FULL           (ANAR_100FULL | ANAR_10FULL | ANAR_CSMA)
#define ANAR_ALL            (ANAR_10HALF  | ANAR_10FULL |              \
                             ANAR_100HALF | ANAR_100FULL)

/* Link partner ability register. */
#define ANLPAR_SLCT         0x001f      /* Same as advertise selector        */
#define ANLPAR_10HALF       0x0020      /* Can do 10mbps half-duplex         */
#define ANLPAR_10FULL       0x0040      /* Can do 10mbps full-duplex         */
#define ANLPAR_100HALF      0x0080      /* Can do 100mbps half-duplex        */
#define ANLPAR_100FULL      0x0100      /* Can do 100mbps full-duplex        */
#define ANLPAR_100BASE4     0x0200      /* Can do 100mbps 4k packets         */
#define ANLPAR_FCS          0x0400      /* Can do Flow Control Support       */
#define ANLPAR_RESV         0x1800      /* Unused...                         */
#define ANLPAR_RFAULT       0x2000      /* Link partner faulted              */
#define ANLPAR_LPACK        0x4000      /* Link partner acked us             */
#define ANLPAR_NPAGE        0x8000      /* Next page bit                     */

#define ANLPAR_DUPLEX       (ANLPAR_10FULL  | ANLPAR_100FULL)
#define ANLPAR_100          (ANLPAR_100FULL | ANLPAR_100HALF | ANLPAR_100BASE4)

/* Expansion register for auto-negotiation. */
#define ANER_LPANABLE       0x0001      /* Link Partner AutoNegotiation able */
#define ANER_PAGERX         0x0002      /* New Page received                 */
#define ANER_NPABLE         0x0004      /* Local Device next page able       */
#define ANER_LPNPABLE       0x0008      /* Link partner supports npage       */
#define ANER_PDF            0x0010      /* Parallel Detection fault          */
#define ANER_RESV           0xFFE0      /* Unused...                         */

/* PHY ID */
#define MII_DM9161_ID       0x0181b8a0
#define MII_AM79C875_ID     0x00225540  /* 0x00225541                        */

#endif

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/

