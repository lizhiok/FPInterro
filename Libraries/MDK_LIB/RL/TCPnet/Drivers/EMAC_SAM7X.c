/*----------------------------------------------------------------------------
 *      RL-ARM - TCPnet
 *----------------------------------------------------------------------------
 *      Name:    EMAC_SAM7X.C
 *      Purpose: Driver for Atmel AT91SAM7X EMAC Ethernet Controller
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <Net_Config.h>
#include "EMAC_SAM7X.h"

/* The following macro definitions may be used to select the speed
   of the physical link:

  _10MBIT_   - connect at 10 MBit only
  _100MBIT_  - connect at 100 MBit only

  By default an autonegotiation of the link speed is used. This may take 
  longer to connect, but it works for 10MBit and 100MBit physical links.     */

/* Net_Config.c */
extern U8 own_hw_adr[];

/* Local variables */
static U32 RxBufIndex;

/* EMAC local IO buffers descriptors. */
static Buf_Desc Rx_Desc[NUM_RX_BUF];
static Buf_Desc Tx_Desc[NUM_TX_BUF];

/* EMAC local buffers must be 8-byte aligned. */
static U64 rx_buf[NUM_RX_BUF][ETH_RX_BUF_SIZE>>3];
static U64 tx_buf[NUM_TX_BUF][ETH_TX_BUF_SIZE>>3];


/*----------------------------------------------------------------------------
 *      EMAC Ethernet Driver Functions
 *----------------------------------------------------------------------------
 *  Required functions for Ethernet driver module:
 *  a. Polling mode: - void init_ethernet ()
 *                   - void send_frame (OS_FRAME *frame)
 *                   - void poll_ethernet (void)
 *  b. Interrupt mode: - void init_ethernet ()
 *                     - void send_frame (OS_FRAME *frame)
 *                     - void int_enable_eth ()
 *                     - void int_disable_eth ()
 *                     - interrupt function 
 *---------------------------------------------------------------------------*/

/* Local Function Prototypes */
static void interrupt_ethernet (void) __irq;
static void def_interrupt (void) __irq;
static void fetch_packet (void);
static void rx_descr_init (void);
static void tx_descr_init (void);
static void write_PHY (U32 PhyReg, U16 Value);
static U16  read_PHY (U32 PhyReg);
static void enable_MDI (void);
static void disable_MDI (void);

/*--------------------------- init_ethernet ---------------------------------*/

void init_ethernet (void) {
  /* Initialize the EMAC ethernet controller. */
  U32 regv,tout,id1,id2;

  /* Enable Peripheral Clock for EMAC Peripherals */
  pPMC->PMC_PCER  = (1 << AT91C_ID_PIOB) | (1 << AT91C_ID_EMAC);

  /* Disable pull up on RXDV => PHY normal mode (not in test mode), */
  /* and set MII mode. PHY has internal pull down.                  */
  pPIOB->PIO_PPUDR= AT91C_PIO_PB16 | AT91C_PIO_PB15;

  /* Clear PB18 <=> PHY powerdown */
  pPIOB->PIO_IDR  =
  pPIOB->PIO_PER  =
  pPIOB->PIO_OER  = AT91C_PIO_PB18;
  pPIOB->PIO_CODR = AT91C_PIO_PB18;

  /* After PHY power up, hardware reset the PHY */
  pRSTC->RSTC_RCR = 0xA5000000 | AT91C_RSTC_EXTRST;

  /* Wait for hardware reset to end. */
  while (!(pRSTC->RSTC_RSR & AT91C_RSTC_NRSTL));

  /* EMAC IO init for EMAC-PHY communication in MII. */
  pPIOB->PIO_IDR  =
  pPIOB->PIO_PPUDR=
  pPIOB->PIO_PDR  =
  pPIOB->PIO_ASR  = AT91C_PB0_ETXCK_EREFCK | AT91C_PB1_ETXEN         |
                    AT91C_PB2_ETX0         | AT91C_PB3_ETX1          |
                    AT91C_PB4_ECRS         | AT91C_PB5_ERX0          |
                    AT91C_PB6_ERX1         | AT91C_PB7_ERXER         |
                    AT91C_PB8_EMDC         | AT91C_PB9_EMDIO         |
                    AT91C_PB10_ETX2        | AT91C_PB11_ETX3         |
                    AT91C_PB12_ETXER       | AT91C_PB13_ERX2         |
                    AT91C_PB14_ERX3        | AT91C_PB15_ERXDV_ECRSDV |
                    AT91C_PB16_ECOL        | AT91C_PB17_ERXCK;

  /* Enable communication between EMAC-PHY. */
  enable_MDI ();
  /* Put the DM9161 in reset mode */
  write_PHY (PHY_REG_BMCR, BMCR_RESET);

  /* Wait for hardware reset to end. */
  for (tout = 0; tout < 0x100000; tout++) {
    regv = read_PHY (PHY_REG_BMCR);
    if (!(regv & BMCR_RESET)) {
      /* Reset complete */
      break;
    }
  }

  /* Check if this is a DM9161 PHY. */
  id1 = read_PHY (PHY_REG_PHYID1);
  id2 = read_PHY (PHY_REG_PHYID2);

  if (((id1 << 16) | (id2 & 0xfff0)) == MII_DM9161_ID) {
    /* Configure the PHY device */
#if defined (_10MBIT_)
    /* Connect at 10MBit */
    write_PHY (PHY_REG_BMCR, PHY_FULLD_10M);
    write_PHY (PHY_REG_DSCR, 0x0000);
#elif defined (_100MBIT_)
    /* Connect at 100MBit */
    write_PHY (PHY_REG_BMCR, PHY_FULLD_100M);
    write_PHY (PHY_REG_DSCR, 0x0400);
#else
    /* Use autonegotiation about the link speed. */
    write_PHY (PHY_REG_BMCR, PHY_AUTO_NEG);
    /* Wait to complete Auto_Negotiation. */
    for (tout = 0; tout < 0x100000; tout++) {
      regv = read_PHY (PHY_REG_BMSR);
      if (regv & BMSR_ANEGCOMPLETE) {
        /* ANEG_ACK set, autonegotiation finished. */
        break;
      }
    }
    /* Check the link status. */
    for (tout = 0; tout < 0x10000; tout++) {
      regv = read_PHY (PHY_REG_BMSR);
      if (regv & BMSR_LINKST) {
        /* Link is on. */
        break;
      }
    }
#endif
  }

  /* Update the MAC register NCFGR. */
  pEMAC->EMAC_NCFGR = 0;
  regv = read_PHY (PHY_REG_BMCR);

  if (regv & BMCR_SPEED100) {
    /* Speed 100Mbit is enabled. */
    pEMAC->EMAC_NCFGR |= AT91C_EMAC_SPD;
  }
  if (regv & BMCR_FULLDPLX) {
    /* Full duplex is enabled. */
    pEMAC->EMAC_NCFGR |= AT91C_EMAC_FD;
  }

  disable_MDI ();

  /* Enable EMAC in MII mode, enable clock ERXCK and ETXCK */
  pEMAC->EMAC_USRIO= AT91C_EMAC_CLKEN;

  /* Transmit and Receive disable. */
  pEMAC->EMAC_NCR &= ~(AT91C_EMAC_RE | AT91C_EMAC_TE);

  /* Initialize Tx and Rx DMA Descriptors */
  rx_descr_init ();
  tx_descr_init ();

  /* The sequence write EMAC_SA1L and write EMAC_SA1H must be respected. */
  pEMAC->EMAC_SA1L = ((U32)own_hw_adr[3] << 24) | ((U32)own_hw_adr[2] << 16) |
                     ((U32)own_hw_adr[1] << 8)  |  (U32)own_hw_adr[0];
  pEMAC->EMAC_SA1H = ((U32)own_hw_adr[5] << 8)  |  (U32)own_hw_adr[4];

  /* Enable receiving of all Multicast packets. */
  pEMAC->EMAC_HRB  = 0xFFFFFFFF;
  pEMAC->EMAC_HRT  = 0xFFFFFFFF;

  /* Clear receive and transmit status registers. */
  pEMAC->EMAC_RSR  = (AT91C_EMAC_OVR | AT91C_EMAC_REC | AT91C_EMAC_BNA);
  pEMAC->EMAC_TSR  = (AT91C_EMAC_UND | AT91C_EMAC_COMP| AT91C_EMAC_BEX |
                      AT91C_EMAC_RLES| AT91C_EMAC_COL | AT91C_EMAC_UBR);

  /* Configure EMAC operation mode, enable Multicast. */
  pEMAC->EMAC_NCFGR |= (AT91C_EMAC_BIG | AT91C_EMAC_DRFCS | AT91C_EMAC_MTI);
  pEMAC->EMAC_NCR   |= (AT91C_EMAC_TE  | AT91C_EMAC_RE | AT91C_EMAC_WESTAT);

  /* Configure the EMAC Interrupts. */
  pEMAC->EMAC_IDR = 0x3fff;
  /* Allow RCOMP, RXUBR and ROVR interrupts. */
  pEMAC->EMAC_IER = AT91C_EMAC_RCOMP | AT91C_EMAC_ROVR | AT91C_EMAC_RXUBR;

  /* Enable EMAC interrupts. */
  pAIC->AIC_IDCR  = (1 << AT91C_ID_EMAC);
  pAIC->AIC_SVR[AT91C_ID_EMAC] = (U32)interrupt_ethernet;
  pAIC->AIC_SMR[AT91C_ID_EMAC] = AT91C_AIC_PRIOR_HIGHEST;
  pAIC->AIC_ICCR  = (1 << AT91C_ID_EMAC);
  pAIC->AIC_SPU   = (U32)def_interrupt;
}


/*--------------------------- def_interrupt ---------------------------------*/

void def_interrupt (void) __irq {
  /* Default Interrupt Function: may be called when timer ISR is disabled */
  pAIC->AIC_EOICR = 0;
}


/*--------------------------- int_enable_eth --------------------------------*/

void int_enable_eth (void) {
  /* Ethernet Interrupt Enable function. */
  pAIC->AIC_IECR = (1 << AT91C_ID_EMAC);
}


/*--------------------------- int_disable_eth -------------------------------*/

void int_disable_eth (void) {
  /* Ethernet Interrupt Disable function. */
  pAIC->AIC_IDCR = (1 << AT91C_ID_EMAC);
}


/*--------------------------- send_frame ------------------------------------*/

void send_frame (OS_FRAME *frame) {
  /* Send frame to EMAC ethernet controller */
  U32 *sp,*dp;
  U32 i;

  /* Packet Transmit in progress, wait until finished. */
  while (pEMAC->EMAC_TSR & AT91C_EMAC_TGO);

  sp = (U32 *)&frame->data[0];
  dp = (U32 *)(Tx_Desc[0].addr & ~3);
  /* Copy frame data to EMAC IO buffer. */
  for (i = (frame->length + 3) >> 2; i; i--) {
    *dp++ = *sp++;
  }
  Tx_Desc[0].stat = frame->length | TD_LAST_BUF | TD_TRANSMIT_WRAP;

  /* Start frame transmission. */
  pEMAC->EMAC_NCR |= AT91C_EMAC_TSTART;
}


/*--------------------------- fetch_packet ----------------------------------*/

static void fetch_packet (void) {
  /* Fetch a packet from DMA buffer and release buffer. */
  OS_FRAME *frame;
  U32 j,ei,si,RxLen;
  U32 *sp,*dp;

  for (ei = RxBufIndex; ; RxBufIndex = ei) {
    /* Scan the receive buffers. */
    for (si = ei; ; ) {
      if (!(Rx_Desc[ei].addr & AT91C_OWNERSHIP_BIT)) {
        /* End of scan, unused buffers found. */
        if (si != ei) {
          /* Found erroneus fragment, release it. */
          ei = si;
          goto rel;
        }
        return;
      }
      /* Search for EOF. */
      if (Rx_Desc[ei].stat & RD_EOF) {
        break;
      }
      /* Check the SOF-SOF sequence */
      if (Rx_Desc[ei].stat & RD_SOF) {
        /* Found one, this is new start of frame. */
        si = ei;
      }
      if (++ei == NUM_RX_BUF) ei = 0;

      if (ei == RxBufIndex) {
        /* Safety limit to prevent deadlock. */
        ei = si;
        goto rel;
      }
    }

    /* Get frame length. */
    RxLen = Rx_Desc[ei].stat & RD_LENGTH_MASK;
    if (++ei == NUM_RX_BUF) ei = 0;

    if (RxLen > ETH_MTU) {
      /* Packet too big, ignore it and free buffer. */
      goto rel;
    }

    /* Flag 0x80000000 to skip sys_error() call when out of memory. */
    frame = alloc_mem (RxLen | 0x80000000);

    /* if 'alloc_mem()' has failed, ignore this packet. */
    if (frame != NULL) {
      /* Make sure that block is 4-byte aligned */
      dp = (U32 *)&frame->data[0];
      for ( ; si != ei; RxLen -= ETH_RX_BUF_SIZE) {
        sp = (U32 *)(Rx_Desc[si].addr & ~0x3);
        j = RxLen;
        if (j > ETH_RX_BUF_SIZE) j = ETH_RX_BUF_SIZE;
        for (j = (j + 3) >> 2; j; j--) {
          *dp++ = *sp++;
        }
        if (++si == NUM_RX_BUF) si = 0;
      }
      put_in_queue (frame);
    }

    /* Release packet fragments from EMAC IO buffer. */
rel:for (j = RxBufIndex; ; ) {
      Rx_Desc[j].addr &= ~AT91C_OWNERSHIP_BIT;
      if (++j == NUM_RX_BUF) j = 0;
      if (j == ei) break;
    }
  }
}


/*--------------------------- interrupt_ethernet ----------------------------*/

static void interrupt_ethernet (void) __irq {
  /* SAM7X EMAC Ethernet Controller Interrupt function. */
  U32 int_stat;

  int_stat = pEMAC->EMAC_ISR;
  if (int_stat & (AT91C_EMAC_RXUBR | AT91C_EMAC_ROVR)) {
    /* Receive buffer overflow, all packets are invalid, because */
    /* a descriptor status is overwritten by DMA engine. */
    pEMAC->EMAC_NCR &= ~AT91C_EMAC_RE;
    rx_descr_init ();
    pEMAC->EMAC_RSR  = AT91C_EMAC_OVR | AT91C_EMAC_REC | AT91C_EMAC_BNA;
    pEMAC->EMAC_NCR |= AT91C_EMAC_RE;
  }
  else if (int_stat & AT91C_EMAC_RCOMP) {
    /* Valid frame has been received. */
    if (pEMAC->EMAC_RSR & AT91C_EMAC_REC) {
      fetch_packet ();
      pEMAC->EMAC_RSR = AT91C_EMAC_REC;
    }
  }
  /* Acknowledge the interrupt. */
  pAIC->AIC_EOICR = 0;
}


/*--------------------------- rx_descr_init ---------------------------------*/

static void rx_descr_init (void) {
  /* Initialize Rx_Desc descriptor. */
  U32 i;

  RxBufIndex = 0;
  for (i = 0; i < NUM_RX_BUF; i++) {
    Rx_Desc[i].addr = (U32)&rx_buf[i];
    Rx_Desc[i].stat = 0;
  }
  /* Set the WRAP bit at the end of the list descriptor. */
  Rx_Desc[NUM_RX_BUF-1].addr |= 0x02;
  /* Set Rx Queue pointer to descriptor list. */
  pEMAC->EMAC_RBQP = (U32)&Rx_Desc[0];
}


/*--------------------------- tx_descr_init ---------------------------------*/

static void tx_descr_init (void) {
  /* Initialize Tx_Desc descriptor. */
  U32 i;

  for (i = 0; i < NUM_TX_BUF; i++) {
    Tx_Desc[i].addr = (U32)&tx_buf[i];
    Tx_Desc[i].stat = 0;
  }
  /* Set the WRAP bit at the end of the list descriptor. */
  Tx_Desc[NUM_TX_BUF-1].stat |= TD_TRANSMIT_WRAP;
  /* Set Tx Queue pointer to descriptor list. */
  pEMAC->EMAC_TBQP = (U32)&Tx_Desc[0];
}


/*--------------------------- write_PHY -------------------------------------*/

static void write_PHY (U32 PhyReg, U16 Value) {
  /* Write a data 'Value' to PHY register 'PhyReg'.         */
  /* MDI interface is assumed to already have been enabled. */

  pEMAC->EMAC_MAN = ((AT91C_EMAC_SOF & (0x01<<30)) | (2 << 16) | (1 << 28) |
                     (AT91C_PHY_ADDR << 23) | (PhyReg << 18))  | Value;

  /* Wait until IDLE bit in Network Status register is cleared */
  while (!(pEMAC->EMAC_NSR & AT91C_EMAC_IDLE));
}


/*--------------------------- read_PHY --------------------------------------*/

static U16 read_PHY (U32 PhyReg) {
  /* Read a PHY register 'PhyReg'.                          */
  /* MDI interface is assumed to already have been enabled. */

  pEMAC->EMAC_MAN = (AT91C_EMAC_SOF & (0x01<<30)) | (2 << 16) | (2 << 28) |
                    (AT91C_PHY_ADDR << 23) | (PhyReg << 18);

  /* Wait until IDLE bit in Network Status register is cleared */
  while (!(pEMAC->EMAC_NSR & AT91C_EMAC_IDLE));
  return (pEMAC->EMAC_MAN & 0x0000ffff);
}


/*--------------------------- enable_MDI ------------------------------------*/

static void enable_MDI (void) {
  /* Enable management port in MAC control register. */
  pEMAC->EMAC_NCR   |= AT91C_EMAC_MPE;
  /* MDC = MCK/32 */
  pEMAC->EMAC_NCFGR |= AT91C_EMAC_CLK_HCLK_32;
}


/*--------------------------- disable_MDI -----------------------------------*/

static void disable_MDI (void) {
  /* Disable management port in MAC control register. */
  pEMAC->EMAC_NCR &= ~AT91C_EMAC_MPE;
}

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/




