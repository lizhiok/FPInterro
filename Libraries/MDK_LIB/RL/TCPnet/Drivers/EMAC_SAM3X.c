/*----------------------------------------------------------------------------
 *      RL-ARM - TCPnet
 *----------------------------------------------------------------------------
 *      Name:    EMAC_SAM3X.C
 *      Purpose: Driver for Atmel SAM3X EMAC Ethernet Controller
 *      Rev.:    V5.00
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <Net_Config.h>
#include <sam3xa.h>
#include "EMAC_SAM3X.h"

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
static EMAC_DMA_Desc Rx_Desc[NUM_RX_BUF];
static EMAC_DMA_Desc Tx_Desc[NUM_TX_BUF];

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
static void fetch_packet (void);
static void rx_descr_init (void);
static void tx_descr_init (void);
static void write_PHY (U32 PhyReg, U16 Value);
static U16  read_PHY (U32 PhyReg);

/*--------------------------- init_ethernet ---------------------------------*/

void init_ethernet (void) {
  /* Initialize the EMAC ethernet controller. */
  U32 regv,tout,id1,id2,conn;

  /* Enable Peripheral Clock for PIOB and EMAC Peripherals */
  PMC->PMC_WPMR  = PMC_WPDIS_KEY;       /* Disable PMC write protection       */
  PMC->PMC_PCDR1 = PMC_PCSR1_PID42;     /* ID_EMAC                            */
  PMC->PMC_PCER0 = PMC_PCER0_PID12;     /* ID_PIOB                            */
  PMC->PMC_PCER1 = PMC_PCSR1_PID42;     /* ID_EMAC                            */
  PMC->PMC_WPMR  = PMC_WPEN_KEY;        /* Enable PMC write protection        */

  /* Configure EMAC pins */
  PIOB->PIO_WPMR = PIO_WPDIS_KEY;       /* Disable PIO write protection       */
  PIOB->PIO_PDR  = EMAC_PIO_PDR_PIN_MSK;
  PIOB->PIO_PUDR = EMAC_PIO_PUDR_PIN_MSK;
  PIOB->PIO_WPMR = PIO_WPEN_KEY;        /* Enable PIO write protection        */
  
  /* Reset configuration, disable receiver and transmitter */
  EMAC->EMAC_NCFGR &= ~(0xFFDFB);
  EMAC->EMAC_NCR   &= ~(EMAC_NCR_RE | EMAC_NCR_TE);
  /* Enable management port in MAC control register. */
  EMAC->EMAC_NCR   |= EMAC_NCR_MPE;
  /* MDC = MCK/64 */
  EMAC->EMAC_NCFGR |= EMAC_NCFGR_CLK_MCK_64;

  /* Put the DM9161 in reset mode */
  write_PHY (PHY_REG_BMCR, BMCR_RESET);

  /* Wait for hardware reset to end. */
  for (tout = 0; tout < 0x800000; tout++) {
    regv = read_PHY (PHY_REG_BMCR);
    if (!(regv & BMCR_RESET)) {
      /* Reset complete */
      break;
    }
  }
  /* Default connection settings */
  conn = 0;

  /* Read PHY ID */
  id1 = read_PHY (PHY_REG_PHYID1);
  id2 = read_PHY (PHY_REG_PHYID2);

/* Check if this is a DM9161 PHY. */
  if (((id1 << 16) | (id2 & 0xfff0)) == PHY_ID_DM9161) {
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
    for (tout = 0; tout < 0x800000; tout++) {
      regv = read_PHY (PHY_REG_BMSR);
      if (regv & BMSR_ANEGCOMPLETE) {
        /* ANEG_ACK set, autonegotiation finished. */
        break;
      }
    }
   #endif
    /* Check the link status. */
    for (tout = 0; tout < 0x80000; tout++) {
      regv = read_PHY (PHY_REG_BMSR);
      if (regv & BMSR_LINKST) {
        /* Link is on, get connection info */
        regv = read_PHY (PHY_REG_BMCR);
        
        if (regv & BMCR_FULLDPLX) {
          /* Full-duplex connection */
          conn |= PHY_CON_SET_FULLD;
        }
        
        if (regv & BMCR_SPEED100) {
          /* Speed is 100Mbit */
          conn |= PHY_CON_SET_100M;
        }
        break;
      }
    }
  }

  /* Configure Full/Half Duplex mode. */
  if (conn & PHY_CON_SET_FULLD) {
    /* Full duplex is enabled. */
    EMAC->EMAC_NCFGR |= EMAC_NCFGR_FD;
  }

  /* Configure 100MBit/10MBit mode. */
  if (conn & PHY_CON_SET_100M) {
    /* Enable 100Mbit speed */
    EMAC->EMAC_NCFGR |= EMAC_NCFGR_SPD;
  }

  /* Disable management port in MAC control register. */
  EMAC->EMAC_NCR &= ~EMAC_NCR_MPE;

  /* Enable EMAC in RMII mode. */
  EMAC->EMAC_USRIO = EMAC_USRIO_CLKEN | EMAC_USRIO_RMII;

  /* Disable transceiver */
  EMAC->EMAC_NCR &= ~(EMAC_NCR_RE | EMAC_NCR_TE);

  /* Initialize Tx and Rx DMA Descriptors */
  rx_descr_init ();
  tx_descr_init ();

  /* The sequence write EMAC_SA1L and write EMAC_SA1H must be respected. */
  EMAC->EMAC_SA[0].EMAC_SAxB = ((U32)own_hw_adr[3] << 24) | ((U32)own_hw_adr[2] << 16) |
                               ((U32)own_hw_adr[1] <<  8) |  (U32)own_hw_adr[0];
  EMAC->EMAC_SA[0].EMAC_SAxT = ((U32)own_hw_adr[5] <<  8) |  (U32)own_hw_adr[4];

  /* Enable receiving of all Multicast packets. */
  EMAC->EMAC_HRB = 0xFFFFFFFF;
  EMAC->EMAC_HRT = 0xFFFFFFFF;

  /* Clear receive and transmit status registers. */
  EMAC->EMAC_RSR = EMAC_RSR_OVR | EMAC_RSR_REC | EMAC_RSR_BNA ;
  EMAC->EMAC_TSR = EMAC_TSR_UND | EMAC_TSR_COMP| EMAC_TSR_BEX |
                   EMAC_TSR_RLES| EMAC_TSR_COL | EMAC_TSR_UBR ;

  /* Configure the EMAC Interrupts. */
  EMAC->EMAC_IDR  = 0x3CFF;
  /* Allow RCOMP and RXUBR interrupts. */
  EMAC->EMAC_IER  = EMAC_IER_RCOMP | EMAC_IER_RXUBR;
  
  /* Configure EMAC operation mode */
  EMAC->EMAC_NCFGR |= (EMAC_NCFGR_RLCE | EMAC_NCFGR_DRFCS | EMAC_NCFGR_BIG);
  EMAC->EMAC_NCR   |= (EMAC_NCR_WESTAT | EMAC_NCR_RE      | EMAC_NCR_TE);
}


/*--------------------------- int_enable_eth --------------------------------*/

void int_enable_eth (void) {
  /* Enable EMAC Interrupt */
  NVIC_EnableIRQ (EMAC_IRQn);
}


/*--------------------------- int_disable_eth -------------------------------*/

void int_disable_eth (void) {
  /* Disable EMAC Interrupt */
  NVIC_DisableIRQ (EMAC_IRQn);
}


/*--------------------------- send_frame ------------------------------------*/

void send_frame (OS_FRAME *frame) {
  /* Send frame to EMAC ethernet controller */
  U32 *sp,*dp;
  U32 i;

  /* Packet Transmit in progress, wait until finished. */
  while (EMAC->EMAC_TSR & EMAC_TSR_TGO);

  sp = (U32 *)&frame->data[0];
  dp = (U32 *)(Tx_Desc[0].Addr & ~3);
  /* Copy frame data to EMAC IO buffer. */
  for (i = (frame->length + 3) >> 2; i; i--) {
    *dp++ = *sp++;
  }
  Tx_Desc[0].Stat = frame->length | TD_LAST_BUF | TD_TRANSMIT_WRAP;

  /* Start frame transmission. */
  EMAC->EMAC_NCR |= EMAC_NCR_TSTART;
}


/*--------------------------- fetch_packet ----------------------------------*/

static void fetch_packet (void) {
  /* Fetch a packet from DMA buffer and release buffer. */
  OS_FRAME *frame;
  U32 j,ei,si,RxLen;
  U8 *sbp, *dbp;

  for (ei = RxBufIndex; ; RxBufIndex = ei) {
    /* Scan the receive buffers. */
    for (si = ei; ; ) {
      if (!(Rx_Desc[ei].Addr & EMAC_DESC_OWN)) {
        /* End of scan, unused buffers found. */
        if (si != ei) {
          /* Found erroneus fragment, release it. */
          ei = si;
          goto rel;
        }
        return;
      }
      /* Search for EOF. */
      if (Rx_Desc[ei].Stat & RD_EOF) {
        break;
      }
      /* Check the SOF-SOF sequence */
      if (Rx_Desc[ei].Stat & RD_SOF) {
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
    RxLen = Rx_Desc[ei].Stat & RD_LENGTH_MASK;
    if (++ei == NUM_RX_BUF) ei = 0;

    if (RxLen > ETH_MTU) {
      /* Packet too big, ignore it and free buffer. */
      goto rel;
    }

    /* Flag 0x80000000 to skip sys_error() call when out of memory. */
    frame = alloc_mem (RxLen | 0x80000000);

    /* if 'alloc_mem()' has failed, ignore this packet. */
    if (frame != NULL) {
      dbp = (U8 *)&frame->data[0];
      
      for ( ; si != ei; ) {
        sbp = (U8 *)(Rx_Desc[si].Addr & ~0x3);
        j = RxLen;
        if (j > ETH_RX_BUF_SIZE) j = ETH_RX_BUF_SIZE;
        RxLen -= j;

        for (; j; j--) {
          *dbp++ = *sbp++;
        }
        if (++si == NUM_RX_BUF) si = 0;
      }
      put_in_queue (frame);
    }

    /* Release packet fragments from EMAC IO buffer. */
rel:for (j = RxBufIndex; ; ) {
      Rx_Desc[j].Addr &= ~EMAC_DESC_OWN;
      if (++j == NUM_RX_BUF) j = 0;
      if (j == ei) break;
    }
  }
}


/*--------------------------- EMAC_IRQHandler -------------------------------*/

void EMAC_IRQHandler (void) {
  /* EMAC Ethernet Controller Interrupt function. */
  U32 stat;

  stat = EMAC->EMAC_ISR;

  EMAC->EMAC_RSR = EMAC_RSR_BNA | EMAC_RSR_REC | EMAC_RSR_OVR;

  if (stat & EMAC_IER_RXUBR) {
    EMAC->EMAC_NCR &= ~EMAC_NCR_RE;
    rx_descr_init ();
    EMAC->EMAC_NCR |= EMAC_NCR_RE;
  }
  else if (stat & EMAC_IER_RCOMP) {
    fetch_packet ();
  }
}


/*--------------------------- rx_descr_init ---------------------------------*/

static void rx_descr_init (void) {
  /* Initialize Rx_Desc descriptor. */
  U32 i;

  RxBufIndex = 0;
  for (i = 0; i < NUM_RX_BUF; i++) {
    Rx_Desc[i].Addr = (U32)&rx_buf[i];
    Rx_Desc[i].Stat = 0;
  }
  /* Set the WRAP bit at the end of the list descriptor. */
  Rx_Desc[NUM_RX_BUF-1].Addr |= EMAC_DESC_WRAP;
  /* Set Rx Queue pointer to descriptor list. */
  EMAC->EMAC_RBQP = (U32)&Rx_Desc[0];
}


/*--------------------------- tx_descr_init ---------------------------------*/

static void tx_descr_init (void) {
  /* Initialize Tx_Desc descriptor. */
  U32 i;

  for (i = 0; i < NUM_TX_BUF; i++) {
    Tx_Desc[i].Addr = (U32)&tx_buf[i];
    Tx_Desc[i].Stat = 0;
  }
  /* Set the WRAP bit at the end of the list descriptor. */
  Tx_Desc[NUM_TX_BUF-1].Stat |= TD_TRANSMIT_WRAP;
  /* Set Tx Queue pointer to descriptor list. */
  EMAC->EMAC_TBQP = (U32)&Tx_Desc[0];
}


/*--------------------------- write_PHY -------------------------------------*/

static void write_PHY (U32 PhyReg, U16 Value) {
  /* Write a data 'Value' to PHY register 'PhyReg'.         */
  /* MDI interface is assumed to already have been enabled. */
  EMAC->EMAC_MAN = EMAC_MAN_SOF (1) | EMAC_MAN_RW (1) | EMAC_MAN_CODE (2) |
                   EMAC_MAN_PHYA (PHY_DEF_ADDR) |
                   EMAC_MAN_REGA (PhyReg)       |
                   Value                        ;

  /* Wait until IDLE bit in Network Status register is cleared */
  while (!(EMAC->EMAC_NSR & EMAC_NSR_IDLE));
}


/*--------------------------- read_PHY --------------------------------------*/

static U16 read_PHY (U32 PhyReg) {
  /* Read a PHY register 'PhyReg'.                          */
  /* MDI interface is assumed to already have been enabled. */
  EMAC->EMAC_MAN = EMAC_MAN_SOF (1) | EMAC_MAN_RW (2) | EMAC_MAN_CODE (2) |
                   EMAC_MAN_PHYA (PHY_DEF_ADDR) |
                   EMAC_MAN_REGA (PhyReg);

  /* Wait until IDLE bit in Network Status register is cleared */
  while (!(EMAC->EMAC_NSR & EMAC_NSR_IDLE));
  return (EMAC->EMAC_MAN & 0x0000ffff);
}


/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
