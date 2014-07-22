/*-----------------------------------------------------------------------------
 *      RL-ARM - TCPnet
 *-----------------------------------------------------------------------------
 *      Name:    ETH_LPC18XX.C
 *      Purpose: Driver for NXP LPC18XX Ethernet Controller
 *      Rev.:    V4.70
 *-----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include <Net_Config.h>
#include <LPC18xx.h>                    /* LPC18xx Definitions                */
#include "ETH_LPC18xx.h"

/* The following macro definitions may be used to select the speed
   of the physical link:

  _10MBIT_   - connect at 10 MBit only
  _100MBIT_  - connect at 100 MBit only

  By default an autonegotiation of the link speed is used. This may take 
  longer to connect, but it works for 10MBit and 100MBit physical links.
  
    The following macro definitions may be used to select PYH interface:
  -MII_      - use MII interface instead of RMII
                                                                              */
//#define _MII_

/* Net_Config.c */
extern U8 own_hw_adr[];

/* Local variables */
static U8 TxBufIndex;
static U8 RxBufIndex;

/* ENET local DMA Descriptors. */
static RX_Desc Rx_Desc[NUM_RX_BUF];
static TX_Desc Tx_Desc[NUM_TX_BUF];

/* ENET local DMA buffers. */
static U32 rx_buf[NUM_RX_BUF][ETH_BUF_SIZE>>2];
static U32 tx_buf[NUM_TX_BUF][ETH_BUF_SIZE>>2];

/*-----------------------------------------------------------------------------
 *      ENET Ethernet Driver Functions
 *-----------------------------------------------------------------------------
 *  Required functions for Ethernet driver module:
 *  a. Polling mode: - void init_ethernet ()
 *                   - void send_frame (OS_FRAME *frame)
 *                   - void poll_ethernet (void)
 *  b. Interrupt mode: - void init_ethernet ()
 *                     - void send_frame (OS_FRAME *frame)
 *                     - void int_enable_eth ()
 *                     - void int_disable_eth ()
 *                     - interrupt function 
 *----------------------------------------------------------------------------*/

/* Local Function Prototypes */
static void rx_descr_init (void);
static void tx_descr_init (void);
static void write_PHY (U32 PhyReg, U16 Value);
static U16  read_PHY (U32 PhyReg);

/*---------------------------- init_ethernet ---------------------------------*/

void init_ethernet (void) {
  /* Initialize the EMAC ethernet controller. */
  int id1, id2, tout, regv;

  /* Enable GPIO register interface clock */
  LPC_CCU1->CLK_M3_GPIO_CFG     |= 1;
  while (!(LPC_CCU1->CLK_M3_GPIO_STAT   & 1));

  /* Enable ethernet branch clock */
  LPC_CCU1->CLK_M3_ETHERNET_CFG |= 1;
  while (!(LPC_CCU1->CLK_M3_ETHERNET_STAT & 1));

  LPC_RGU->RESET_EXT_STAT19 |= (1 << 2);
  LPC_RGU->RESET_EXT_STAT22 |= (1 << 2);

  LPC_RGU->RESET_EXT_STAT19 &= ~(1 << 2);
  LPC_RGU->RESET_EXT_STAT22 &= ~(1 << 2);

  LPC_CREG->CREG6 = (LPC_CREG->CREG6 & ~0x7) | 4;

  /* Ethernet pins configuration */
  LPC_SCU->SFSP0_0  = (1 << 6) | (1 << 5) | 0x2; /* P0.0  = ENET_RXD1         */
  LPC_SCU->SFSP0_1  = (1 << 6) | (1 << 5) | 0x6; /* P0.1  = ENET_TX_EN        */

  LPC_SCU->SFSP1_15 = (1 << 6) | (1 << 5) | 0x3; /* P1.15 = ENET_RXD0         */
  LPC_SCU->SFSP1_16 = (1 << 6) | (1 << 5) | 0x7; /* P1.16 = ENET_RX_DV        */
  LPC_SCU->SFSP1_17 = (1 << 6) |            0x3; /* P1.17 = ENET_MDIO         */
  LPC_SCU->SFSP1_18 = (1 << 6) | (1 << 5) | 0x3; /* P1.18 = ENET_TXD0         */
  LPC_SCU->SFSP1_19 = (1 << 6) | (1 << 5) | 0x0; /* P1.19 = ENET_TX_CLK       */
  LPC_SCU->SFSP1_20 = (1 << 6) | (1 << 5) | 0x3; /* P1.20 = ENET_TXD1         */

  LPC_SCU->SFSPC_1  = (1 << 6) | (1 << 5) | 0x3; /* PC.1  = ENET_MDC          */

#ifdef _MII_
  LPC_CREG->CREG6 &= ~0x7;
  
  LPC_SCU->SFSP0_1 = (1 << 6) | (1 << 5) | 0x2;  /* P0.1  = ENET_COL          */
  
  LPC_SCU->SFSPC_0 = (1 << 6) | (1 << 5) | 0x0;  /* PC.0  = ENET_RX_CLK       */
  LPC_SCU->SFSPC_2 = (1 << 6) | (1 << 5) | 0x3;  /* PC.2  = ENET_TXD2         */
  LPC_SCU->SFSPC_3 = (1 << 6) | (1 << 5) | 0x3;  /* PC.3  = ENET_TXD3         */
  LPC_SCU->SFSPC_5 = (1 << 6) | (1 << 5) | 0x3;  /* PC.5  = ENET_TX_ER        */
  LPC_SCU->SFSPC_6 = (1 << 6) | (1 << 5) | 0x3;  /* PC.6  = ENET_RXD2         */
  LPC_SCU->SFSPC_7 = (1 << 6) | (1 << 5) | 0x3;  /* PC.7  = ENET_RXD3         */
  LPC_SCU->SFSPC_8 = (1 << 6) | (1 << 5) | 0x3;  /* PC.8  = ENET_RX_DV        */
#endif

  /* Reset Ethernet Controller peripheral */
  LPC_RGU->RESET_CTRL0 = ETHERNET_RST;
  while (!(LPC_RGU->RESET_ACTIVE_STATUS0 & ETHERNET_RST));

  /* Reset MAC Subsystem internal registers */
  LPC_ETHERNET->DMA_BUS_MODE |= DBMR_SWR;
  while (LPC_ETHERNET->DMA_BUS_MODE & DBMR_SWR);

  /* Put the DP83848C in reset mode */
  write_PHY (PHY_REG_BMCR, PHY_BMCR_RESET);

  /* Wait for hardware reset to end. */
  for (tout = 0; tout < TIMEOUT; tout++) {
    regv = read_PHY (PHY_REG_BMCR);
    if (!(regv & PHY_BMCR_RESET)) {
      /* Reset complete */
      break;
    }
  }
  
  /* Check if this is a DP83848C PHY. */
  id1 = read_PHY (PHY_REG_IDR1);
  id2 = read_PHY (PHY_REG_IDR2);

  if (((id1 << 16) | (id2 & 0xFFF0)) == DP83848C_ID) {
	/* Configure the PHY device */
#if defined (_10MBIT_)
    /* Connect at 10MBit */
    write_PHY (PHY_REG_BMCR, PHY_FULLD_10M);
#elif defined (_100MBIT_)
    /* Connect at 100MBit */
    write_PHY (PHY_REG_BMCR, PHY_FULLD_100M);
#else    
    /* Use autonegotiation about the link speed. */
    write_PHY (0x04, 0x01E1);
    write_PHY (PHY_REG_BMCR, PHY_AUTO_NEG);
    /* Wait to complete Auto_Negotiation. */
    for (tout = 0; tout < TIMEOUT; tout++) {
      regv = read_PHY (PHY_REG_BMSR);
      if (regv & PHY_AUTO_NEG_DONE) {
        /* Autonegotiation Complete. */
        break;
      }
    }
#endif
  }

  /* Check the link status. */
  for (tout = 0; tout < TIMEOUT; tout++) {
    regv = read_PHY (PHY_REG_STS);
    if (regv & LINK_VALID_STS) {
      /* Link is on. */
      break;
    }
  }

  /* Initialize MAC control register */
  LPC_ETHERNET->MAC_CONFIG = MCR_DO;

  /* Configure Full/Half Duplex mode. */
  if (regv & FULL_DUP_STS) {
    /* Full duplex is enabled. */
    LPC_ETHERNET->MAC_CONFIG |= MCR_DM;
  }

  /* Configure 100MBit/10MBit mode. */
  if (~(regv & SPEED_10M_STS)) {
    /* 100MBit mode. */
    LPC_ETHERNET->MAC_CONFIG |= MCR_FES;  
  }
   
  /* Set the Ethernet MAC Address registers */
  LPC_ETHERNET->MAC_ADDR0_HIGH = ((U32)own_hw_adr[5] <<  8) | (U32)own_hw_adr[4];
  LPC_ETHERNET->MAC_ADDR0_LOW  = ((U32)own_hw_adr[3] << 24) |
                                 ((U32)own_hw_adr[2] << 16) |
                                 ((U32)own_hw_adr[1] <<  8) |
                                  (U32)own_hw_adr[0];

  /* Reset all interrupts */
  LPC_ETHERNET->DMA_STAT = 0x0001FFFF;

  /* Enable Rx interrupts */
  LPC_ETHERNET->DMA_INT_EN =  INT_NISE | INT_AISE | INT_RBUIE | INT_RIE;

  /* Initialize Descriptor Lists    */
  rx_descr_init();
  tx_descr_init();

  /* Configure Frame Filtering */
  LPC_ETHERNET->MAC_FRAME_FILTER = MFFR_PM | MFFR_PAM | MFFR_RA;
  LPC_ETHERNET->MAC_FLOW_CTRL    = MFCR_DZQP;

  /* Start Transmission & Receive processes */
  LPC_ETHERNET->DMA_OP_MODE = DOMR_FTF | DOMR_ST | DOMR_SR;
  
  /* Enable Receiver and Transmitter */  
  LPC_ETHERNET->MAC_CONFIG |= MCR_TE | MCR_RE;
}

/*---------------------------- int_enable_eth --------------------------------*/

void int_enable_eth (void) {
  /* Ethernet Interrupt Enable function. */
  NVIC_EnableIRQ (ETHERNET_IRQn);
}


/*---------------------------- int_disable_eth -------------------------------*/

void int_disable_eth (void) {
  /* Ethernet Interrupt Disable function. */
  NVIC_DisableIRQ (ETHERNET_IRQn);
}


/*--------------------------- send_frame -------------------------------------*/

void send_frame (OS_FRAME *frame) {
  /* Send frame to EMAC ethernet controller */
  U32 *sp,*dp;
  U32 i,j;

  j = TxBufIndex;
  /* Wait until previous packet transmitted. */
  while (Tx_Desc[j].CtrlStat & DMA_TX_OWN);

  sp = (U32 *)&frame->data[0];
  dp = (U32 *)(Tx_Desc[j].Addr & ~3);

  /* Copy frame data to ETH IO buffer. */
  for (i = (frame->length + 3) >> 2; i; i--) {
    *dp++ = *sp++;
  }
  Tx_Desc[j].Size      = frame->length;
  Tx_Desc[j].CtrlStat |= DMA_TX_OWN;
  if (++j == NUM_TX_BUF) j = 0;
  TxBufIndex = j;
  /* Start frame transmission. */
  LPC_ETHERNET->DMA_STAT = DSR_TPSS;
  LPC_ETHERNET->DMA_TRANS_POLL_DEMAND = 0;
}


/*--------------------------- interrupt_ethernet -----------------------------*/

void ETH_IRQHandler (void) {
  OS_FRAME *frame;
  U32 i, RxLen;
  U32 *sp,*dp;

  i = RxBufIndex;
  do {
    if (Rx_Desc[i].Stat & DMA_RX_ERROR_MASK) {
      goto rel;
    }
    if ((Rx_Desc[i].Stat & DMA_RX_SEG_MASK) != DMA_RX_SEG_MASK) {
      goto rel;
    }
    RxLen = ((Rx_Desc[i].Stat >> 16) & 0x3FFF) - 4;
    if (RxLen > ETH_MTU) {
      /* Packet too big, ignore it and free buffer. */
      goto rel;
    }
    /* Flag 0x80000000 to skip sys_error() call when out of memory. */
    frame = alloc_mem (RxLen | 0x80000000);
    /* if 'alloc_mem()' has failed, ignore this packet. */
    if (frame != NULL) {
      sp = (U32 *)(Rx_Desc[i].Addr & ~3);
      dp = (U32 *)&frame->data[0];
      for (RxLen = (RxLen + 3) >> 2; RxLen; RxLen--) {
        *dp++ = *sp++;
      }
      put_in_queue (frame);
    }
    /* Release this frame from ETH IO buffer. */
rel:Rx_Desc[i].Stat = DMA_RX_OWN;

    if (++i == NUM_RX_BUF) i = 0;
    RxBufIndex = i;  
  }
  while (!(Rx_Desc[i].Stat & DMA_RX_OWN));
  RxBufIndex = i;

  if (LPC_ETHERNET->DMA_STAT & INT_RBUIE) {
    /* Receive buffer unavailable, resume DMA */
    LPC_ETHERNET->DMA_STAT = INT_RBUIE;
    LPC_ETHERNET->DMA_REC_POLL_DEMAND = 0;       
  }
  /* Clear pending interrupt bits */
  LPC_ETHERNET->DMA_STAT = INT_NISE | INT_AISE | INT_RBUIE | INT_RIE;
}


/*---------------------------- rx_descr_init ---------------------------------*/

static void rx_descr_init (void) {
  /* Initialize Receive DMA Descriptor array. */
  U32 i, next;

  RxBufIndex = 0;
  for (i = 0, next = 0; i < NUM_RX_BUF; i++) {
    if (++next == NUM_RX_BUF) next = 0;
    Rx_Desc[i].Stat = DMA_RX_OWN;
    Rx_Desc[i].Ctrl = DMA_RX_RCH | ETH_BUF_SIZE;
    Rx_Desc[i].Addr = (U32)&rx_buf[i];
    Rx_Desc[i].Next = (U32)&Rx_Desc[next];
  }
  LPC_ETHERNET->DMA_REC_DES_ADDR = (U32)&Rx_Desc[0];
}


/*---------------------------- tx_descr_init ---------------------------------*/

static void tx_descr_init (void) {
  /* Initialize Transmit DMA Descriptor array. */
  U32 i,next;

  TxBufIndex = 0;
  for (i = 0, next = 0; i < NUM_TX_BUF; i++) {
    if (++next == NUM_TX_BUF) next = 0;
    Tx_Desc[i].CtrlStat = DMA_TX_TCH | DMA_TX_LS | DMA_TX_FS;
    Tx_Desc[i].Addr     = (U32)&tx_buf[i];
    Tx_Desc[i].Next     = (U32)&Tx_Desc[next];
  }
  LPC_ETHERNET->DMA_TRANS_DES_ADDR = (U32)&Tx_Desc[0];
}


/*---------------------------- write_PHY -------------------------------------*/

static void write_PHY (U32 PhyReg, U16 Value) {
  /* Write a data 'Value' to PHY register 'PhyReg'. */
  U32 tout;

  /* Write a data 'Value' to PHY register 'PhyReg'. */
  while (LPC_ETHERNET->MAC_MII_ADDR & MMAR_GB);
  LPC_ETHERNET->MAC_MII_DATA  = Value;
  LPC_ETHERNET->MAC_MII_ADDR  = MMAR_GB               |
                                MMAR_MW               |
                                CSR_CLK_RANGE    << 2 |
                                PhyReg           << 6 |
                                DP83848C_DEF_ADR << 11;

  /* Wait until operation completed */
  for (tout = 0; tout < MII_WR_TOUT; tout++) {
    if ((LPC_ETHERNET->MAC_MII_ADDR & MMAR_GB) == 0) {
      break;
    }
  }
}


/*---------------------------- read_PHY --------------------------------------*/

static U16 read_PHY (U32 PhyReg) {
  /* Read a PHY register 'PhyReg'. */   
  U32 tout;

  /* Read a PHY register 'PhyReg'. */
  while(LPC_ETHERNET->MAC_MII_ADDR & MMAR_GB);
  LPC_ETHERNET->MAC_MII_ADDR =  MMAR_GB               |
                                CSR_CLK_RANGE    << 2 |
                                PhyReg           << 6 |
                                DP83848C_DEF_ADR << 11;

  /* Wait until operation completed */
  for (tout = 0; tout < MII_RD_TOUT; tout++) {
    if ((LPC_ETHERNET->MAC_MII_ADDR & MMAR_GB) == 0) {
      break;
    }
  }
  return (LPC_ETHERNET->MAC_MII_DATA & MMDR_GD);
}

/*-----------------------------------------------------------------------------
 * End Of File
 *----------------------------------------------------------------------------*/
