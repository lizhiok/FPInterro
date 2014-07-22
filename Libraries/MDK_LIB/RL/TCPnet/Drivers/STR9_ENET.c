/*----------------------------------------------------------------------------
 *      RL-ARM - TCPnet
 *----------------------------------------------------------------------------
 *      Name:    STR9_ENET.C
 *      Purpose: Driver for ST STR912 ENET Ethernet Controller
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <Net_Config.h>
#include "STR9_ENET.h"
#include <91x_lib.h>                    /* STR912F definitions               */

/* The following macro definitions may be used to select the speed
   of the physical link:

  _10MBIT_   - connect at 10 MBit only
  _100MBIT_  - connect at 100 MBit only

  By default an autonegotiation of the link speed is used. This may take 
  longer to connect, but it works for 10MBit and 100MBit physical links.     */

/* Net_Config.c */
extern U8 own_hw_adr[];

/* Local variables */
static U32 TxBufIndex;
static U32 RxBufIndex;

/* ENET local DMA Descriptors. */
static DMA_Desc Rx_Desc[NUM_RX_BUF];
static DMA_Desc Tx_Desc[NUM_TX_BUF];

/* ENET local DMA buffers. */
static U32 rx_buf[NUM_RX_BUF][ETH_BUF_SIZE>>2];
static U32 tx_buf[NUM_TX_BUF][ETH_BUF_SIZE>>2];

/*----------------------------------------------------------------------------
 *      ENET Ethernet Driver Functions
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
static void fetch_packet (BOOL check2);
static void rx_descr_init (void);
static void tx_descr_init (void);
static void write_PHY (U32 PhyReg, U16 Value);
static U16  read_PHY (U32 PhyReg);

/*--------------------------- init_ethernet ---------------------------------*/

void init_ethernet (void) {
  /* Initialize the ENET ethernet controller. */
  U32 regv,tout,id1,id2;

  /* Reset ENET and DMA devices. */
  ENET_DMA->SCR |= SCR_SRESET;

  /* Enable GPIO1 Ethernet Pins. */
  GPIO1->DDR         |=  0x9E;
  GPIO1->DR[0x9E<<2]  =  0x00;
  SCU->GPIOTYPE[1]   &= ~0x9E;
  SCU->GPIOIN[1]     &= ~0x9E;
  SCU->GPIOOUT[1]    &= ~0xC3FC;
  SCU->GPIOOUT[1]    |=  0x82A8;

  /* Enable GPIO5 Ethernet Pins, drive MII clock 25MHz. */
  GPIO5->DDR         |=  0x0C;
  GPIO5->DR[0x0C<<2]  =  0x00;
  SCU->GPIOTYPE[5]   &= ~0x0C;
  SCU->GPIOIN[5]     &= ~0x0C;
  SCU->GPIOOUT[5]    &= ~0x00F0;
  SCU->GPIOOUT[5]    |=  0x00A0;

  /* Remove reset for ENET and MAC devices */
  ENET_DMA->SCR &= ~SCR_SRESET;

  /* Initialize MAC control register, accept multicast packets. */
  ENET_MAC->MCR = MCR_PFM_MCAST | MCR_RVFF | MCR_APR | MCR_DCE;

  /* If HCLK > 50MHz enable the following line. */
  //ENET_MAC->MCR |= MCR_PS_DEF;

  /* Setup Tx & Rx burst size. */
  regv = ENET_DMA->SCR & ~(SCR_TX_BURST_SIZE | SCR_RX_BURST_SIZE);
  ENET_DMA->SCR = regv |  (SCR_TX_BURST_DEF  | SCR_RX_BURST_DEF);

  /* Set clock to PCLK */
  ENET_DMA->CCR = (ENET_DMA->CCR & CCR_SEL_CLK) | CCR_SEL_CLK_DEF;

  /* Put the STE100P in reset mode */
  write_PHY (PHY_REG_XCR, 0x8000);

  /* Wait for hardware reset to end. */
  for (tout = 0; tout < 0x100000; tout++) {
    regv = read_PHY (PHY_REG_XCR);
    if (!(regv & 0x8000)) {
      /* Reset complete */
      break;
    }
  }
  /* Check if this is a STE100P PHY. */
  id1 = read_PHY (PHY_REG_PID1);
  id2 = read_PHY (PHY_REG_PID2);

  if (((id1 << 16) | (id2 & 0xFFF0)) == STE100P_ID) {
    /* Configure the PHY device */
#if defined (_10MBIT_)
    /* Connect at 10MBit */
    write_PHY (PHY_REG_XCR, PHY_FULLD_10M);
#elif defined (_100MBIT_)
    /* Connect at 100MBit */
    write_PHY (PHY_REG_XCR, PHY_FULLD_100M);
#else
    /* Use autonegotiation about the link speed. */
    write_PHY (PHY_REG_XCR, PHY_AUTO_NEG);
    /* Wait to complete Auto_Negotiation. */
    for (tout = 0; tout < 0x100000; tout++) {
      regv = read_PHY (PHY_REG_XSR);
      if (regv & 0x0020) {
        /* ANEG_ACK set, autonegotiation finished. */
        break;
      }
    }
#endif
  }

  /* Check the link status. */
  for (tout = 0; tout < 0x10000; tout++) {
    regv = read_PHY (PHY_REG_XSR);
    if (regv & 0x0004) {
      /* Link is on. */
      break;
    }
  }
  
  regv = read_PHY (PHY_REG_XCIIS);
  if (regv & 0x0100) {
    /* Full duplex is enabled. */
    ENET_MAC->MCR |= MCR_FDM;
  }
  else {
    /* Half duplex mode. */
    ENET_MAC->MCR |= MCR_DRO;
  }

  /* Set the Ethernet MAC Address registers */
  ENET_MAC->MAH = ((U32)own_hw_adr[5] << 8)  |  (U32)own_hw_adr[4];
  ENET_MAC->MAL = ((U32)own_hw_adr[3] << 24) | ((U32)own_hw_adr[2] << 16) |
                  ((U32)own_hw_adr[1] << 8)  |  (U32)own_hw_adr[0];

  /* Initialize Tx and Rx DMA Descriptors */
  rx_descr_init ();
  tx_descr_init ();

  /* Force a ENET abort by software */
  ENET_DMA->RXSTR |= RXSTR_DMA_EN;
  ENET_DMA->TXSTR |= TXSTR_DMA_EN;

  /* Setup Descriptor Fetch Delay */
  ENET_DMA->RXSTR = (ENET_DMA->RXSTR & ~RXSTR_DFETCH_DLY) | RXSTR_DFETCH_DEF |
                     RXSTR_COLL_SEEN |  RXSTR_RUNT_FRAME  | RXSTR_FILTER_FAIL;
  ENET_DMA->TXSTR = (ENET_DMA->TXSTR & ~TXSTR_DFETCH_DLY) | TXSTR_DFETCH_DEF |
                     TXSTR_UNDER_RUN;

  /* Enable ENET interrupts. */
  ENET_DMA->IER   = INT_RX_CURR_DONE | INT_TX_CURR_DONE | INT_RX_DONE;

  /* Reset all interrupts */
  ENET_DMA->ISR   = 0xFFFFFFFF;

  /* Enable receive and transmit mode of MAC Ethernet core */
  ENET_MAC->MCR  |= (MCR_TE | MCR_RE);

  /* Start the receive operation */
  ENET_DMA->RXSTR |= RXSTR_START_FETCH;

  /* Configure VIC for EMAC interrupt. */
  VIC0->DVAR     = (U32)def_interrupt;
  VIC0->VAiR[14] = (U32)interrupt_ethernet;
  VIC0->INTSR   &= ~(1 << 11);
  VIC0->VCiR[14] = 0x20 | 11;
}


/*--------------------------- def_interrupt ---------------------------------*/

static void def_interrupt (void) __irq  {
  /* Default Interrupt Function: may be called when timer ISR is disabled */
  VIC0->VAR = 0;
  VIC1->VAR = 0;
}


/*--------------------------- int_enable_eth --------------------------------*/

void int_enable_eth (void) {
  /* Ethernet Interrupt Enable function. */
  VIC0->INTER = 1 << 11;
}


/*--------------------------- int_disable_eth -------------------------------*/

void int_disable_eth (void) {
  /* Ethernet Interrupt Disable function. */
  VIC0->INTECR = 1 << 11;
}


/*--------------------------- send_frame ------------------------------------*/

void send_frame (OS_FRAME *frame) {
  /* Send frame to ENET ethernet controller */
  static BOOL started;
  U32 *sp,*dp;
  U32 i,j;

  if (!(Tx_Desc[TxBufIndex].Stat & DMA_TX_VALID)) {
    j  = TxBufIndex;
    sp = (U32 *)&frame->data[0];
    dp = (U32 *)(Tx_Desc[j].Addr & ~3);

    /* Copy frame data to ENET IO buffer. */
    for (i = (frame->length + 3) >> 2; i; i--) {
      *dp++ = *sp++;
    }

    /* A small delay before updating descriptors. */
    for (i = 0; i < 10; i++) {
      __nop ();
    }

    Tx_Desc[j].Ctrl = frame->length | DMA_CTRL_NEXT_EN;
    Tx_Desc[j].Stat = DMA_TX_VALID;
    if (++j == NUM_TX_BUF) j = 0;
    TxBufIndex = j;
    if (!started) {
      /* Start Transmit DMA. */
      ENET_DMA->TXSTR |= TXSTR_START_FETCH;
      started = __TRUE;
    }
  }
}


/*--------------------------- fetch_packet ----------------------------------*/

static void fetch_packet (BOOL check2) {
  /* Fetch a packet from DMA buffer and release buffer. */
  OS_FRAME *frame;
  U32 i,RxLen;
  U32 *sp,*dp;

  i = RxBufIndex;
fetch:
  /* Check if a packet is valid. */
  if (Rx_Desc[i].Stat & DMA_RX_ERROR_MASK) {
    /* Erroneous Frame, ignore it. */
    goto rel;
  }

  /* Frame valid, get frame length. */
  RxLen = (Rx_Desc[i].Stat & DMA_RX_FLEN) - 4;
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
  /* Release this frame from ENET IO buffer. */
rel:Rx_Desc[i].Stat = DMA_RX_VALID;

  if (++i == NUM_RX_BUF) i = 0;

  /* Check for possible ISR RX event loss. */
  if (check2 == __TRUE) {
    if ((Rx_Desc[i].Stat & DMA_RX_VALID) == 0) {
      /* Yes, this must be the case because another packet is ready. */
      check2 = __FALSE;
      goto fetch;
    }
  }

  RxBufIndex = i;
}


/*--------------------------- interrupt_ethernet ----------------------------*/

static void interrupt_ethernet (void) __irq {
  /* ENET Ethernet Controller Interrupt function. */
  U32 i,int_stat;

  while ((int_stat = (ENET_DMA->ISR & ENET_DMA->IER)) != 0) {
    if (int_stat & INT_RX_DONE) {
      /* RX master DMA completed, all descriptors are used. */
      for (i = 0; i < NUM_RX_BUF; i++) {
        fetch_packet (__FALSE);
      }
      /* Restart Receive DMA. */
      ENET_DMA->RXNDAR = (U32)&Rx_Desc[0];
      RxBufIndex = 0;
      ENET_DMA->RXSTR |= RXSTR_START_FETCH;
      ENET_DMA->ISR = INT_RX_DONE | INT_RX_CURR_DONE;
    }
    else if (int_stat & INT_RX_CURR_DONE) {
      /* Valid frame has been received. */
      fetch_packet (__TRUE);
      ENET_DMA->ISR = INT_RX_CURR_DONE;
    }
    if (int_stat & INT_TX_CURR_DONE) {
      /* Frame transmit completed. */
      ENET_DMA->ISR = INT_TX_CURR_DONE;
    }
  }
  /* Acknowledge the VIC interrupt. */
  VIC0->VAR = 0;
}


/*--------------------------- rx_descr_init ---------------------------------*/

static void rx_descr_init (void) {
  /* Initialize Receive DMA Descriptor array. */
  U32 i,next;

  RxBufIndex = 0;
  for (i = 0, next = 0; i < NUM_RX_BUF; i++) {
    if (++next == NUM_RX_BUF) next = 0;
    Rx_Desc[i].Ctrl = ETH_BUF_SIZE | DMA_CTRL_NEXT_EN;
    Rx_Desc[i].Addr = (U32)&rx_buf[i];
    Rx_Desc[i].Next = (U32)&Rx_Desc[next];
    Rx_Desc[i].Stat = DMA_RX_VALID;
  }
  ENET_DMA->RXNDAR = (U32)&Rx_Desc[0];
}


/*--------------------------- tx_descr_init ---- ----------------------------*/

static void tx_descr_init (void) {
  /* Initialize Transmit DMA Descriptor array. */
  U32 i,next;

  TxBufIndex = 0;
  for (i = 0, next = 0; i < NUM_TX_BUF; i++) {
    if (++next == NUM_TX_BUF) next = 0;
    Tx_Desc[i].Ctrl = 0;
    Tx_Desc[i].Addr = (U32)&tx_buf[i];
    Tx_Desc[i].Next = (U32)&Tx_Desc[next] | DMA_NEXT_POL_EN;
    Tx_Desc[i].Stat = 0;
  }
  ENET_DMA->TXNDAR = (U32)&Tx_Desc[0] | TXNDAR_NPOL_EN;
}


/*--------------------------- write_PHY -------------------------------------*/

static void write_PHY (U32 PhyReg, U16 Value) {
  /* Write a data 'Value' to PHY register 'PhyReg'. */
  U32 tout;

  ENET_MAC->MIID = Value;
  ENET_MAC->MIIA = STE100P_DEF_ADR | (PhyReg << 6) | MIIA_WR | MIIA_BUSY;

  /* Wait utill operation completed */
  tout = 0;
  for (tout = 0; tout < MII_WR_TOUT; tout++) {
    if ((ENET_MAC->MIIA & MIIA_BUSY) == 0) {
      break;
    }
  }
}


/*--------------------------- read_PHY --------------------------------------*/

static U16 read_PHY (U32 PhyReg) {
  /* Read a PHY register 'PhyReg'. */
  U32 tout;

  ENET_MAC->MIIA = STE100P_DEF_ADR | (PhyReg << 6) | MIIA_BUSY;

  /* Wait until operation completed */
  tout = 0;
  for (tout = 0; tout < MII_RD_TOUT; tout++) {
    if ((ENET_MAC->MIIA & MIIA_BUSY) == 0) {
      break;
    }
  }
  return (ENET_MAC->MIID);
}


/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
