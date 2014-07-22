/*-----------------------------------------------------------------------------
 *      RL-ARM - TCPnet
 *-----------------------------------------------------------------------------
 *      Name:    EMAC_LPC177x_8x.c
 *      Purpose: Driver for NXP LPC177x_8x EMAC Ethernet Controller
 *      Rev.:    V4.70
 *-----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include <Net_Config.h>
#include "EMAC_LPC177x_8x.h"
#include <LPC177x_8x.h>                 /* LPC177x/8x definitions             */


/* The following macro definitions may be used to select the speed
   of the physical link:

  _10MBIT_   - connect at 10 MBit only
  _100MBIT_  - connect at 100 MBit only

  By default an autonegotiation of the link speed is used. This may take 
  longer to connect, but it works for 10MBit and 100MBit physical links.      */

/* Net_Config.c */
extern U8 own_hw_adr[];

/* EMAC local DMA Descriptors. */
static            RX_Desc Rx_Desc[NUM_RX_FRAG];
static __align(8) RX_Stat Rx_Stat[NUM_RX_FRAG]; /* Must be 8-Byte alligned    */
static            TX_Desc Tx_Desc[NUM_TX_FRAG];
static            TX_Stat Tx_Stat[NUM_TX_FRAG];

/* EMAC local DMA buffers. */
static U32 rx_buf[NUM_RX_FRAG][ETH_FRAG_SIZE>>2];
static U32 tx_buf[NUM_TX_FRAG][ETH_FRAG_SIZE>>2];

/*-----------------------------------------------------------------------------
 *      EMAC Ethernet Driver Functions
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
static U16  read_PHY  (U32 PhyReg);

/*--------------------------- init_ethernet ----------------------------------*/

void init_ethernet (void) {
  /* Initialize the EMAC ethernet controller. */
  U32 regv,tout,id1,id2;

  /* Power Up the EMAC controller. */
  LPC_SC->PCONP |= (1U << 30);

  /* Configure Ethernet Pins. */
  LPC_IOCON->P1_0  = EMAC_PIN | 1;  /* P1.0  == ENET_TXD0    */
  LPC_IOCON->P1_1  = EMAC_PIN | 1;  /* P1.1  == ENET_TXD1    */
  LPC_IOCON->P1_4  = EMAC_PIN | 1;  /* P1.4  == nENET_TX_EN  */
  LPC_IOCON->P1_8  = EMAC_PIN | 1;  /* P1.8  == ENET_CRS_DV  */
  LPC_IOCON->P1_9  = EMAC_PIN | 1;  /* P1.9  == ENET_RXD0    */
  LPC_IOCON->P1_10 = EMAC_PIN | 1;  /* P1.10 == ENET_RXD1    */
  LPC_IOCON->P1_14 = EMAC_PIN | 1;  /* P1.14 == ENET_RX_ER   */
  LPC_IOCON->P1_15 = EMAC_PIN | 1;  /* P1.15 == ENET_REF_CLK */
  LPC_IOCON->P1_16 = EMAC_PIN | 1;  /* P1.16 == ENET_MDC     */
  LPC_IOCON->P1_17 = EMAC_PIN | 1;  /* P1.17 == ENET_MDIO    */

  /* Reset all EMAC internal modules. */
  LPC_EMAC->MAC1    = MAC1_RES_TX | MAC1_RES_MCS_TX | MAC1_RES_RX | 
                      MAC1_RES_MCS_RX | MAC1_SIM_RES | MAC1_SOFT_RES;
  LPC_EMAC->Command = CR_REG_RES | CR_TX_RES | CR_RX_RES | CR_PASS_RUNT_FRM;

  /* A short delay after reset. */
  for (tout = 100; tout; tout--);

  /* Initialize MAC control registers. */
  LPC_EMAC->MAC1 = MAC1_PASS_ALL;
  LPC_EMAC->MAC2 = MAC2_CRC_EN | MAC2_PAD_EN;
  LPC_EMAC->MAXF = ETH_MAX_FLEN;
  LPC_EMAC->CLRT = CLRT_DEF;
  LPC_EMAC->IPGR = IPGR_DEF;

  /* Enable Reduced MII interface. */
  LPC_EMAC->MCFG = MCFG_CLK_DIV64 | MCFG_RES_MII;
  for (tout = 100; tout; tout--);
  LPC_EMAC->MCFG = MCFG_CLK_DIV64;

  /* Enable Reduced MII interface. */
  LPC_EMAC->Command = CR_RMII | CR_PASS_RUNT_FRM;

  /* Reset Reduced MII Logic. */
  LPC_EMAC->SUPP = SUPP_RES_RMII;
  for (tout = 100; tout; tout--);
  LPC_EMAC->SUPP = 0;

  /* Put the LAN8700 in reset mode */
  write_PHY (PHY_REG_BCR, 0x8000);

  /* Wait for hardware reset to end. */
  for (tout = 0; tout < 0x100000; tout++) {
    regv = read_PHY (PHY_REG_BCR);
    if (!(regv & 0x8000)) {
      /* Reset complete */
      break;
    }
  }

  /* Check if this is a LAN8700 PHY. */
  id1 = read_PHY (PHY_REG_PID1);
  id2 = read_PHY (PHY_REG_PID2);

  if (((id1 << 16) | (id2 & 0xFFF0)) == LAN8700_ID) {
    /* Configure the PHY device */
#if defined (_10MBIT_)
    /* Connect at 10MBit */
    write_PHY (PHY_REG_BCR, PHY_FULLD_10M);
#elif defined (_100MBIT_)
    /* Connect at 100MBit */
    write_PHY (PHY_REG_BCR, PHY_FULLD_100M);
#else
    /* Use autonegotiation about the link speed. */
    write_PHY (PHY_REG_BCR, PHY_AUTO_NEG);
    /* Wait to complete Auto_Negotiation. */
    for (tout = 0; tout < 0x100000; tout++) {
      regv = read_PHY (PHY_REG_BSR);
      if (regv & 0x0020) {
        /* Autonegotiation Complete. */
        break;
      }
    }
#endif
  }

  /* Check the link status. */
  for (tout = 0; tout < 0x10000; tout++) {
    regv = read_PHY (PHY_REG_BSR);
    if (regv & 0x0004) {
      /* Link is on. */
      break;
    }
  }

  /* Configure Full/Half Duplex mode. */
  if (regv & 0x0004) {
    /* Full duplex is enabled. */
    LPC_EMAC->MAC2    |= MAC2_FULL_DUP;
    LPC_EMAC->Command |= CR_FULL_DUP;
    LPC_EMAC->IPGT     = IPGT_FULL_DUP;
  }
  else {
    /* Half duplex mode. */
    LPC_EMAC->IPGT = IPGT_HALF_DUP;
  }

  /* Configure 100MBit/10MBit mode. */
  if (regv & 0x0002) {
    /* 10MBit mode. */
    LPC_EMAC->SUPP = 0;
  }
  else {
    /* 100MBit mode. */
    LPC_EMAC->SUPP = SUPP_SPEED;
  }

  /* Set the Ethernet MAC Address registers */
  LPC_EMAC->SA0 = ((U32)own_hw_adr[5] << 8) | (U32)own_hw_adr[4];
  LPC_EMAC->SA1 = ((U32)own_hw_adr[3] << 8) | (U32)own_hw_adr[2];
  LPC_EMAC->SA2 = ((U32)own_hw_adr[1] << 8) | (U32)own_hw_adr[0];

  /* Initialize Tx and Rx DMA Descriptors */
  rx_descr_init ();
  tx_descr_init ();

  /* Receive Broadcast, Multicast and Perfect Match Packets */
  LPC_EMAC->RxFilterCtrl = RFC_MCAST_EN | RFC_BCAST_EN | RFC_PERFECT_EN;

  /* Enable EMAC interrupts. */
  LPC_EMAC->IntEnable = INT_RX_DONE | INT_TX_DONE;

  /* Reset all interrupts */
  LPC_EMAC->IntClear  = 0xFFFF;

  /* Enable receive and transmit mode of MAC Ethernet core */
  LPC_EMAC->Command  |= (CR_RX_EN | CR_TX_EN);
  LPC_EMAC->MAC1     |= MAC1_REC_EN;
}


/*--------------------------- int_enable_eth ---------------------------------*/

void int_enable_eth (void) {
  /* Ethernet Interrupt Enable function. */
  NVIC_EnableIRQ(ENET_IRQn);
}


/*--------------------------- int_disable_eth --------------------------------*/

void int_disable_eth (void) {
  /* Ethernet Interrupt Disable function. */
  NVIC_DisableIRQ(ENET_IRQn);
}


/*--------------------------- send_frame -------------------------------------*/

void send_frame (OS_FRAME *frame) {
  /* Send frame to EMAC ethernet controller */
  U32 idx,len;
  U32 *sp,*dp;

  idx = LPC_EMAC->TxProduceIndex;
  sp  = (U32 *)&frame->data[0];
  dp  = (U32 *)Tx_Desc[idx].Packet;

  /* Copy frame data to EMAC packet buffers. */
  for (len = (frame->length + 3) >> 2; len; len--) {
    *dp++ = *sp++;
  }
  Tx_Desc[idx].Ctrl = (frame->length-1) | (TCTRL_INT | TCTRL_LAST);

  /* Start frame transmission. */
  if (++idx == NUM_TX_FRAG) idx = 0;
  LPC_EMAC->TxProduceIndex = idx;
}


/*--------------------------- interrupt_ethernet -----------------------------*/

void ENET_IRQHandler (void) {
  /* EMAC Ethernet Controller Interrupt function. */
  OS_FRAME *frame;
  U32 idx,int_stat,RxLen,info;
  U32 *sp,*dp;

  while ((int_stat = (LPC_EMAC->IntStatus & LPC_EMAC->IntEnable)) != 0) {
    LPC_EMAC->IntClear = int_stat;
    if (int_stat & INT_RX_DONE) {
      /* Packet received, check if packet is valid. */
      idx = LPC_EMAC->RxConsumeIndex;
      while (idx != LPC_EMAC->RxProduceIndex) {
        info = Rx_Stat[idx].Info;
        if (!(info & RINFO_LAST_FLAG)) {
          goto rel;
        }

        RxLen = (info & RINFO_SIZE) - 3;
        if (RxLen > ETH_MTU || (info & RINFO_ERR_MASK)) {
          /* Invalid frame, ignore it and free buffer. */
          goto rel;
        }
        /* Flag 0x80000000 to skip sys_error() call when out of memory. */
        frame = alloc_mem (RxLen | 0x80000000);
        /* if 'alloc_mem()' has failed, ignore this packet. */
        if (frame != NULL) {
          dp = (U32 *)&frame->data[0];
          sp = (U32 *)Rx_Desc[idx].Packet;
          for (RxLen = (RxLen + 3) >> 2; RxLen; RxLen--) {
            *dp++ = *sp++;
          }
          put_in_queue (frame);
        }
rel:    if (++idx == NUM_RX_FRAG) idx = 0;
        /* Release frame from EMAC buffer. */
        LPC_EMAC->RxConsumeIndex = idx;
      }
    }
    if (int_stat & INT_TX_DONE) {
      /* Frame transmit completed. */
    }
  }

}


/*--------------------------- rx_descr_init ----------------------------------*/

static void rx_descr_init (void) {
  /* Initialize Receive Descriptor and Status array. */
  U32 i;

  for (i = 0; i < NUM_RX_FRAG; i++) {
    Rx_Desc[i].Packet  = (U32)&rx_buf[i];
    Rx_Desc[i].Ctrl    = RCTRL_INT | (ETH_FRAG_SIZE-1);
    Rx_Stat[i].Info    = 0;
    Rx_Stat[i].HashCRC = 0;
  }

  /* Set EMAC Receive Descriptor Registers. */
  LPC_EMAC->RxDescriptor       = (U32)&Rx_Desc[0];
  LPC_EMAC->RxStatus           = (U32)&Rx_Stat[0];
  LPC_EMAC->RxDescriptorNumber = NUM_RX_FRAG-1;

  /* Rx Descriptors Point to 0 */
  LPC_EMAC->RxConsumeIndex  = 0;
}


/*--------------------------- tx_descr_init ---- -----------------------------*/

static void tx_descr_init (void) {
  /* Initialize Transmit Descriptor and Status array. */
  U32 i;

  for (i = 0; i < NUM_TX_FRAG; i++) {
    Tx_Desc[i].Packet = (U32)&tx_buf[i];
    Tx_Desc[i].Ctrl   = 0;
    Tx_Stat[i].Info   = 0;
  }

  /* Set EMAC Transmit Descriptor Registers. */
  LPC_EMAC->TxDescriptor       = (U32)&Tx_Desc[0];
  LPC_EMAC->TxStatus           = (U32)&Tx_Stat[0];
  LPC_EMAC->TxDescriptorNumber = NUM_TX_FRAG-1;

  /* Tx Descriptors Point to 0 */
  LPC_EMAC->TxProduceIndex  = 0;
}


/*--------------------------- write_PHY --------------------------------------*/

static void write_PHY (U32 PhyReg, U16 Value) {
  /* Write a data 'Value' to PHY register 'PhyReg'. */
  U32 tout;

  LPC_EMAC->MADR = PHY_DEF_ADDR | PhyReg;
  LPC_EMAC->MWTD = Value;

  /* Wait utill operation completed */
  for (tout = 0; tout < MII_WR_TOUT; tout++) {
    if ((LPC_EMAC->MIND & MIND_BUSY) == 0) {
      break;
    }
  }
}


/*--------------------------- read_PHY ---------------------------------------*/

static U16 read_PHY (U32 PhyReg) {
  /* Read a PHY register 'PhyReg'. */
  U32 tout, val;

  LPC_EMAC->MADR = PHY_DEF_ADDR | PhyReg;
  LPC_EMAC->MCMD = MCMD_READ;

  /* Wait until operation completed */
  for (tout = 0; tout < MII_RD_TOUT; tout++) {
    if ((LPC_EMAC->MIND & MIND_BUSY) == 0) {
      break;
    }
  }
  LPC_EMAC->MCMD = 0;
  val = LPC_EMAC->MRDD;
  return (val);
}

/*-----------------------------------------------------------------------------
 * end of file
 *----------------------------------------------------------------------------*/
