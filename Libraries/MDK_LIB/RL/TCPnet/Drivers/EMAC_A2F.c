/*----------------------------------------------------------------------------
 *      RL-ARM - TCPnet
 *----------------------------------------------------------------------------
 *      Name:    EMAC_A2F.c
 *      Purpose: Driver for Actel SmartFusion Ethernet MAC Controller
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <Net_Config.h>
#include "EMAC_A2F.h"
#include <a2fxxxm3.h>

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

/* EMAC local DMA Descriptors. */
static DMA_Desc Rx_Desc[NUM_RX_BUF];
static DMA_Desc Tx_Desc[NUM_TX_BUF];

/* EMAC local DMA buffers. */
static U32 rx_buf[NUM_RX_BUF][ETH_RX_BUF_SIZE>>2];
static U32 tx_buf[NUM_TX_BUF][ETH_TX_BUF_SIZE>>2];

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
static void rx_desc_init (void);
static void tx_desc_init (void);
static void send_setup (void);
static void write_PHY (U32 PhyReg, U16 Value);
static U16  read_PHY (U32 PhyReg);
static void output_MDIO (U32 val, U32 n);
static void turnaround_MDIO (void);
static U32  input_MDIO (void);

/*--------------------------- init_ethernet ---------------------------------*/

void init_ethernet (void) {
  /* Initialize the EMAC ethernet controller. */
  U32 regv,tout,id1,id2;

  /* EMAC Software Reset. */
  MAC->CSR0 |= CSR0_SWR;
  while (MAC->CSR0 & CSR0_SWR);

  /* Initialize MAC control registers. */
  MAC->CSR0 &= ~(CSR0_BAR | CSR0_BLE | CSR0_DBO | CSR0_SPD |
                 CSR0_DSL | CSR0_PBL | CSR0_TAP);

  MAC->CSR6 &= ~(CSR6_HP  | CSR6_SR  | CSR6_HO  | CSR6_PB  |
                 CSR6_IF  | CSR6_PR  | CSR6_PM  | CSR6_FD  |
                 CSR6_ST  | CSR6_TR  | CSR6_TTM | CSR6_RA);

  /* Enable Store and Forward. */
  MAC->CSR6 |= CSR6_SF;

  /* Put the DP83848C in reset mode */
  write_PHY (PHY_REG_BMCR, 0x8000);

  /* Wait for hardware reset to end. */
  for (tout = 0; tout < 0x100000; tout++) {
    regv = read_PHY (PHY_REG_BMCR);
    if (!(regv & 0x8800)) {
      /* Reset complete, device not Power Down. */
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
    write_PHY (PHY_REG_BMCR, PHY_AUTO_NEG);
    /* Wait to complete Auto_Negotiation. */
    for (tout = 0; tout < 0x100000; tout++) {
      regv = read_PHY (PHY_REG_BMSR);
      if (regv & 0x0020) {
        /* Autonegotiation Complete. */
        break;
      }
    }
#endif
  }

  /* Check the link status. */
  for (tout = 0; tout < 0x10000; tout++) {
    regv = read_PHY (PHY_REG_STS);
    if (regv & 0x0001) {
      /* Link is on. */
      break;
    }
  }

  /* Configure Full/Half Duplex mode. */
  if (regv & 0x0004) {
    /* Full duplex is enabled. */
    MAC->CSR6 |= CSR6_FD;
  }

  /* Configure 100MBit/10MBit mode. */
  if ((regv & 0x0002) == 0) {
    /* 100MBit mode. */
    MAC->CSR6 |= CSR6_TTM;
  }

  /* Set the Ethernet MAC Address registers. */
  send_setup ();

  /* Initialize Tx and Rx DMA Descriptors */
  rx_desc_init ();
  tx_desc_init ();

  /* Receive Broadcast, Multicast and Perfect Match Packets */
  MAC->CSR6 |= CSR6_PM;
  
  /* Enable EMAC interrupts. */
  MAC->CSR7 = CSR7_TIE | CSR7_RIE | CSR7_NIE;

  /* Reset all interrupts */
  MAC->CSR5 = CSR5_TI  | CSR5_RI  | CSR5_NIS;

  /* Enable receive and transmit mode of MAC Ethernet core. */
  MAC->CSR6 |= (CSR6_SR | CSR6_ST);
}


/*--------------------------- int_enable_eth --------------------------------*/

void int_enable_eth (void) {
  /* Ethernet Interrupt Enable function. */
  NVIC_EnableIRQ(EthernetMAC_IRQn);
}


/*--------------------------- int_disable_eth -------------------------------*/

void int_disable_eth (void) {
  /* Ethernet Interrupt Disable function. */
  NVIC_DisableIRQ(EthernetMAC_IRQn);
}


/*--------------------------- send_frame ------------------------------------*/

void send_frame (OS_FRAME *frame) {
  /* Send frame to EMAC ethernet controller */
  U32 idx,len;
  U32 *sp,*dp;

  idx = TxBufIndex;
  sp  = (U32 *)&frame->data[0];
  dp  = (U32 *)Tx_Desc[idx].Buf1;

  /* Copy frame data to EMAC packet buffers. */
  for (len = (frame->length + 3) >> 2; len; len--) {
    *dp++ = *sp++;
  }
  Tx_Desc[idx].Ctrl = (Tx_Desc[idx].Ctrl & ~TDES1_TBS1) | frame->length;
  Tx_Desc[idx].Stat = RDES0_OWN;

  /* Start frame transmission. */
  MAC->CSR6 |= CSR6_ST;
  MAC->CSR1  = 1;
  if (++idx == NUM_TX_BUF) idx = 0;
  TxBufIndex = idx;
}


/*--------------------------- interrupt_ethernet ----------------------------*/

void EthernetMAC_IRQHandler (void) {
  /* EMAC Ethernet Controller Interrupt function. */
  OS_FRAME *frame;
  U32 i,j,idx,int_stat,frm_err,RxLen;
  U32 *sp,*dp;

  int_stat = MAC->CSR5;
  if (int_stat & CSR5_RI) {
    /* Packet received, check if packet is valid. */
    idx = RxBufIndex;
    /* Get frame length. */
    for (j = 0; j < NUM_RX_BUF; j++) {
      if (Rx_Desc[idx].Stat & RDES0_LS) {
        break;
      }
      if (++idx == NUM_RX_BUF) idx = 0;
    }

fetch:
    RxLen   = ((Rx_Desc[idx].Stat >> 16) & 0x3FFF) - 4;
    frm_err = Rx_Desc[idx].Stat & RDES0_ES;

    if (++idx == NUM_RX_BUF) idx = 0;

    if (RxLen > ETH_MTU || frm_err) {
      /* Packet error, ignore it and free buffer. */
      goto rel;
    }

    /* Flag 0x80000000 to skip sys_error() call when out of memory. */
    frame = alloc_mem (RxLen | 0x80000000);

    /* if 'alloc_mem()' has failed, ignore this packet. */
    if (frame != NULL) {
      /* Make sure that block is 4-byte aligned */
      dp = (U32 *)&frame->data[0];
      for (i = RxBufIndex; i != idx; RxLen -= ETH_RX_BUF_SIZE) {
        sp = (U32 *)(Rx_Desc[i].Buf1);
        j = RxLen;
        if (j > ETH_RX_BUF_SIZE) j = ETH_RX_BUF_SIZE;
        for (j = (j + 3) >> 2; j; j--) {
          *dp++ = *sp++;
        }
        if (++i == NUM_RX_BUF) i = 0;
      }
      put_in_queue (frame);
    }

    /* Release this frame from FEC IO buffer. */
rel:for (j = RxBufIndex; j != idx;  ) {
      Rx_Desc[j].Stat |= RDES0_OWN;
      if (++j == NUM_RX_BUF) j = 0;
    }
    RxBufIndex = idx;

    /* Check if a new packet received. */
    for (j = 0; j < ETH_RX_BUF_NUM; j++) {
      if (Rx_Desc[idx].Stat & RDES0_OWN) {
        break;
      }
      if (Rx_Desc[idx].Stat & RDES0_LS) {
        /* Yes, fetch it and release FEC IO buffer. */
        goto fetch;
      }
      if (++idx == NUM_RX_BUF) idx = 0;
    }
  }
  MAC->CSR5 = int_stat;
}


/*--------------------------- rx_desc_init ----------------------------------*/

static void rx_desc_init (void) {
  /* Initialize Receive Descriptor list. */
  U32 i;

  RxBufIndex = 0;
  for (i = 0; i < NUM_RX_BUF; i++) {
    Rx_Desc[i].Stat = RDES0_OWN;
    Rx_Desc[i].Ctrl = ETH_RX_BUF_SIZE;
    Rx_Desc[i].Buf1 = (U32)&rx_buf[i];
    Rx_Desc[i].Buf2 = 0;
  }
  Rx_Desc[NUM_RX_BUF-1].Ctrl |= RDES1_RER;

  /* Set EMAC Receive List base address. */
  MAC->CSR3 = (U32)&Rx_Desc[0];
}


/*--------------------------- tx_desc_init ----------------------------------*/

static void tx_desc_init (void) {
  /* Initialize Transmit Descriptor list. */
  U32 i;

  TxBufIndex = 0;
  for (i = 0; i < NUM_TX_BUF; i++) {
    Tx_Desc[i].Stat = 0;
    Tx_Desc[i].Ctrl = TDES1_LS | TDES1_FS;
    Tx_Desc[i].Buf1 = (U32)&tx_buf[i];
    Tx_Desc[i].Buf2 = 0;
  }
  Tx_Desc[NUM_TX_BUF-1].Ctrl |= TDES1_TER;

  /* Set EMAC Transmit List base address. */
  MAC->CSR4 = (U32)&Tx_Desc[0];
}


/*--------------------------- send_setup ------------------------------------*/

static void send_setup (void) {
  DMA_Desc desc;
  U8 *frm;
  U32 i;

  
  frm = (U8 *)alloc_mem (192 | 0x80000000);
  desc.Stat = TDES0_OWN;
  desc.Ctrl = TDES1_SET | TDES1_TER | TDES1_FT0 | 192;
  desc.Buf1 = (U32)frm;
  desc.Buf2 = 0;
  MAC->CSR4 = (U32)&desc;

  /* Clear Setup buffer. */
  for (i = 0; i < (192/4); i++) {
    ((U32 *)frm)[i] = 0;
  }

  /* Set own MAC Address. */
  i = 156;
  frm[i]   = own_hw_adr[0];
  frm[i+1] = own_hw_adr[1];
  frm[i+4] = own_hw_adr[2];
  frm[i+5] = own_hw_adr[3];
  frm[i+8] = own_hw_adr[4];
  frm[i+9] = own_hw_adr[5];

  /* Start Transmission */
  MAC->CSR6 |= CSR6_ST;

  /* Wait until TS Suspended. */
  while ((MAC->CSR5 & CSR5_TS) != (6 << 20)) {
    MAC->CSR1 = 1;
  }

  /* Stop Transmission */
  MAC->CSR6 &= ~CSR6_ST;
}


/*--------------------------- write_PHY -------------------------------------*/

static void write_PHY (U32 PhyReg, U16 Value) {
  /* Write a data 'Value' to PHY register 'PhyReg'. */

  /* enable MDO */
  MAC->CSR9 &= ~CSR9_MDEN;

  /* 32 consecutive ones on MDO to establish sync */
  output_MDIO (0xFFFFFFFF, 32);

  /* start code (01), write command (01) */
  output_MDIO (0x05, 4);

  /* write PHY address */
  output_MDIO (DP83848C_DEF_ADR >> 8, 5);

  /* write the PHY register to write */
  output_MDIO (PhyReg, 5);

  /* turnaround MDIO (1,0)*/
  output_MDIO (0x02, 2);

  /* write the data value */
  output_MDIO (Value, 16);

  /* turnaround MDO is tristated */
  turnaround_MDIO ();
}


/*--------------------------- read_PHY --------------------------------------*/

static U16 read_PHY (U32 PhyReg) {
  /* Read a PHY register 'PhyReg'. */
  U32 val;
    
  /* enable MDI, tristate MDO */
  MAC->CSR9 |= CSR9_MDEN;

  /* 32 consecutive ones on MDO to establish sync */
  output_MDIO (0xFFFFFFFF, 32);

  /* start code (01), read command (10) */
  output_MDIO (0x06, 4);

  /* write PHY address */
  output_MDIO (DP83848C_DEF_ADR >> 8, 5);

  /* write the PHY register to write */
  output_MDIO (PhyReg, 5);

  /* turnaround MDO is tristated */
  turnaround_MDIO ();

  /* read the data value */
  val = input_MDIO ();

  /* turnaround MDIO is tristated */
  turnaround_MDIO ();

  return (val);
}

/*--------------------------- output_MDIO -----------------------------------*/

#define delay()     __nop(); __nop();

static void output_MDIO (U32 val, U32 n) {
  /* Output a value to the MII PHY management interface. */
  U32 csr9 = MAC->CSR9;

  for (val <<= (32 - n); n; val <<= 1, n--) {
    if (val & 0x80000000) {
      csr9 |=  CSR9_MDO;
    }
    else {
      csr9 &= ~CSR9_MDO;
    }
    MAC->CSR9 = csr9;
    delay ();
    MAC->CSR9 = csr9 | CSR9_MDC;
    delay ();
    MAC->CSR9 = csr9;
  }
}

/*--------------------------- turnaround_MDIO -------------------------------*/

static void turnaround_MDIO (void) {
  /* Turnaround MDO is tristated. */
  U32 csr9 = MAC->CSR9 | CSR9_MDEN;
  
  MAC->CSR9 = csr9 | CSR9_MDC;
  delay ();
  MAC->CSR9 = csr9;
  delay ();
}

/*--------------------------- input_MDIO ------------------------------------*/

static U32 input_MDIO (void) {
  /* Input a value from the MII PHY management interface. */
  U32 i,csr9,val = 0;

  csr9 = MAC->CSR9;
  for (i = 0; i < 16; i++) {
    val <<= 1;
    MAC->CSR9 = csr9 | CSR9_MDC;
    delay ();
    MAC->CSR9 = csr9;
    if (MAC->CSR9 & CSR9_MDI) {
      val |= 1;
    }
  }
  return (val);
}


/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
