/*-----------------------------------------------------------------------------
 *      RL-ARM - TCPnet
 *-----------------------------------------------------------------------------
 *      Name:    ETH_XMC45xx.C
 *      Purpose: Driver for Infineon XMC4500 Ethernet Controller
 *      Rev.:    V4.71
 *-----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include <Net_Config.h>
#include <XMC4500.h>                    /* XMC4500 Definitions                */
#include "ETH_XMC45xx.h"

/* The following macro definitions may be used to select the speed
   of the physical link:

  _10MBIT_   - connect at 10 MBit only
  _100MBIT_  - connect at 100 MBit only

  By default an autonegotiation of the link speed is used. This may take 
  longer to connect, but it works for 10MBit and 100MBit physical links.      */

/* Net_Config.c */
extern U8 own_hw_adr[];

/* Local variables */
static U32 TxBufIndex;
static U32 RxBufIndex;

/* ETH local DMA Descriptors. */
static RX_Desc Rx_Desc[NUM_RX_BUF];
static TX_Desc Tx_Desc[NUM_TX_BUF];

/* ETH local DMA buffers. */
static U32 rx_buf[NUM_RX_BUF][ETH_RX_BUF_SIZE>>2];
static U32 tx_buf[NUM_TX_BUF][ETH_TX_BUF_SIZE>>2];

/*-----------------------------------------------------------------------------
 *      ETH Ethernet Driver Functions
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

/*--------------------------- init_ethernet ----------------------------------*/

void init_ethernet (void) {
  /* Initialize the ETH ethernet controller. */
  U32 regv,tout,id1,id2;

  /* Disable parity error and unaligned access trap. */
  SCU_PARITY->PETE = 0;
  PPB->CCR &= ~PPB_CCR_UNALIGN_TRP_Msk;

  /* Enable Clock for Ethernet MAC */
  SCU_CLK->CLKSET = SCU_CLK_CLKSET_ETH0CEN_Msk;

  /* Configure Ethernet Pins  */
  /* P2.0 [ETH_MDIO]  [HWI0]  */
  /* P2.2 [ETH_RXD0A] [Input] */
  /* P2.3 [ETH_RXD1A] [Input] */
  /* P2.7 [ETH_MDC]   [ALT1 ] */
  /* P2.8 [ETH_TXD0]  [ALT1 ] */
  /* P2.9 [ETH_TXD1]  [ALT1 ] */
  PORT2->IOCR0  &= ~(PORT2_IOCR0_PC0_Msk |
                     PORT2_IOCR0_PC2_Msk |
                     PORT2_IOCR0_PC3_Msk);

  PORT2->IOCR4  &= ~(PORT2_IOCR4_PC7_Msk);
  PORT2->IOCR4  |=  (0x11U << PORT2_IOCR4_PC7_Pos);
  
  PORT2->IOCR8  &= ~(PORT2_IOCR8_PC8_Msk |
                     PORT2_IOCR8_PC9_Msk);
  PORT2->IOCR8  |=  (0x11U << PORT2_IOCR8_PC8_Pos) |
                    (0x11U << PORT2_IOCR8_PC9_Pos) ;

  PORT2->PDR0   &= ~(PORT0_PDR0_PD2_Msk | PORT0_PDR0_PD3_Msk);
  PORT2->PDR1   &= ~(PORT0_PDR1_PD8_Msk | PORT0_PDR1_PD9_Msk);
  
  PORT2->HWSEL  &= ~(PORT0_HWSEL_HW0_Msk);
  PORT2->HWSEL  |=  (0x01U << PORT0_HWSEL_HW0_Pos);

  /* P5.3 - ETH_RXERD [Input] */
  /* P5.9 - ETH_TXEN  [ALT4 ] */
  PORT5->IOCR0  &= ~(PORT5_IOCR0_PC3_Msk);
  PORT5->IOCR8  &= ~(PORT5_IOCR8_PC9_Msk);
  PORT5->IOCR8  |=  (0x14U << PORT5_IOCR8_PC9_Pos);
  PORT5->PDR1   &= ~(PORT5_PDR1_PD9_Msk);
  
  /* P15.8 - ETH_CLK_RMIIC [Input] */
  /* P15.9 - ETH_CRS_DVC   [Input] */
  PORT15->IOCR8 &= ~(PORT15_IOCR8_PC8_Msk | PORT15_IOCR8_PC9_Msk);

  /* Configure ETH0 Port Control, enable RMII */
  ETH0_CON->CON = 0x0440CA00;

  /* Reset Ethernet MAC */
  SCU_RESET->PRSET2 = SCU_RESET_PRSET2_ETH0RS_Msk;
  for (tout = 0; tout < 0xFFFF; tout++);
  SCU_RESET->PRCLR2 = SCU_RESET_PRCLR2_ETH0RS_Msk;

  /* Reset Ethernet DMA */
  ETH0->BUS_MODE |= ETH_BUS_MODE_SWR_Msk;
  while (ETH0->BUS_MODE & ETH_BUS_MODE_SWR_Msk);

  /* Initialize MAC control registers. */
  ETH0->MAC_CONFIGURATION =  ETH_MAC_CONFIGURATION_CST_Msk |
                             ETH_MAC_CONFIGURATION_ACS_Msk | 0x00008000;

  /* Set MAC address */
  ETH0->MAC_ADDRESS0_HIGH = (U32)own_hw_adr[5] <<  8 | (U32)own_hw_adr[4];
  ETH0->MAC_ADDRESS0_LOW  = (U32)own_hw_adr[3] << 24 | (U32)own_hw_adr[2] << 16 |
                            (U32)own_hw_adr[1] <<  8 | (U32)own_hw_adr[0];

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
    ETH0->MAC_CONFIGURATION |= ETH_MAC_CONFIGURATION_DM_Msk;
  }
  else {
    /* Half duplex mode. */
    ETH0->MAC_CONFIGURATION |= ETH_MAC_CONFIGURATION_DO_Msk;
  }

  /* Configure 100MBit/10MBit mode. */
  if (regv & 0x0002) {
    /* 10MBit mode. */
  }
  else {
    /* 100MBit mode. */
    ETH0->MAC_CONFIGURATION |= ETH_MAC_CONFIGURATION_FES_Msk;
  }

  /* Initialize Tx and Rx DMA Descriptors */
  rx_descr_init ();
  tx_descr_init ();

  /* Receive Broadcast, Multicast and Perfect Match Packets */
  ETH0->MAC_FRAME_FILTER = ETH_MAC_FRAME_FILTER_PM_Msk |
                           ETH_MAC_FRAME_FILTER_HPF_Msk;

  /* Enable ETH interrupts. */
  ETH0->INTERRUPT_ENABLE = ETH_INTERRUPT_ENABLE_RIE_Msk |
                           ETH_INTERRUPT_ENABLE_TIE_Msk |
                           ETH_INTERRUPT_ENABLE_NIE_Msk;

  /* Reset all interrupts */
  ETH0->STATUS           = ETH_INTERRUPT_ENABLE_RIE_Msk |
                           ETH_INTERRUPT_ENABLE_TIE_Msk |
                           ETH_INTERRUPT_ENABLE_NIE_Msk;

  /* Start MAC receive descriptor ring pooling */
  ETH0->RECEIVE_POLL_DEMAND  = 0;
  ETH0->TRANSMIT_POLL_DEMAND = 0;

  /* Enable receive and transmit mode of MAC Ethernet core. */
  ETH0->MAC_CONFIGURATION |= (ETH_MAC_CONFIGURATION_RE_Msk |
                              ETH_MAC_CONFIGURATION_TE_Msk);
  ETH0->OPERATION_MODE    |= (ETH_OPERATION_MODE_SR_Msk |
                              ETH_OPERATION_MODE_ST_Msk);
}

/*--------------------------- int_enable_eth ---------------------------------*/

void int_enable_eth (void) {
  /* Ethernet Interrupt Enable function. */
  NVIC_EnableIRQ (ETH0_0_IRQn);
}


/*--------------------------- int_disable_eth --------------------------------*/

void int_disable_eth (void) {
  /* Ethernet Interrupt Disable function. */
  NVIC_DisableIRQ (ETH0_0_IRQn);
}


/*--------------------------- send_frame -------------------------------------*/

void send_frame (OS_FRAME *frame) {
  /* Send frame to ETH ethernet controller */
  U32 idx,len;
  U32 *sp,*dp;

  idx = TxBufIndex;
  /* Wait until previous frame transmitted */
  while (Tx_Desc[idx].CtrlStat & TDES0_OWN);

  sp  = (U32 *)&frame->data[0];
  dp  = (U32 *)Tx_Desc[idx].Addr;

  /* Copy frame data to ETH packet buffer. */
  for (len = (frame->length + 3) >> 2; len; len--) {
    *dp++ = *sp++;
  }
  Tx_Desc[idx].Size      = frame->length;
  Tx_Desc[idx].CtrlStat |= TDES0_OWN;
  if (++idx == NUM_TX_BUF) idx = 0;
  TxBufIndex = idx;

  /* Start frame transmission. */
  ETH0->TRANSMIT_POLL_DEMAND = 0;
}


/*--------------------------- interrupt_ethernet -----------------------------*/

void ETH0_0_IRQHandler (void) {
  OS_FRAME *frame;
  U32 i,j,idx,int_stat,frm_err,RxLen;
  U32 *sp,*dp;

  int_stat = ETH0->STATUS;
  if (int_stat & ETH_STATUS_RI_Msk) {
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
    RxLen   = (Rx_Desc[idx].Stat >> 16) & 0x3FFF;
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
        sp = (U32 *)(Rx_Desc[i].Addr);
        j = RxLen;
        if (j > ETH_RX_BUF_SIZE) j = ETH_RX_BUF_SIZE;
        for (j = (j + 3) >> 2; j; j--) {
          *dp++ = *sp++;
        }
        if (++i == NUM_RX_BUF) i = 0;
      }
      put_in_queue (frame);
    }

    /* Release this frame from ETH IO buffer. */
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
        /* Yes, fetch it and release ETH IO buffer. */
        goto fetch;
      }
      if (++idx == NUM_RX_BUF) idx = 0;
    }
  }
  ETH0->STATUS = int_stat;
}


/*--------------------------- rx_descr_init ----------------------------------*/

static void rx_descr_init (void) {
  /* Initialize Receive DMA Descriptor array. */
  U32 i;

  RxBufIndex = 0;
  for (i = 0; i < NUM_RX_BUF; i++) {
    Rx_Desc[i].Stat = RDES0_OWN;
    Rx_Desc[i].Ctrl = RDES1_RCH | ETH_RX_BUF_SIZE;
    Rx_Desc[i].Addr = (U32)&rx_buf[i];
    Rx_Desc[i].Next = (U32)&Rx_Desc[i+1];
  }
  Rx_Desc[NUM_RX_BUF-1].Next = (U32)&Rx_Desc[0];

  /* Set ETH Receive List base address. */
  ETH0->RECEIVE_DESCRIPTOR_LIST_ADDRESS = (U32)&Rx_Desc[0];
}


/*--------------------------- tx_descr_init ----------------------------------*/

static void tx_descr_init (void) {
  /* Initialize Transmit DMA Descriptor array. */
  U32 i;

  TxBufIndex = 0;
  for (i = 0; i < NUM_TX_BUF; i++) {
    Tx_Desc[i].CtrlStat = TDES0_TCH | TDES0_LS | TDES0_FS;
    Tx_Desc[i].Addr = (U32)&tx_buf[i];
    Tx_Desc[i].Next = (U32)&Tx_Desc[i+1];
  }
  Tx_Desc[NUM_TX_BUF-1].Next = (U32)&Tx_Desc[0];

  /* Set ETH Transmit List base address. */
  ETH0->TRANSMIT_DESCRIPTOR_LIST_ADDRESS = (U32)&Tx_Desc[0];
}


/*--------------------------- write_PHY --------------------------------------*/

static void write_PHY (U32 PhyReg, U16 Value) {
  /* Write a data 'Value' to PHY register 'PhyReg'. */
  U32 tout;

  /* Send MDIO write command */
  ETH0->GMII_DATA    = Value;
  ETH0->GMII_ADDRESS = DP83848C_DEF_ADR << 11  | PhyReg << 6 | 0x01 << 2 |
                       ETH_GMII_ADDRESS_MW_Msk | ETH_GMII_ADDRESS_MB_Msk;

  /* Wait until operation completed */
  for (tout = 0; tout < GMII_WR_TOUT; tout++) {
    if (!(ETH0->GMII_ADDRESS & ETH_GMII_ADDRESS_MB_Msk)) {
      break;
    }
  }
}


/*--------------------------- read_PHY ---------------------------------------*/

static U16 read_PHY (U32 PhyReg) {
  /* Read a PHY register 'PhyReg'. */
  U32 tout;

  /* Send MDIO read command */
  ETH0->GMII_ADDRESS = DP83848C_DEF_ADR << 11 | PhyReg << 6 | 0x01 << 2 |
                                                ETH_GMII_ADDRESS_MB_Msk;

  /* Wait until operation completed */
  for (tout = 0; tout < GMII_RD_TOUT; tout++) {
    if (!(ETH0->GMII_ADDRESS & ETH_GMII_ADDRESS_MB_Msk)) {
      break;
    }
  }
  return (ETH0->GMII_DATA);
}

/*-----------------------------------------------------------------------------
 * End of file
 *----------------------------------------------------------------------------*/
