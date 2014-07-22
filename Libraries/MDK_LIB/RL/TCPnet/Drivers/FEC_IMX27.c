/*----------------------------------------------------------------------------
 *      RL-ARM - TCPnet
 *----------------------------------------------------------------------------
 *      Name:    FEC_IMX27.C
 *      Purpose: Driver for Freescale i.MX27 Fast Ethernet Controller
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <Net_Config.h>
#include "FEC_IMX27.h"
#include <iMX27.h>                      /* i.MX27 definitions                */

/* The following macro definitions may be used to select the speed
   of the physical link:

  _10MBIT_   - connect at 10 MBit only
  _100MBIT_  - connect at 100 MBit only

  By default an autonegotiation of the link speed is used. This may take 
  longer to connect, but it works for 10MBit and 100MBit physical links.     */

/* Net_Config.c */
extern U8 own_hw_adr[];

/* Local variables */

/* FEC local DMA Descriptors. */
/*static*/ U32 RxBufIndex;
/*static*/ U32 TxBufIndex;
/*static*/ __align(16) RX_Desc Rx_Desc[NUM_RX_BUF];
/*static*/ __align(16) TX_Desc Tx_Desc[NUM_TX_BUF];

/* FEC local DMA buffers must be 16-byte aligned. */
static __align(16) U32 rx_buf[NUM_RX_BUF][ETH_RX_BUF_SIZE>>2];
static __align(16) U32 tx_buf[NUM_TX_BUF][ETH_TX_BUF_SIZE>>2];

/*----------------------------------------------------------------------------
 *      FEC Ethernet Driver Functions
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
static void init_rbd (void);
static void init_tbd (void);
static void write_PHY (U32 PhyReg, U16 Value);
static U16  read_PHY (U32 PhyReg);

/*--------------------------- init_ethernet ---------------------------------*/

void init_ethernet (void) {
  /* Initialize the FEC ethernet controller. */
  U32 regv,tout,id1,id2;

  /* Initialize GPIO pins for FEC. */
  /* Pullup enable */
  GPIO_PTD_PUEN    &= ~0x0001FFFF;
  GPIO_PTD_PUEN    |=  0x00000200;
  GPIO_PTF_PUEN    &= ~0x00800000;

  /* Data direction */
  GPIO_PTD_DDIR    &= ~0x0001FFFF;
  GPIO_PTD_DDIR    |=  0x0001020F;
  GPIO_PTF_DDIR    |=  0x00800000;

  /* Primary / Alternate function */
  GPIO_PTD_GPR     &= ~0x0001FFFF;
  GPIO_PTD_GPR     |=  0x00000100;
  GPIO_PTF_GPR     &= ~0x00800000;

  /* Use as GPIO ? */
  GPIO_PTD_GIUS    |=  0x0001FFFF;
  GPIO_PTD_GIUS    &= ~0x00000100;
  GPIO_PTF_GIUS    |=  0x00800000;

  /* Input / Output configuration */
  GPIO_PTD_OCR1     =  0x00000000;
  GPIO_PTD_OCR2    &= ~0x00000003;
  GPIO_PTF_OCR2    &= ~0x0000C000;
  GPIO_PTD_ICONFA1  =  0x00000000;
  GPIO_PTD_ICONFA2 &= ~0x00000003;
  GPIO_PTD_ICONFB1  =  0x00000000;
  GPIO_PTD_ICONFB2 &= ~0x00000003;
  GPIO_PTF_ICONFA2 &= ~0x0000C000;
  GPIO_PTF_ICONFB2 &= ~0x0000C000;

  /* Enable the clock for the FEC. */
  PLLCLK_PCCR0 |= (1 << 26);

  /* Reset all FEC internal modules. */
  FEC_ECR = ECR_RESET;

  /* A short delay after reset. */
  for (tout = 200; tout; tout--);

  /* Reset all interrupts */
  FEC_EIMR = 0x00000000;
  FEC_EIR  = 0xFFFFFFFF;

  /* Initialize Rx and Tx control registers. */
  FEC_RCR  = (ETH_MAX_FLEN << 16) | RCR_FCE | RCR_MII_MODE;
  FEC_TCR  = TCR_HBC;

  /* Set the Ethernet MAC Address registers */
  FEC_PAUR = ((U32)own_hw_adr[4] << 24) | ((U32)own_hw_adr[5] << 16) | 0x8808;
  FEC_PALR = ((U32)own_hw_adr[0] << 24) | ((U32)own_hw_adr[1] << 16) |
             ((U32)own_hw_adr[2] << 8)  | ((U32)own_hw_adr[3] <<  0);

  /* Group Address filter, receive all Multicast packets. */
  FEC_GAUR = 0xFFFFFFFF;
  FEC_GALR = 0xFFFFFFFF;

  /* Disable Individual Address Hash filter. */
  FEC_IAUR = 0x00000000;
  FEC_IALR = 0x00000000;

  /* Set the Receive buffer size. */
  FEC_EMRBR = ETH_RX_BUF_SIZE;

  /* Initialize Receive and Transmit Buffer Descriptors */
  init_rbd ();
  init_tbd ();

  /* Set the MII clock to 2.5MHz from 133MHz. */
  FEC_MSCR = 27 << 1;

  /* Put the KSZ8001 in reset mode */
  write_PHY (PHY_REG_BCR, 0x8000);

  /* Wait for hardware reset to end. */
  for (tout = 0; tout < 0x100000; tout++) {
    regv = read_PHY (PHY_REG_BCR);
    if (!(regv & 0x8800)) {
      /* Reset complete */
      break;
    }
  }

  /* Check if this is a KSZ8001 PHY. */
  id1 = read_PHY (PHY_REG_IDR1);
  id2 = read_PHY (PHY_REG_IDR2);

  if (((id1 << 16) | (id2 & 0xFFF0)) == KSZ8001_ID) {
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
    regv = read_PHY (PHY_REG_100TPCR);
    if (regv & 0x001C) {
      /* Link is on. */
      break;
    }
  }

  /* Configure Full/Half Duplex mode. */
  if (regv & 0x0010) {
    /* Full duplex is enabled. */
    FEC_TCR |= TCR_FDEN;
  }
  else {
    /* Half duplex mode. */
    FEC_RCR |= RCR_DRT;
  }

  /* Enable FEC Receive interrupts. */
  FEC_EIMR = INT_RXF;

  /* Clear all interrupt event flags */
  FEC_EIR  = 0xFFFFFFFF;

  /* Enable the FEC. */
  FEC_ECR  = ECR_ETHER_EN;
  FEC_RDAR = 1 << 24;
}


/*--------------------------- int_enable_eth --------------------------------*/

void int_enable_eth (void) {
  /* Ethernet Interrupt Enable function. */
  AITC_INTENNUM = 18 + 32;
}


/*--------------------------- int_disable_eth -------------------------------*/

void int_disable_eth (void) {
  /* Ethernet Interrupt Disable function. */
  AITC_INTDISNUM = 18 + 32;
}


/*--------------------------- send_frame ------------------------------------*/

void send_frame (OS_FRAME *frame) {
  /* Send frame to FEC ethernet controller */
  U32 idx,len;
  U32 *sp,*dp;

  idx = TxBufIndex;
  sp  = (U32 *)&frame->data[0];
  dp  = (U32 *)Tx_Desc[idx].Buf;

  /* Copy frame data to FEC packet buffer. */
  for (len = (frame->length + 3) >> 2; len; len--) {
    *dp++ = *sp++;
  }
  Tx_Desc[idx].Len   = frame->length;
  Tx_Desc[idx].Ctrl |= TBD_READY | TBD_LAST | TBD_TC;

  if (++idx == NUM_TX_BUF) idx = 0;
  TxBufIndex = idx;

  /* Start frame transmission. */
  FEC_TDAR = 1 << 24;
}


/*--------------------------- interrupt_ethernet ----------------------------*/

void interrupt_ethernet (void) __irq {
  /* FEC Ethernet Controller Interrupt function. */
  OS_FRAME *frame;
  U32 i,j,int_stat,idx,frm_err,RxLen;
  U32 *sp,*dp;

  int_stat = (FEC_EIR & FEC_EIMR);
  if (int_stat & INT_RXF) {
    /* Packet received, check if packet is valid. */
    idx = RxBufIndex;
    /* Get frame length. */
    for (j = 0; j < NUM_RX_BUF; j++) {
      if (Rx_Desc[idx].Stat & RBD_LAST) {
        break;
      }
      if (++idx == NUM_RX_BUF) idx = 0;
    }

fetch:
    RxLen   = Rx_Desc[idx].Len - 4;
    frm_err = Rx_Desc[idx].Stat & RBD_ERR;

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
        sp = (U32 *)(Rx_Desc[i].Buf);
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
      Rx_Desc[j].Stat |= RBD_EMPTY;
      if (++j == NUM_RX_BUF) j = 0;
    }
    RxBufIndex = idx;

    /* Check if a new packet received. */
    for (j = 0; j < ETH_RX_BUF_NUM; j++) {
      if (Rx_Desc[idx].Stat & RBD_EMPTY) {
        break;
      }
      if (Rx_Desc[idx].Stat & RBD_LAST) {
        /* Yes, fetch it and release FEC IO buffer. */
        goto fetch;
      }
      if (++idx == NUM_RX_BUF) idx = 0;
    }
  }
  FEC_RDAR = 1 << 24;
  FEC_EIR  = int_stat;
}


/*--------------------------- init_rbd --------------------------------------*/

static void init_rbd (void) {
  /* Initialize Receive Descriptor and Status array. */
  U32 i;

  for (i = 0; i < NUM_RX_BUF; i++) {
    Rx_Desc[i].Stat = RBD_EMPTY;
    Rx_Desc[i].Len  = 0;
    Rx_Desc[i].Buf  = (U32)&rx_buf[i];
  }

  /* Set the WRAP bit at the end of the descriptor list. */
  Rx_Desc[i-1].Stat |= RBD_WRAP;

  /* Receive BD Ring Start Address. */
  FEC_ERDSR  = (U32)&Rx_Desc;
  RxBufIndex = 0;
}


/*--------------------------- init_tbd --------------------------------------*/

static void init_tbd (void) {
  /* Initialize Transmit Descriptor and Status array. */
  U32 i;

  for (i = 0; i < NUM_TX_BUF; i++) {
    Tx_Desc[i].Ctrl = 0;
    Tx_Desc[i].Len  = 0;
    Tx_Desc[i].Buf  = (U32)&tx_buf[i];
  }
  /* Set the WRAP bit at the end of the descriptor list. */
  Tx_Desc[i-1].Ctrl |= TBD_WRAP;

  /* Transmit BD Ring Start Address. */
  FEC_ETDSR  = (U32)&Tx_Desc;
  TxBufIndex = 0;
}


/*--------------------------- write_PHY -------------------------------------*/

static void write_PHY (U32 PhyReg, U16 Value) {
  /* Write a data 'Value' to PHY register 'PhyReg'. */
  U32 tout;

  FEC_MMFR = MMFR_MII_ST | MMFR_MII_WR | MMFR_MII_TA |
             (KSZ8001_DEF_ADR << MMFR_PA_SHIFT)      |
             (PhyReg << MMFR_RA_SHIFT) | Value;

  /* Wait util operation completed */
  tout = 0;
  for (tout = 0; tout < MII_WR_TOUT; tout++) {
    if ((FEC_EIR & INT_MII) != 0) {
      break;
    }
  }
  /* Clear MII interrupt flag. */
  FEC_EIR = INT_MII;
}


/*--------------------------- read_PHY --------------------------------------*/

static U16 read_PHY (U32 PhyReg) {
  /* Read a PHY register 'PhyReg'. */
  U32 tout;

  FEC_MMFR = MMFR_MII_ST | MMFR_MII_RD | MMFR_MII_TA |
             (KSZ8001_DEF_ADR << MMFR_PA_SHIFT)      |
             (PhyReg << MMFR_RA_SHIFT);

  /* Wait until operation completed */
  tout = 0;
  for (tout = 0; tout < MII_RD_TOUT; tout++) {
    if ((FEC_EIR & INT_MII) != 0) {
      break;
    }
  }
  /* Clear MII interrupt flag. */
  FEC_EIR = INT_MII;
  return ((U16)FEC_MMFR);
}

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
