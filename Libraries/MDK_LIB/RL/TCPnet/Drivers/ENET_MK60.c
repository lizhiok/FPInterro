/*-----------------------------------------------------------------------------
 *      RL-ARM - TCPnet
 *-----------------------------------------------------------------------------
 *      Name:    ENET_MK6x.C
 *      Purpose: Driver for Freescale MK60 Ethernet Controller
 *      Rev.:    V4.70
 *-----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include <Net_Config.h>
#include <MK60F12.H>
#include "ENET_MKxx.h"

/* The following macro definitions may be used to select the speed
   of the physical link:

  _10MBIT_   - connect at 10 MBit only
  _100MBIT_  - connect at 100 MBit only

  By default an autonegotiation of the link speed is used. This may take 
  longer to connect, but it works for 10MBit and 100MBit physical links.      */

/* Net_Config.c */
extern U8 own_hw_adr[];

/* Local variables */
static U8 TxBufIndex;
static U8 RxBufIndex;

/* ENET local DMA Descriptors. */
static __align(16) RX_Desc Rx_Desc[NUM_RX_BUF];
static __align(16) TX_Desc Tx_Desc[NUM_TX_BUF];

/* ENET local DMA buffers. */
static U32 rx_buf[NUM_RX_BUF][ETH_BUF_SIZE>>2];
static U32 tx_buf[NUM_TX_BUF][ETH_BUF_SIZE>>2];

#define IDX(x) (x/2)
#define SWE(n) ((((n) & 0x00FF) << 8) | (((n) & 0xFF00) >> 8))

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

/*--------------------------- init_ethernet ----------------------------------*/
void init_ethernet (void) {
  /* Initialize the ETH ethernet controller. */
  U32 regv,tout,id1,id2,ctrl;
  
  OSC0->CR   |= OSC_CR_ERCLKEN_MASK;    /* Enable external reference clock    */
  SIM->SCGC2 |= SIM_SCGC2_ENET_MASK;    /* Enable ENET module gate clocking   */
  SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK |  /* Enable Port A module gate clocking */
                SIM_SCGC5_PORTB_MASK ;  /* Enable Port B module gate clocking */

  /* Configure Ethernet Pins  */
  PORTA->PCR[5]  &= ~(PORT_PCR_MUX_MASK | PORT_PCR_PS_MASK);
  PORTA->PCR[5]  |=  PORT_PCR_MUX(4);   /* Pull-down on RX_ER is enabled      */

  PORTA->PCR[12]  = (PORTA->PCR[12] & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(4);
  PORTA->PCR[13]  = (PORTA->PCR[13] & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(4);
  PORTA->PCR[14]  = (PORTA->PCR[14] & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(4);
  PORTA->PCR[15]  = (PORTA->PCR[15] & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(4);
  PORTA->PCR[16]  = (PORTA->PCR[16] & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(4);
  PORTA->PCR[17]  = (PORTA->PCR[17] & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(4);

  PORTB->PCR[0]   = (PORTB->PCR[0]  & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(4)| PORT_PCR_ODE_MASK;
  PORTB->PCR[1]   = (PORTB->PCR[1]  & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(4);

  /* Reset Ethernet MAC */
  ENET->ECR =  ENET_ECR_RESET_MASK;
  while (ENET->ECR & ENET_ECR_RESET_MASK);

  /* Set MDC clock frequency @ 50MHz MAC clock frequency */
  ENET->MSCR = ENET_MSCR_MII_SPEED(0x13);

  /* Set receive control */
  ENET->RCR = ENET_RCR_MAX_FL (0x5EE) |
              ENET_RCR_RMII_MODE_MASK | /* MAC Configured for RMII operation  */
              ENET_RCR_MII_MODE_MASK  ; /* This bit must always be set        */

  /* Set transmit control */
  ENET->TCR = ENET_TCR_ADDINS_MASK |    /* MAC overwrites the source MAC */
              ENET_TCR_ADDSEL(0)   ;    /* MAC address is in PADDR 1 and 2 */

  ENET->MRBR = ETH_BUF_SIZE;

  /* Set thresholds */
  ENET->RAEM = 4;
  ENET->RAFL = 4;
  ENET->TAEM = 4;
  ENET->TAFL = 8;
  ENET->RSFL = 0;                       /* Store and forward on the RX FIFO   */
  ENET->FTRL = 0x7FF;                   /* Frame Truncation Length            */

  /* Read PHY ID */
  id1 = read_PHY (PHY_REG_ID1);
  id2 = read_PHY (PHY_REG_ID2);

  /* Check if this is a KSZ8041NL PHY. */
  if (((id1 << 16) | (id2 & 0xFFF0)) == PHY_ID_KSZ8041) {
    /* Put the PHY in reset mode */
    write_PHY (PHY_REG_BCTRL, 0x8000);

    /* Wait for hardware reset to end. */
    for (tout = 0; tout < 0x10000; tout++) {
      regv = read_PHY (PHY_REG_BCTRL);
      if (!(regv & 0x8800)) {
        /* Reset complete, device not Power Down. */
        break;
      }
    }
    /* Configure the PHY device */
#if defined (_10MBIT_)
    /* Connect at 10MBit */
    write_PHY (PHY_REG_BCTRL, PHY_FULLD_10M);
#elif defined (_100MBIT_)
    /* Connect at 100MBit */
    write_PHY (PHY_REG_BCTRL, PHY_FULLD_100M);
#else
    /* Use autonegotiation about the link speed. */
    write_PHY (PHY_REG_BCTRL, PHY_AUTO_NEG);
    /* Wait to complete Auto_Negotiation. */
    for (tout = 0; tout < 0x10000; tout++) {
      regv = read_PHY (PHY_REG_BSTAT);
      if (regv & 0x0020) {        
        break;                          /* Autonegotiation Complete           */
      }
    }
#endif
    /* Check the link status. */
    for (tout = 0; tout < 0x10000; tout++) {
      regv = read_PHY (PHY_REG_BSTAT);
      if (regv & 0x0004) {        
        break;                          /* Link is on                         */
      }
    }

    /* Check Operation Mode Indication in PHY Control register */ 
    regv = read_PHY (PHY_REG_PC2);
    /* Configure Full/Half Duplex mode. */
    switch ((regv & 0x001C) >> 2) {
      case 1:  ctrl = PHY_CON_10M  | PHY_CON_HD; break;
      case 2:  ctrl = PHY_CON_100M | PHY_CON_HD; break;
      case 5:  ctrl = PHY_CON_10M  | PHY_CON_FD; break;
      case 6:  ctrl = PHY_CON_100M | PHY_CON_FD; break;
      default: ctrl = 0;                         break;
    }
    if (ctrl & PHY_CON_FD) {
      ENET->TCR |= ENET_TCR_FDEN_MASK;  /* Enable Full duplex                 */
    }

    /* Configure 100MBit/10MBit mode. */
    if (ctrl & PHY_CON_100M) {      
      ENET->RCR &= ~ENET_RCR_RMII_10T_MASK; /* 100MBit mode.                  */
    }
  }

  /* MAC address filtering, accept multicast packets. */

  /* Set the Ethernet MAC Address registers */
  ENET->PAUR = ENET_PAUR_TYPE (0x8808) |
               ENET_PAUR_PADDR2(((U32)own_hw_adr[4] << 8) | (U32)own_hw_adr[5]);
  ENET->PALR = ((U32)own_hw_adr[0] << 24) | (U32)own_hw_adr[1] << 16 |
               ((U32)own_hw_adr[2] <<  8) | (U32)own_hw_adr[3];

  ENET->MIBC |= ENET_MIBC_MIB_CLEAR_MASK;

  /* Initialize Tx and Rx DMA Descriptors */
  rx_descr_init ();
  tx_descr_init ();

  /* Reset all interrupts */
  ENET->EIR = 0x7FFF8000;
  
  /* Enable receive interrupt */
  ENET->EIMR = ENET_EIMR_RXF_MASK;

  /* Enable Ethernet, reception and transmission are possible */
  ENET->ECR  |= ENET_ECR_EN1588_MASK | ENET_ECR_ETHEREN_MASK;

  /* Start MAC receive descriptor ring pooling */
  ENET->RDAR = ENET_RDAR_RDAR_MASK;
}

/*--------------------------- int_enable_eth ---------------------------------*/

void int_enable_eth (void) {
  /* Ethernet Interrupt Enable function. */
  NVIC_EnableIRQ (ENET_Receive_IRQn);
}


/*--------------------------- int_disable_eth --------------------------------*/

void int_disable_eth (void) {
  /* Ethernet Interrupt Disable function. */
  NVIC_DisableIRQ (ENET_Receive_IRQn);
}


/*--------------------------- send_frame -------------------------------------*/

void send_frame (OS_FRAME *frame) {
  /* Send frame to ETH ethernet controller */
  U32 *sp,*dp;
  U32 i,j, adr;

  j = TxBufIndex;
  /* Wait until previous packet transmitted. */
  while (Tx_Desc[j].TBD[IDX(0)] & SWE(DESC_TX_R));

  adr  = SWE(Tx_Desc[j].TBD[IDX(4)]) << 16;
  adr |= SWE(Tx_Desc[j].TBD[IDX(6)]);

  dp = (U32 *)(adr & ~3);
  sp = (U32 *)&frame->data[0];

  /* Copy frame data to ETH IO buffer. */
  for (i = (frame->length + 3) >> 2; i; i--) {
    *dp++ = *sp++;
  }
  Tx_Desc[j].TBD[IDX(2)]  = SWE(frame->length);
  Tx_Desc[j].TBD[IDX(0)] |= SWE(DESC_TX_R | DESC_TX_L | DESC_TX_TC);
  Tx_Desc[j].TBD[IDX(0x10)] = 0;
  if (++j == NUM_TX_BUF) j = 0;
  TxBufIndex = j;
  /* Start frame transmission. */
  ENET->TDAR = ENET_TDAR_TDAR_MASK;
}


/*--------------------------- interrupt_ethernet -----------------------------*/

void ENET_Receive_IRQHandler (void) {
  OS_FRAME *frame;
  U32 i, adr, RxLen;
  U32 *sp,*dp;

  i = RxBufIndex;
  do {
    if (Rx_Desc[i].RBD[IDX(0)] & SWE(DESC_RX_TR)) {
      /* Frame is truncated */
      goto rel;
    }
    RxLen = SWE(Rx_Desc[i].RBD[IDX(2)]);
    
    if (Rx_Desc[i].RBD[IDX(0)] & SWE(DESC_RX_L)) {
      /* Last in a frame, length includes CRC bytes */
      RxLen -= 4;
      if (Rx_Desc[i].RBD[IDX(0)] & SWE(DESC_RX_OV)) {
        /* Receive FIFO overrun */
        goto rel;
      }
      if (Rx_Desc[i].RBD[IDX(0)] & SWE(DESC_RX_NO)) {
        /* Received non-octet aligned frame */
        goto rel;
      }
      if (Rx_Desc[i].RBD[IDX(0)] & SWE(DESC_RX_CR)) {
        /* CRC or frame error */
        goto rel;
      }
    }

    if (RxLen > ETH_MTU) {
      /* Packet too big, ignore it and free buffer. */
      goto rel;
    }
    /* Flag 0x80000000 to skip sys_error() call when out of memory. */
    frame = alloc_mem (RxLen | 0x80000000);
    /* if 'alloc_mem()' has failed, ignore this packet. */
    if (frame != NULL) {
      adr  = SWE(Rx_Desc[i].RBD[IDX(4)]) << 16;
      adr |= SWE(Rx_Desc[i].RBD[IDX(6)]);
      sp = (U32 *)(adr & ~3);
      dp = (U32 *)&frame->data[0];
      for (RxLen = (RxLen + 3) >> 2; RxLen; RxLen--) {
        *dp++ = *sp++;
      }
      put_in_queue (frame);
    }
    /* Release this frame from ETH IO buffer. */
rel:Rx_Desc[i].RBD[IDX(0)] |= SWE(DESC_RX_E);
    Rx_Desc[i].RBD[IDX(0x10)] = 0;

    if (++i == NUM_RX_BUF) i = 0;
    RxBufIndex = i;
  }
  while (!(Rx_Desc[i].RBD[IDX(0)] & SWE(DESC_RX_E)));
  RxBufIndex = i;

  /* Clear pending interrupt bits */
  ENET->EIR = ENET_EIR_RXB_MASK | ENET_EIR_RXF_MASK;
  
  /* Start MAC receive descriptor ring pooling */
  ENET->RDAR = ENET_RDAR_RDAR_MASK;
}


/*--------------------------- rx_descr_init ----------------------------------*/

static void rx_descr_init (void) {
  /* Initialize Receive DMA Descriptor array. */
  U32 i,next;

  RxBufIndex = 0;
  for (i = 0, next = 0; i < NUM_RX_BUF; i++) {
    if (++next == NUM_RX_BUF) next = 0;
    Rx_Desc[i].RBD[IDX(0x00)] = SWE(DESC_RX_E);
    Rx_Desc[i].RBD[IDX(0x04)] = SWE((U32)&rx_buf[i] >> 16);
    Rx_Desc[i].RBD[IDX(0x06)] = SWE((U32)&rx_buf[i] & 0xFFFF);
    Rx_Desc[i].RBD[IDX(0x08)] = SWE(DESC_RX_INT);
    Rx_Desc[i].RBD[IDX(0x10)] = 0;
  }
  Rx_Desc[i-1].RBD[IDX(0)] |= SWE(DESC_RX_W);
  ENET->RDSR = (U32)&Rx_Desc[0];
}



/*--------------------------- tx_descr_init ----------------------------------*/

static void tx_descr_init (void) {
  /* Initialize Transmit DMA Descriptor array. */
  U32 i,next;

  TxBufIndex = 0;
  for (i = 0, next = 0; i < NUM_TX_BUF; i++) {
    if (++next == NUM_TX_BUF) next = 0;
    Tx_Desc[i].TBD[IDX(0x00)] = SWE(DESC_TX_L);
    Tx_Desc[i].TBD[IDX(0x04)] = SWE((U32)&tx_buf[i] >> 16);
    Tx_Desc[i].TBD[IDX(0x06)] = SWE((U32)&tx_buf[i] & 0xFFFF);
    Tx_Desc[i].TBD[IDX(0x10)] = 0;
  }
  Tx_Desc[i-1].TBD[IDX(0x00)] |= SWE(DESC_TX_W);
  ENET->TDSR = (U32)&Tx_Desc[0];
}


/*--------------------------- write_PHY --------------------------------------*/

static void write_PHY (U32 PhyReg, U16 Value) {
  /* Write a data 'Value' to PHY register 'PhyReg'. */
  U32 tout;

  /* Clear MII Interrupt */
  ENET->EIR = ENET_EIR_MII_MASK;

  /* Send MDIO write command */
  ENET->MMFR =  ENET_MMFR_ST (1)            |
                ENET_MMFR_OP (1)            |
                ENET_MMFR_PA (PHY_DEF_ADDR) |
                ENET_MMFR_RA (PhyReg)       |
                ENET_MMFR_TA (2)            |
                ENET_MMFR_DATA (Value)      ;

  /* Wait until operation completed */
  tout = 0;
  for (tout = 0; tout < MII_WR_TOUT; tout++) {
    if (ENET->EIR & ENET_EIR_MII_MASK) {
      break;
    }
  }
}


/*--------------------------- read_PHY ---------------------------------------*/

static U16 read_PHY (U32 PhyReg) {
  /* Read a PHY register 'PhyReg'. */
  U32 tout;

  /* Clear MII Interrupt */
  ENET->EIR = ENET_EIR_MII_MASK;

  /* Send MDIO read command */
  ENET->MMFR =  ENET_MMFR_ST (1)            |
                ENET_MMFR_OP (2)            |
                ENET_MMFR_PA (PHY_DEF_ADDR) |
                ENET_MMFR_RA (PhyReg)       |
                ENET_MMFR_TA (2)            ;

  /* Wait until operation completed */
  tout = 0;
  for (tout = 0; tout < MII_RD_TOUT; tout++) {
    if (ENET->EIR & ENET_EIR_MII_MASK) {
      break;
    }
  }
  return (ENET->MMFR & ENET_MMFR_DATA_MASK);
}

/*-----------------------------------------------------------------------------
 * End of file
 *----------------------------------------------------------------------------*/
