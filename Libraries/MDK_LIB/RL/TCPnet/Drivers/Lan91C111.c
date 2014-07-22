/*----------------------------------------------------------------------------
 *      RL-ARM - TCPnet
 *----------------------------------------------------------------------------
 *      Name:    LAN91C111.C
 *      Purpose: Driver for SMSC LAN91C111 Ethernet Controller
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <Net_Config.h>
#include "Lan91C111.h"
#include <LPC21xx.H>                          /* LPC21xx definitions         */

/* The following macro definitions may be used to select the speed
   of the physical link:

  _10MBIT_   - connect at 10 MBit only
  _100MBIT_  - connect at 100 MBit only

  By default an autonegotiation of the link speed is used. This may take 
  longer to connect, but it works for 10MBit and 100MBit physical links.     */

/* Net_Config.c */
extern U8 own_hw_adr[];

/* Local variables */

/*----------------------------------------------------------------------------
 *      LAN91C111 Ethernet Driver Functions
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
static void delay_1ms (U32 time);
static void output_MDO (int bit_value);
static void output_MDO (int bit_value);
static int  input_MDI (void);
static void write_PHY (U32 PhyReg, int Value);
static U16  read_PHY (U32 PhyReg);
static void def_interrupt (void) __irq;
static void interrupt_ethernet (void) __irq;

/*--------------------------- init_ethernet ---------------------------------*/

void init_ethernet (void) {
  /* Initialize the Lan91C111 ethernet controller. */
  U32 tcr,stat;
  int i;

  /* Mask off all interrupts */
  LREG (U16, BSR)    = 2;
  LREG (U8,  B2_MSK) = 0;

  /* Output reset to LAN controller */
  LREG (U16, BSR)    = 0;
  LREG (U16, B0_RCR) = RCR_SOFT_RST;

  /* Wait 50 ms for PHY to reset. */
  delay_1ms (50);

  /* Clear control registers. */
  LREG (U16, B0_RCR) = 0;
  LREG (U16, B0_TCR) = 0;

  /* Read MAC address stored to external EEPROM */
  LREG (U16, BSR) = 1;

  i = LREG (U16, B1_IAR0);
  own_hw_adr[0] = i;
  own_hw_adr[1] = i >> 8;
  i = LREG (U16, B1_IAR2);
  own_hw_adr[2] = i;
  own_hw_adr[3] = i >> 8;
  i = LREG (U16, B1_IAR4);
  own_hw_adr[4] = i;
  own_hw_adr[5] = i >> 8;

  /* Write Configuration Registers */
  LREG (U16, B1_CR) = CR_EPH_POW_EN | CR_DEFAULT;

  /* Wait 50 ms for MMU operation to finish. */
  delay_1ms (50);

  /* Establish the link */

  /* Reset the PHY, timeout is 3 sec */
  write_PHY (0, 0x8000);
  for (i = 0; i < 30; i++) {
    delay_1ms (100);
    if (!(read_PHY (0) & 0x8000)) {
      /* reset complete */
      break;
    }
  }

  /* Before auto negotiation, clear phy 18 status */
  read_PHY (18);

  /* Set the MAC Register and select the speed. */
  LREG (U16, BSR)     = 0;
#if defined (_10MBIT_)
  /* Connect at 10MBit */
  LREG (U16, B0_RPCR) = LEDA_10M_100M | LEDB_TX_RX;
#elif defined (_100MBIT_)
  /* Connect at 100MBit */
  LREG (U16, B0_RPCR) = RPCR_SPEED | LEDA_10M_100M | LEDB_TX_RX;
#else
  /* Use autonegotiation about the link speed. */
  LREG (U16, B0_RPCR) = RPCR_ANEG | LEDA_10M_100M | LEDB_TX_RX;
#endif
  /* Turn off the isolation mode, start Auto_Negotiation process. */
  write_PHY (0, 0x3000);

#if !defined (_10MBIT_) && !defined (_100MBIT_)
  /* Wait to complete Auto_Negotiation. */
  for (i = 0; i < 150; i++) {
    delay_1ms (100);
    stat = read_PHY (1);
    if (stat & 0x0020) {
      /* ANEG_ACK set, autonegotiation finished. */
      break;
    }
  }

  /* Check for the output status of the autoneg. */
  for (i = 0; i < 30; i++) {
    delay_1ms (100);
    stat = read_PHY (18);
    if (!(stat & 0x4000)) {
      break;
    }
  }
#else
  stat = read_PHY (18);
#endif

  /* Set the Control Register */
  LREG (U16, BSR)    = 1;
  LREG (U16, B1_CTR) = CTR_LE_ENABLE | CTR_CR_ENABLE | CTR_TE_ENABLE |
                       CTR_AUTO_REL  | CTR_DEFAULT;

  /* Set Receive Control Register, accept Multicast. */
  LREG (U16, BSR)    = 0;
  LREG (U16, B0_RCR) = RCR_RXEN | RCR_STRIP_CRC | RCR_ALMUL;

  /* Setup Transmit Control Register. */
  tcr = TCR_TXENA | TCR_PAD_EN;
  if (stat & 0x0040) {
    tcr |= TCR_FDUPLX;
  }
  LREG (U16, B0_TCR) = (U16)tcr;

  /* Reset MMU */
  LREG (U16, BSR)    = 2;
  LREG (U16, B2_MMUCR) = MMU_RESET;
  while (LREG (U16, B2_MMUCR) & MMUCR_BUSY);

  /* Configure the Interrupts, allow  RX_OVRN and RCV intr. */
  LREG (U8,  B2_MSK) = MSK_RX_OVRN | MSK_RCV;

  /* Enable EINT0 */
  PINSEL1 &= ~0x00000003;
  PINSEL1 |=  0x00000001;

  /* LPC2294 chip problem.                 */
  /* The following sequence does not work: */
  /* EXTMODE  = 0x00;                      */
  /* EXTPOLAR = 0x01;                      */

  /* Configure EINT0 for level triggered hi-active interrupt. */
  EXTMODE  = 0x00;
  EXTPOLAR = 0x03;

  VICDefVectAddr = (U32)def_interrupt;
  VICVectAddr14  = (U32)interrupt_ethernet;
  VICVectCntl14  = 0x20 | 14;
}


/*--------------------------- def_interrupt ---------------------------------*/

void def_interrupt (void) __irq {
  /* Default Interrupt Function: may be called when timer ISR is disabled */
  VICVectAddr = 0;
}


/*--------------------------- int_enable_eth --------------------------------*/

void int_enable_eth (void) {
  /* Ethernet Interrupt Enable function. */
  VICIntEnable |= (1 << 14);
}


/*--------------------------- int_disable_eth -------------------------------*/

void int_disable_eth (void) {
  /* Ethernet Interrupt Disable function. */
  VICIntEnClr = (1 << 14);
}

/*--------------------------- send_frame ------------------------------------*/

void send_frame (OS_FRAME *frame) {
  /* Send frame to Lan91C111 ethernet controller */
  U16 *dp;
  U8 packnr;
  int i;

  int_disable_eth ();
  /* Allocate buffer in Lan91C111 memory */
  LREG (U16, BSR) = 2;
  LREG (U16, B2_MMUCR) = MMU_ALLOC_TX;
  i = 300000;
  do {
    if (--i == 0) {
      /* Prevent dead loops. */
      break;
    }
  } while (!(LREG (U16, B2_IST) & IST_ALLOC_INT));

  if (i == 0) {
    /* Failed, Reset MMU */
    LREG (U16, B2_MMUCR) = MMU_RESET;
    while (LREG (U16, B2_MMUCR) & MMUCR_BUSY);
    goto x;
  }

  packnr = LREG (U8, B2_ARR);
  LREG (U8, B2_PNR) = packnr;
  LREG (U16, B2_PTR) = PTR_AUTO_INCR;

  /* Reserve space for Status */
  LREG (U16, B2_DATA0) = 0x0000;
  LREG (U16, B2_DATA0) = frame->length + 6;

  /* Copy frame data to Ethernet controller */
  dp = (U16 *)&frame->data[0];
  for (i = frame->length; i > 1; i -= 2) {
    LREG (U16, B2_DATA0) = *dp++;
  }
  if (i) {
    /* Add a control word and odd byte. */
    LREG (U16, B2_DATA0) = (RFC_CRC | RFC_ODD) | (*dp & 0xFF);
  }
  else {
    /* Add control word. */
    LREG (U16, B2_DATA0) = RFC_CRC;
  }

  /* Enable transmitter. */
  LREG (U16, BSR)     = 0;
  LREG (U16, B0_TCR)  = TCR_TXENA | TCR_PAD_EN;

  /* Enqueue the packet. */
  LREG (U16, BSR)      = 2;
  LREG (U16, B2_MMUCR) = MMU_ENQ_TX;
x: int_enable_eth ();
}


/*--------------------------- interrupt_ethernet ----------------------------*/

static void interrupt_ethernet (void) __irq {
  /* Ethernet Controller Interrupt function. */
  OS_FRAME *frame;
  U32 State, RxLen;
  U32 val, *dp;

  LREG (U16, BSR) = 2;

  while ((val = (LREG (U8, B2_IST) & (IST_RX_OVRN | IST_RCV))) != 0) {
    if (val & IST_RX_OVRN) {
      /* Clear the RX overrun bit. */
      LREG (U8, B2_ACK) = ACK_RX_OVRN;
      continue;
    }

    State = LREG (U16, B2_FIFO);
    if (State & FIFO_REMPTY) {
      /* Check if empty packet. */
      continue;
    }

    /* Read status and packet length */
    LREG (U16, B2_PTR) = PTR_RCV | PTR_AUTO_INCR | PTR_READ;
    val = LREG (U32, B2_DATA);
    State = val & 0xFFFF;
    RxLen = (val >> 16) - 6;
    if (State & RFS_ODDFRM) {
      /* Odd number of bytes in a frame. */
      RxLen++;
    }

    if (RxLen > ETH_MTU) {
      /* Packet too big, ignore it and free MMU. */
      LREG (U16, B2_MMUCR) = MMU_REMV_REL_RX;
      continue;
    }

    /* Flag 0x80000000 to skip sys_error() call when out of memory. */
    frame = alloc_mem (RxLen | 0x80000000);

    /* if 'alloc_mem()' has failed, ignore this packet. */
    if (frame != NULL) {
      /* Make sure that block is dword aligned */
      RxLen = (RxLen + 3) >> 2;

      dp = (U32 *)&frame->data[0];
      for (  ; RxLen; RxLen--) {
        *dp++ = LREG (U32, B2_DATA);
      }
      put_in_queue (frame);
    }

    /* MMU free packet. */
    LREG (U16, B2_MMUCR) = MMU_REMV_REL_RX;

  }
  /* Acknowledge Interrupt */
  EXTINT = 0x01;
  VICVectAddr = 0;
}


/*--------------------------- delay_1ms -------------------------------------*/

static void delay_1ms (U32 time) {
  /* Implements delays for the driver */
  /* Note: The execution time of this function depends on the CPU clock,    */
  /* type of memory where the code is executed etc. It should be calibrated.*/
  U32 dly;

  for (  ; time; time--) {
    /* Wait 1ms - execution time depends on CPU speed. */
    for (dly = 2500; dly; dly--);
  }
}


/*--------------------------- output_MDO ------------------------------------*/

static void output_MDO (int bit_value) {
  /* Output a bit value to the MII PHY management interface. */
  U32 val = MGMT_MDOE;

  if (bit_value) {
    val |= MGMT_MDO;
  }
  LREG (U16, B3_MGMT) = (U16)val;
  LREG (U16, B3_MGMT) = (U16)(val | MGMT_MCLK);
  LREG (U16, B3_MGMT) = (U16)val;
}


/*--------------------------- input_MDI -------------------------------------*/

static int input_MDI (void) {
  /* Input a bit value from the MII PHY management interface. */
  int val = 0;

  LREG (U16, B3_MGMT) = 0;
  LREG (U16, B3_MGMT) = MGMT_MCLK;
  if (LREG (U16, B3_MGMT) & MGMT_MDI) {
    val = 1;
  }
  LREG (U16, B3_MGMT) = 0;
  return (val);
}


/*--------------------------- write_PHY -------------------------------------*/

static void write_PHY (U32 PhyReg, int Value) {
  /* Write a data 'Value' to PHY register 'PhyReg'. */
  int i;

  LREG (U16, BSR)     = 3;
  LREG (U16, B3_MGMT) = MGMT_MDOE | MGMT_MDO;

  /* 32 consecutive ones on MDO to establish sync */
  for (i = 0; i < 32; i++) {
    LREG (U16, B3_MGMT) = MGMT_MDOE | MGMT_MDO;
    LREG (U16, B3_MGMT) = MGMT_MDOE | MGMT_MDO | MGMT_MCLK;
  }
  LREG (U16, B3_MGMT) = MGMT_MDOE;

  /* start code (01) */
  output_MDO (0);
  output_MDO (1);

  /* write command (01) */
  output_MDO (0);
  output_MDO (1);

  /* write PHY address - which is five 0s for 91C111 */
  for (i = 0; i < 5; i++) {
    output_MDO (0);
  }

  /* write the PHY register to write (highest bit first) */
  for (i = 0; i < 5; i++) {
    output_MDO ((PhyReg >> 4) & 0x01);
    PhyReg <<= 1;
  }

  /* turnaround MDO */
  output_MDO (1);
  output_MDO (0);

  /* write the data value (highest bit first) */
  for (i = 0; i < 16; i++) {
    output_MDO ((Value >> 15) & 0x01);
    Value <<= 1;
  }

  /* turnaround MDO is tristated */
  LREG (U16, B3_MGMT) = 0;
  LREG (U16, B3_MGMT) = MGMT_MCLK;
  LREG (U16, B3_MGMT) = 0;
}


/*--------------------------- read_PHY --------------------------------------*/

static U16 read_PHY (U32 PhyReg) {
  /* Read a PHY register 'PhyReg'. */
  int i, val;

  LREG (U16, BSR)     = 3;
  LREG (U16, B3_MGMT) = MGMT_MDOE | MGMT_MDO;

  /* 32 consecutive ones on MDO to establish sync */
  for (i = 0; i < 32; i++) {
    LREG (U16, B3_MGMT) = MGMT_MDOE | MGMT_MDO;
    LREG (U16, B3_MGMT) = MGMT_MDOE | MGMT_MDO | MGMT_MCLK;
  }
  LREG (U16, B3_MGMT) = MGMT_MDOE;

  /* start code (01) */
  output_MDO (0);
  output_MDO (1);

  /* read command (10) */
  output_MDO (1);
  output_MDO (0);

  /* write PHY address - which is five 0s for 91C111 */
  for (i = 0; i < 5; i++) {
    output_MDO (0);
  }

  /* write the PHY register to read (highest bit first) */
  for (i = 0; i < 5; i++) {
    output_MDO ((PhyReg >> 4) & 0x01);
    PhyReg <<= 1;
  }

  /* turnaround MDO is tristated */
  LREG (U16, B3_MGMT) = 0;
  LREG (U16, B3_MGMT) = MGMT_MCLK;
  LREG (U16, B3_MGMT) = 0;

  /* read the data value */
  val = 0;
  for (i = 0; i < 16; i++) {
    val <<= 1;
    val |= input_MDI ();
  }

  /* turnaround MDO is tristated */
  LREG (U16, B3_MGMT) = 0;
  LREG (U16, B3_MGMT) = MGMT_MCLK;
  LREG (U16, B3_MGMT) = 0;

  return (val);
}

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
