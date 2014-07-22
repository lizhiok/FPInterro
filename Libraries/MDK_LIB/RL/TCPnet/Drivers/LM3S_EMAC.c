/*----------------------------------------------------------------------------
 *      RL-ARM - TCPnet
 *----------------------------------------------------------------------------
 *      Name:    LM3S_EMAC.C
 *      Purpose: Driver for Luminary Micro LM3Sxxxx EMAC Ethernet Controller
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <Net_Config.h>
#include <LM3Sxxxx.H>
#include "LM3S_EMAC.h"

/* The following macro definitions may be used to select the speed
   of the physical link:

  _10MBIT_   - connect at 10 MBit only
  _100MBIT_  - connect at 100 MBit only

  By default an autonegotiation of the link speed is used. This may take 
  longer to connect, but it works for 10MBit and 100MBit physical links.     */


/* Net_Config.c */
extern U8 own_hw_adr[];

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
static void write_PHY (U32 PhyReg, U16 Value);
static U16  read_PHY (U32 PhyReg);


/*--------------------------- init_ethernet ---------------------------------*/

void init_ethernet (void) {
  /* Initialize the EMAC ethernet controller. */
  U32 tout,regv;

  /* Enable and Reset the Ethernet Controller */
  SysCtlPeripheralEnable (SYSCTL_PERIPH_ETH);
  SysCtlPeripheralReset (SYSCTL_PERIPH_ETH);

  /* Enable Port F for Ethernet LEDs */
  SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOF);
  GPIODirModeSet  (GPIO_PORTF_BASE, (GPIO_PIN_2 | GPIO_PIN_3),
                   GPIO_DIR_MODE_HW);
  GPIOPadConfigSet(GPIO_PORTF_BASE, (GPIO_PIN_2 | GPIO_PIN_3),
                   GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);

  /* Disable all Ethernet Interrupts */
  pEMAC->IEN = 0;
  pEMAC->ISR = INT_ALL;

  /* Set MII interface clock (max. 2.5MHz) */
  pEMAC->MCDIV = 10;

  /* Put the PHY in reset mode */
  write_PHY (PHY_REG_CTRL, 0x8000);

  /* Wait for hardware reset to end. */
  for (tout = 0; tout < 0x20000; tout++) {
    regv = read_PHY (PHY_REG_CTRL);
    if (!(regv & 0x8000)) {
      /* Reset complete. */
      break;
    }
  }

#if defined (_10MBIT_)
  /* Connect at 10MBit */
  write_PHY (PHY_REG_CTRL, PHY_FULLD_10M);
#elif defined (_100MBIT_)
  /* Connect at 100MBit */
  write_PHY (PHY_REG_CTRL, PHY_FULLD_100M);
#else
  /* Use autonegotiation about the link speed. */
  write_PHY (PHY_REG_CTRL, PHY_AUTO_NEG);
  /* Wait to complete Auto_Negotiation. */
  for (tout = 0; tout < 0x20000; tout++) {
    regv = read_PHY (PHY_REG_STAT);
    if (regv & 0x0020) {
      /* Autonegotiation Complete. */
      break;
    }
  }
#endif

  /* Check the link status. */
  for (tout = 0; tout < 0x10000; tout++) {
    regv = read_PHY (PHY_REG_STAT);
    if (regv & 0x0004) {
      /* Link is on. */
      break;
    }
  }

  /* Configure TX/RX Control Registers, enable Multicast. */
  pEMAC->RXCTL = RXCTL_BAD_CRC | RXCTL_RST_FIFO | RXCTL_MCAST_EN;
  pEMAC->TXCTL = TXCTL_PAD_EN  | TXCTL_CRC_EN;

  /* Configure Full/Half Duplex mode. */
  if (regv & 0x0800) {
    /* Full duplex is enabled. */
    pEMAC->TXCTL |= TXCTL_DUP_EN;
  }

  /* Set the Ethernet MAC Address registers */
  pEMAC->IAR0 = ((U32)own_hw_adr[3] << 24) | ((U32)own_hw_adr[2] << 16) |
                ((U32)own_hw_adr[1] << 8)  |  (U32)own_hw_adr[0];

  pEMAC->IAR1 = ((U32)own_hw_adr[5] << 8)  |  (U32)own_hw_adr[4];

  /* Enable the Ethernet Controller */
  pEMAC->RXCTL |= RXCTL_RX_EN;
  pEMAC->TXCTL |= TXCTL_TX_EN;

  /* Enable Interrupts */
  pEMAC->IEN = INT_RX;
  IntEnable(INT_ETH);
}


/*--------------------------- int_enable_eth --------------------------------*/

void int_enable_eth (void) {
  /* Ethernet Interrupt Enable function. */
  IntEnable(INT_ETH);
}


/*--------------------------- int_disable_eth -------------------------------*/

void int_disable_eth (void) {
  /* Ethernet Interrupt Disable function. */
  IntDisable(INT_ETH);
}


/*--------------------------- send_frame ------------------------------------*/

void send_frame (OS_FRAME *frame) {
  /* Send frame to EMAC ethernet controller */
  U32 len,val;
  U16 *sp;

  /* Wait for current packet to complete. */
  while (pEMAC->TXRQ & TXRQ_NEW_TX);

  sp = (U16 *)&frame->data[0];
  val = frame->length - 14;
  val |= (*sp++) << 16;
  pEMAC->DATA = val;
  /* Copy frame data to EMAC packet buffers. */
  for (len = (frame->length + 1) >> 2; len; sp += 2, len--) {
    pEMAC->DATA = sp[0] | ((U32)sp[1] << 16);
  }

  /* Start Transmitting */
  pEMAC->TXRQ = TXRQ_NEW_TX;
}


/*--------------------------- interrupt_ethernet ----------------------------*/

void interrupt_ethernet (void) __irq {
  /* EMAC Ethernet Controller Interrupt function. */
  OS_FRAME *frame;
  U32 val,int_stat,RxLen,len;
  U16 *dp;

  while ((int_stat = pEMAC->ISR & INT_ALL) != 0) {
    pEMAC->ISR = int_stat;
    if (int_stat & INT_RX) {
      /* Packet received, get the packet size. */
      val = pEMAC->DATA;
      RxLen = (val & 0xFFFF) - 6;
      /* DMA adds also 4-byte CRC and 2-byte 'size' to packet size. */

      if (RxLen > ETH_MTU || (int_stat & INT_RX_ERR)) {
        /* Invalid frame, ignore it and free buffer. */
        goto rel;
      }
      /* Flag 0x80000000 to skip sys_error() call when out of memory. */
      frame = alloc_mem (RxLen | 0x80000000);
      if (frame != NULL) {
        dp  = (U16 *)&frame->data[0];
        for (len = (RxLen + 3) >> 2; len; dp += 2, len--) {
          dp[0] = val >> 16;
          val = pEMAC->DATA;
          dp[1] = val & 0xFFFF;
        }
        if ((RxLen - 1) & 0x02) {
          /* Drain the remaining 1 or 2 byte(s) of the CRC. */
          val = pEMAC->DATA;
        }
        put_in_queue (frame);
      }
      else {
        /* In case of error discard the entire RX FIFO. */
rel:      pEMAC->RXCTL &= ~RXCTL_RX_EN;
        pEMAC->RXCTL |= RXCTL_RST_FIFO;
        pEMAC->RXCTL |= RXCTL_RX_EN;
      }
    }
  }
}


/*--------------------------- write_PHY -------------------------------------*/

static void write_PHY (U32 PhyReg, U16 Value) {
  /* Write a data 'Value' to PHY register 'PhyReg'. */
  U32 tout;

  pEMAC->MADR = PHY_DEF_ADR;
  pEMAC->MTXD = Value;
  pEMAC->MCTL = (PhyReg << 3) | MCTL_WR | MCTL_START;

  /* Wait util operation completed */
  for (tout = 0; tout < MII_WR_TOUT; tout++) {
    if ((pEMAC->MCTL & MCTL_START) == 0) {
      break;
    }
  }
}


/*--------------------------- read_PHY --------------------------------------*/

static U16 read_PHY (U32 PhyReg) {
  /* Read a PHY register 'PhyReg'. */
  U32 tout;

  pEMAC->MADR = PHY_DEF_ADR;
  pEMAC->MCTL = (PhyReg << 3) | MCTL_START;

  /* Wait until operation completed */
  for (tout = 0; tout < MII_RD_TOUT; tout++) {
    if ((pEMAC->MCTL & MCTL_START) == 0) {
      break;
    }
  }
  return (pEMAC->MRXD & MRXD_MASK);
}

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
