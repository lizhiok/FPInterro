/*------------------------------------------------------------------------------
 *      RL-ARM - USB
 *------------------------------------------------------------------------------
 *      Name:    usbh_ohci_lpc32x0.c
 *      Purpose: OHCI Hardware Specific Layer Driver for the 
 *               NXP LPC32x0 Device Series
 *      Rev.:    V4.70
 *------------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include <RTL.h>
#include <rl_usb.h>
#include <LPC325x.h>


/************************* OHCI Hardware Driver Configuration *****************/

// *** <<< Use Configuration Wizard in Context Menu >>> ***

// <h> USB Host OHCI Settings
//   <o0> USB Host OHCI Controller Base Address
//   <h> Root Hub ports used by OHCI Controller
//     <i> These are the ports that OHCI will use.
//     <o1.0>  Port 1
//     <o1.1>  Port 2
//     <o1.2>  Port 3
//     <o1.3>  Port 4
//     <o1.4>  Port 5
//     <o1.5>  Port 6
//     <o1.6>  Port 7
//     <o1.7>  Port 8
//     <o1.8>  Port 9
//     <o1.9>  Port 10
//     <o1.10> Port 11
//     <o1.11> Port 12
//     <o1.12> Port 13
//     <o1.13> Port 14
//     <o1.14> Port 15
//     </h>
//
//   <o2> Start address of memory used by OHCI
//     <i> This is a start address of memory that OHCI will use for descriptors 
//     <i> and USB communication data.
//
//   <o3> Size of memory used for USB data by OHCI <1-1048576>
//     <i> This is a size of memory (in bytes) that OHCI will use for  
//     <i> USB communication data (maximum sum of data at a single point in 
//     <i> time, for example for HID = size of max Configuration Descriptor + 
//     <i> size of max HID Report Descriptor).
//
//   <o4> Maximum number of Endpoint Descriptors used by OHCI <1-64>
//     <i> This is a maximum number of Endpoints that OHCI will use.
//
//   <o5> Maximum number of Transfer Descriptors used by OHCI <1-64>
//     <i> This is a maximum number of Transfers that OHCI will use.
//
//   <o6> Maximum number of Isochronous Transfer Descriptors used by OHCI <0-64>
//     <i> This is a maximum number of Isochronous Transfers that OHCI will use.
// </h>
#define USBH_OHCI_ADR               0x31020000
#define USBH_OHCI_PORTS             0x00000001
#define USBH_OHCI_MEM_ADR           0x08001000
#define USBH_OHCI_MEM_DATA_SZ       4096
#define USBH_OHCI_NUM_ED            3
#define USBH_OHCI_NUM_TD            1
#define USBH_OHCI_NUM_ITD           0

// *** <<< End of Configuration section             >>> ***


/************************** OHCI Host Controller Hardware Driver Variables ****/

#define USBH_OHCI_MEM_SZ           (256                   +  /* HCCA sz    */  \
                                    USBH_OHCI_NUM_ED * 16 +  /* EDs sz     */  \
                                    USBH_OHCI_NUM_TD * 16 +  /* TDs sz     */  \
                                    USBH_OHCI_MEM_DATA_SZ +  /* DATA sz    */  \
                                    USBH_OHCI_NUM_TD * 8  +  /* alloc ovhd */  \
                                    4                     )  /* alloc ovhd */

#define USBH_OHCI_MEM_HCCA         (USBH_OHCI_MEM_ADR)
#define USBH_OHCI_MEM_ED           (USBH_OHCI_MEM_ADR+256)
#define USBH_OHCI_MEM_TD           (USBH_OHCI_MEM_ED+(USBH_OHCI_NUM_ED<<4))
#define USBH_OHCI_MEM_ITD          (USBH_OHCI_MEM_TD+(USBH_OHCI_NUM_TD<<4))
#define USBH_OHCI_MEM_MPOOL        (USBH_OHCI_MEM_ITD+(USBH_OHCI_NUM_ITD<<5))
#define USBH_OHCI_MEM_SZ_MPOOL     (USBH_OHCI_MEM_SZ-(USBH_OHCI_MEM_MPOOL-USBH_OHCI_MEM_ADR))
#define USBH_OHCI_NUM_TDURB        (USBH_OHCI_NUM_TD+USBH_OHCI_NUM_ITD)

U32     usbh_ohci_hcca  [64]                        __attribute__((at(USBH_OHCI_MEM_ADR)));
U32     usbh_ohci_ed    [USBH_OHCI_NUM_ED<<2]       __attribute__((at(USBH_OHCI_MEM_ED)));
U32     usbh_ohci_td    [USBH_OHCI_NUM_TD<<2]       __attribute__((at(USBH_OHCI_MEM_TD)));
#if    (USBH_OHCI_NUM_ITD > 0)
U32     usbh_ohci_itd   [USBH_OHCI_NUM_ITD<<3]      __attribute__((at(USBH_OHCI_MEM_ITD)));
#endif
U32     usbh_ohci_mpool [USBH_OHCI_MEM_SZ_MPOOL>>2] __attribute__((at(USBH_OHCI_MEM_MPOOL)));
U32     usbh_ohci_tdurb [USBH_OHCI_NUM_TDURB<<1];


/************************** OHCI Host Controller Hardware Function Prototypes */

void usbh_ohci_hw_get_capabilities (USBH_HCI_CAP *cap);
void usbh_ohci_hw_delay_ms         (U32 ms);
void usbh_ohci_hw_reg_wr           (U32 reg_ofs, U32 val);
U32  usbh_ohci_hw_reg_rd           (U32 reg_ofs);
BOOL usbh_ohci_hw_pins_config      (BOOL on);
BOOL usbh_ohci_hw_init             (BOOL on);
BOOL usbh_ohci_hw_port_power       (BOOL on);
BOOL usbh_ohci_hw_irq_en           (BOOL on);

void USB_IRQHandler                (void);


/************************** OHCI Host Controller Hardware Driver Structure ****/

USBH_HWD_OHCI usbh0_hwd_ohci = {        /* OHCI Host Controller Hardware Drv  */
  USBH_OHCI_PORTS,                      /* Ports (bits 0..15)                 */
  USBH_OHCI_NUM_ED,                     /* Maximum Endpoint Descriptors       */
  USBH_OHCI_NUM_TD,                     /* Maximum Transfer Descriptors       */
  USBH_OHCI_NUM_ITD,                    /* Maximum Iso Transfer Descriptors   */
  (U32 *) &usbh_ohci_hcca,              /* Pointer to HCCA memory start       */
  (U32 *) &usbh_ohci_ed,                /* Pointer to ED memory start         */
  (U32 *) &usbh_ohci_td,                /* Pointer to TD memory start         */
#if    (USBH_OHCI_NUM_ITD > 0)
  (U32 *) &usbh_ohci_itd,               /* Pointer to ITD memory start        */
#else
  NULL,
#endif
  (U32 *) &usbh_ohci_tdurb,             /* Pointer to TDURB memory start      */
  usbh_ohci_hw_get_capabilities,        /* Get driver capabilities            */
  usbh_ohci_hw_delay_ms,                /* Delay in ms                        */
  usbh_ohci_hw_reg_wr,                  /* Write register                     */
  usbh_ohci_hw_reg_rd,                  /* Read register                      */
  usbh_ohci_hw_pins_config,             /* Config/Unconfig pins               */
  usbh_ohci_hw_init,                    /* Init/Uninit Host Controller        */
  usbh_ohci_hw_port_power,              /* On/Off Port Power                  */
  usbh_ohci_hw_irq_en                   /* Enable/Disable interrupt           */
};


/************************** Imported Functions ********************************/

extern void USBH_OHCI_IRQHandler (void);


/************************** Auxiliary Functions *******************************/

/*------------------------- isp1301_i2c_wr -------------------------------------
 *
 *  Write register of ISP1301 over I2C interface.
 *
 *  Parameter:  reg:        Register address
 *              val:        Register value 
 *  Return:                 Error code
 *----------------------------------------------------------------------------*/

static BOOL isp1301_i2c_wr (U32 reg, U32 val) {
  OTG_I2C_TX  =  (1    <<  8) |         /* START                              */
                 (0x2C <<  1) |         /* ADDRESS (0x2C)                     */
                  0;                    /* WR                                 */
  while (!(OTG_I2C_STS & (1 << 11)));   /* Wait Transmit FIFO Empty    (TFE)  */
  if (OTG_I2C_STS & (1 <<  2))          /* If No Acknowledge received  (NAI)  */
    return (__FALSE);
  OTG_I2C_TX  =   reg;                  /* Register address                   */
  while (!(OTG_I2C_STS & (1 << 11)));   /* Wait Transmit FIFO Empty    (TFE)  */
  if (OTG_I2C_STS & (1 <<  2))          /* If No Acknowledge received  (NAI)  */
    return (__FALSE);
  OTG_I2C_TX  =  (1    <<  9) |         /* STOP                               */
                  val         ;         /* Register value                     */
  while (!(OTG_I2C_STS & (1 << 11)));   /* Wait Transmit FIFO Empty    (TFE)  */
  if (OTG_I2C_STS & (1 <<  2))          /* If No Acknowledge received  (NAI)  */
    return (__FALSE);
  while (!(OTG_I2C_STS & (1 <<  0)));   /* Wait Transaction Done       (TDI)  */
  OTG_I2C_STS =  (1    <<  0);          /* Clear Transaction Done      (TDI)  */

  return (__TRUE);
}


/*------------------------- isp1301_i2c_rd -------------------------------------
 *
 *  Read register of ISP1301 over I2C interface.
 *
 *  Parameter:  reg:        Register address
 *              sz:         Register size (in bytes 1..4)
 *             *val:        Pointer to register value 
 *  Return:                 Error code
 *----------------------------------------------------------------------------*/

static BOOL isp1301_i2c_rd (U32 reg, U32 sz, U32 *val) {
  U32 i;

  if ((!sz) || (sz > 4))
    return (__FALSE);

  OTG_I2C_TX  =  (1    <<  8) |         /* START                              */
                 (0x2C <<  1) |         /* ADDRESS (0x2C)                     */
                  0;                    /* WR                                 */
  while (!(OTG_I2C_STS & (1 << 11)));   /* Wait Transmit FIFO Empty    (TFE)  */
  if (OTG_I2C_STS & (1 <<  2))          /* If No Acknowledge received  (NAI)  */
    return (__FALSE);
  OTG_I2C_TX  =   reg;                  /* Register address                   */
  while (!(OTG_I2C_STS & (1 << 11)));   /* Wait Transmit FIFO Empty    (TFE)  */
  if (OTG_I2C_STS & (1 <<  2))          /* If No Acknowledge received  (NAI)  */
    return (__FALSE);
  OTG_I2C_TX  =  (1    <<  8) |         /* START                              */
                 (0x2C <<  1) |         /* ADDRESS (0x2C)                     */
                  1;                    /* RD                                 */
  while (!(OTG_I2C_STS & (1 << 11)));   /* Wait Transmit FIFO Empty    (TFE)  */
  if (OTG_I2C_STS & (1 <<  2))          /* If No Acknowledge received  (NAI)  */
    return (__FALSE);
  while (!(OTG_I2C_STS & (1 <<  3)));   /* Wait for Master Data Request(DRMI) */
  OTG_I2C_TX  =   0xAA;                 /* Dummy write to start reception     */
  while (!(OTG_I2C_STS & (1 <<  3)));   /* Wait for Master Data Request(DRMI) */
  *val = 0;
  i    = 0;
  while (sz--) {
    if (!sz)
      OTG_I2C_TX  =  (1    <<  8) |     /* STOP                               */
                      0xAA;             /* Dummy write to read and STOP       */
    else
      OTG_I2C_TX  =   0xAA;             /* Dummy write to start reception     */
    while (OTG_I2C_STS & (1 <<  9));    /* Wait Receive FIFO not Empty (RFE)  */
    *val |= (OTG_I2C_RX & 0xFF) << i;   /* Read byte                          */
    i    += 8;
  }
  while (!(OTG_I2C_STS & (1 <<  0)));   /* Wait Transaction Done       (TDI)  */
  OTG_I2C_STS =  (1    <<  0);          /* Clear Transaction Done      (TDI)  */

  return (__TRUE);
}


/************************** Module Functions **********************************/

/*------------------------- usbh_ohci_hw_get_capabilities ----------------------
 *
 *  Get capabilities of Host Controller Driver
 *
 *  Parameter:  cap:        Pointer to USBH_HCI_CAP structure where 
 *                          capabilities are loaded
 *  Return:
 *----------------------------------------------------------------------------*/

void usbh_ohci_hw_get_capabilities (USBH_HCI_CAP *cap) {
  cap->MultiPckt = __TRUE;
  cap->MaxDataSz = USBH_OHCI_MEM_DATA_SZ;
  cap->CtrlNAKs  = 1000;
  cap->BulkNAKs  = 1000000;
}


/*------------------------- usbh_ohci_hw_delay_ms ------------------------------
 *
 *  Delay execution (in milliseconds on 208 MHz CPU clock)
 *
 *  Parameter:  ms:         Delay in ms
 *  Return:
 *----------------------------------------------------------------------------*/

void usbh_ohci_hw_delay_ms (U32 ms) {

  ms <<= 14;
  while (ms--) {
    __nop(); __nop(); __nop(); __nop(); __nop(); __nop(); __nop(); __nop();
  }
}


/*------------------------- usbh_ohci_hw_reg_wr --------------------------------
 *
 *  Write hardware register.
 *
 *  Parameter:  reg:        Register offset
 *              val:        Register value 
 *  Return:
 *----------------------------------------------------------------------------*/

void usbh_ohci_hw_reg_wr (U32 reg_ofs, U32 val) {
  *((U32 *)(USBH_OHCI_ADR + reg_ofs)) = val;
}


/*------------------------- usbh_ohci_hw_reg_rd --------------------------------
 *
 *  Read hardware register.
 *
 *  Parameter:  reg:        Register offset
 *  Return:                 Register value 
 *----------------------------------------------------------------------------*/

U32 usbh_ohci_hw_reg_rd (U32 reg_ofs) {
  return (*((U32 *)(USBH_OHCI_ADR + reg_ofs)));
}


/*------------------------- usbh_ohci_hw_pins_cfg ------------------------------
 *
 *  Configurate or unconfigurate pins used by the USB Host.
 *
 *  Parameter:  on:         __TRUE = configurate, __FALSE = unconfigurate
 *  Return:                 __TRUE = Ok, __FALSE = error
 *----------------------------------------------------------------------------*/

BOOL usbh_ohci_hw_pins_config (BOOL on) {

  return (__TRUE);
}


/*------------------------- usbh_ohci_hw_init ----------------------------------
 *
 *  Initialize or uninitialize the USB Host Controller.
 *
 *  Parameter:  on:         __TRUE = initialize, __FALSE = uninitialize
 *  Return:                 __TRUE = Ok, __FALSE = error
 *----------------------------------------------------------------------------*/

BOOL usbh_ohci_hw_init (BOOL on) {
  U32 tout, val;

  if (on) {
    /* Initialize memory pool for data                                        */
    if (!usbh_mem_init(0, (U32 *)&usbh_ohci_mpool, sizeof(usbh_ohci_mpool)))
      return (__FALSE);

    USB_DIV       =   13 - 1;           /* USB input rate = 13MHz / (rate + 1)*/

    USBCLK_CTRL   =  (1   << 19);       /* Restore register reset value       */
    USBCLK_CTRL  |=  (1   << 17);       /* USB_Clken1: USB PLL clocked        */
    USBCLK_CTRL  |=  (191 <<  1);       /* PLL feedback div (M): * 192 (M=192)*/
    USBCLK_CTRL  &= ~(3   <<  9);       /* PLL pre-divider  (N): / 1   (N=1)  */
    USBCLK_CTRL  |=  (1   << 11);       /* PLL post-divider (P): / 4   (P=2)  */
    USBCLK_CTRL  &= ~(1   << 13);       /* Feed CCO back to PLL               */
    USBCLK_CTRL  &= ~(1   << 14);       /* Post divider is PLL clock output   */
    USBCLK_CTRL  &= ~(1   << 15);       /* CCO is sent to post divider        */
    USBCLK_CTRL  |=  (1   << 16);       /* Power up the USB PLL               */

    for (tout = 10000; ; tout--) {
      if ((USBCLK_CTRL & 1) == 1)       /* Wait for USB PLL lock              */
        break;
      if (!tout) 
        return (__FALSE);
    }

    USBCLK_CTRL  |=  (1   << 18) ;      /* USB_Clken2: USB block clocked      */
    for (tout = 100; ; tout--) {
      if (USBCLK_CTRL & (1 << 18))      /* Wait for enabled                   */
        break;
      if (!tout) 
        return (__FALSE);
    }

    USBCLK_CTRL  |=  (1   << 24);       /* Slave HCLK enabled                 */
    OTG_CLK_CTRL |=  0x1C;              /* Enable OTG,I2C and AHB clock       */

    for (tout = 100; ; tout--) {
      if ((OTG_CLK_STAT & 0x1C) == 0x1C)/* Wait for clocks enabled            */
        break;
      if (!tout) 
        return (__FALSE);
    }

    I2CCLK_CTRL  |=  (1 <<  4);         /* USB I2C pins high drive mode       */
    OTG_I2C_CTL   =  (1 <<  8);         /* Soft reset the I2C                 */
    for (tout = 100; ; tout--) {
      if (!(OTG_I2C_CTL & (1 << 8)))    /* Wait for reset to finish           */
        break;
     if (!tout) 
        return (__FALSE);
    }
    OTG_I2C_CLKHI =  65;                /* I2C clock = PERIPH_CLK / 130       */
    OTG_I2C_CLKLO =  65;                /* = 13 MHz / 130 = 100 kHz           */

    isp1301_i2c_rd (0x00, 2, &val);     /* Read ISP1301 Vendor ID             */
    if (val == 0x04CC) {                /* If Philips ISP1301 transceiver used*/
      isp1301_i2c_wr (0x12,(1<<6)|(1<<2));/* Mode Control 2:PSW_OE = 1,BI_DI=1*/
      isp1301_i2c_wr (0x04,       (1<<2));/* Mode Control 1:DAT_SE0 = 1       */
    }

    USBCLK_CTRL  |=  (1   << 21);       /* usb_host_need_clk_en: enabled      */
    for (tout = 100; ; tout--) {
      if (USBCLK_CTRL & (1 << 21))      /* Wait for enabled                   */
        break;
      if (!tout) 
        return (__FALSE);
    }

    OTG_CLK_CTRL |=  0x1D;              /* Enable Host,OTG,I2C and AHB clock  */

    for (tout = 100; ; tout--) {
      if ((OTG_CLK_STAT & 0x1D) == 0x1D)/* Wait for clocks enabled            */
        break;
      if (!tout) 
        return (__FALSE);
    }

    OTG_STAT_CTRL =   1;                /* OTG: USB Host enabled              */

    /* Enable USB host interrupt                                              */
    SIC1_APR     |=  (1 << 27);         /* Interrupt polarity, high           */
    SIC1_ATR     &= ~(1 << 27);         /* Interrupt activation type, level   */
    SIC1_ITR     &= ~(1 << 27);         /* Interrupt routed to IRQ            */
    MIC_APR      &= ~(1 <<  0);         /* Interrupt polarity, low            */
    MIC_ATR      &= ~(1 <<  0);         /* Interrupt activation type, level   */
    MIC_ITR      &= ~(1 <<  0);         /* Interrupt routed to IRQ            */
    MIC_ER       |=  (1 <<  0);         /* Enable IRQ from SIC1               */
  } else {
    OTG_STAT_CTRL =   0;                /* OTG: USB Device enabled            */

    OTG_CLK_CTRL &= ~0x1D;              /* Disable Host,OTG,I2C and AHB clock */

    for (tout = 100; ; tout--) {
      if ((OTG_CLK_STAT & 0x1D) == 0)   /* Wait for clocks disabled           */
        break;
      if (!tout) 
        return (__FALSE);
    }

    I2CCLK_CTRL   =   0;                /* USB I2C clock control to reset val */
    OTG_I2C_CTL   =  (1 <<  8);         /* Soft reset the I2C                 */
    for (tout = 100; ; tout--) {
      if (!(OTG_I2C_CTL & (1 << 8)))    /* Wait for reset to finish           */
        break;
      if (!tout) 
        return (__FALSE);
    }
    OTG_I2C_CLKHI =  65;                /* Restore register reset value       */
    OTG_I2C_CLKLO =  65;                /* Restore register reset value       */

    USBCLK_CTRL   =  (1   << 19);       /* Restore register reset value       */
    USB_DIV       =   12;               /* Restore register reset value       */
  }

  return (__TRUE);
}


/*------------------------- usbh_ohci_hw_port_power ----------------------------
 *
 *  Turn Port Power on or off with pin if not handled through OHCI.
 *
 *  Parameter:  on:         __TRUE = turn power on, __FALSE = turn power off
 *  Return:                 __TRUE = Ok, __FALSE = error
 *----------------------------------------------------------------------------*/

BOOL usbh_ohci_hw_port_power (BOOL on) {

  return (__TRUE);
}


/*------------------------- usbh_ohci_hw_irq_en --------------------------------
 *
 *  USB Host OHCI Controller interrupt enable or disable.
 *
 *  Parameter:  on:         __TRUE = enable, __FALSE = disable
 *  Return:                 __TRUE = Ok, __FALSE = error
 *----------------------------------------------------------------------------*/

BOOL usbh_ohci_hw_irq_en (BOOL on) {

  if (on) {
    SIC1_ER      |=  (1 << 27);         /* Enable USB Host interrupt          */
  } else {
    SIC1_ER      &= ~(1 << 27);         /* Disable USB Host interrupt         */
  }

  return (__TRUE);
}


/*------------------------- USB_IRQHandler -------------------------------------
 *
 *  Hardware USB Interrupt Handler Routine.
 *
 *  Parameter:
 *  Return:
 *----------------------------------------------------------------------------*/

void USB_IRQHandler (void) {

  if (SIC1_SR & (1 << 27)) {            /* If USB Host interrupt occured      */
    USBH_OHCI_IRQHandler();             /* Handle USB Host interrupt events   */
    SIC1_RSR    = (1 << 27);            /* Clear USB Host interrupt           */
    MIC_RSR     = (1 <<  0);            /* Clear IRQ from SIC1                */
  }
}
