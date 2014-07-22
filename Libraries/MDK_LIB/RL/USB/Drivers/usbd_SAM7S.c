/*----------------------------------------------------------------------------
 *      RL-ARM - USB
 *----------------------------------------------------------------------------
 *      Name:    usbd_SAM7S.c
 *      Purpose: Hardware Layer module for Atmel AT91SAM7S
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <RTL.h>
#include <rl_usb.h>
#include <AT91SAM7S256.H>

#define __NO_USB_LIB_C
#include "usb_config.c"


const U8  DualBankEP = 0x06;            /* Dual Bank Endpoint Bit Mask        */

const U32 RX_DATA_BK[2] = {
  AT91C_UDP_RX_DATA_BK0,
  AT91C_UDP_RX_DATA_BK1
};


AT91PS_UDP pUDP = AT91C_BASE_UDP;       /* Global UDP Pointer                 */

U8  RxDataBank[USBD_EP_NUM+1];
U8  TxDataBank[USBD_EP_NUM+1];

void USBD_ISR (void) __irq;

/*
 *  Set / Clear functions to modify UDP_CSR register
 */

static void USBD_SetCSR(U32 EPNum, U32 flags) {
  U32 timeout = 100;

  EPNum &= 0x0F;
  pUDP->UDP_CSR[EPNum] |= (flags);
  while ((pUDP->UDP_CSR[EPNum] & (flags)) != (flags)) {
    if (!timeout--) break;
  }
}

static void USBD_ClrCSR(U32 EPNum, U32 flags) {
  U32 timeout = 100;

  EPNum &= 0x0F;
  pUDP->UDP_CSR[EPNum] &= ~(flags);
  while (pUDP->UDP_CSR[EPNum] & (flags)) {
    if (!timeout--) break;
  }
}


/*
 *  Retrieve maximum EP size Function
 *   Called during EndPoint configuration
 *    Return Value:    maximum size for given EP
 */

static int USB_GetSizeEP (U32 EPNum) {
  switch (EPNum & 0x0F) {
    case 0:
      return (8);                       /* Maximum size is 8 bytes            */
    case 1:
    case 2:
    case 3:
      return (64);                      /* Maximum size is 64 bytes           */
    default:
      return (0);                       /* Non existant endpoint              */
  }
}


/*
 *  USB Device Initialize Function
 *   Called by the User to initialize USB Device
 *    Return Value:    None
 */

void USBD_Init (void) {
  /* Enables the 48MHz USB Clock UDPCK and System Peripheral USB Clock        */
  AT91C_BASE_PMC->PMC_SCER = AT91C_PMC_UDP;
  while (!(AT91C_BASE_PMC->PMC_SCSR & AT91C_PMC_UDP));
  AT91C_BASE_PMC->PMC_PCER = (1 << AT91C_ID_UDP);
  while (!(AT91C_BASE_PMC->PMC_PCSR & (1 << AT91C_ID_UDP)));

  /* Global USB Interrupt: Mode and Vector with Highest Priority and Enable   */
  AT91C_BASE_AIC->AIC_SMR[AT91C_ID_UDP] = AT91C_AIC_SRCTYPE_INT_POSITIVE_EDGE |
                                          AT91C_AIC_PRIOR_HIGHEST;
  AT91C_BASE_AIC->AIC_SVR[AT91C_ID_UDP] = (U32) USBD_ISR;
  AT91C_BASE_AIC->AIC_ICCR = (1 << AT91C_ID_UDP);
  AT91C_BASE_AIC->AIC_IECR = (1 << AT91C_ID_UDP);

  /* UDP PullUp (USB_DP_PUP): PA16 Pin                                        */
  /*   Configure as Output and Set to disable Pull-up Resistor                */
  AT91C_BASE_PIOA->PIO_PER  = AT91C_PIO_PA16;
  AT91C_BASE_PIOA->PIO_SODR = AT91C_PIO_PA16;
  AT91C_BASE_PIOA->PIO_OER  = AT91C_PIO_PA16;
}


/*
 *  USB Device Connect Function
 *   Called by the User to Connect/Disconnect USB Device
 *    Parameters:      con:   Connect/Disconnect
 *    Return Value:    None
 */

void USBD_Connect (BOOL con) {
  if (con) {
    /* Enable UDP PullUp (USB_DP_PUP)                                         */
    AT91C_BASE_PIOA->PIO_CODR = AT91C_PIO_PA16;
  } else {
    /* Disable UDP PullUp (USB_DP_PUP)                                        */
    AT91C_BASE_PIOA->PIO_SODR = AT91C_PIO_PA16;
  }
}


/*
 *  USB Device Reset Function
 *   Called automatically on USB Device Reset
 *    Return Value:    None
 */

void USBD_Reset (void) {
  U32 ep;

  /* Global USB Device Reset                                                  */
  pUDP->UDP_GLBSTATE  = 0;                        /* Reset global status to 0 */
  pUDP->UDP_FADDR     = AT91C_UDP_FEN;            /* Set address to 0         */
  pUDP->UDP_ICR       = 0xFFFFFFFF;               /* Clear all pending ints   */

  /* Reset & Disable USB Device Endpoints                                     */
  for (ep = 0; ep <= USBD_EP_NUM; ep++) {
    pUDP->UDP_CSR[ep] = 0;
    RxDataBank[ep]    = 0;
    TxDataBank[ep]    = 0;
  }
  pUDP->UDP_RSTEP     = 0xFFFFFFFF;
  pUDP->UDP_RSTEP     = 0;

  /* Setup USB Interrupts                                                     */
#ifdef __RTX
  pUDP->UDP_IER = ((USBD_RTX_DevTask     != 0) ? AT91C_UDP_RXSUSP   : 0) |
                  ((USBD_RTX_DevTask     != 0) ? AT91C_UDP_RXRSM    : 0) |
                  ((USBD_RTX_DevTask     != 0) ? AT91C_UDP_SOFINT   : 0) |
                  ((USBD_RTX_DevTask     != 0) ? AT91C_UDP_WAKEUP   : 0) |
#else
  pUDP->UDP_IER = ((USBD_P_Suspend_Event != 0) ? AT91C_UDP_RXSUSP   : 0) |
                  ((USBD_P_Resume_Event  != 0) ? AT91C_UDP_RXRSM    : 0) |
                  ((USBD_P_SOF_Event     != 0) ? AT91C_UDP_SOFINT   : 0) |
                  ((USBD_P_WakeUp_Event  != 0) ? AT91C_UDP_WAKEUP   : 0) |
#endif
                  ((1 << (USBD_EP_NUM+1))-1);

  /* Setup Control Endpoint 0                                                 */
  USBD_SetCSR(0, AT91C_UDP_EPEDS | AT91C_UDP_EPTYPE_CTRL);
}


/*
 *  USB Device Suspend Function
 *   Called automatically on USB Device Suspend
 *    Return Value:    None
 */

void USBD_Suspend (void) {
  /* Performed by Hardware */
}


/*
 *  USB Device Resume Function
 *   Called automatically on USB Device Resume
 *    Return Value:    None
 */

void USBD_Resume (void) {
  /* Performed by Hardware */
}


/*
 *  USB Device Remote Wakeup Function
 *   Called automatically on USB Device Remote Wakeup
 *    Return Value:    None
 */

void USBD_WakeUp (void) {
  /* Performed by Hardware */
}


/*
 *  USB Device Remote Wakeup Configuration Function
 *    Parameters:      cfg:   Device Enable/Disable
 *    Return Value:    None
 */

void USBD_WakeUpCfg (BOOL cfg) {
  if (cfg) {
    pUDP->UDP_GLBSTATE |=  AT91C_UDP_RMWUPE;
  } else {
    pUDP->UDP_GLBSTATE &= ~AT91C_UDP_RMWUPE;
  }
}


/*
 *  USB Device Set Address Function
 *    Parameters:      adr:   USB Device Address
 *                     setup: Called in setup stage (!=0), else after status stage
 *    Return Value:    None
 */

void USBD_SetAddress (U32 adr, U32 setup) {
  if (setup) return;
  pUDP->UDP_FADDR = AT91C_UDP_FEN | (adr & AT91C_UDP_FADD); /* Set the address*/
  if (adr) {                                      /* If address is non-zero   */
    pUDP->UDP_GLBSTATE |=  AT91C_UDP_FADDEN;      /* Device enters adr state  */
  } else {
    pUDP->UDP_GLBSTATE &= ~AT91C_UDP_FADDEN;      /* Device enters def state  */
  }
}


/*
 *  USB Device Configure Function
 *    Parameters:      cfg:   Device Configure/Deconfigure
 *    Return Value:    None
 */

void USBD_Configure (BOOL cfg) {
  if (cfg) {                                      /* If config is non-zero    */
    pUDP->UDP_GLBSTATE |=  AT91C_UDP_CONFG;       /* Device enters cfg state  */
  } else {
    pUDP->UDP_GLBSTATE &= ~AT91C_UDP_CONFG;       /* Device clears cfg state  */
  }
}


/*
 *  Configure USB Device Endpoint according to Descriptor
 *    Parameters:      pEPD:  Pointer to Device Endpoint Descriptor
 *    Return Value:    None
 */

void USBD_ConfigEP (USB_ENDPOINT_DESCRIPTOR *pEPD) {
  U32 ep, dir, type, csr;

  ep   = pEPD->bEndpointAddress & 0x0F;
  type = pEPD->bmAttributes     & USB_ENDPOINT_TYPE_MASK;
  dir  = pEPD->bEndpointAddress >> 7;
  csr  = ((type | (dir << 2)) << 8);

  /* Check if MaxPacketSize fits for EndPoint                                 */
  if (pEPD->wMaxPacketSize <= USB_GetSizeEP(ep)) {
    USBD_SetCSR(ep, csr);               /* Configure the EP                   */
  }
}


/*
 *  Set Direction for USB Device Control Endpoint
 *    Parameters:      dir:   Out (dir == 0), In (dir <> 0)
 *    Return Value:    None
 */

void USBD_DirCtrlEP (U32 dir) {
  if (dir) {
    USBD_SetCSR(0, AT91C_UDP_DIR);
  } else {
    USBD_ClrCSR(0, AT91C_UDP_DIR);
  }
}


/*
 *  Enable USB Device Endpoint
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_EnableEP (U32 EPNum) {
  EPNum &= 0x0F;                        /* Get endpoint number                */
  pUDP->UDP_IER = (1 << EPNum);         /* Enable EP interrupts               */
  USBD_SetCSR(EPNum, AT91C_UDP_EPEDS);
}


/*
 *  Disable USB Device Endpoint
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_DisableEP (U32 EPNum) {
  EPNum &= 0x0F;                        /* Get endpoint number                */
  USBD_ClrCSR(EPNum, AT91C_UDP_EPEDS);
  pUDP->UDP_IDR = (1 << EPNum);         /* Disable EP interrupts              */
}


/*
 *  Reset USB Device Endpoint
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_ResetEP (U32 EPNum) {
  EPNum &= 0x0F;
  USBD_ClrCSR(EPNum, (AT91C_UDP_TXCOMP      | AT91C_UDP_RXSETUP      |
                      AT91C_UDP_RX_DATA_BK0 | AT91C_UDP_RX_DATA_BK1  |
                      AT91C_UDP_TXPKTRDY    | AT91C_UDP_FORCESTALL   |
                      AT91C_UDP_STALLSENT                             ));

  pUDP->UDP_RSTEP  |=  (1 << EPNum);
  pUDP->UDP_RSTEP  &= ~(1 << EPNum);
  RxDataBank[EPNum] =   0;
  TxDataBank[EPNum] =   0;
}


/*
 *  Set Stall for USB Device Endpoint
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_SetStallEP (U32 EPNum) {
  USBD_SetCSR(EPNum & 0x0F, AT91C_UDP_FORCESTALL);
}


/*
 *  Clear Stall for USB Device Endpoint
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_ClrStallEP (U32 EPNum) {
  USBD_ClrCSR(EPNum & 0x0F, AT91C_UDP_FORCESTALL);
}


/*
 *  Read USB Device Endpoint Data
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *                     pData: Pointer to Data Buffer
 *    Return Value:    Number of bytes read
 */

U32 USBD_ReadEP (U32 EPNum, U8 *pData) {
  U32 cnt, n;
#ifdef __RTX
  U8 *pDat = pData;
#endif

  EPNum &= 0x0F;
  cnt = (pUDP->UDP_CSR[EPNum] >> 16) & 0x07FF;
  for (n = 0; n < cnt; n++) {           /* Read data                          */
    *pData++ = (U8)pUDP->UDP_FDR[EPNum];
  }
  USBD_ClrCSR(EPNum, RX_DATA_BK[RxDataBank[EPNum]]);
  if (DualBankEP & (1 << EPNum)) {
    RxDataBank[EPNum] ^= 1;
  }
#ifdef __RTX
  /* Leave RXSETUP bit and interrupt disabled if next packet is IN and we  
     need to send data                                                        */
  if (EPNum || !((pUDP->UDP_CSR[0] & AT91C_UDP_RXSETUP) && (pDat[0] & 0x80))) {
    USBD_ClrCSR(EPNum, AT91C_UDP_RXSETUP);
    pUDP->UDP_IER = (1 << EPNum);       /* Reenable EP int                    */
  }
#else
  USBD_ClrCSR(EPNum, AT91C_UDP_RXSETUP);
#endif

  return (cnt);
}


/*
 *  Write USB Device Endpoint Data
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *                     pData: Pointer to Data Buffer
 *                     cnt:   Number of bytes to write
 *    Return Value:    Number of bytes written
 */

U32 USBD_WriteEP (U32 EPNum, U8 *pData, U32 cnt) {
  U32 n;

  EPNum &= 0x0F;
  if (pUDP->UDP_CSR[EPNum] & AT91C_UDP_FORCESTALL) {  /* If stalled don't send*/
    return (cnt);
  }

  if (TxDataBank[EPNum])                /* If hardware not available to send  */
    return (0);

  if (pUDP->UDP_CSR[EPNum] & AT91C_UDP_TXPKTRDY) {
    if ((DualBankEP & (1 << EPNum)) && (TxDataBank[EPNum] == 0)) {
      TxDataBank[EPNum] = 1;
    } else {
      return (0);
    }
  }
  USBD_ClrCSR(EPNum, AT91C_UDP_RXSETUP);
  for (n = 0; n < cnt; n++) {           /* Write data                         */
    pUDP->UDP_FDR[EPNum] = *pData++;
  }
#ifdef __RTX
  pUDP->UDP_IER = (1 << EPNum);         /* Reenable EP int                    */
#endif
  USBD_SetCSR(EPNum, AT91C_UDP_TXPKTRDY);

  return (cnt);
}


/*
 *  Get USB Device Last Frame Number
 *    Parameters:      None
 *    Return Value:    Frame Number
 */

U32 USBD_GetFrame (void) {
  U32 val;

  while ((pUDP->UDP_NUM & (AT91C_UDP_FRM_OK | AT91C_UDP_FRM_ERR)) == 0);
  if (pUDP->UDP_NUM & AT91C_UDP_FRM_OK) {
    val = pUDP->UDP_NUM & AT91C_UDP_FRM_NUM;
  } else {
    val = 0xFFFFFFFF;
  }

  return (val);
}


#ifdef __RTX
U32 LastError;                          /* Last Error                         */

/*
 *  Get USB Device Last Error Code
 *    Parameters:      None
 *    Return Value:    Error Code
 */

U32 USBD_GetError (void) {
  return (LastError);
}
#endif


/*
 *  USB Device Interrupt Service Routine
 */

void USBD_ISR (void) __irq {
  U32 isr, csr, bkm, n;

  do {
    isr = pUDP->UDP_ISR & pUDP->UDP_IMR;

    /* End of Bus Reset Interrupt                                             */
    if (isr & AT91C_UDP_ENDBUSRES) {
      USBD_Reset();
      usbd_reset_core();
#ifdef __RTX
      if (USBD_RTX_DevTask) {
        isr_evt_set(USBD_EVT_RESET, USBD_RTX_DevTask);
      }
#else
      if (USBD_P_Reset_Event) {
        USBD_P_Reset_Event();
      }
#endif
      pUDP->UDP_ICR = AT91C_UDP_ENDBUSRES;
    }

    /* USB Suspend Interrupt                                                  */
    if (isr & AT91C_UDP_RXSUSP) {
      USBD_Suspend();
#ifdef __RTX
      if (USBD_RTX_DevTask) {
        isr_evt_set(USBD_EVT_SUSPEND, USBD_RTX_DevTask);
      }
#else
      if (USBD_P_Suspend_Event) {
        USBD_P_Suspend_Event();
      }
#endif
      pUDP->UDP_ICR = AT91C_UDP_RXSUSP;
    }

    /* USB Resume Interrupt                                                   */
    if (isr & AT91C_UDP_RXRSM) {
      USBD_Resume();
#ifdef __RTX
      if (USBD_RTX_DevTask) {
        isr_evt_set(USBD_EVT_RESUME,  USBD_RTX_DevTask);
      }
#else
      if (USBD_P_Resume_Event) {
        USBD_P_Resume_Event();
      }
#endif
      pUDP->UDP_ICR = AT91C_UDP_RXRSM;
    }

    /* Start of Frame Interrupt                                               */
    if (isr & AT91C_UDP_SOFINT) {
#ifdef __RTX
      if (USBD_RTX_DevTask) {
        isr_evt_set(USBD_EVT_SOF, USBD_RTX_DevTask);
      }
#else
      if (USBD_P_SOF_Event) {
        USBD_P_SOF_Event();
      }
#endif
      pUDP->UDP_ICR = AT91C_UDP_SOFINT;
    }

    /* Endpoint Interrupts                                                    */
    for (n = 0; n <= USBD_EP_NUM; n++) {
      if (isr & (1 << n)) {
        csr = pUDP->UDP_CSR[n];

        /* Data Packet Sent Interrupt                                         */
        if (csr & AT91C_UDP_TXCOMP) {
          USBD_ClrCSR(n, AT91C_UDP_TXCOMP);
#ifdef __RTX
          if (USBD_RTX_EPTask[n]) {       /* IN Packet                        */
            isr_evt_set(USBD_EVT_IN,  USBD_RTX_EPTask[n]);
          }
#else
          if (USBD_P_EP[n]) {
            USBD_P_EP[n](USBD_EVT_IN);
          }
#endif
        }

        /* Data Packet Received Interrupt                                     */
        bkm = RX_DATA_BK[RxDataBank[n]];
        if (csr & bkm) {
#ifdef __RTX
          if (USBD_RTX_EPTask[n]) {     /* OUT Packet                         */
            isr_evt_set(USBD_EVT_OUT, USBD_RTX_EPTask[n]);
            pUDP->UDP_IDR = (1 << n);   /* Disable EP int until read          */
          }
#else
          if (USBD_P_EP[n]) {
            USBD_P_EP[n](USBD_EVT_OUT);
          }
#endif
        }

        /* STALL Packet Sent Interrupt                                        */
        if (csr & AT91C_UDP_STALLSENT) {
          if ((csr & AT91C_UDP_EPTYPE) == AT91C_UDP_EPTYPE_CTRL) {
#ifdef __RTX
            if (USBD_RTX_EPTask[n]) {
              isr_evt_set(USBD_EVT_IN_STALL, USBD_RTX_EPTask[n]);
            }
#else
            if (USBD_P_EP[n]) {
              USBD_P_EP[n](USBD_EVT_IN_STALL);
            }
#endif
          }
          USBD_ClrCSR(n, AT91C_UDP_STALLSENT);
        }

        /* Setup Packet Received Interrupt                                    */
        if (csr & AT91C_UDP_RXSETUP) {
#ifdef __RTX
          if (USBD_RTX_EPTask[n]) {     /* SETUP Packet                       */
            isr_evt_set(USBD_EVT_SETUP, USBD_RTX_EPTask[n]);
            pUDP->UDP_IDR = (1 << n);   /* Disable EP int until read          */
          }
#else
          if (USBD_P_EP[n]) {
            USBD_P_EP[n](USBD_EVT_SETUP);
          }
#endif
        }
      }
    }
  } while (isr);

  *AT91C_AIC_EOICR = 0;                 /* End of Interrupt                   */
}
