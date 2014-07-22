/*----------------------------------------------------------------------------
 *      RL-ARM - CAN
 *----------------------------------------------------------------------------
 *      Name:    CAN_LPC29xx.c
 *      Purpose: CAN Driver, Hardware specific module for NXP LPC29xx
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <RTL.h>                      /* RTX kernel functions & defines      */
#include <RTX_CAN.h>                  /* CAN Generic functions & defines     */
#include <LPC29xx.h>                  /* LPC29xx definitions                 */
#include "CAN_reg.h"                  /* LPC29xx CAN registers               */

#pragma diag_suppress 550

/* Values of bit time register for different baudrates
   NT = Nominal bit time = TSEG1 + TSEG2 + 3
   SP = Sample point     = ((1+TSEG1+1)/(1+TSEG1+1+TSEG2+1)) * 100%
                                            SAM,  SJW, TSEG1, TSEG2, NT,  SP */
const U32 CAN_BIT_TIME[] = {          0, /*             not used             */
                                      0, /*             not used             */
                                      0, /*             not used             */
                                      0, /*             not used             */
                             0x0001C000, /* 0+1,  3+1,   1+1,   0+1,  4, 75% */
                                      0, /*             not used             */
                             0x0012C000, /* 0+1,  3+1,   2+1,   1+1,  6, 67% */
                                      0, /*             not used             */
                             0x0023C000, /* 0+1,  3+1,   3+1,   2+1,  8, 63% */
                                      0, /*             not used             */
                             0x0025C000, /* 0+1,  3+1,   5+1,   2+1, 10, 70% */
                                      0, /*             not used             */
                             0x0036C000, /* 0+1,  3+1,   6+1,   3+1, 12, 67% */
                                      0, /*             not used             */
                                      0, /*             not used             */
                             0x0048C000, /* 0+1,  3+1,   8+1,   4+1, 15, 67% */
                             0x0049C000, /* 0+1,  3+1,   9+1,   4+1, 16, 69% */
                           };


/************************* CAN Hardware Configuration ************************/

// *** <<< Use Configuration Wizard in Context Menu >>> ***

// <o> CAN Peripheral Clock (in Hz) <1-1000000000>
//     <i> Default: 80000000 (= system clock)                                    
#define PCLK             80000000

// *** <<< End of Configuration section             >>> ***


/*----------------------------------------------------------------------------
 *      CAN RTX Hardware Specific Driver Functions
 *----------------------------------------------------------------------------
 *  Functions implemented in this module:
 *    static CAN_ERROR CAN_hw_set_baudrate (U32 ctrl, U32 baudrate)
 *           CAN_ERROR CAN_hw_setup        (U32 ctrl)
 *           CAN_ERROR CAN_hw_init         (U32 ctrl, U32 baudrate)
 *           CAN_ERROR CAN_hw_start        (U32 ctrl)
 *           CAN_ERROR CAN_hw_tx_empty     (U32 ctrl)
 *           CAN_ERROR CAN_hw_wr           (U32 ctrl, CAN_msg *msg)
 *    static      void CAN_hw_rd           (U32 ctrl, CAN_msg *msg)
 *           CAN_ERROR CAN_hw_set          (U32 ctrl, CAN_msg *msg)
 *           CAN_ERROR CAN_hw_rx_object    (U32 ctrl, U32 ch, U32 id, U32 object_para)
 *           CAN_ERROR CAN_hw_tx_object    (U32 ctrl, U32 ch,         U32 object_para)
 *    Interrupt fuctions
 *---------------------------------------------------------------------------*/

/* Static functions used only in this module                                 */
static void CAN_hw_rd          (U32 ctrl, CAN_msg *msg);
#if USE_CAN_CTRL1 == 1
  __irq void CAN0_Tx_Handler  (void);
  __irq void CAN0_Rx_Handler  (void);
#endif
#if USE_CAN_CTRL2 == 1
  __irq void CAN1_Tx_Handler  (void);
  __irq void CAN1_Rx_Handler  (void);
#endif


/************************* Auxiliary Functions *******************************/

/*------------------------ CAN_hw_set_baudrate ------------------------------
 *
 *  Setup the requested baudrate
 *
 *  Parameter:  ctrl:       Index of the hardware CAN controller (1 .. x)
 *              baudrate:   Baudrate
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

static CAN_ERROR CAN_hw_set_baudrate (U32 ctrl, U32 baudrate)  {
  U32 ctrl0 = ctrl-1;                 /* Controller index 0 .. x-1           */
  regCAN    *ptrcan    = (regCAN *) CAN_BASE[ctrl0];
  U32 result = 0;
  U32 nominal_time;

  /* Determine which nominal time to use for baudrate according to PCLK      */
  nominal_time = 16;

  /* Prepare value appropriate for bit time register                         */
  result  = (PCLK / nominal_time) / baudrate - 1;
  result &= 0x000003FF;
  result |= CAN_BIT_TIME[nominal_time];

  ptrcan->CCBT = result;              /* Set bit timing                      */

  return CAN_OK;
}


/*************************** Module Functions ********************************/

/*--------------------------- CAN_hw_setup ----------------------------------
 *
 *  Setup CAN transmit and receive PINs and interrupt vectors
 *
 *  Parameter:  ctrl:       Index of the hardware CAN controller (1 .. x)
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_setup (U32 ctrl)  {

  /* Configure CAN/LIN clock = PLL clock / 2 = 80 MHz                        */
  IVNSS_CLK_CONF = (2UL << 24) | (1 << 11) | (1 << 2);

  switch (ctrl) {
    case 1: 
      #if USE_CAN_CTRL1 == 1
        SFSP0_0  = 0x06;              /* Configure P0.0  as hw CAN0 TxD      */
        SFSP0_1  = 0x06;              /* Configure P0.1  as hw CAN0 RxD      */
    
        /* Set interrupt vector for hw CAN0 Tx                               */
        INT_REQUEST_43 = (1 << 28) |  /* Enable setting of priority level    */
                         (1 << 27) |  /* Enable setting interrupt target     */
                         (1 << 26) |  /* Write enable of new settings        */
                         (1 << 16) |  /* Enable interrupt request            */
                         (1 <<  8) |  /* Interrupt target is IRQ interrupt   */
                         (15 << 0) ;  /* Priority level 15, disable nesting  */
        /* Set interrupt vector for hw CAN0 Rx                               */
        INT_REQUEST_37 = (1 << 28) |  /* Enable setting of priority level    */
                         (1 << 27) |  /* Enable setting interrupt target     */
                         (1 << 26) |  /* Write enable of new settings        */
                         (1 << 16) |  /* Enable interrupt request            */
                         (1 <<  8) |  /* Interrupt target is IRQ interrupt   */
                         (15 << 0) ;  /* Priority level 15, disable nesting  */
      #endif
      break;
    case 2: 
      #if USE_CAN_CTRL2 == 1
        SFSP0_24 = 0x06;              /* Configure P0.24 as hw CAN1 TxD      */
        SFSP0_25 = 0x06;              /* Configure P0.25 as hw CAN1 RxD      */
    
        /* Set interrupt vector for hw CAN1 Tx                               */
        INT_REQUEST_44 = (1 << 28) |  /* Enable setting of priority level    */
                         (1 << 27) |  /* Enable setting interrupt target     */
                         (1 << 26) |  /* Write enable of new settings        */
                         (1 << 16) |  /* Enable interrupt request            */
                         (1 <<  8) |  /* Interrupt target is IRQ interrupt   */
                         (15 << 0) ;  /* Priority level 15, disable nesting  */
        /* Set interrupt vector for hw CAN1 Rx                               */
        INT_REQUEST_38 = (1 << 28) |  /* Enable setting of priority level    */
                         (1 << 27) |  /* Enable setting interrupt target     */
                         (1 << 26) |  /* Write enable of new settings        */
                         (1 << 16) |  /* Enable interrupt request            */
                         (1 <<  8) |  /* Interrupt target is IRQ interrupt   */
                         (15 << 0) ;  /* Priority level 15, disable nesting  */
      #endif
      break;
    default:
      return CAN_UNEXIST_CTRL_ERROR;
  }

  return CAN_OK;
}


/*--------------------------- CAN_hw_init -----------------------------------
 *
 *  Initialize the CAN hardware
 *
 *  Parameter:  ctrl:       Index of the hardware CAN controller (1 .. x)
 *              baudrate:   Baudrate
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_init (U32 ctrl, U32 baudrate)  {
  U32 ctrl0 = ctrl-1;                 /* Controller index 0 .. x-1           */
  regCAN_AFR *ptrcan_afr = (regCAN_AFR *) CAN_AFR_BASE;
  regCAN     *ptrcan     = (regCAN *)     CAN_BASE[ctrl0];

  ptrcan_afr->CAMODE = 1;             /* Disable acceptance filter           */
  ptrcan->CCMODE     = 1;             /* Enter reset mode                    */
  ptrcan->CCIE       = 0;             /* Disable all interrupts              */
  ptrcan->CCGS       = 0;             /* Clear status register               */
  CAN_hw_set_baudrate(ctrl, baudrate);/* Set bit timing                      */
  ptrcan->CCIE       = 0x0603;        /* Enable Tx and Rx interrupt          */
      
  return CAN_OK;
}


/*--------------------------- CAN_hw_start ----------------------------------
 *
 *  Enable the CAN interrupts (recive or/and transmit)
 *
 *  Parameter:  ctrl:       Index of the hardware CAN controller (1 .. x)
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_start (U32 ctrl)  {
  U32 ctrl0 = ctrl-1;                 /* Controller index 0 .. x-1           */
  regCAN *ptrcan = (regCAN *) CAN_BASE[ctrl0];

  ptrcan->CCMODE = 0;                 /* Enter normal operating mode         */

  return CAN_OK;
}


/*--------------------------- CAN_hw_tx_empty -------------------------------
 *
 *  Check if controller is available for transmission
 *
 *  Parameter:  ctrl:       Index of the hardware CAN controller (1 .. x)
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_tx_empty (U32 ctrl)  {
  U32 ctrl0 = ctrl-1;                 /* Controller index 0 .. x-1           */
  regCAN *ptrcan = (regCAN *) CAN_BASE[ctrl0];

  if ((os_sem_wait (wr_sem[ctrl-1], 0) != OS_R_TMO)){ /* If semaphore is free*/
    if (ptrcan->CCSTAT & (1 << 2))    /* Transmitter ready for transmission  */
      return CAN_OK;
    else 
      os_sem_send(wr_sem[ctrl-1]);    /* Return a token back to semaphore    */
  }

  return CAN_TX_BUSY_ERROR;
}


/*--------------------------- CAN_hw_wr -------------------------------------
 *
 *  Write CAN_msg to the hardware registers of the requested controller
 *
 *  Parameter:  ctrl:       Index of the hardware CAN controller (1 .. x)
 *              msg:        Pointer to CAN message to be written to hardware
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_wr (U32 ctrl, CAN_msg *msg)  {
  U32 ctrl0 = ctrl-1;                 /* Controller index 0 .. x-1           */
  U32 CANData;
  regCAN *ptrcan = (regCAN *) CAN_BASE[ctrl0];

  CANData       = (((U32) msg->len) << 16)          & 0x000F0000 | 
                  (msg->format == EXTENDED_FORMAT ) * 0x80000000 |
                  (msg->type   == REMOTE_FRAME)     * 0x40000000;

  if (ptrcan->CCSTAT & (1 << 2)) {    /* Transmit buffer 1 free              */
    ptrcan->CCTXB1MI = CANData;       /* Write frame informations            */
    ptrcan->CCTXB1ID = msg->id;       /* Write CAN message identifier        */
    ptrcan->CCTXB1DA = *(U32 *) &msg->data[0];/* Write first 4 data bytes    */
    ptrcan->CCTXB1DB = *(U32 *) &msg->data[4];/* Write second 4 data bytes   */
    ptrcan->CCCMD    = 0x21;          /* Start transmission without loopback */
  }
  else
    return CAN_TX_BUSY_ERROR;

  return CAN_OK;
}


/*--------------------------- CAN_hw_rd -------------------------------------
 *
 *  Read CAN_msg from the hardware registers of the requested controller
 *
 *  Parameter:  ctrl:       Index of the hardware CAN controller (1 .. x)
 *              msg:        Pointer where CAN message will be read
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

static void CAN_hw_rd (U32 ctrl, CAN_msg *msg)  {
  U32 ctrl0 = ctrl-1;                 /* Controller index 0 .. x-1           */
  U32 CANData;
  U32 *CANAddr;
  regCAN *ptrcan = (regCAN *) CAN_BASE[ctrl0];

  /* Read frame informations                                                 */
  CANData       = ptrcan->CCRXBMI;
  msg->format   = (CANData & 0x80000000) == 0x80000000;
  msg->type     = (CANData & 0x40000000) == 0x40000000;
  msg->len      = ((U8)(CANData >> 16)) & 0x0F;

  /* Read CAN message identifier                                             */
  msg->id       = ptrcan->CCRXBID;

  /* Read the data if received message was DATA FRAME                        */
  if (msg->type == DATA_FRAME)  {     

    /* Read first 4 data bytes                                               */
    CANAddr = (U32 *) &msg->data[0];
    *CANAddr++  = ptrcan->CCCRXBDA;

    /* Read second 4 data bytes                                              */
    *CANAddr    = ptrcan->CCCRXBDB;
  }
}


/*--------------------------- CAN_hw_set ------------------------------------
 *  Set a message that will automatically be sent as an answer to the REMOTE
 *  FRAME message, as this functionality is not enabled by hardware this 
 *  function is not implemented
 *
 *  Parameter:  ctrl:       Ignored
 *              msg:        Pointer to CAN message to be set
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_set (U32 ctrl, CAN_msg *msg)  {

  return CAN_NOT_IMPLEMENTED_ERROR;
}


/*--------------------------- CAN_hw_rx_object ------------------------------
 *
 *  Enable reception of messages, on specified controller with specified 
 *  identifier, by setting acceptance filter
 *
 *  Parameter:  ctrl:       Index of the hardware CAN controller (1 .. x)
 *              ch:         Ignored for LPC2xxx
 *              id:         CAN message identifier
 *              object_para:Object parameters (standard or extended format, 
 *                          data or remote frame)
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_rx_object (U32 ctrl, U32 ch, U32 id, U32 object_para)  {
  static S16 CAN_std_cnt = 0;
  static S16 CAN_ext_cnt = 0;
  regCAN_AFR *ptrcan_afr = (regCAN_AFR *) CAN_AFR_BASE;
  regCAN_AFM *ptrcan_afm = (regCAN_AFM *) CAN_AFM_BASE;
  U32 buf0, buf1;
  S16 cnt1, cnt2, bound1;

  ctrl -= 1;

  /* Acceptance Filter Memory full                                           */
  if ((((CAN_std_cnt + 1) >> 1) + CAN_ext_cnt) >= 512)
    return CAN_OBJECTS_FULL_ERROR;

  /* Setup Acceptance Filter Configuration                                   
     Acceptance Filter Mode Register = Off                                   */
  ptrcan_afr->CAMODE = 1;

  if ((object_para & FORMAT_TYPE) == STANDARD_TYPE)  { /* Add mask for std id*/
    id |= ctrl << 13;                 /* Add controller number               */
    id &= 0x0000F7FF;                 /* Mask out 16-bits of ID              */

    /* Move all remaining extended mask entries one place up                 
       if new entry will increase standard ID filters list                   */
    if ((CAN_std_cnt & 0x0001) == 0 && CAN_ext_cnt != 0) {
      cnt1   = (CAN_std_cnt >> 1);
      bound1 = CAN_ext_cnt;
      buf0   = ptrcan_afm->mask[cnt1];
      while (bound1--)  {
        cnt1++;
        buf1 = ptrcan_afm->mask[cnt1];
        ptrcan_afm->mask[cnt1] = buf0;
        buf0 = buf1;
      }        
    }

    if (CAN_std_cnt == 0)  {          /* For entering first  ID              */
      ptrcan_afm->mask[0] = 0x0000FFFF | (id << 16);
    }  else if (CAN_std_cnt == 1)  {  /* For entering second ID              */
      if ((ptrcan_afm->mask[0] >> 16) > id)
        ptrcan_afm->mask[0] = (ptrcan_afm->mask[0] >> 16) | (id << 16);
      else
        ptrcan_afm->mask[0] = (ptrcan_afm->mask[0] & 0xFFFF0000) | id;
    }  else  {
      /* Find where to insert new ID                                         */
      cnt1 = 0;
      cnt2 = CAN_std_cnt;
      bound1 = (CAN_std_cnt - 1) >> 1;
      while (cnt1 <= bound1)  {       /* Loop through standard existing IDs  */
        if ((ptrcan_afm->mask[cnt1] >> 16) > id)  {
          cnt2 = cnt1 * 2;
          break;
        }
        if ((ptrcan_afm->mask[cnt1] & 0x0000FFFF) > id)  {
          cnt2 = cnt1 * 2 + 1;
          break;
        }
        cnt1++;                       /* cnt1 = U32 where to insert new ID   */
      }                               /* cnt2 = U16 where to insert new ID   */

      if (cnt1 > bound1)  {           /* Adding ID as last entry             */
        if ((CAN_std_cnt & 0x0001) == 0)/* Even number of IDs exists         */
          ptrcan_afm->mask[cnt1]  = 0x0000FFFF | (id << 16);
        else                              /* Odd  number of IDs exists       */
          ptrcan_afm->mask[cnt1]  = (ptrcan_afm->mask[cnt1] & 0xFFFF0000) | id;
      }  else  {
        buf0 = ptrcan_afm->mask[cnt1];/* Remember current entry            */
        if ((cnt2 & 0x0001) == 0)     /* Insert new mask to even address     */
          buf1 = (id << 16) | (buf0 >> 16);
        else                          /* Insert new mask to odd  address     */
          buf1 = (buf0 & 0xFFFF0000) | id;
     
        ptrcan_afm->mask[cnt1] = buf1;/* Insert mask                       */

        bound1 = CAN_std_cnt >> 1;
        /* Move all remaining standard mask entries one place up             */
        while (cnt1 < bound1)  {
          cnt1++;
          buf1  = ptrcan_afm->mask[cnt1];
          ptrcan_afm->mask[cnt1] = (buf1 >> 16) | (buf0 << 16);
          buf0  = buf1;
        }

        if ((CAN_std_cnt & 0x0001) == 0)/* Even number of IDs exists         */
          ptrcan_afm->mask[cnt1] = (ptrcan_afm->mask[cnt1] & 0xFFFF0000) | (0x0000FFFF);
      }
    }
    CAN_std_cnt++;
  }  else  {                          /* Add mask for extended identifiers   */
    id |= (ctrl) << 29;               /* Add controller number               */

    cnt1 = ((CAN_std_cnt + 1) >> 1);
    cnt2 = 0;
    while (cnt2 < CAN_ext_cnt)  {     /* Loop through extended existing masks*/
      if (ptrcan_afm->mask[cnt1] > id)
        break;
      cnt1++;                         /* cnt1 = U32 where to insert new mask */
      cnt2++;
    }

    buf0 = ptrcan_afm->mask[cnt1];  /* Remember current entry              */
    ptrcan_afm->mask[cnt1] = id;    /* Insert mask                         */

    CAN_ext_cnt++;

    bound1 = CAN_ext_cnt - 1;
    /* Move all remaining extended mask entries one place up                 */
    while (cnt2 < bound1)  {
      cnt1++;
      cnt2++;
      buf1 = ptrcan_afm->mask[cnt1];
      ptrcan_afm->mask[cnt1] = buf0;
      buf0 = buf1;
    }        
  }
  
  /* Calculate std ID start address (buf0) and ext ID start address (buf1)   */
  buf0 = ((CAN_std_cnt + 1) >> 1) << 2;
  buf1 = buf0 + (CAN_ext_cnt << 2);

  /* Setup acceptance filter pointers                                        */
  ptrcan_afr->CASFESA = 0;
  ptrcan_afr->CASFGSA = buf0;
  ptrcan_afr->CAEFESA = buf0;
  ptrcan_afr->CAEFGSA = buf1;
  ptrcan_afr->CAEOTA  = buf1;

  ptrcan_afr->CAMODE  = 0;            /* Use acceptance filter               */

  return CAN_OK;
}

/*--------------------------- CAN_hw_tx_object ------------------------------
 *
 *  This function has no usage on LPC2xxx, and so it does nothing
 *
 *  Parameter:  ctrl:       Index of the hardware CAN controller (1 .. x)
 *              ch:         Ignored for LPC2xxx
 *              id:         CAN message identifier
 *              object_para:Object parameters (standard or extended format, 
 *                          data or remote frame)
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_tx_object (U32 ctrl, U32 ch, U32 object_para)  {

  return CAN_OK;
}


/************************* Interrupt Functions *******************************/

#if USE_CAN_CTRL1 == 1

#define CTRL0                 0
#define CTRL                  1
#define ptrCANc               ((regCAN *)CAN1_BASE)

/*--------------------------- CAN0_Tx_Handler ------------------------------
 *
 *  CAN transmit interrupt function for controller 1 (hw CAN0)
 *  If there are messages in mailbox for transmit it writes it to hardware
 *  and starts the transmission on controller 1 (hw CAN0)
 *
 *  Parameter:  none
 *
 *  Return:     none
 *---------------------------------------------------------------------------*/

__irq void CAN0_Tx_Handler (void)  {
  CAN_msg *ptrmsg;

  /* If there is message in mailbox ready for send, 
     read the message from mailbox and send it                               */
  if (isr_mbx_receive (MBX_tx_ctrl[CTRL0], (void **)&ptrmsg) != OS_R_OK) {
    CAN_hw_wr (CTRL, ptrmsg);
    _free_box(CAN_mpool, ptrmsg);
  } else {
    isr_sem_send(wr_sem[CTRL0]);      /* Return a token back to semaphore    */
  }

  /* Read from interrupt register to acknowledge interrupt                   */
  ptrCANc->CCIC;
}


/*--------------------------- CAN0_Rx_Handler ------------------------------
 *
 *  CAN receive interrupt function, for CAN controller 1 (hw CAN0)
 *  Reads message from hardware registers and puts it into receive mailbox
 *  for controller 1 (hw CAN0)
 *
 *  Parameter:  none
 *
 *  Return:     none
 *---------------------------------------------------------------------------*/

__irq void CAN0_Rx_Handler (void)  {
  CAN_msg *ptrmsg;

  /* If mailbox isn't full read message from hardware and send it to message 
     queue                                                                   */
  if (os_mbx_check (MBX_rx_ctrl[CTRL0]) > 0) {
    ptrmsg = _alloc_box (CAN_mpool);
    if (ptrmsg) {
      CAN_hw_rd (CTRL, ptrmsg);
      isr_mbx_send (MBX_rx_ctrl[CTRL0], ptrmsg);
    }
  }

  ptrCANc->CCCMD = 0x04;              /* Release receive buffer              */
}

#endif


#if USE_CAN_CTRL2 == 1

#undef  CTRL0
#define CTRL0                 1
#undef  CTRL
#define CTRL                  2
#undef  ptrCANc
#define ptrCANc               ((regCAN *)CAN2_BASE)

/*--------------------------- CAN1_Tx_Handler ------------------------------
 *
 *  CAN transmit interrupt function for controller 2 (hw CAN1)
 *  If there are messages in mailbox for transmit it writes it to hardware
 *  and starts the transmission on controller 2 (hw CAN1)
 *
 *  Parameter:  none
 *
 *  Return:     none
 *---------------------------------------------------------------------------*/

__irq void CAN1_Tx_Handler (void)  {
  CAN_msg *ptrmsg;

  /* If there is message in mailbox ready for send, 
     read the message from mailbox and send it                               */
  if (isr_mbx_receive (MBX_tx_ctrl[CTRL0], (void **)&ptrmsg) != OS_R_OK) {
    CAN_hw_wr (CTRL, ptrmsg);
    _free_box(CAN_mpool, ptrmsg);
  } else {
    isr_sem_send(wr_sem[CTRL0]);      /* Return a token back to semaphore    */
  }

  /* Read from interrupt register to acknowledge interrupt                   */
  ptrCANc->CCIC;
}


/*--------------------------- CAN1_Rx_Handler ------------------------------
 *
 *  CAN receive interrupt function, for CAN controller 2 (hw CAN1)
 *  Reads message from hardware registers and puts it into receive mailbox
 *  for controller 2 (hw CAN1)
 *
 *  Parameter:  none
 *
 *  Return:     none
 *---------------------------------------------------------------------------*/

__irq void CAN1_Rx_Handler (void)  {
  CAN_msg *ptrmsg;

  /* If mailbox isn't full read message from hardware and send it to message 
     queue                                                                   */
  if (os_mbx_check (MBX_rx_ctrl[CTRL0]) > 0) {
    ptrmsg = _alloc_box (CAN_mpool);
    if (ptrmsg) {
      CAN_hw_rd (CTRL, ptrmsg);
      isr_mbx_send (MBX_rx_ctrl[CTRL0], ptrmsg);
    }
  }

  ptrCANc->CCCMD = 0x04;              /* Release receive buffer              */
}


#endif


/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/

