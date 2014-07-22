/*----------------------------------------------------------------------------
 *      RL-ARM - CAN
 *----------------------------------------------------------------------------
 *      Name:    CAN_LPC17xx.c
 *      Purpose: CAN Driver, Hardware specific module for NXP LPC17xx
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <RTL.h>                      /* RTX kernel functions & defines      */
#include <RTX_CAN.h>                  /* CAN Generic functions & defines     */
#include <LPC17xx.h>                  /* LPC17xx definitions                 */

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

// <o> PCLK value (in Hz) <1-1000000000>
//     <i> Peripheral clock, depends on VPBDIV
//     <i> Default: 25000000
#define PCLK             25000000

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
  LPC_CAN_TypeDef *pCAN = (ctrl == 1) ? LPC_CAN1 : LPC_CAN2;  /* CAN ctrl    */
  U32 result = 0;
  U32 nominal_time;

  /* Determine which nominal time to use for PCLK */
  if (((PCLK / 1000000) % 6) == 0) {
    nominal_time = 12;                   /* PCLK based on  72MHz CCLK */
  } else {
    nominal_time = 10;                   /* PCLK based on 100MHz CCLK */
  }

  /* Prepare value appropriate for bit time register                         */
  result  = (PCLK / nominal_time) / baudrate - 1;
  result &= 0x000003FF;
  result |= CAN_BIT_TIME[nominal_time];

  pCAN->BTR  = result;                           /* Set bit timing           */

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
  switch (ctrl) {
    case 1: 
      #if USE_CAN_CTRL1 == 1
        LPC_SC->PCONP       |=  (1 << 13);    /* Enable power to CAN1 block  */
        LPC_PINCON->PINSEL0 |=  (1 <<  0);    /* Pin P0.0 used as RD1 (CAN1) */
        LPC_PINCON->PINSEL0 |=  (1 <<  2);    /* Pin P0.1 used as TD1 (CAN1) */
    
        NVIC_EnableIRQ(CAN_IRQn);             /* Enable CAN interrupt        */
      #endif
      break;
    case 2: 
      #if USE_CAN_CTRL2 == 1
        LPC_SC->PCONP       |=  (1 << 14);    /* Enable power to CAN2 block  */
        LPC_PINCON->PINSEL4 |=  (1 << 14);    /* Pin P2.7 used as RD2 (CAN2) */
        LPC_PINCON->PINSEL4 |=  (1 << 16);    /* Pin P2.8 used as TD2 (CAN2) */
    
        NVIC_EnableIRQ(CAN_IRQn);             /* Enable CAN interrupt        */
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
  LPC_CAN_TypeDef *pCAN = (ctrl == 1) ? LPC_CAN1 : LPC_CAN2;  /* CAN ctrl    */

  LPC_CANAF->AFMR = 2;                /* By default filter is not used       */
  pCAN->MOD       = 1;                /* Enter reset mode                    */
  pCAN->IER       = 0;                /* Disable all interrupts              */
  pCAN->GSR       = 0;                /* Clear status register               */
  CAN_hw_set_baudrate(ctrl, baudrate);/* Set bit timing                      */
  pCAN->IER       = 0x0003;           /* Enable Tx and Rx interrupt          */
      
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
  LPC_CAN_TypeDef *pCAN = (ctrl == 1) ? LPC_CAN1 : LPC_CAN2;  /* CAN ctrl    */

  pCAN->MOD = 0;                      /* Enter normal operating mode         */

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
  LPC_CAN_TypeDef *pCAN = (ctrl == 1) ? LPC_CAN1 : LPC_CAN2;  /* CAN ctrl    */

  if ((os_sem_wait (wr_sem[ctrl-1], 0) != OS_R_TMO)){ /* If semaphore is free*/
    if (pCAN->SR & 0x00000004)                    /* Transmitter ready for   */
                                                  /* transmission            */
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
  LPC_CAN_TypeDef *pCAN = (ctrl == 1) ? LPC_CAN1 : LPC_CAN2;  /* CAN ctrl    */
  U32 CANData;

  CANData       = (((U32) msg->len) << 16)          & 0x000F0000 | 
                  (msg->format == EXTENDED_FORMAT ) * 0x80000000 |
                  (msg->type   == REMOTE_FRAME)     * 0x40000000;

  if (pCAN->SR & 0x00000004)  {       /* Transmit buffer 1 free              */
    pCAN->TFI1  = CANData;            /* Write frame informations            */
    pCAN->TID1 = msg->id;             /* Write CAN message identifier        */
    pCAN->TDA1 = *(U32 *) &msg->data[0];      /* Write first 4 data bytes    */
    pCAN->TDB1 = *(U32 *) &msg->data[4];      /* Write second 4 data bytes   */
    //pCAN->CMR  = 0x31;              /* Select Tx1 for Self Tx/Rx           */
    pCAN->CMR  = 0x21;                /* Start transmission without loop-back*/
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
  LPC_CAN_TypeDef *pCAN = (ctrl == 1) ? LPC_CAN1 : LPC_CAN2;  /* CAN ctrl    */
  U32 CANData;
  U32 *CANAddr;

  /* Read frame informations                                                 */
  CANData = pCAN->RFS;
  msg->format   = (CANData & 0x80000000) == 0x80000000;
  msg->type     = (CANData & 0x40000000) == 0x40000000;
  msg->len      = ((U8)(CANData >> 16)) & 0x0F;

  /* Read CAN message identifier                                             */
  msg->id = pCAN->RID;

  /* Read the data if received message was DATA FRAME                        */
  if (msg->type == DATA_FRAME)  {     

    /* Read first 4 data bytes                                               */
    CANAddr = (U32 *) &msg->data[0];
    *CANAddr++ = pCAN->RDA;

    /* Read second 4 data bytes                                              */
    *CANAddr   = pCAN->RDB;
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
         U32 buf0, buf1;
         S16 cnt1, cnt2, bound1;

  /* Acceptance Filter Memory full                                           */
  if ((((CAN_std_cnt + 1) >> 1) + CAN_ext_cnt) >= 512)
    return CAN_OBJECTS_FULL_ERROR;

  /* Setup Acceptance Filter Configuration                                   
     Acceptance Filter Mode Register = Off                                   */
  LPC_CANAF->AFMR = 0x00000001;

  if ((object_para & FORMAT_TYPE) == STANDARD_TYPE)  { /* Add mask for std id*/
    id |= (ctrl-1) << 13;             /* Add controller number               */
    id &= 0x0000F7FF;                 /* Mask out 16-bits of ID              */

    /* Move all remaining extended mask entries one place up                 
       if new entry will increase standard ID filters list                   */
    if ((CAN_std_cnt & 0x0001) == 0 && CAN_ext_cnt != 0) {
      cnt1   = (CAN_std_cnt >> 1);
      bound1 = CAN_ext_cnt;
      buf0   = LPC_CANAF_RAM->mask[cnt1];
      while (bound1--)  {
        cnt1++;
        buf1 = LPC_CANAF_RAM->mask[cnt1];
        LPC_CANAF_RAM->mask[cnt1] = buf0;
        buf0 = buf1;
      }        
    }

    if (CAN_std_cnt == 0)  {          /* For entering first  ID              */
      LPC_CANAF_RAM->mask[0] = 0x0000FFFF | (id << 16);
    }  else if (CAN_std_cnt == 1)  {  /* For entering second ID              */
      if ((LPC_CANAF_RAM->mask[0] >> 16) > id)
        LPC_CANAF_RAM->mask[0] = (LPC_CANAF_RAM->mask[0] >> 16) | (id << 16);
      else
        LPC_CANAF_RAM->mask[0] = (LPC_CANAF_RAM->mask[0] & 0xFFFF0000) | id;
    }  else  {
      /* Find where to insert new ID                                         */
      cnt1 = 0;
      cnt2 = CAN_std_cnt;
      bound1 = (CAN_std_cnt - 1) >> 1;
      while (cnt1 <= bound1)  {       /* Loop through standard existing IDs  */
        if ((LPC_CANAF_RAM->mask[cnt1] >> 16) > id)  {
          cnt2 = cnt1 * 2;
          break;
        }
        if ((LPC_CANAF_RAM->mask[cnt1] & 0x0000FFFF) > id)  {
          cnt2 = cnt1 * 2 + 1;
          break;
        }
        cnt1++;                       /* cnt1 = U32 where to insert new ID   */
      }                               /* cnt2 = U16 where to insert new ID   */

      if (cnt1 > bound1)  {           /* Adding ID as last entry             */
        if ((CAN_std_cnt & 0x0001) == 0)/* Even number of IDs exists         */
          LPC_CANAF_RAM->mask[cnt1]  = 0x0000FFFF | (id << 16);
        else                              /* Odd  number of IDs exists       */
          LPC_CANAF_RAM->mask[cnt1]  = (LPC_CANAF_RAM->mask[cnt1] & 0xFFFF0000) | id;
      }  else  {
        buf0 = LPC_CANAF_RAM->mask[cnt1];/* Remember current entry           */
        if ((cnt2 & 0x0001) == 0)     /* Insert new mask to even address     */
          buf1 = (id << 16) | (buf0 >> 16);
        else                          /* Insert new mask to odd  address     */
          buf1 = (buf0 & 0xFFFF0000) | id;
     
        LPC_CANAF_RAM->mask[cnt1] = buf1;/* Insert mask                      */

        bound1 = CAN_std_cnt >> 1;
        /* Move all remaining standard mask entries one place up             */
        while (cnt1 < bound1)  {
          cnt1++;
          buf1  = LPC_CANAF_RAM->mask[cnt1];
          LPC_CANAF_RAM->mask[cnt1] = (buf1 >> 16) | (buf0 << 16);
          buf0  = buf1;
        }

        if ((CAN_std_cnt & 0x0001) == 0)/* Even number of IDs exists         */
          LPC_CANAF_RAM->mask[cnt1] = (LPC_CANAF_RAM->mask[cnt1] & 0xFFFF0000) | (0x0000FFFF);
      }
    }
    CAN_std_cnt++;
  }  else  {                          /* Add mask for extended identifiers   */
    id |= (ctrl-1) << 29;             /* Add controller number               */

    cnt1 = ((CAN_std_cnt + 1) >> 1);
    cnt2 = 0;
    while (cnt2 < CAN_ext_cnt)  {     /* Loop through extended existing masks*/
      if (LPC_CANAF_RAM->mask[cnt1] > id)
        break;
      cnt1++;                         /* cnt1 = U32 where to insert new mask */
      cnt2++;
    }

    buf0 = LPC_CANAF_RAM->mask[cnt1];  /* Remember current entry             */
    LPC_CANAF_RAM->mask[cnt1] = id;    /* Insert mask                        */

    CAN_ext_cnt++;

    bound1 = CAN_ext_cnt - 1;
    /* Move all remaining extended mask entries one place up                 */
    while (cnt2 < bound1)  {
      cnt1++;
      cnt2++;
      buf1 = LPC_CANAF_RAM->mask[cnt1];
      LPC_CANAF_RAM->mask[cnt1] = buf0;
      buf0 = buf1;
    }        
  }
  
  /* Calculate std ID start address (buf0) and ext ID start address (buf1)   */
  buf0 = ((CAN_std_cnt + 1) >> 1) << 2;
  buf1 = buf0 + (CAN_ext_cnt << 2);

  /* Setup acceptance filter pointers                                        */
  LPC_CANAF->SFF_sa     = 0;
  LPC_CANAF->SFF_GRP_sa = buf0;
  LPC_CANAF->EFF_sa     = buf0;
  LPC_CANAF->EFF_GRP_sa = buf1;
  LPC_CANAF->ENDofTable = buf1;

  LPC_CANAF->AFMR = 0x00000000;   /* Use acceptance filter               */

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

/*------------------------ CAN_IRQHandler -----------------------------------
 *
 *  CAN receive and transmit interrupt function for all controllers
 *  Reads message from hardware registers and puts it into receive mailboxes
 *  If there are messages in mailbox for transmit it writes it to hardware
 *  and starts the transmission
 *
 *  Parameter:  none
 *
 *  Return:     none
 *---------------------------------------------------------------------------*/

void CAN_IRQHandler (void)  {
  CAN_msg *ptrmsg;
  U32 run1, run2, pconp, mbx_free;
  U32 icr1 = 0, icr2 = 0;

  /* Check which channels are running                                        */
  pconp = LPC_SC->PCONP;
  run1  = (pconp & (1 << 13)) != 0;
  run2  = (pconp & (1 << 14)) != 0;

  /* If message is received and if mailbox isn't full read message from 
     hardware and send it to message queue                                   */
#if USE_CAN_CTRL1 == 1
  if (run1) {
    icr1 = LPC_CAN1->ICR;
    if (icr1 & (1 << 0)) {
      mbx_free = (isr_mbx_check (MBX_rx_ctrl[0]) > 0);
      if (mbx_free) {
        ptrmsg = _alloc_box (CAN_mpool);
        if (ptrmsg) CAN_hw_rd (1, ptrmsg);
      }
      LPC_CAN1->CMR = 0x04;             /* Release receive buffer            */
      if (mbx_free && ptrmsg) isr_mbx_send (MBX_rx_ctrl[0], ptrmsg);
    }
  }
#endif
#if USE_CAN_CTRL2 == 1
  if (run2) {
    icr2 = LPC_CAN2->ICR;
    if (icr2 & (1 << 0)) {
      mbx_free = (isr_mbx_check (MBX_rx_ctrl[1]) > 0);
      if (mbx_free) {
        ptrmsg = _alloc_box (CAN_mpool);
        if (ptrmsg) CAN_hw_rd (2, ptrmsg);
      }
      LPC_CAN2->CMR = 0x04;             /* Release receive buffer            */
      if (mbx_free && ptrmsg) isr_mbx_send (MBX_rx_ctrl[1], ptrmsg);
    }
  }
#endif

  /* If there is message in mailbox ready for send, and if transmission
     hardware is ready, read the message from mailbox and send it            */
#if USE_CAN_CTRL1 == 1
  if (run1) {
    if (icr1 & (1 << 1)) {
      if (isr_mbx_receive (MBX_tx_ctrl[0], (void **)&ptrmsg) != OS_R_OK) {
        CAN_hw_wr (1, ptrmsg);
        _free_box(CAN_mpool, ptrmsg);
      } else {
        isr_sem_send(wr_sem[0]);  /* Return a token back to semaphore        */
      }
    }
  }
#endif
#if USE_CAN_CTRL2 == 1
  if (run2) {
    if (icr2 & (1 << 1)) {
      if (isr_mbx_receive (MBX_tx_ctrl[1], (void **)&ptrmsg) != OS_R_OK) {
        CAN_hw_wr (2, ptrmsg);
        _free_box(CAN_mpool, ptrmsg);
      } else {
        isr_sem_send(wr_sem[1]);  /* Return a token back to semaphore        */
      }
    }
  }
#endif
}


/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/

