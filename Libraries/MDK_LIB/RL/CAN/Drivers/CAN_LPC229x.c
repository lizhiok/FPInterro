/*----------------------------------------------------------------------------
 *      RL-ARM - CAN
 *----------------------------------------------------------------------------
 *      Name:    CAN_LPC229x.c
 *      Purpose: CAN Driver, Hardware specific module for NXP LPC229x
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <RTL.h>                      /* RTX kernel functions & defines      */
#include <RTX_CAN.h>                  /* CAN Generic functions & defines     */
#include <LPC22xx.h>                  /* LPC22xx definitions                 */
#include "CAN_reg.h"                  /* LPC2xxx CAN registers               */

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

// <o> PCLK value (in Hz) <1-1000000000>
//     <i> Peripheral clock, depends on VPBDIV
//     <i> Default: 60000000 (VPBDIV => VPB Clock = CPU Clock, MSEL = 5, xtal = 12 MHz => CPU Clock = 12 Mhz * 5 = 60 MHz)
#define PCLK             60000000

// <o0> VIC Interrupt number for CAN Controller 1 transmit <0-15> 
//      <i> Default: 11
// <o1> VIC Interrupt number for CAN Controller 1 receive  <0-15>            
//      <i> Default:  7
#define VIC_NUM_CTRL1_TX       11
#define VIC_NUM_CTRL1_RX        7

// <o0> VIC Interrupt number for CAN Controller 2 transmit <0-15>
//      <i> Default: 12
// <o1> VIC Interrupt number for CAN Controller 2 receive  <0-15>
//      <i> Default:  8
#define VIC_NUM_CTRL2_TX       12
#define VIC_NUM_CTRL2_RX        8

// <o0> VIC Interrupt number for CAN Controller 3 transmit <0-15>
//      <i> Default: 13
// <o1> VIC Interrupt number for CAN Controller 3 receive  <0-15>
//      <i> Default:  9
#define VIC_NUM_CTRL3_TX       13
#define VIC_NUM_CTRL3_RX        9

// <o0> VIC Interrupt number for CAN Controller 4 transmit <0-15>
//      <i> Default: 14
// <o1> VIC Interrupt number for CAN Controller 4 receive  <0-15>
//      <i> Default: 10
#define VIC_NUM_CTRL4_TX       14
#define VIC_NUM_CTRL4_RX       10

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
  static void CAN_TX1_ISR      (void) __irq;
  static void CAN_RX1_ISR      (void) __irq;
#endif
#if USE_CAN_CTRL2 == 1
  static void CAN_TX2_ISR      (void) __irq;
  static void CAN_RX2_ISR      (void) __irq;
#endif
#if USE_CAN_CTRL3 == 1
  static void CAN_TX3_ISR      (void) __irq;
  static void CAN_RX3_ISR      (void) __irq;
#endif
#if USE_CAN_CTRL4 == 1
  static void CAN_TX4_ISR      (void) __irq;
  static void CAN_RX4_ISR      (void) __irq;
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

  /* Determine which nominal time to use for PCLK and baudrate               */
  if (baudrate <= 500000)  {
    nominal_time = 12;
  }  else if (((PCLK / 1000000) % 15) == 0)  {
    nominal_time = 15;
  }  else if (((PCLK / 1000000) % 16) == 0)  {
    nominal_time = 16;
  }  else  {
    nominal_time = 10;
  }

  /* Prepare value appropriate for bit time register                         */
  result  = (PCLK / nominal_time) / baudrate - 1;
  result &= 0x000003FF;
  result |= CAN_BIT_TIME[nominal_time];

  ptrcan->CANBTR  = result;                      /* Set bit timing           */

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
        PINSEL1 &= 0xFFF3FFFF;        /* Reset CAN1 bits                     */
        PINSEL1 |= 0x00040000;        /* Set CAN1 bits to bin 01             */
    
        /* Set interrupt vector for Tx1                                      */
        *(&VICVectAddr0 + VIC_NUM_CTRL1_TX) = (unsigned long) CAN_TX1_ISR;
        *(&VICVectCntl0 + VIC_NUM_CTRL1_TX) = 0x20 | 20;
        /* Set interrupt vector for Rx1                                      */
        *(&VICVectAddr0 + VIC_NUM_CTRL1_RX) = (unsigned long) CAN_RX1_ISR;       
        *(&VICVectCntl0 + VIC_NUM_CTRL1_RX) = 0x20 | 26;
    
        VICIntEnable = 0x04100000;    /* Enable CAN Tx1 and Rx1 Interrupt    */
      #endif
      break;
    case 2: 
      #if USE_CAN_CTRL2 == 1
        PINSEL1 &= 0xFFFC3FFF;        /* Reset CAN2 bits                     */
        PINSEL1 |= 0x00014000;        /* Set CAN2 bits to bin 0101           */
    
        /* Set interrupt vector for Tx2                                      */
        *(&VICVectAddr0 + VIC_NUM_CTRL2_TX) = (unsigned long) CAN_TX2_ISR;
        *(&VICVectCntl0 + VIC_NUM_CTRL2_TX) = 0x20 | 21;
        /* Set interrupt vector for Rx2                                      */
        *(&VICVectAddr0 + VIC_NUM_CTRL2_RX) = (unsigned long) CAN_RX2_ISR;
        *(&VICVectCntl0 + VIC_NUM_CTRL2_RX) = 0x20 | 27;
    
        VICIntEnable = 0x08200000;    /* Enable CAN Tx2 and Rx2 Interrupt    */
      #endif
      break;
    case 3: 
      #if USE_CAN_CTRL3 == 1
        PINSEL1 &= 0xFFFFC3FF;        /* Reset CAN3 bits                     */
        PINSEL1 |= 0x00001800;        /* Set CAN3 bits to bin 0110           */
    
        /* Set interrupt vector for Tx3                                      */
        *(&VICVectAddr0 + VIC_NUM_CTRL3_TX) = (unsigned long) CAN_TX3_ISR;
        *(&VICVectCntl0 + VIC_NUM_CTRL3_TX) = 0x20 | 22;
        /* Set interrupt vector for Rx3                                      */
        *(&VICVectAddr0 + VIC_NUM_CTRL3_RX) = (unsigned long) CAN_RX3_ISR;
        *(&VICVectCntl0 + VIC_NUM_CTRL3_RX) = 0x20 | 28;
    
        VICIntEnable = 0x10400000;    /* Enable CAN Tx3 and Rx3 Interrupt    */
      #endif
      break;
    case 4: 
      #if USE_CAN_CTRL4 == 1
        PINSEL0 &= 0xF0FFFFFF;        /* Reset CAN4 bits                     */
        PINSEL0 |= 0x0F000000;        /* Set CAN4 bits to bin 1111           */
    
        /* Set interrupt vector for Tx4                                      */
        *(&VICVectAddr0 + VIC_NUM_CTRL4_TX) = (unsigned long) CAN_TX4_ISR;
        *(&VICVectCntl0 + VIC_NUM_CTRL4_TX) = 0x20 | 23;
        /* Set interrupt vector for Rx4                                      */
        *(&VICVectAddr0 + VIC_NUM_CTRL4_RX) = (unsigned long) CAN_RX4_ISR;
        *(&VICVectCntl0 + VIC_NUM_CTRL4_RX) = 0x20 | 29;
    
        VICIntEnable = 0x20800000;    /* Enable CAN Tx4 and Rx4 Interrupt    */
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
  regCAN_AF *ptrcan_af = (regCAN_AF *) CAN_AF_BASE;
  regCAN    *ptrcan    = (regCAN *) CAN_BASE[ctrl0];

  ptrcan_af->AFMR = 1;                /* Disable acceptance filter           */
  ptrcan->CANMOD  = 1;                /* Enter reset mode                    */
  ptrcan->CANIER  = 0;                /* Disable all interrupts              */
  ptrcan->CANGSR  = 0;                /* Clear status register               */
  CAN_hw_set_baudrate(ctrl, baudrate);/* Set bit timing                      */
  ptrcan->CANIER  = 0x0603;           /* Enable Tx and Rx interrupt          */
      
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

  ptrcan->CANMOD = 0;                 /* Enter normal operating mode         */

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
    if (ptrcan->CANSR & 0x00000004)               /* Transmitter ready for   */
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
  U32 ctrl0 = ctrl-1;                 /* Controller index 0 .. x-1           */
  U32 CANData;
  regCAN *ptrcan = (regCAN *) CAN_BASE[ctrl0];

  CANData       = (((U32) msg->len) << 16)          & 0x000F0000 | 
                  (msg->format == EXTENDED_FORMAT ) * 0x80000000 |
                  (msg->type   == REMOTE_FRAME)     * 0x40000000;

  if (ptrcan->CANSR & 0x00000004)  {  /* Transmit buffer 1 free              */
    ptrcan->CANTF1  = CANData;        /* Write frame informations            */
    ptrcan->CANTID1 = msg->id;        /* Write CAN message identifier        */
    ptrcan->CANTDA1 = *(U32 *) &msg->data[0]; /* Write first 4 data bytes    */
    ptrcan->CANTDB1 = *(U32 *) &msg->data[4]; /* Write second 4 data bytes   */
    ptrcan->CANCMR  = 0x30;           /* Start self-transmission             */
    //ptrcan->CANCMR  = 0x20;         /* Start transmission without loop-back*/
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
  CANData = ptrcan->CANRFS;
  msg->format   = (CANData & 0x80000000) == 0x80000000;
  msg->type     = (CANData & 0x40000000) == 0x40000000;
  msg->len      = ((U8)(CANData >> 16)) & 0x0F;

  /* Read CAN message identifier                                             */
  msg->id = ptrcan->CANRID;

  /* Read the data if received message was DATA FRAME                        */
  if (msg->type == DATA_FRAME)  {     

    /* Read first 4 data bytes                                               */
    CANAddr = (U32 *) &msg->data[0];
    *CANAddr++ = ptrcan->CANRDA;

    /* Read second 4 data bytes                                              */
    *CANAddr   = ptrcan->CANRDB;
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
  regCAN_AF    *ptrcan_af    = (regCAN_AF *) CAN_AF_BASE;
  regCAN_AFRAM *ptrcan_afram = (regCAN_AFRAM *) CAN_AFRAM_BASE;
  U32 buf0, buf1;
  S16 cnt1, cnt2, bound1;

  /* Acceptance Filter Memory full                                           */
  if ((((CAN_std_cnt + 1) >> 1) + CAN_ext_cnt) >= 512)
    return CAN_OBJECTS_FULL_ERROR;

  /* Setup Acceptance Filter Configuration                                   
     Acceptance Filter Mode Register = Off                                   */
  ptrcan_af->AFMR = 0x00000001;

  if ((object_para & FORMAT_TYPE) == STANDARD_TYPE)  { /* Add mask for std id*/
    id |= ctrl << 13;                 /* Add controller number               */
    id &= 0x0000F7FF;                 /* Mask out 16-bits of ID              */

    /* Move all remaining extended mask entries one place up                 
       if new entry will increase standard ID filters list                   */
    if ((CAN_std_cnt & 0x0001) == 0 && CAN_ext_cnt != 0) {
      cnt1   = (CAN_std_cnt >> 1);
      bound1 = CAN_ext_cnt;
      buf0   = ptrcan_afram->mask[cnt1];
      while (bound1--)  {
        cnt1++;
        buf1 = ptrcan_afram->mask[cnt1];
        ptrcan_afram->mask[cnt1] = buf0;
        buf0 = buf1;
      }        
    }

    if (CAN_std_cnt == 0)  {          /* For entering first  ID              */
      ptrcan_afram->mask[0] = 0x0000FFFF | (id << 16);
    }  else if (CAN_std_cnt == 1)  {  /* For entering second ID              */
      if ((ptrcan_afram->mask[0] >> 16) > id)
        ptrcan_afram->mask[0] = (ptrcan_afram->mask[0] >> 16) | (id << 16);
      else
        ptrcan_afram->mask[0] = (ptrcan_afram->mask[0] & 0xFFFF0000) | id;
    }  else  {
      /* Find where to insert new ID                                         */
      cnt1 = 0;
      cnt2 = CAN_std_cnt;
      bound1 = (CAN_std_cnt - 1) >> 1;
      while (cnt1 <= bound1)  {       /* Loop through standard existing IDs  */
        if ((ptrcan_afram->mask[cnt1] >> 16) > id)  {
          cnt2 = cnt1 * 2;
          break;
        }
        if ((ptrcan_afram->mask[cnt1] & 0x0000FFFF) > id)  {
          cnt2 = cnt1 * 2 + 1;
          break;
        }
        cnt1++;                       /* cnt1 = U32 where to insert new ID   */
      }                               /* cnt2 = U16 where to insert new ID   */

      if (cnt1 > bound1)  {           /* Adding ID as last entry             */
        if ((CAN_std_cnt & 0x0001) == 0)/* Even number of IDs exists         */
          ptrcan_afram->mask[cnt1]  = 0x0000FFFF | (id << 16);
        else                              /* Odd  number of IDs exists       */
          ptrcan_afram->mask[cnt1]  = (ptrcan_afram->mask[cnt1] & 0xFFFF0000) | id;
      }  else  {
        buf0 = ptrcan_afram->mask[cnt1];/* Remember current entry            */
        if ((cnt2 & 0x0001) == 0)     /* Insert new mask to even address     */
          buf1 = (id << 16) | (buf0 >> 16);
        else                          /* Insert new mask to odd  address     */
          buf1 = (buf0 & 0xFFFF0000) | id;
     
        ptrcan_afram->mask[cnt1] = buf1;/* Insert mask                       */

        bound1 = CAN_std_cnt >> 1;
        /* Move all remaining standard mask entries one place up             */
        while (cnt1 < bound1)  {
          cnt1++;
          buf1  = ptrcan_afram->mask[cnt1];
          ptrcan_afram->mask[cnt1] = (buf1 >> 16) | (buf0 << 16);
          buf0  = buf1;
        }

        if ((CAN_std_cnt & 0x0001) == 0)/* Even number of IDs exists         */
          ptrcan_afram->mask[cnt1] = (ptrcan_afram->mask[cnt1] & 0xFFFF0000) | (0x0000FFFF);
      }
    }
    CAN_std_cnt++;
  }  else  {                          /* Add mask for extended identifiers   */
    id |= (ctrl) << 29;               /* Add controller number               */

    cnt1 = ((CAN_std_cnt + 1) >> 1);
    cnt2 = 0;
    while (cnt2 < CAN_ext_cnt)  {     /* Loop through extended existing masks*/
      if (ptrcan_afram->mask[cnt1] > id)
        break;
      cnt1++;                         /* cnt1 = U32 where to insert new mask */
      cnt2++;
    }

    buf0 = ptrcan_afram->mask[cnt1];  /* Remember current entry              */
    ptrcan_afram->mask[cnt1] = id;    /* Insert mask                         */

    CAN_ext_cnt++;

    bound1 = CAN_ext_cnt - 1;
    /* Move all remaining extended mask entries one place up                 */
    while (cnt2 < bound1)  {
      cnt1++;
      cnt2++;
      buf1 = ptrcan_afram->mask[cnt1];
      ptrcan_afram->mask[cnt1] = buf0;
      buf0 = buf1;
    }        
  }
  
  /* Calculate std ID start address (buf0) and ext ID start address (buf1)   */
  buf0 = ((CAN_std_cnt + 1) >> 1) << 2;
  buf1 = buf0 + (CAN_ext_cnt << 2);

  /* Setup acceptance filter pointers                                        */
  ptrcan_af->SFF_sa     = 0;
  ptrcan_af->SFF_GRP_sa = buf0;
  ptrcan_af->EFF_sa     = buf0;
  ptrcan_af->EFF_GRP_sa = buf1;
  ptrcan_af->ENDofTable = buf1;

  ptrcan_af->AFMR = 0x00000000;       /* Use acceptance filter               */

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

/*--------------------------- CAN_TX1_ISR ----------------------------------
 *
 *  CAN transmit interrupt function for controller 1
 *  If there are messages in mailbox for transmit it writes it to hardware
 *  and starts the transmission on controller 1
 *
 *  Parameter:  none
 *
 *  Return:     none
 *---------------------------------------------------------------------------*/

static void CAN_TX1_ISR (void) __irq  {
  CAN_msg *ptrmsg;
  U32 temp;

    /* If there is message in mailbox ready for send, 
       read the message from mailbox and send it                             */
    if (isr_mbx_receive (MBX_tx_ctrl[0], (void **)&ptrmsg) != OS_R_OK) {
      CAN_hw_wr (1, ptrmsg);
      _free_box(CAN_mpool, ptrmsg);
  } else {
    isr_sem_send(wr_sem[0]);      /* Return a token back to semaphore        */
  }

  /* Read from interrupt register to acknowledge interrupt                   */
  temp = ptrCAN1->CANICR;
  VICVectAddr = 0xFFFFFFFF;           /* Acknowledge Interrupt               */
}


/*--------------------------- CAN_RX1_ISR -----------------------------------
 *
 *  CAN receive interrupt function, for CAN controller 1
 *  Reads message from hardware registers and puts it into receive mailbox
 *  for controller 1
 *
 *  Parameter:  none
 *
 *  Return:     none
 *---------------------------------------------------------------------------*/

static void CAN_RX1_ISR (void) __irq  {
  CAN_msg *ptrmsg;

  /* If mailbox isn't full read message from hardware and send it to message queue */
  if (os_mbx_check (MBX_rx_ctrl[0]) > 0) {
    ptrmsg = _alloc_box (CAN_mpool);
    CAN_hw_rd (1, ptrmsg);
    if (ptrmsg) isr_mbx_send (MBX_rx_ctrl[0], ptrmsg);
  }

  ptrCAN1->CANCMR = 0x04;             /* Release receive buffer              */
  VICVectAddr = 0xFFFFFFFF;           /* Acknowledge interrupt               */
}

#endif


#if USE_CAN_CTRL2 == 1

/*--------------------------- CAN_TX2_ISR ----------------------------------
 *
 *  CAN transmit interrupt function for controller 2
 *  If there are messages in mailbox for transmit it writes it to hardware
 *  and starts the transmission on controller 2
 *
 *  Parameter:  none
 *
 *  Return:     none
 *---------------------------------------------------------------------------*/

static void CAN_TX2_ISR (void) __irq  {
  CAN_msg *ptrmsg;
  U32 temp;

    /* If there is message in mailbox ready for send, 
       read the message from mailbox and send it                             */
    if (isr_mbx_receive (MBX_tx_ctrl[1], (void **)&ptrmsg) != OS_R_OK) {
      CAN_hw_wr (2, ptrmsg);
      _free_box(CAN_mpool, ptrmsg);
  } else {
    isr_sem_send(wr_sem[1]);      /* Return a token back to semaphore        */
  }

  /* Read from interrupt register to acknowledge interrupt                   */
  temp = ptrCAN2->CANICR;
  VICVectAddr = 0xFFFFFFFF;           /* Acknowledge Interrupt               */
}


/*--------------------------- CAN_RX2_ISR -----------------------------------
 *
 *  CAN receive interrupt function, for CAN controller 2
 *  Reads message from hardware registers and puts it into receive mailbox
 *  for controller 2
 *
 *  Parameter:  none
 *
 *  Return:     none
 *---------------------------------------------------------------------------*/

static void CAN_RX2_ISR (void) __irq  {
  CAN_msg *ptrmsg;

  /* If mailbox isn't full read message from hardware and send it to message queue */
  if (os_mbx_check (MBX_rx_ctrl[1]) > 0) {
    ptrmsg = _alloc_box (CAN_mpool);
    CAN_hw_rd (2, ptrmsg);
    if (ptrmsg) isr_mbx_send (MBX_rx_ctrl[1], ptrmsg);
  }

  ptrCAN2->CANCMR = 0x04;             /* Release receive buffer              */
  VICVectAddr = 0xFFFFFFFF;           /* Acknowledge interrupt               */
}

#endif


#if USE_CAN_CTRL3 == 1

/*--------------------------- CAN_TX3_ISR ----------------------------------
 *
 *  CAN transmit interrupt function for controller 3
 *  If there are messages in mailbox for transmit it writes it to hardware
 *  and starts the transmission on controller 3
 *
 *  Parameter:  none
 *
 *  Return:     none
 *---------------------------------------------------------------------------*/

static void CAN_TX3_ISR (void) __irq  {
  CAN_msg *ptrmsg;
  U32 temp;

    /* If there is message in mailbox ready for send, 
       read the message from mailbox and send it                             */
    if (isr_mbx_receive (MBX_tx_ctrl[2], (void **)&ptrmsg) != OS_R_OK) {
      CAN_hw_wr (3, ptrmsg);
      _free_box(CAN_mpool, ptrmsg);
  } else {
    isr_sem_send(wr_sem[2]);      /* Return a token back to semaphore        */
  }

  /* Read from interrupt register to acknowledge interrupt                   */
  temp = ptrCAN3->CANICR;
  VICVectAddr = 0xFFFFFFFF;           /* Acknowledge Interrupt               */
}


/*--------------------------- CAN_RX3_ISR -----------------------------------
 *
 *  CAN receive interrupt function, for CAN controller 3
 *  Reads message from hardware registers and puts it into receive mailbox
 *  for controller 3
 *
 *  Parameter:  none
 *
 *  Return:     none
 *---------------------------------------------------------------------------*/

static void CAN_RX3_ISR (void) __irq  {
  CAN_msg *ptrmsg;

  /* If mailbox isn't full read message from hardware and send it to message queue */
  if (os_mbx_check (MBX_rx_ctrl[2]) > 0) {
    ptrmsg = _alloc_box (CAN_mpool);
    CAN_hw_rd (3, ptrmsg);
    if (ptrmsg) isr_mbx_send (MBX_rx_ctrl[2], ptrmsg);
  }

  ptrCAN3->CANCMR = 0x04;             /* Release receive buffer              */
  VICVectAddr = 0xFFFFFFFF;           /* Acknowledge interrupt               */
}

#endif


#if USE_CAN_CTRL4 == 1

/*--------------------------- CAN_TX4_ISR ----------------------------------
 *
 *  CAN transmit interrupt function for controller 4
 *  If there are messages in mailbox for transmit it writes it to hardware
 *  and starts the transmission on controller 4
 *
 *  Parameter:  none
 *
 *  Return:     none
 *---------------------------------------------------------------------------*/

static void CAN_TX4_ISR (void) __irq  {
  CAN_msg *ptrmsg;
  U32 temp;

    /* If there is message in mailbox ready for send, 
       read the message from mailbox and send it                             */
    if (isr_mbx_receive (MBX_tx_ctrl[3], (void **)&ptrmsg) != OS_R_OK) {
      CAN_hw_wr (4, ptrmsg);
      _free_box(CAN_mpool, ptrmsg);
  } else {
    isr_sem_send(wr_sem[0]);      /* Return a token back to semaphore        */
  }

  /* Read from interrupt register to acknowledge interrupt                   */
  temp = ptrCAN4->CANICR;
  VICVectAddr = 0xFFFFFFFF;           /* Acknowledge Interrupt               */
}


/*--------------------------- CAN_RX4_ISR -----------------------------------
 *
 *  CAN receive interrupt function, for CAN controller 4
 *  Reads message from hardware registers and puts it into receive mailbox
 *  for controller 4
 *
 *  Parameter:  none
 *
 *  Return:     none
 *---------------------------------------------------------------------------*/

static void CAN_RX4_ISR (void) __irq  {
  CAN_msg *ptrmsg;

  /* If mailbox isn't full read message from hardware and send it to message queue */
  if (os_mbx_check (MBX_rx_ctrl[3]) > 0) {
    ptrmsg = _alloc_box (CAN_mpool);
    CAN_hw_rd (4, ptrmsg);
    if (ptrmsg) isr_mbx_send (MBX_rx_ctrl[3], ptrmsg);
  }

  ptrCAN4->CANCMR = 0x04;             /* Release receive buffer              */
  VICVectAddr = 0xFFFFFFFF;           /* Acknowledge interrupt               */
}

#endif


/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/

