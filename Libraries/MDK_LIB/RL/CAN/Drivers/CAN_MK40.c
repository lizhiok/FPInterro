/*------------------------------------------------------------------------------
 *      RL-ARM - CAN
 *------------------------------------------------------------------------------
 *      Name:    CAN_MK40.c
 *      Purpose: CAN Driver, Hardware specific module for Freescale MK40
 *      Rev.:    V4.70
 *------------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include <RTL.h>                        /* RTX kernel functions & defines     */
#include <RTX_CAN.h>                    /* CAN Generic functions & defines    */
#include <MK40N512MD100.h>


/************************* CAN Hardware Configuration *************************/

// *** <<< Use Configuration Wizard in Context Menu >>> ***

// <o0> CAN0 Engine Clock (in Hz) <1-1000000000>
//     <i> Same as Bus Clock
// <o1> CAN1 Engine Clock (in Hz) <1-1000000000>
//     <i> Same as Bus Clock
#define CAN0_CLK              48000000
#define CAN1_CLK              48000000

// *** <<< End of Configuration section             >>> ***


/*------------------------------------------------------------------------------
 *      CAN RTX Hardware Specific Driver Functions
 *------------------------------------------------------------------------------
 * Functions implemented in this module and used externally:
 *         CAN_ERROR CAN_hw_testmode    (U32 ctrl, U32 testmode);
 *         CAN_ERROR CAN_hw_setup       (U32 ctrl);
 *         CAN_ERROR CAN_hw_init        (U32 ctrl, U32 baudrate);
 *         CAN_ERROR CAN_hw_start       (U32 ctrl);
 *         CAN_ERROR CAN_hw_tx_empty    (U32 ctrl);
 *         CAN_ERROR CAN_hw_wr          (U32 ctrl, CAN_msg *msg);
 *         CAN_ERROR CAN_hw_set         (U32 ctrl, CAN_msg *msg);
 *         CAN_ERROR CAN_hw_rx_object   (U32 ctrl, U32 ch, U32 id, U32 object_para);
 *         CAN_ERROR CAN_hw_tx_object   (U32 ctrl, U32 ch,         U32 object_para);
 *----------------------------------------------------------------------------*/

/* CAN Controller pointer to register structure                               */
CAN_Type *CAN_CTRL[] = { CAN0, CAN1 };

/* Global variables                                                           */
U32 rx_mb  [2]    = { 0 };
U32 tx_rtr_mb [2] = { 0 };
U32 tx_mb  [2]    = { 0 };
U32 rx_rtr_mb [2] = { 0 };


/************************* Auxiliary Functions ********************************/

/*--------------------------- CAN_set_timing -----------------------------------
 *
 *  Setup the CAN timing with specific parameters
 *  Bit time looks like below:
 *  /--------------|--------------------------------------|-------------------\ 
 *  | sync seg = 1 |        prop_seg    +      tseg1      |       tseg2       | 
 *  \--------------|--------------------------------------|-------------------/
 *  <-------------------- around 70 % ------------------->|<-- around 30 % -->
 *
 *  Parameter:  ctrl:       Index of the hardware CAN controller (1 .. x)
 *              presdiv:    Prescaler Division Factor
 *              tseg1:      Specifies Time Segment before the sample point 
 *                          (in time quanta) 
 *              tseg2:      Time Segment after the sample point (in time quanta)
 *              prop_seg:   Propagation segment (in time quanta)
 *              brp:        Baud Rate Prescaler
 *
 *  Return:                 __TRUE = success, __FALSE = fail
 *----------------------------------------------------------------------------*/

static BOOL CAN_set_timing (U32 ctrl, U32 presdiv, U32 pseg1, U32 pseg2, U32 prop_seg) {
  CAN_Type *CANx = CAN_CTRL[ctrl-1];
  U32 rjw;

  /* Check input parameters are in expected boundaries                        */
  if ((!presdiv)  || (presdiv > 256)  ||
      (!pseg1)    || (pseg1   >   8)  || 
      (!pseg2)    || (pseg2   >   8)  ||
      (!prop_seg) || (prop_seg >   8))  {
    return (__FALSE);
  }

  /* Calculate other needed values                                            */
  rjw = (((pseg1 + prop_seg) < pseg2) ? (pseg1 + prop_seg) : (pseg2));
  if (rjw > 4)  rjw = 4;
  if (!rjw)     rjw = 1;

  CANx->CTRL1 &= ~0xFFFF03FF;
  CANx->CTRL1 |=  ((presdiv-1) << 24) | ((rjw-1) << 22) | ((pseg1-1) << 19) | ((pseg2-1) << 16) | (1 << 7) | (prop_seg-1);

  return (__TRUE);
}


/*------------------------ CAN_hw_set_baudrate ---------------------------------
 *
 *  Setup the requested baudrate
 *
 *  Parameter:  ctrl:       Index of the hardware CAN controller (1 .. x)
 *              baudrate:   Baudrate
 *
 *  Return:     CAN_ERROR:  Error code
 *----------------------------------------------------------------------------*/

static CAN_ERROR CAN_hw_set_baudrate (U32 ctrl, U32 baudrate)  {

  /* Prepare values so that sample point is at about 70% from bit start
     PSEG1, PSEG2, PROP_SEG are values that should be written into register, 
     these values are used with + 1 for timing values
     PSEG1    = 3 (tseg1    = PSEG1    + 1),  \
     PSEG2    = 3 (tseg2    = PSEG2    + 1),  - => 1 CAN bit = 12 TQ, sample at 66.7% 
     PROP_SEG = 2 (prop_seg = PROP_SEG + 1)   /
     sample point = ((1+prop_seg+tseg1)/(1+prop_seg+tseg1+tseg2)) * 100%        

    /--------------|--------------------------------------|-------------------\ 
    | sync_seg = 1 |    prop_seg = 3   +    tseg1 = 4     |    tseg2 = 4      | 
    \--------------|--------------------------------------|-------------------/
    <----------------------- 66.7 % --------------------->|<----- 33.3 % ---->
  */

  if (!CAN_set_timing (ctrl, (((ctrl == 1) ? CAN0_CLK : CAN1_CLK) / 12) / baudrate, 3+1, 3+1, 2+1)) 
    return CAN_BAUDRATE_ERROR;

  return CAN_OK;
}


/*************************** Module Functions *********************************/

/*--------------------------- CAN_hw_setup -------------------------------------
 *
 *  Setup CAN clocks, transmit and receive PINs and interrupt vectors
 *
 *  Parameter:  ctrl:       Index of the hardware CAN controller (1 .. x)
 *
 *  Return:     CAN_ERROR:  Error code
 *----------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_setup (U32 ctrl)  {
  switch (ctrl) {
    case 1:
      #if USE_CAN_CTRL1 == 1
        /* Enable CAN0 clock                                                  */
        SIM->SCGC6     |=  (1 <<  4);   /* FlexCAN0 Clock Enabled             */
        CAN0->CTRL1    |=  (1 << 13);   /* CAN Engine Clock Src is Bus Clock  */

        /* Setup CAN0 pins                                                    */
#ifdef CAN0_PORTB
        SIM->SCGC5     |=  (1 << 10);   /* Enable Port B Clock                */
        PORTB->PCR[18] &= ~(7 <<  8);
        PORTB->PCR[18] |=  (2 <<  8);   /* PTB18 is CAN0_TX pin (ALT2)        */
        PORTB->PCR[19] &= ~(7 <<  8);
        PORTB->PCR[19] |=  (2 <<  8);   /* PTB19 is CAN0_RX pin (ALT2)        */
#else                                   /* Default CAN0 pins on Port A        */
        SIM->SCGC5     |=  (1 <<  9);   /* Enable Port A Clock                */
        PORTA->PCR[12] &= ~(7 <<  8);
        PORTA->PCR[12] |=  (2 <<  8);   /* PTA12 is CAN0_TX pin (ALT2)        */
        PORTA->PCR[13] &= ~(7 <<  8);
        PORTA->PCR[13] |=  (2 <<  8);   /* PTA13 is CAN0_RX pin (ALT2)        */
#endif

        /* Enable CAN0 interrupt                                              */
        NVIC_SetPriority (CAN0_ORed_Message_buffer_IRQn, 1);
        NVIC_EnableIRQ   (CAN0_ORed_Message_buffer_IRQn);
      #endif
      break;
    case 2: 
      #if USE_CAN_CTRL2 == 1
        /* Enable CAN1 clock                                                  */
        SIM->SCGC3     |=  (1 <<  4);   /* FlexCAN1 Clock Enabled             */
        CAN1->CTRL1    |=  (1 << 13);   /* CAN Engine Clock Src is Bus Clock  */

        /* Setup CAN1 pins                                                    */
#ifdef CAN1_PORTC
        SIM->SCGC5     |=  (1 << 11);   /* Enable Port C Clock                */
        PORTC->PCR[17] &= ~(7 <<  8);
        PORTC->PCR[17] |=  (2 <<  8);   /* PTC17 is CAN1_TX pin (ALT2)        */
        PORTC->PCR[16] &= ~(7 <<  8);
        PORTC->PCR[16] |=  (2 <<  8);   /* PTC16 is CAN1_RX pin (ALT2)        */
#else                                   /* Default CAN10 pins on Port E       */
        SIM->SCGC5     |=  (1 << 13);   /* Enable Port E Clock                */
        PORTE->PCR[24] &= ~(7 <<  8);
        PORTE->PCR[24] |=  (2 <<  8);   /* PTE24 is CAN1_TX pin (ALT2)        */
        PORTE->PCR[25] &= ~(7 <<  8);
        PORTE->PCR[25] |=  (2 <<  8);   /* PTE25 is CAN1_RX pin (ALT2)        */
#endif

        /* Enable CAN1 interrupt                                              */
        NVIC_SetPriority (CAN1_ORed_Message_buffer_IRQn, 1);
        NVIC_EnableIRQ   (CAN1_ORed_Message_buffer_IRQn);
      #endif
      break;
    default:
      return CAN_UNEXIST_CTRL_ERROR;
  }

  return CAN_OK;
}

/*--------------------------- CAN_hw_init --------------------------------------
 *
 *  Initialize the CAN hardware
 *
 *  Parameter:  ctrl:       Index of the hardware CAN controller (1 .. x)
 *              baudrate:   Baudrate
 *
 *  Return:     CAN_ERROR:  Error code
 *----------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_init (U32 ctrl, U32 baudrate)  {
  CAN_Type *CANx = CAN_CTRL[ctrl-1];
  U32 i;

  OSC->CR      =  (1   <<  7);          /* Enable OSCERCLK errata e2583       */

  CANx->MCR   &= ~(1UL << 31);          /* Enable FlexCAN module              */
  CANx->MCR   |=  (1   << 25);          /* Start Soft Reset                   */
  while (CANx->MCR & (1 << 25));        /* Wait for reset to finish           */
  CANx->MCR   |=  (1   << 30);          /* Freeze enable                      */
  CANx->MCR   |=  (1   << 28);          /* Halt FlexCAN                       */
  while (!(CANx->MCR & (1 << 24)));     /* Wait for freeze mode acknowledge   */

  CANx->CTRL2 &=~((15  << 24) |         /* RFFN = 0, Number of Rx FIFO Filters*/
                  (1   << 17));         /* RRS = 0, Remote Request generated  */
  CANx->CTRL2 |=  (1   << 16) ;         /* EACEN = 0, IDE and RTR compared    */
  CANx->MCR   |=  (1   << 17) |         /* SRXDIS = 1 (Self Reception Disable)*/
                  (1   << 16) |         /* IRQM = 1 (Individual Rx Msk and Q) */
                  (15  <<  0) ;         /* Number of Mailboxes is 16          */

  if (CAN_hw_set_baudrate(ctrl, baudrate) != CAN_OK)         /* Set baudrate  */
    return CAN_BAUDRATE_ERROR;

  rx_mb[ctrl-1]     = 0;
  rx_rtr_mb[ctrl-1] = 0;
  tx_mb[ctrl-1]     = 0;
  tx_rtr_mb[ctrl-1] = 0;
  for (i = 0; i < 16; i++)              /* Disable all Mailboxes              */
    CANx->MB[i].CS = 0;

  return CAN_OK;
}


/*--------------------------- CAN_hw_start -------------------------------------
 *
 *  Enable the CAN controller
 *
 *  Parameter:  ctrl:       Index of the hardware CAN controller (1 .. x)
 *
 *  Return:     CAN_ERROR:  Error code
 *----------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_start (U32 ctrl)  {
  CAN_Type *CANx = CAN_CTRL[ctrl-1];

  CANx->MCR &= ~(1   << 28);            /* Exit halt mode                     */
  while (CANx->MCR & (1 << 24));        /* Wait for freeze mode to exit       */

  return CAN_OK;
}

/*--------------------------- CAN_set_testmode ---------------------------------
 *
 *  Enable the CAN testmode
 *
 *  Parameter:  ctrl:       Index of the hardware CAN controller (1 .. x)
 *              testmode:   Test mode (loopback bit value)
 *
 *  Return:     CAN_ERROR:  Error code
 *----------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_testmode (U32 ctrl, U32 testmode) {
  CAN_Type *CANx = CAN_CTRL[ctrl-1];

  CANx->CTRL1 |= ((testmode & 1) << 12);

  return CAN_OK;
}

/*--------------------------- CAN_hw_tx_empty ----------------------------------
 *
 *  Check if controller is available for transmission
 *
 *  Parameter:  ctrl:       Index of the hardware CAN controller (1 .. x)
 *
 *  Return:     CAN_ERROR:  Error code
 *----------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_tx_empty (U32 ctrl)  {
  CAN_Type *CANx = CAN_CTRL[ctrl-1];

  if ((os_sem_wait (wr_sem[ctrl-1], 0) != OS_R_TMO)){ /* If semaphore is free */
    if (!(CANx->ESR1 & (1 << 6)))       /* If FlexCAN is not in transmission  */
      return CAN_OK;
    else 
      os_sem_send(wr_sem[ctrl-1]);      /* Return a token back to semaphore   */
  }

  return CAN_TX_BUSY_ERROR;
}


/*--------------------------- CAN_hw_wr ----------------------------------------
 *
 *  Write CAN_msg to the hardware registers of the requested controller
 *
 *  Parameter:  ctrl:       Index of the hardware CAN controller (1 .. x)
 *              msg:        Pointer to CAN message to be written to hardware
 *
 *  Return:     CAN_ERROR:  Error code
 *----------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_wr (U32 ctrl, CAN_msg *msg)  {
  CAN_Type *CANx = CAN_CTRL[ctrl-1];
  U32  cs; 
  U8   ch; 
  BOOL remote_type, standard_id;

  remote_type = (msg->type   == REMOTE_FRAME);
  standard_id = (msg->format == STANDARD_FORMAT);
  ch          = (msg->ch) - 1;
  cs          =  0;

  /* Clear interrupt if it is active                                          */
  if (CANx->IFLAG1 & (1 << ch)) CANx->IFLAG1 = (1 << ch);

  /* Setup the identifier information                                         */
  if (standard_id)  {                                      /* Standard ID     */
    CANx->MB[ch].ID = (U32)(msg->id << 18);
  }  else  {                                               /* Extended ID     */
    CANx->MB[ch].ID = (U32)(msg->id <<  0);
    cs = (1 << 21);
  }

  if (remote_type) 
    cs |= (1 << 20);

  cs |= (0x0C       << 24) |            /* CODE = DATA or REMOTE              */
        ((msg->len) << 16) ;            /* DLC                                */

  /* Setup data bytes                                                         */
  CANx->MB[ch].WORD0 = ((U32)msg->data[0] << 24) | 
                       ((U32)msg->data[1] << 16) | 
                       ((U32)msg->data[2] <<  8) | 
                       ((U32)msg->data[3]      ) ;
  CANx->MB[ch].WORD1 = ((U32)msg->data[4] << 24) | 
                       ((U32)msg->data[5] << 16) | 
                       ((U32)msg->data[6] <<  8) | 
                       ((U32)msg->data[7]      ) ;

  if (remote_type) { 
    tx_rtr_mb[ctrl-1] |= (1 << ch);     /* Send RTR                           */
    rx_mb[ctrl-1]     |= (1 << ch);     /* Receive response                   */
    rx_rtr_mb[ctrl-1] &=~(1 << ch);
    tx_mb[ctrl-1]     &=~(1 << ch);
  } else { 
    tx_mb[ctrl-1]     |= (1 << ch);     /* Send DATA                          */
    tx_rtr_mb[ctrl-1] &=~(1 << ch);
    rx_mb[ctrl-1]     &=~(1 << ch);
    rx_rtr_mb[ctrl-1] &=~(1 << ch);
  }

  CANx->IMASK1  |=  (1 << ch);          /* Enable channel interrupt           */

  CANx->MB[ch].CS = cs;                 /* Activate transfer                  */

  return CAN_OK;
}


/*--------------------------- CAN_hw_rd ----------------------------------------
 *
 *  Read CAN_msg from the hardware registers of the requested controller
 *
 *  Parameter:  ctrl:       Index of the hardware CAN controller (1 .. x)
 *              ch:         Index of object used for reception
 *              msg:        Pointer where CAN message will be read
 *
 *  Return:     none
 *----------------------------------------------------------------------------*/

static void CAN_hw_rd (U32 ctrl, U32 ch, CAN_msg *msg)  {
  CAN_Type *CANx = CAN_CTRL[ctrl-1];
  U32       cs;

  cs  = CANx->MB[ch].CS;
  ch -= 1;

  while (cs & (3 << 24)) {              /* If BUST, wait for BUSY to end      */
    cs = CANx->MB[ch].CS;
  }

  /* Read identifier information                                              */
  if (!(cs & (1 << 21))) {                                  /* Standard ID    */
    msg->format =  STANDARD_FORMAT;
    msg->id     =  0x000007FFUL & (CANx->MB[ch].ID >> 18);
  }  else  {                                                /* Extended ID    */
    msg->format =  EXTENDED_FORMAT;
    msg->id     =  0x1FFFFFFFUL & CANx->MB[ch].ID;
  }

  /* Read type information                                                    */
  msg->type     =  DATA_FRAME;                              /* DATA   FRAME   */

  /* Read length (number of received bytes)                                   */
  msg->len      = (CANx->MB[ch].CS  >> 16) & 0x0F;

  /* Read data bytes                                                          */
  msg->data[0]  = (CANx->MB[ch].WORD0 >> 24);
  msg->data[1]  = (CANx->MB[ch].WORD0 >> 16);
  msg->data[2]  = (CANx->MB[ch].WORD0 >>  8);
  msg->data[3]  = (CANx->MB[ch].WORD0      );
  msg->data[4]  = (CANx->MB[ch].WORD1 >> 24);
  msg->data[5]  = (CANx->MB[ch].WORD1 >> 16);
  msg->data[6]  = (CANx->MB[ch].WORD1 >>  8);
  msg->data[7]  = (CANx->MB[ch].WORD1      );

  CANx->TIMER;                          /* Read timer to unlock Mailbox       */
}


/*--------------------------- CAN_hw_set ---------------------------------------
 *  Set a message that will automatically be sent as an answer to the REMOTE
 *  FRAME message
 *
 *  Parameter:  ctrl:       Index of the hardware CAN controller (1 .. x)
 *              msg:        Pointer to CAN message to be set
 *
 *  Return:     CAN_ERROR:  Error code
 *----------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_set (U32 ctrl, CAN_msg *msg)  {
  CAN_Type *CANx = CAN_CTRL[ctrl-1];
  U32  cs; 
  U8   ch; 
  BOOL standard_id;

  standard_id = (msg->format == STANDARD_FORMAT);
  ch          = (msg->ch) - 1;
  cs          =  0;

  /* Clear interrupt if it is active                                          */
  if (CANx->IFLAG1 & (1 << ch)) CANx->IFLAG1 = (1 << ch);

  /* Setup the identifier information                                         */
  if (standard_id)  {                                      /* Standard ID     */
    CANx->MB[ch].ID = (U32)(msg->id << 18);
  }  else  {                                               /* Extended ID     */
    CANx->MB[ch].ID = (U32)(msg->id <<  0);
    cs = (1 << 21);
  }

  cs |= (0x0A       << 24) |            /* CODE = RANSWER                     */
        ((msg->len) << 16) ;            /* DLC                                */

  /* Setup data bytes                                                         */
  CANx->MB[ch].WORD0 = ((U32)msg->data[0] << 24) | 
                       ((U32)msg->data[1] << 16) | 
                       ((U32)msg->data[2] <<  8) | 
                       ((U32)msg->data[3]      ) ;
  CANx->MB[ch].WORD1 = ((U32)msg->data[4] << 24) | 
                       ((U32)msg->data[5] << 16) | 
                       ((U32)msg->data[6] <<  8) | 
                       ((U32)msg->data[7]      ) ;

  rx_rtr_mb[ctrl-1] |= (1 << ch);       /* Receive RTR                        */
  tx_mb[ctrl-1]     |= (1 << ch);       /* Transmit response                  */
  tx_rtr_mb[ctrl-1] &=~(1 << ch);
  rx_mb[ctrl-1]     &=~(1 << ch);

  CANx->IMASK1   |=  (1 << ch);         /* Enable channel interrupt           */

  CANx->MB[ch].CS = cs;                 /* Activate transfer                  */

  return CAN_OK;
}


/*--------------------------- CAN_hw_rx_object ---------------------------------
 *
 *  This function setups object that is going to be used for the message 
 *  reception
 *
 *  Parameter:  ctrl:       Index of the hardware CAN controller (1 .. x)
 *              ch:         Index of object used for reception
 *              id:         Identifier of receiving messages
 *              object_para:Object parameters (standard or extended format, 
 *                          data or remote frame)
 *
 *  Return:     CAN_ERROR:  Error code
 *----------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_rx_object (U32 ctrl, U32 ch, U32 id, U32 object_para)  {
  CAN_Type *CANx = CAN_CTRL[ctrl-1];
  U32  cs; 
//BOOL remote_type;
  BOOL standard_id;

//remote_type = (object_para &  FRAME_TYPE) == REMOTE_TYPE;
  standard_id = (object_para & FORMAT_TYPE) == STANDARD_TYPE;
  ch         -=  1;
  cs          =  0;

  /* Clear interrupt if it is active                                          */
  if (CANx->IFLAG1 & (1 << ch)) CANx->IFLAG1 = (1 << ch);

  /* Setup the identifier information                                         */
  if (standard_id)  {                                      /* Standard ID     */
    CANx->MB[ch].ID = (U32)(id << 18);
  }  else  {                                               /* Extended ID     */
    CANx->MB[ch].ID = (U32)(id <<  0);
    cs = (1 << 21);
  }

  cs |= (0x04       << 24);             /* CODE = EMPTY                       */

  rx_mb[ctrl-1]     |= (1 << ch);       /* Receive DATA                       */
  tx_rtr_mb[ctrl-1] &=~(1 << ch);
  rx_rtr_mb[ctrl-1] &=~(1 << ch);
  tx_mb[ctrl-1]     &=~(1 << ch);

  CANx->IMASK1  |= (1 << ch);           /* Enable channel interrupt           */

  CANx->MB[ch].CS = cs;                 /* Activate transfer                  */

  return CAN_OK;
}

/*--------------------------- CAN_hw_tx_object ---------------------------------
 *
 *  This function setups object that is going to be used for the message 
 *  transmission, the setup of transmission object is not necessery so this 
 *  function is not implemented
 *
 *  Parameter:  ctrl:       Index of the hardware CAN controller (1 .. x)
 *              ch:         Index of object used for transmission
 *              object_para:Object parameters (standard or extended format, 
 *                          data or remote frame)
 *
 *  Return:     CAN_ERROR:  Error code
 *----------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_tx_object (U32 ctrl, U32 ch, U32 object_para)  {

  return CAN_NOT_IMPLEMENTED_ERROR;
}


/************************* Interrupt Functions ********************************/

/*--------------------------- CANx_IRQHandler ----------------------------------
 *
 *  CAN interrupt functions 
 *  If transmit interrupt occured and there are messages in mailbox for 
 *  transmit it writes it to hardware and starts the transmission
 *  If receive interrupt occured it reads message from hardware registers 
 *  and puts it into receive mailbox
 *----------------------------------------------------------------------------*/

#define CTRL0                 0
#define CTRL                  1
#define CANc                  CAN0

void CAN0_ORed_Message_buffer_IRQHandler (void) {
  CAN_msg *ptrmsg;
  U32      iflag1, mb;

  iflag1 = CANc->IFLAG1;
  while (iflag1) {
    for (mb = 0; mb < 32; mb ++) {
      if (iflag1 & (1 << mb)) {         /* If interrupt on Mailbox happened   */
        if ((rx_rtr_mb[CTRL0] & tx_mb[CTRL0]) & (1 << mb)) {
                                        /* If RTR received and answer sent    */
                                        /* Disable further RTR reception      */
          CANc->MB[mb].CS &= ~(0x0F << 24);    /* Set CODE to INACTIVE        */
        }

        if ((tx_rtr_mb[CTRL0] | tx_mb[CTRL0]) & (1 << mb)) {
                                        /* If RTR or DATA transmitted         */
          /* If there is a message in the mailbox ready for send, read the 
             message from the mailbox and send it                             */
          if (isr_mbx_receive (MBX_tx_ctrl[CTRL0], (void **)&ptrmsg) != OS_R_OK) {
            CAN_hw_wr (CTRL, ptrmsg);
            _free_box(CAN_mpool, ptrmsg);
          } else {
            isr_sem_send(wr_sem[CTRL0]);/* Return a token back to semaphore   */
          }
        } else if (rx_mb[CTRL0] & (1 << mb)) { /* If DATA received            */
          /* If the mailbox isn't full read the message from the hardware and
             send it to the message queue otherwise dump the rcvd message     */
          if (isr_mbx_check (MBX_rx_ctrl[CTRL0]) > 0)  {
            ptrmsg = _alloc_box (CAN_mpool);
            if (ptrmsg) {
              CAN_hw_rd (CTRL, mb+1, ptrmsg);/* Read received message         */
              isr_mbx_send (MBX_rx_ctrl[CTRL0], ptrmsg);
            }
          }
        }
        iflag1 &= ~(1 << mb);
        CANc->IFLAG1 |= (1 << mb);
        if (!iflag1)                    /* If all interrupts were handled     */
          return;
      }
    }
    iflag1 = CANc->IFLAG1;              /* Allow processing of new interrupts */
  }
}


#undef  CTRL0
#define CTRL0                 1
#undef  CTRL
#define CTRL                  2
#undef  CANc
#define CANc                  CAN1

void CAN1_ORed_Message_buffer_IRQHandler (void) {
  CAN_msg *ptrmsg;
  U32      iflag1, mb;

  iflag1 = CANc->IFLAG1;
  while (iflag1) {
    for (mb = 0; mb < 32; mb ++) {
      if (iflag1 & (1 << mb)) {         /* If interrupt on Mailbox happened   */
        if ((rx_rtr_mb[CTRL0] & tx_mb[CTRL0]) & (1 << mb)) {
                                        /* If RTR received and answer sent    */
                                        /* Disable further RTR reception      */
          CANc->MB[mb].CS &= ~(0x0F << 24);    /* Set CODE to INACTIVE        */
        }

        if ((tx_rtr_mb[CTRL0] | tx_mb[CTRL0]) & (1 << mb)) {
                                        /* If RTR or DATA transmitted         */
          /* If there is a message in the mailbox ready for send, read the 
             message from the mailbox and send it                             */
          if (isr_mbx_receive (MBX_tx_ctrl[CTRL0], (void **)&ptrmsg) != OS_R_OK) {
            CAN_hw_wr (CTRL, ptrmsg);
            _free_box(CAN_mpool, ptrmsg);
          } else {
            isr_sem_send(wr_sem[CTRL0]);/* Return a token back to semaphore   */
          }
        } else if (rx_mb[CTRL0] & (1 << mb)) { /* If DATA received            */
          /* If the mailbox isn't full read the message from the hardware and
             send it to the message queue otherwise dump the rcvd message     */
          if (isr_mbx_check (MBX_rx_ctrl[CTRL0]) > 0)  {
            ptrmsg = _alloc_box (CAN_mpool);
            if (ptrmsg) {
              CAN_hw_rd (CTRL, mb+1, ptrmsg);/* Read received message         */
              isr_mbx_send (MBX_rx_ctrl[CTRL0], ptrmsg);
            }
          }
        }
        iflag1 &= ~(1 << mb);
        CANc->IFLAG1 |= (1 << mb);
        if (!iflag1)                    /* If all interrupts were handled     */
          return;
      }
    }
    iflag1 = CANc->IFLAG1;              /* Allow processing of new interrupts */
  }
}


/*------------------------------------------------------------------------------
 * end of file
 *----------------------------------------------------------------------------*/

