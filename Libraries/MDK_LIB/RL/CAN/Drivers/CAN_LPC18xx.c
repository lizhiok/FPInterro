/*----------------------------------------------------------------------------
 *      RL-ARM - CAN
 *----------------------------------------------------------------------------
 *      Name:    CAN_LPC18xx.c
 *      Purpose: CAN Driver, Hardware specific module for NXP LPC18xx
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <RTL.h>                      /* RTX kernel functions & defines      */
#include <RTX_CAN.h>                  /* CAN Generic functions & defines     */
#include <LPC18xx.h>


/************************* CAN Hardware Configuration ************************/

// *** <<< Use Configuration Wizard in Context Menu >>> ***

// <o0> C_CAN0 Peripheral Clock (in Hz) <1-1000000000>
//     <i> Same as BASE_APB3_CLK
// <o1> C_CAN1 Peripheral Clock (in Hz) <1-1000000000>
//     <i> Same as BASE_APB1_CLK
#define C_CAN0_CLK           180000000
#define C_CAN1_CLK           180000000

// *** <<< End of Configuration section             >>> ***


/*----------------------------------------------------------------------------
 *      CAN RTX Hardware Specific Driver Functions
 *----------------------------------------------------------------------------
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
 *
 * Functions implemented in this module and used only in this module:
 *  static void      CAN_set_timing     (U32 ctrl, U32 tseg1, U32 tseg2, U32 sjw, U32 brp)
 *  static CAN_ERROR CAN_hw_set_baudrate(U32 ctrl, U32 baudrate)
 *  static void      CAN_hw_rd          (U32 ctrl, U32 ch, CAN_msg *msg)
 *  Interrupt function(s)
 *---------------------------------------------------------------------------*/

/* CAN Controller Register Addresses                                         */
LPC_C_CANn_Type *CAN_CTRL[] = { (LPC_C_CANn_Type *)LPC_C_CAN0_BASE, (LPC_C_CANn_Type *)LPC_C_CAN1_BASE };

/* Global variables                                                          */
U32 tx_mo [2] = { 0 };
U32 rx_mo [2] = { 0 };
U32 rtr_mo[2] = { 0 };


/************************* Auxiliary Functions *******************************/

/*--------------------------- CAN_set_timing --------------------------------
 *
 *  Setup the CAN timing with specific parameters
 *
 *  Parameter:  ctrl:       Index of the hardware CAN controller (1 .. x)
 *              tseg1:      Specifies Time Segment before the sample point
 *              tseg2:      Time Segment after the sample point
 *              sjw:        Synchronisation Jump Width
 *              brp:        Baud Rate Prescaler
 *
 *---------------------------------------------------------------------------*/

static void CAN_set_timing (U32 ctrl, U32 tseg1, U32 tseg2, U32 sjw, U32 brp) {
  LPC_C_CANn_Type *CANx = CAN_CTRL[ctrl-1];
  U8  brpe;
  
  CANx->CLKDIV = 1;                   /* PCLK = C_CANx_CLK / 2               */
  brp  /= 2;

  brpe  = (brp >> 6) & 0x0F;
  brp  &= 0x3F;

  CANx->BT   = (((tseg2-1) & 0x07) << 12) | (((tseg1-1) & 0x0F) << 8) | ((((sjw-1) & 0x03) << 6) | ((brp-1) & 0x3F));
  CANx->BRPE = brpe;
}


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

  /* Load the baudrate register BT 
     so that sample point is at about 70% bit time from bit start 
     TSEG1 = 6, TSEG2 = 3 SJW = 3 => 1 CAN bit = 10 TQ, sample at 70%        */

  CAN_set_timing(ctrl, 6, 3, 3, (((ctrl == 1) ? C_CAN0_CLK : C_CAN1_CLK) / 10) / baudrate);

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
        /* Enable clock for C_CAN0                                           */
        LPC_CGU->BASE_APB3_CLK = (0    <<  0) |   /* Disable Power-down      */
                                 (1    << 11) |   /* Autoblock En            */
                                 (0x09 << 24) ;   /* Clock source PLL1       */
        LPC_CCU1->CLK_APB3_CAN0_CFG = 1;          /* Run enable              */
        while (!(LPC_CCU1->CLK_APB3_CAN0_CFG&1)); /* Wait for run enable     */

        /* Enable C_CAN0 pins                                                */
        LPC_SCU->SFSP3_1  =        2  | /* P3_1 - CAN0_RD                    */
                             (1 << 4) | /*        EPUN = 1 (disable pull-up) */
                             (1 << 6) ; /*        EZI  = 1 (input buffer en) */
        LPC_SCU->SFSP3_2  =        2  | /* P3_2 - CAN0_TD                    */
                             (1 << 4) ; /*        EPUN = 1 (disable pull-up) */

        /* Enable C_CAN0 interrupt                                           */
        NVIC_SetPriority (C_CAN0_IRQn, 1);  /* Set C_CAN0 irq priority       */
        NVIC_EnableIRQ  (C_CAN0_IRQn);      /* Enable C_CAN0 interrupt       */
      #endif
      break;
    case 2: 
      #if USE_CAN_CTRL2 == 1
        /* Enable clock for C_CAN1                                           */
        LPC_CGU->BASE_APB1_CLK = (0    <<  0) |   /* Disable Power-down      */
                                 (1    << 11) |   /* Autoblock En            */
                                 (0x09 << 24) ;   /* Clock source PLL1       */
        LPC_CCU1->CLK_APB1_CAN1_CFG = 1;          /* Run enable              */
        while (!(LPC_CCU1->CLK_APB1_CAN1_CFG&1)); /* Wait for run enable     */

        /* Enable C_CAN1 pins                                                */
        LPC_SCU->SFSP4_9  =        6  | /* P4_9 - CAN1_RD                    */
                             (1 << 4) | /*        EPUN = 1 (disable pull-up) */
                             (1 << 6) ; /*        EZI = 1 (input buffer en)  */
        LPC_SCU->SFSP4_8  =        6  | /* P4_8 - CAN1_TD                    */
                             (1 << 4) ; /*        EPUN = 1 (disable pull-up) */

        /* Enable C_CAN1 interrupt                                           */
        NVIC_SetPriority (C_CAN1_IRQn, 2);  /* Set C_CAN1 irq priority       */
        NVIC_EnableIRQ  (C_CAN1_IRQn);      /* Enable C_CAN1 interrupt       */
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
  LPC_C_CANn_Type *CANx = CAN_CTRL[ctrl-1];
  U32 i;

  tx_mo [ctrl-1] = 0;                 /* Clear transmit message objects mask */
  rx_mo [ctrl-1] = 0;                 /* Clear receive  message objects mask */
  rtr_mo[ctrl-1] = 0;                 /* Clear RTR      message objects mask */

  CANx->CNTL  =  (1 << 6) |           /* Configuration change enable         */
                 (1 << 0) ;           /* Initialization                      */

  if (CAN_hw_set_baudrate(ctrl, baudrate) != CAN_OK)         /* Set baudrate */
    return CAN_BAUDRATE_ERROR;

  /* Initialize all objects as unused                                        */
  while (CANx->IF1_CMDREQ & 0x8000);
  CANx->IF1_ARB2 = 0;
  CANx->IF1_ARB1 = 0;
  for (i = 1; i <= 32; i++) {
    CANx->IF1_CMDMSK_W = 0xA0;        /* Set ARB = 0                         */
    CANx->IF1_CMDREQ   = i;           /* Execute IF command                  */
    while (CANx->IF1_CMDREQ & 0x8000);
    CANx->IF1_CMDMSK_R = 0x08;        /* CLRINTPND                           */
    CANx->IF1_CMDREQ   = i;           /* Execute IF command                  */
    while (CANx->IF1_CMDREQ & 0x8000);
  }
  CANx->STAT = 0;                     /* Clear interrupt statuses            */

  CANx->CNTL &= ~(1 << 6);            /* Configuration change disable        */

  return CAN_OK;
}


/*--------------------------- CAN_hw_start ----------------------------------
 *
 *  Enable the CAN interrupts (recive or/and transmit) and enable the CAN 
 *  controller
 *
 *  Parameter:  ctrl:       Index of the hardware CAN controller (1 .. x)
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_start (U32 ctrl)  {
  LPC_C_CANn_Type *CANx = CAN_CTRL[ctrl-1];

  CANx->CNTL &= ~(1 << 0);            /* Normal operation mode               */
  while (CANx->CNTL & (1 << 0));

  CANx->CNTL |=  (1 << 1);            /* Module interrupt enable             */
  while (!(CANx->CNTL & (1 << 1)));

  return CAN_OK;
}

/*--------------------------- CAN_set_testmode ------------------------------
 *
 *  Enable the CAN testmode
 *
 *  Parameter:  ctrl:       Index of the hardware CAN controller (1 .. x)
 *              testmode:   Test mode type
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_testmode (U32 ctrl, U32 testmode) {
  LPC_C_CANn_Type *CANx = CAN_CTRL[ctrl-1];

  CANx->CNTL |= (1 << 7);             /* Enable test mode                    */
  CANx->TEST  = testmode;             /* Set test mode type                  */

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
  LPC_C_CANn_Type *CANx = CAN_CTRL[ctrl-1];

  if ((os_sem_wait (wr_sem[ctrl-1], 0) != OS_R_TMO)){ /* If semaphore is free*/
    if (!(CANx->IF1_CMDREQ&(1<<15)))  /* Message interface 1 not busy        */
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
  LPC_C_CANn_Type *CANx = CAN_CTRL[ctrl-1];
  U8   ch; 
  BOOL remote_type, standard_id;

  remote_type = (msg->type   == REMOTE_FRAME);
  standard_id = (msg->format == STANDARD_FORMAT);
  ch          = (msg->ch);

  while (CANx->IF1_CMDREQ & 0x8000);  /* Wait while IF1 is busy              */

  /* Setup data bytes                                                        */
  CANx->IF1_DA1 = ((U32)msg->data[1] << 8) | ((U32)msg->data[0]);
  CANx->IF1_DA2 = ((U32)msg->data[3] << 8) | ((U32)msg->data[2]);
  CANx->IF1_DB1 = ((U32)msg->data[5] << 8) | ((U32)msg->data[4]);
  CANx->IF1_DB2 = ((U32)msg->data[7] << 8) | ((U32)msg->data[6]);

  /* Setup the identifier information                                        */
  if (standard_id)  {                                      /* Standard ID    */
    CANx->IF1_MSK1 =  0;
    CANx->IF1_MSK2 = (U32)(msg->id <<  2) | (1 << 14);
    CANx->IF1_ARB1 =  0;
    CANx->IF1_ARB2 = (U32)(msg->id <<  2) | ((!remote_type) << 13) | (1 << 15);
  }  else  {                                               /* Extended ID    */
    CANx->IF1_MSK1 = (U32)(msg->id      );
    CANx->IF1_MSK2 = (U32)(msg->id >> 16) | (1 << 15) | (1 << 14);
    CANx->IF1_ARB1 = (U32)(msg->id      );
    CANx->IF1_ARB2 = (U32)(msg->id >> 16) | ((!remote_type) << 13) | (1 << 14) | (1 << 15);
  }

  /* Setup transfer parameters                                               */
  CANx->IF1_MCTRL = (msg->len & 0x0F)   |  /* DLC - data lenght code         */
                    (1 <<  7)           |  /* EOB - end of buffer            */
                    (1 <<  8)           |  /* TXRQST - transmit request      */
                    (remote_type<<9)    |  /* RMTEN - remote enable          */
                    (remote_type<<10)   |  /* RXIE - receive interrupt en    */
                    ((!remote_type)<<11)|  /* TXIE - transmit interrupt en   */
                    (1 << 12)           ;  /* UMASK - use acceptance mask    */

  CANx->IF1_CMDMSK_W = 0xF3;          /* Access all except TXRQST, CLRINTPND */
  if (remote_type) {
    rtr_mo[ctrl-1]  |= (1 << (ch-1)); /* Mark msg object as used to send RTR */
    rx_mo [ctrl-1]  |= (1 << (ch-1)); /* and receive answer                  */
  } else {
    tx_mo [ctrl-1]  |= (1 << (ch-1)); /* Mark msg object as used for transmit*/
  }
  CANx->IF1_CMDREQ   = ch;            /* Use message object 1 for transfers  */
  while (CANx->IF1_CMDREQ & 0x8000);  /* Wait while IF1 is busy              */

  return CAN_OK;
}


/*--------------------------- CAN_hw_rd -------------------------------------
 *
 *  Read CAN_msg from the hardware registers of the requested controller
 *
 *  Parameter:  ctrl:       Index of the hardware CAN controller (1 .. x)
 *              ch:         Index of object used for reception
 *              msg:        Pointer where CAN message will be read
 *
 *  Return:     none
 *---------------------------------------------------------------------------*/

static void CAN_hw_rd (U32 ctrl, U32 ch, CAN_msg *msg)  {
  LPC_C_CANn_Type *CANx = CAN_CTRL[ctrl-1];

  while (CANx->IF2_CMDREQ & 0x8000);  /* Wait while IF2 is busy              */
  CANx->IF2_CMDMSK_R = 0x7F;          /* Access all fields in read request   */
  CANx->IF2_CMDREQ   = ch;            /* Message object ch                   */
  while (CANx->IF2_CMDREQ & 0x8000);  /* Wait while IF2 is busy              */
  
  /* Read identifier information                                             */
  if (!(CANx->IF2_ARB2 & (1 << 14))) {                      /* Standard ID   */
    msg->format = STANDARD_FORMAT;
    msg->id     = 0x000007FFUL & (CANx->IF2_ARB2 >> 2);
  }  else  {                                                /* Extended ID   */
    msg->format = EXTENDED_FORMAT;
    msg->id     = 0x1FFFFFFFUL & ((CANx->IF2_ARB2 << 16) | (CANx->IF2_ARB1 & 0xFFFF));
  }

  /* Read type information                                                   */
  msg->type     = DATA_FRAME;                               /* DATA   FRAME  */

  /* Read length (number of received bytes)                                  */
  msg->len      = CANx->IF2_MCTRL & 0x0F;

  /* Read data bytes                                                         */
  msg->data[0] = (CANx->IF2_DA1);
  msg->data[1] = (CANx->IF2_DA1 >> 8);
  msg->data[2] = (CANx->IF2_DA2);
  msg->data[3] = (CANx->IF2_DA2 >> 8);
  msg->data[4] = (CANx->IF2_DB1);
  msg->data[5] = (CANx->IF2_DB1 >> 8);
  msg->data[6] = (CANx->IF2_DB2);
  msg->data[7] = (CANx->IF2_DB2 >> 8);
}


/*--------------------------- CAN_hw_set ------------------------------------
 *  Set a message that will automatically be sent as an answer to the REMOTE
 *  FRAME message
 *
 *  Parameter:  ctrl:       Index of the hardware CAN controller (1 .. x)
 *              msg:        Pointer to CAN message to be set
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_set (U32 ctrl, CAN_msg *msg)  {
  LPC_C_CANn_Type *CANx = CAN_CTRL[ctrl-1];
  U8   ch; 
  BOOL standard_id;

  standard_id = (msg->format == STANDARD_FORMAT);
  ch          = (msg->ch);

  while (CANx->IF1_CMDREQ & 0x8000);  /* Wait while IF1 is busy              */

  /* Setup data bytes                                                        */
  CANx->IF1_DA1 = ((U32)msg->data[1] << 8) | ((U32)msg->data[0]);
  CANx->IF1_DA2 = ((U32)msg->data[3] << 8) | ((U32)msg->data[2]);
  CANx->IF1_DB1 = ((U32)msg->data[5] << 8) | ((U32)msg->data[4]);
  CANx->IF1_DB2 = ((U32)msg->data[7] << 8) | ((U32)msg->data[6]);

  /* Setup the identifier information                                        */
  if (standard_id)  {                                      /* Standard ID    */
    CANx->IF1_MSK1 =  0;
    CANx->IF1_MSK2 = (U32)(msg->id <<  2) | (1 << 14);
    CANx->IF1_ARB1 =  0;
    CANx->IF1_ARB2 = (U32)(msg->id <<  2) | (1 << 13) | (1 << 15);
  }  else  {                                               /* Extended ID    */
    CANx->IF1_MSK1 = (U32)(msg->id      );
    CANx->IF1_MSK2 = (U32)(msg->id >> 16) | (1 << 15) | (1 << 14);
    CANx->IF1_ARB1 = (U32)(msg->id      );
    CANx->IF1_ARB2 = (U32)(msg->id >> 16) | (1 << 13) | (1 << 14) | (1 << 15);
  }

  /* Setup transfer parameters                                               */
  CANx->IF1_MCTRL = (msg->len & 0x0F)   |  /* DLC - data lenght code         */
                    (1 <<  7)           |  /* EOB - end of buffer            */
                    (1 <<  9)           |  /* RMTEN - remote enable          */
                    (1 << 11)           |  /* TXIE - transmit interrupt en   */
                    (1 << 12)           ;  /* UMASK - use acceptance mask    */

  CANx->IF1_CMDMSK_W = 0xF3;          /* Access all except TXRQST, CLRINTPND */
  rtr_mo[ctrl-1]    |= (1 << (ch-1)); /* Mark msg object as used to rece RTR */
  tx_mo [ctrl-1]    |= (1 << (ch-1)); /* and transmit answer                 */
  CANx->IF1_CMDREQ   = ch;            /* Use message object 1 for transfers  */
  while (CANx->IF1_CMDREQ & 0x8000);  /* Wait while IF1 is busy              */

  return CAN_OK;
}


/*--------------------------- CAN_hw_rx_object ------------------------------
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
 *---------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_rx_object (U32 ctrl, U32 ch, U32 id, U32 object_para)  {
  LPC_C_CANn_Type *CANx = CAN_CTRL[ctrl-1];
  BOOL remote_type, standard_id;

  if ((tx_mo[ctrl-1] & (1 << (ch-1))))
    return (CAN_OBJECTS_FULL_ERROR);   /* If msg object used by transmit     */

  remote_type = (object_para &  FRAME_TYPE) == REMOTE_TYPE;
  standard_id = (object_para & FORMAT_TYPE) == STANDARD_TYPE;
  
  while (CANx->IF1_CMDREQ & 0x8000);  /* Wait while IF1 is busy              */

  /* Setup the identifier information                                        */
  if (standard_id)  {                                      /* Standard ID    */
    CANx->IF1_MSK1 = 0;
    CANx->IF1_MSK2 = (U32)(id <<  2) | (1 << 15) | (1 << 14);
    CANx->IF1_ARB1 = 0;
    CANx->IF1_ARB2 = (U32)(id <<  2) | ((remote_type) << 13) | (1 << 15);
  }  else  {                                               /* Extended ID    */
    CANx->IF1_MSK1 = (U32)(id      );
    CANx->IF1_MSK2 = (U32)(id >> 16) | (1 << 15) | (1 << 14);
    CANx->IF1_ARB1 = (U32)(id      );
    CANx->IF1_ARB2 = (U32)(id >> 16) | ((remote_type) << 13) | (1 << 14) | (1 << 15);
  }

  /* Setup transfer parameters                                               */
  CANx->IF1_MCTRL = (8 &0x0F)         |    /* DLC - data lenght code         */
                    (1 <<  7)         |    /* EOB - end of buffer            */
                    (remote_type << 9)|    /* RMTEN - remote enable          */
                    (1 << 10)         |    /* RXIE - receive interrupt en    */
                    (1 << 12)         ;    /* UMASK - use acceptance mask    */

  CANx->IF1_CMDMSK_W = 0xF3;          /* Access all except TXRQST, CLRINTPND */
  rx_mo[ctrl-1]     |= (1 << (ch-1)); /* Mark msg object as used for receive */
  CANx->IF1_CMDREQ   = ch;            /* Use message object ch for receive   */
  while (CANx->IF1_CMDREQ & 0x8000);  /* Wait while IF1 is busy              */

  return CAN_OK;
}

/*--------------------------- CAN_hw_tx_object ------------------------------
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
 *---------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_tx_object (U32 ctrl, U32 ch, U32 object_para)  {

  return CAN_NOT_IMPLEMENTED_ERROR;
}


/************************* Interrupt Functions *******************************/

/*--------------------------- CANx_IRQHandler -------------------------------
 *
 *  CAN interrupt functions 
 *  If transmit interrupt occured and there are messages in mailbox for 
 *  transmit it writes it to hardware and starts the transmission
 *  If receive interrupt occured it reads message from hardware registers 
 *  and puts it into receive mailbox
 *---------------------------------------------------------------------------*/

#define CTRL0                 0
#define CTRL                  1
#define CANc                  LPC_C_CAN0

void CAN0_IRQHandler (void) {
  CAN_msg *ptrmsg;
  U32      ch, stat, nd;
  
  while (1) {
    ch   = CANc->INT;
    if (!ch) 
      break;
    nd   = (((U32)CANc->ND2) << 16) | CANc->ND1;
    stat = CANc->STAT;
    if (ch && ch <= 0x20) {
      if (rtr_mo[CTRL0] & (1 << (ch-1))) {     /* If RTR transmitted or 
                                                  received                   */
        if (tx_mo[CTRL0] & (1 << (ch-1))) {    
                                               /* If RTR received and data 
                                                  should be sent as answer   */
          rtr_mo[CTRL0] &= ~(1 << (ch-1));     /* Just remove RTR bit and  
                                                  automatically answer with 
                                                  prepared data              */
          CANc->STAT = stat & ~(1 << 4);       /* Clear RX status            */
        } else if (rx_mo[CTRL0] & (1 << (ch-1))) {
                                               /* If RTR transmitted and data 
                                                  should be received         */
          rtr_mo[CTRL0] &= ~(1 << (ch-1));     /* Just remove RTR bit and 
                                                  check if there is anything
                                                  else to be sent            */
          /* If there is a message in the mailbox ready for send, read the 
             message from the mailbox and send it                            */
          if (isr_mbx_receive (MBX_tx_ctrl[CTRL0], (void **)&ptrmsg) != OS_R_OK) {
            CAN_hw_wr (CTRL, ptrmsg);
            _free_box(CAN_mpool, ptrmsg);
          } else {
            isr_sem_send(wr_sem[CTRL0]);       /* Return a token back to sem */
          }
          CANc->STAT = stat & ~(1 << 3);       /* Clear TX status            */
        }
      }
      else if (tx_mo[CTRL0] & (1 << (ch-1))) { /* If data transmitted        */
        /* Release message object used for transmit                          */
        while (CANc->IF2_CMDREQ & 0x8000);     /* Wait while IF2 is busy     */
        CANc->IF2_CMDMSK_R =  (1 <<  5);       /* Access only ARB bits       */
        CANc->IF2_CMDREQ   =   ch;             /* Read ARB bits              */
        while (CANc->IF2_CMDREQ & 0x8000);     /* Wait while IF2 is busy     */
        CANc->IF2_ARB2    &= ~(1 << 15);       /* Clear MSGVAL bit in ARB2   */
        CANc->IF2_CMDMSK_W =   0xA0;           /* Access only ARB bits       */
        CANc->IF2_CMDREQ   =   ch;             /* Write ARB bits             */
        while (CANc->IF2_CMDREQ & 0x8000);     /* Wait while IF2 is busy     */
        CANc->IF2_CMDMSK_R = (1 << 3);         /* Access only CLRINTPND bit  */
        CANc->IF2_CMDREQ   =   ch;             /* Clear INTPND bit           */
        while (CANc->IF2_CMDREQ & 0x8000);     /* Wait while IF2 is busy     */
        tx_mo[CTRL0] &= ~(1 << (ch-1));        /* Clear obj as not used by tx*/
        /* If there is a message in the mailbox ready for send, read the 
           message from the mailbox and send it                              */
        if (isr_mbx_receive (MBX_tx_ctrl[CTRL0], (void **)&ptrmsg) != OS_R_OK) {
          CAN_hw_wr (CTRL, ptrmsg);
          _free_box(CAN_mpool, ptrmsg);
        } else {
          isr_sem_send(wr_sem[CTRL0]);/* Return a token back to semaphore    */
        }
        CANc->STAT = stat & ~(1 << 3);         /* Clear TX status            */
      }
      else if (rx_mo[CTRL0] & nd & (1 << (ch-1))) { 
                                               /* If receive msg obj int and 
                                                  new data awailable         */
        /* If the mailbox isn't full read the message from the hardware and
           send it to the message queue                                      */
        if (isr_mbx_check (MBX_rx_ctrl[CTRL0]) > 0)  {
          ptrmsg = _alloc_box (CAN_mpool);
          if (ptrmsg) {
            CAN_hw_rd (CTRL, ch, ptrmsg);      /* Read received message      */
            isr_mbx_send (MBX_rx_ctrl[CTRL0], ptrmsg);
          }
        }
        CANc->STAT = stat & ~(1 << 4);         /* Clear RX status            */
      }
    } else if (ch) {                           /* If error interrupt         */
      CANc->STAT = stat & ~(7 << 0);           /* Clear LEC status           */
    }
  }
}


#undef  CTRL0
#define CTRL0                 1
#undef  CTRL
#define CTRL                  2
#undef  CANc
#define CANc                  LPC_C_CAN1

void CAN1_IRQHandler (void) {
  CAN_msg *ptrmsg;
  U32      ch, stat, nd;
  
  while (1) {
    ch   = CANc->INT;
    if (!ch) 
      break;
    nd   = (((U32)CANc->ND2) << 16) | CANc->ND1;
    stat = CANc->STAT;
    if (ch && ch <= 0x20) {
      if (tx_mo[CTRL0] & (1 << (ch-1))) {      /* If transmit msg obj int    */
        /* Release message object used for transmit                          */
        while (CANc->IF2_CMDREQ & 0x8000);     /* Wait while IF2 is busy     */
        CANc->IF2_CMDMSK_R =  (1 <<  5);       /* Access only ARB bits       */
        CANc->IF2_CMDREQ   =   ch;             /* Read ARB bits              */
        while (CANc->IF2_CMDREQ & 0x8000);     /* Wait while IF2 is busy     */
        CANc->IF2_ARB2    &= ~(1 << 15);       /* Clear MSGVAL bit in ARB2   */
        CANc->IF2_CMDMSK_W =   0xA0;           /* Access only ARB bits       */
        CANc->IF2_CMDREQ   =   ch;             /* Write ARB bits             */
        while (CANc->IF2_CMDREQ & 0x8000);     /* Wait while IF2 is busy     */
        CANc->IF2_CMDMSK_R = (1 << 3);         /* Access only CLRINTPND bit  */
        CANc->IF2_CMDREQ   =   ch;             /* Clear INTPND bit           */
        while (CANc->IF2_CMDREQ & 0x8000);     /* Wait while IF2 is busy     */
        tx_mo[CTRL0] &= ~(1 << (ch-1));        /* Clear obj as not used by tx*/
        /* If there is a message in the mailbox ready for send, read the 
           message from the mailbox and send it                              */
        if (isr_mbx_receive (MBX_tx_ctrl[CTRL0], (void **)&ptrmsg) != OS_R_OK) {
          CAN_hw_wr (CTRL, ptrmsg);
          _free_box(CAN_mpool, ptrmsg);
        } else {
          isr_sem_send(wr_sem[CTRL0]);/* Return a token back to semaphore    */
        }
        CANc->STAT = stat & ~(1 << 3);         /* Clear TX status            */
      }
      if (rx_mo[CTRL0] & nd & (1 << (ch-1))) { /* If receive msg obj int & nd*/
        /* If the mailbox isn't full read the message from the hardware and
           send it to the message queue                                      */
        if (isr_mbx_check (MBX_rx_ctrl[CTRL0]) > 0)  {
          ptrmsg = _alloc_box (CAN_mpool);
          if (ptrmsg) {
            CAN_hw_rd (CTRL, ch, ptrmsg);      /* Read received message      */
            isr_mbx_send (MBX_rx_ctrl[CTRL0], ptrmsg);
          }
        }
        CANc->STAT = stat & ~(1 << 4);         /* Clear RX status            */
      }
    } else if (ch) {                           /* If error interrupt         */
      CANc->STAT = stat & ~(7 << 0);           /* Clear LEC status           */
    }
  }
}


/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/

