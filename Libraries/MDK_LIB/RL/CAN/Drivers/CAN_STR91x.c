/*----------------------------------------------------------------------------
 *      RL-ARM - CAN
 *----------------------------------------------------------------------------
 *      Name:    CAN_STR91x.c
 *      Purpose: CAN Driver, Hardware specific module for ST STR91x
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <RTL.h>                      /* RTX kernel functions & defines      */
#include <RTX_CAN.h>                  /* CAN Generic functions & defines     */
#include <91x_lib.h>                  /* STR91x library definitions          */


/*----------------------------------------------------------------------------
 *      CAN RTX Hardware Specific Driver Functions
 *----------------------------------------------------------------------------
 *  Functions implemented in this module:
 *    static CAN_ERROR CAN_hw_set_baudrate (U32 ctrl, U32 baudrate)
 *           CAN_ERROR CAN_hw_setup        (U32 ctrl)
 *           CAN_ERROR CAN_hw_init         (U32 ctrl, U32 baudrate)
 *           CAN_ERROR CAN_hw_start        (U32 ctrl)
 *           CAN_ERROR CAN_hw_tx_empty     (U32 ctrl)
 *           CAN_ERROR CAN_hw_wr           (U32 ctrl,         CAN_msg *msg)
 *    static      void CAN_hw_rd           (U32 ctrl, U32 ch, CAN_msg *msg)
 *           CAN_ERROR CAN_hw_set          (U32 ctrl,         CAN_msg *msg)
 *           CAN_ERROR CAN_hw_rx_object    (U32 ctrl, U32 ch, U32 id, U32 object_para)
 *           CAN_ERROR CAN_hw_tx_object    (U32 ctrl, U32 ch,         U32 object_para)
 *    Interrupt fuction
 *---------------------------------------------------------------------------*/

/* Static functions used only in this module                                 */
static void CAN_hw_rd                (U32 ctrl, U32 ch, CAN_msg *msg);
static void CAN_IRQ_Handler          (void) __irq;


/************************* Auxiliary Functions *******************************/

/*--------------------------- CAN_set_baudrate ------------------------------
 *
 *  Setup the requested baudrate
 *
 *  Parameter:  ctrl:       Ignored
 *              baudrate:   Baudrate
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

static CAN_ERROR CAN_hw_set_baudrate (U32 ctrl, U32 baudrate)  {
  U32 brp = SCU_GetPCLKFreqValue() * 1000;  /* CAN clock in MHz              */

  /* Determine which nominal time to use for requested baudrate and set
     appropriate bit timing                                                  */
  if (baudrate <= 500000)  {
    brp  = (brp / 16) / baudrate;
                                                                          
    /* Load the baudrate registers BTR and BRPR registers                    */
    CAN_SetTiming( 11, 4, 4, brp);
  }  else if (baudrate <= 1000000)  {
    brp  = (brp / 8 ) / baudrate;

    /* Load the baudrate registers BTR and BRPR registers                    */
    CAN_SetTiming(  4, 3, 3, brp);
  }  else  {
    return CAN_BAUDRATE_ERROR;
  }  

  return CAN_OK;
}


/*************************** Module Functions ********************************/

/*--------------------------- CAN_hw_setup ----------------------------------
 *
 *  Setup CAN transmit and receive PINs and interrupt vectors
 *
 *  Parameter:  ctrl:       Ignored
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_setup (U32 ctrl)  {
  GPIO_InitTypeDef  GPIO_InitStruct;    /* Declare GPIO  structure           */

  /* 1. Enable CAN controller Clock - enabled in startup file                */

  /* 2. Setup CAN Tx and Rx pins                                             */
  GPIO_InitStruct.GPIO_Pin              = GPIO_Pin_0;       /* Setup Rx pin  */
  GPIO_InitStruct.GPIO_Direction        = GPIO_PinInput;
  GPIO_InitStruct.GPIO_IPInputConnected = GPIO_IPInputConnected_Enable;
  GPIO_Init(GPIO5, &GPIO_InitStruct);
  GPIO_InitStruct.GPIO_Pin              = GPIO_Pin_1;       /* Setup Tx pin  */
  GPIO_InitStruct.GPIO_Direction        = GPIO_PinOutput;                    
  GPIO_InitStruct.GPIO_IPInputConnected = GPIO_IPInputConnected_Disable;
  GPIO_InitStruct.GPIO_Type             = GPIO_Type_PushPull;
  GPIO_InitStruct.GPIO_Alternate        = GPIO_OutputAlt2;
  GPIO_Init(GPIO5, &GPIO_InitStruct);

  /* 3. Setup IRQ vector for CAN interrupt                                   */
  VIC0->VAiR[CAN_ITLine]  = (U32)CAN_IRQ_Handler;   /* Setup IRQ handler addr*/
  VIC0->VCiR[CAN_ITLine] |= 0x20;                   /* Enable the vector intr*/
  VIC0->VCiR[CAN_ITLine] |= CAN_ITLine;             /* Specify interupt nmber*/

  /* 4. Enable CAN interrupt                                                 */
  VIC0->INTER            |= (1<<CAN_ITLine);        /* Enable CAN interrupt  */

  return CAN_OK;
}

/*--------------------------- CAN_hw_init -----------------------------------
 *
 *  Initialize the CAN hardware
 *
 *  Parameter:  ctrl:       Ignored
 *              baudrate:   Baudrate
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_init (U32 ctrl, U32 baudrate)  {

  /* Enable interrrupt, enter init mode, enable automatic retransmission     */
  CAN_EnterInitMode(CAN_CR_CCE | CAN_CR_IE /*| CAN_CR_DAR*/);  

  if (CAN_hw_set_baudrate(ctrl, baudrate) != CAN_OK)    /* Set baudrate      */
    return CAN_BAUDRATE_ERROR;

  CAN_SetUnusedAllMsgObj();           /* Initialize all objects as unused    */

  return CAN_OK;
}

/*--------------------------- CAN_hw_start ----------------------------------
 *
 *  Enable the CAN interrupts
 *
 *  Parameter:  ctrl:       Ignored
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_start (U32 ctrl)  {

  CAN_LeaveInitMode ();               /* Enter normal operating mode         */ 

  return CAN_OK;
}

/*--------------------------- CAN_hw_tx_empty -------------------------------
 *
 *  Check if interface register group 0 (IF0) is available for usage
 *
 *  Parameter:  ctrl:       Ignored
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_tx_empty (U32 ctrl)  {

  if ((os_sem_wait (wr_sem[ctrl-1], 0) != OS_R_TMO)){ /* If semaphore is free*/
  if ((CAN->sMsgObj[0].CRR & CAN_CRR_BUSY) == 0)  /* Interface 0 is free for */
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
 *  Parameter:  ctrl:       Ignored
 *              msg:        Pointer to CAN message to be written to hardware
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_wr (U32 ctrl, CAN_msg *msg)  {

  /* Set flags for informations that will be changed                         */
  /*  =    CAN_CMR_WRRD
         | CAN_CMR_ARB
         | CAN_CMR_CONTROL
         | CAN_CMR_DATAA
         | CAN_CMR_DATAB      = 0x00B3                                       */
  CAN->sMsgObj[0].CMR = 0x00B3;       

  /* Reset the MSGVAL bit to enable ID, DIR and DLC change                   */
  CAN->sMsgObj[0].A2R = 0;

  /* Setup the identifier information                                        */
  if (msg->format == STANDARD_FORMAT)  {  /* Standard ID                     */
    CAN->sMsgObj[0].A1R =    0;
    CAN->sMsgObj[0].A2R =   (U16) (msg->id << 2);
  }  else  {                              /* Extended ID                     */
    CAN->sMsgObj[0].A1R =   (U16) (msg->id);
    CAN->sMsgObj[0].A2R =   (U16)((msg->id) >> 16) | CAN_A2R_XTD;
  }

  /* Setup type information                                                  */
  if (msg->type == DATA_FRAME)  {         /* DATA FRAME                      */
    CAN->sMsgObj[0].A2R |=  CAN_A2R_DIR;
  }

  /* Setup data bytes                                                        */
  CAN->sMsgObj[0].DA1R = ((U16) msg->data[1]<<8) | msg->data[0];
  CAN->sMsgObj[0].DA2R = ((U16) msg->data[3]<<8) | msg->data[2];
  CAN->sMsgObj[0].DB1R = ((U16) msg->data[5]<<8) | msg->data[4];
  CAN->sMsgObj[0].DB2R = ((U16) msg->data[7]<<8) | msg->data[6];

  /* Setup length and message control bits                                   */
  /*  =    CAN_MCR_NEWDAT 
         | CAN_MCR_TXIE
         | CAN_MCR_TXRQST 
         | CAN_MCR_EOB        = 0x8980                                       */
  CAN->sMsgObj[0].MCR = 0x8980 | (msg->len & 0x000F);

  /* Set the message valid bit                                               */
  CAN->sMsgObj[0].A2R |= CAN_A2R_MSGVAL;

  /* Update informations from interface registers                            */
  CAN->sMsgObj[0].CRR = msg->ch;

  return CAN_OK;
}

/*--------------------------- CAN_hw_rd -------------------------------------
 *
 *  Read CAN_msg from the hardware registers of the requested controller
 *
 *  Parameter:  ctrl:       Ignored
 *              ch:         Index of object used for reception (1..32)
 *              msg:        Pointer where CAN message will be read
 *
 *  Return:     none
 *---------------------------------------------------------------------------*/

static void CAN_hw_rd (U32 ctrl, U32 ch, CAN_msg *msg)  {

  /* Read the message contents to interface registers                        */
  /* =     CAN_CMR_ARB
         | CAN_CMR_CONTROL
         | CAN_CMR_CLRINTPND
         | CAN_CMR_DATAA
         | CAN_CMR_DATAB      = 0x003B                                       */
  CAN->sMsgObj[1].CMR = 0x003B;

  /* Request read to interface registers                                     */
  CAN->sMsgObj[1].CRR = ch;

  /* Read identifier information                                             */
  if ((CAN->sMsgObj[1].A2R & CAN_A2R_XTD) == 0)  {  /* Standard ID           */
    msg->format = STANDARD_FORMAT;
    msg->id     = (CAN->sMsgObj[1].A2R >> 2) & 0x07FF;
  }  else  {                                        /* Extended ID           */
    msg->format = EXTENDED_FORMAT;
    msg->id     =  (((U32)(CAN->sMsgObj[1].A2R) & 0x1FFF) << 16)
                  | ((U32)(CAN->sMsgObj[1].A1R));
  }

  /* Read type information                                                   */
  if ((CAN->sMsgObj[1].A2R & 0x2000) == 0x2000)  {          /* DATA   FRAME  */
    msg->type =   DATA_FRAME;
  }  else  {                                                /* REMOTE FRAME  */
    msg->type = REMOTE_FRAME;
  }

  /* Read length (number of received bytes)                                  */
  msg->len = CAN->sMsgObj[1].MCR & 0x0F;

  /* Read data bytes                                                         */
  msg->data[0] = (U8) CAN->sMsgObj[1].DA1R;
  msg->data[1] = (U8)(CAN->sMsgObj[1].DA1R >> 8);
  msg->data[2] = (U8) CAN->sMsgObj[1].DA2R;
  msg->data[3] = (U8)(CAN->sMsgObj[1].DA2R >> 8);
  msg->data[4] = (U8) CAN->sMsgObj[1].DB1R;
  msg->data[5] = (U8)(CAN->sMsgObj[1].DB1R >> 8);
  msg->data[6] = (U8) CAN->sMsgObj[1].DB2R;
  msg->data[7] = (U8)(CAN->sMsgObj[1].DB2R >> 8);
}

/*--------------------------- CAN_hw_set ------------------------------------
 *  Set a message that will automatically be sent as an answer to the REMOTE
 *  FRAME message
 *
 *  Parameter:  ctrl:       Ignored
 *              msg:        Pointer to CAN message to be set
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_set (U32 ctrl, CAN_msg *msg)  {

  /* Update the contents needed for reply on REMOTE FRAME                    */
  /* =     CAN_CMR_WRRD
         | CAN_CMR_ARB
         | CAN_CMR_CONTROL
         | CAN_CMR_DATAA
         | CAN_CMR_DATAB      = 0x00B3                                       */
  CAN->sMsgObj[0].CMR = 0x00B3;

  /* Reset the message valid bit to enable ID, DIR and DLC change            */
  CAN->sMsgObj[0].A2R = 0;

  /* Setup the identifier information                                        */
  if (msg->format == STANDARD_FORMAT)  {  /* Standard ID                     */   
    CAN->sMsgObj[0].A1R = 0;
    CAN->sMsgObj[0].A2R = (U16) (msg->id << 2);
  }  else  {                              /* Extended ID                     */
    CAN->sMsgObj[0].A1R = (U16) (msg->id);
    CAN->sMsgObj[0].A2R = (U16)((msg->id) >> 16) | CAN_A2R_XTD;
  }

  /* Setup type information                                                  */
  if (msg->type == DATA_FRAME)  {     /* DATA FRAME                          */
    CAN->sMsgObj[0].A2R |= CAN_A2R_DIR;
  }

  /* Setup data bytes                                                        */
  CAN->sMsgObj[0].DA1R = ((U16) msg->data[1]<<8) | msg->data[0];
  CAN->sMsgObj[0].DA2R = ((U16) msg->data[3]<<8) | msg->data[2];
  CAN->sMsgObj[0].DB1R = ((U16) msg->data[5]<<8) | msg->data[4];
  CAN->sMsgObj[0].DB2R = ((U16) msg->data[7]<<8) | msg->data[6];

  /* Setup length and message control bits                                   */
  /* =     CAN_MCR_NEWDAT
         | CAN_MCR_RMTEN 
         | CAN_MCR_EOB        = 0x8280                                       */
  CAN->sMsgObj[0].MCR = 0x8280 | (msg->len & 0x000F);

  /* Set the message valid bit                                               */
  CAN->sMsgObj[0].A2R |= CAN_A2R_MSGVAL;

  /* Update from interface registers                                         */
  CAN->sMsgObj[0].CRR = msg->ch;

  return CAN_OK;
}

/*--------------------------- CAN_hw_rx_object ------------------------------
 *
 *  This function setups object that is going to be used for the message 
 *  reception
 *
 *  Parameter:  ctrl:       Ignored
 *              ch:         Index of object used for reception
 *              id:         Identifier of receiving messages
 *              object_para:Object parameters (standard or extended format, 
 *                          data or remote frame)
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_rx_object (U32 ctrl, U32 ch, U32 id, U32 object_para)  {
  U32 free_if;

  /* Wait for the first free interface register                              */
  do  {
    if ((CAN->sMsgObj[0].CRR & CAN_CRR_BUSY) == 0)  {
      free_if = 0;
      break;
    }  else if ((CAN->sMsgObj[1].CRR & CAN_CRR_BUSY) == 0)  {
      free_if = 1;
      break;
    }
  }  while (1);

  /* Set flags for informations that will be changed                         */
  /*  =    CAN_CMR_WRRD
         | CAN_CMR_MASK
         | CAN_CMR_ARB
         | CAN_CMR_CONTROL
         | CAN_CMR_DATAA
         | CAN_CMR_DATAB      = 0x00F3                                       */
  CAN->sMsgObj[free_if].CMR = 0x00F3;

  /* Setup the identifier information                                        */
  if ((object_para & FORMAT_TYPE) == STANDARD_TYPE)  {  /* Standard ID       */
    CAN->sMsgObj[free_if].M1R = 0;
    CAN->sMsgObj[free_if].M2R = (U16)(0x7FF << 2);

    CAN->sMsgObj[free_if].A1R = 0;
    CAN->sMsgObj[free_if].A2R = CAN_A2R_MSGVAL | (U16)(id << 2);
  }  else  {                                            /* Extended ID       */
    CAN->sMsgObj[free_if].M1R = (U16)(0xFFFF);
    CAN->sMsgObj[free_if].M2R = CAN_M2R_MXTD | (U16)(0x1FFF);

    CAN->sMsgObj[free_if].A1R = (U16)(id);
    CAN->sMsgObj[free_if].A2R = CAN_A2R_MSGVAL | CAN_A2R_XTD | (U16)(id >> 16);
  }

  /* Setup length and message control bits                                   */
  /* =     CAN_MCR_UMASK
         | CAN_MCR_RXIE  
         | CAN_MCR_EOB        = 0x1480                                       */
  CAN->sMsgObj[free_if].MCR = 0x1480;

  /* Setup data bytes                                                        */
  CAN->sMsgObj[free_if].DA1R = 0;
  CAN->sMsgObj[free_if].DA2R = 0;
  CAN->sMsgObj[free_if].DB1R = 0;
  CAN->sMsgObj[free_if].DB2R = 0;

  /* Update from interface registers                                         */
  CAN->sMsgObj[free_if].CRR = ch;

  return CAN_OK;
}

/*--------------------------- CAN_hw_tx_object ------------------------------
 *
 *  This function setups object that is going to be used for the message 
 *  transmission, the setup of transmission object is not necessery so this 
 *  function is not implemented
 *
 *  Parameter:  ctrl:       Ignored
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

/*--------------------------- CAN_IRQ_Handler -------------------------------
 *
 *  CAN interrupt function 
 *  If transmit interrupt occured and there are messages in mailbox for 
 *  transmit it writes it to hardware and starts the transmission
 *  If receive interrupt occured it reads message from hardware registers 
 *  and puts it into receive mailbox
 *---------------------------------------------------------------------------*/

#define CTRL0                 0
#define CTRL                  1
#define CANc                  CAN

static void CAN_IRQ_Handler (void) __irq  {
  U32 ch;
  CAN_msg *ptrmsg;

  ch = CANc->IDR;                     /* Index of object that caused         */
                                      /* interrupt                           */
  if ((ch-1)<32)  {                   /* Check if index in range 1 - 32      */

    /* Wait for IF[1] to becomes free                                        */
    while ((CANc->sMsgObj[1].CRR & CAN_CRR_BUSY) != 0);

    if ((CANc->SR & CAN_SR_TXOK) == CAN_SR_TXOK)  {         /* Tx interrupt  */
      CANc->SR &= ~CAN_SR_TXOK;       /* Clear Tx Ok flag in Status register */

      CANc->sMsgObj[1].CMR = CAN_CMR_CLRINTPND;   /* Release transmit object */
      CANc->sMsgObj[1].CRR = ch;

      /* Wait until Interface Register 1 becomes free                        */
      while ((CANc->sMsgObj[1].CRR & CAN_CRR_BUSY) != 0);

        /* If there is a message in the mailbox ready for send, read the 
           message from the mailbox and send it                              */
        if (isr_mbx_receive (MBX_tx_ctrl[CTRL0], (void **)&ptrmsg) != OS_R_OK) {
          /* Wait until Interface Register 0 becomes free                    */
          while ((CANc->sMsgObj[0].CRR & CAN_CRR_BUSY) != 0);

          CAN_hw_wr (CTRL, ptrmsg);
          _free_box(CAN_mpool, ptrmsg);
      } else {
        isr_sem_send(wr_sem[CTRL0]);/* Return a token back to semaphore      */
      }
    }  else  {                                              /* Rx interrupt  */
                                      /* It should have been checked like
                                         "if ((CANc->SR & CAN_SR_RXOK) == CAN_SR_RXOK)"
                                         but isn't because in TEST mode RxOk 
                                         flag in Status Register isn't 
                                         activated upon reception of message */
      CANc->SR &= ~CAN_SR_RXOK;       /* Clear Rx Ok flag in Status register */

      CANc->sMsgObj[1].CMR = CAN_CMR_CLRINTPND;   /* Release receive object  */
      CANc->sMsgObj[1].CRR = ch;

      /* If the mailbox isn't full read the message from the hardware and
         send it to the message queue                                        */
      if (os_mbx_check (MBX_rx_ctrl[CTRL0]) > 0)  {
        /* Wait until Interface Register 1 becomes free                      */
        while ((CANc->sMsgObj[1].CRR & CAN_CRR_BUSY) != 0);

        ptrmsg = _alloc_box (CAN_mpool);
        if (ptrmsg) {
          CAN_hw_rd (CTRL, ch, ptrmsg); /* Read received message             */
          isr_mbx_send (MBX_rx_ctrl[CTRL0], ptrmsg);
        }
      }
    }
  }

  VIC0->VAR = 0;                        /* Acknowledge Interrupt              */  
  VIC1->VAR = 0;
}


/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/

