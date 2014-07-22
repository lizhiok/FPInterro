/*----------------------------------------------------------------------------
 *      RL-ARM - CAN
 *----------------------------------------------------------------------------
 *      Name:    CAN_STR73x.c
 *      Purpose: CAN Driver, Hardware specific module for ST STR73x
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <RTL.h>                      /* RTX kernel functions & defines      */
#include <RTX_CAN.h>                  /* CAN Generic functions & defines     */
#include <73x_lib.h>                  /* STR73x library definitions          */


/*----------------------------------------------------------------------------
 *      CAN RTX Hardware Specific Driver Functions
 *----------------------------------------------------------------------------
 *  Functions implemented in this module:
 *    static CAN_ERROR CAN_get_ctrl        (U32 ctrl, CAN_TypeDef **ptr_can) 
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
 *    Interrupt fuctions
 *---------------------------------------------------------------------------*/

/* Static functions used only in this module                                 */
static void CAN_hw_rd                (U32 ctrl, U32 ch, CAN_msg *msg);
#if USE_CAN_CTRL1 == 1
  static void CAN0_IRQHandler        (void) __irq;
#endif
#if USE_CAN_CTRL2 == 1
  static void CAN1_IRQHandler        (void) __irq;
#endif
#if USE_CAN_CTRL3 == 1
  static void CAN2_IRQHandler        (void) __irq;
#endif


/************************* Auxiliary Functions *******************************/

/*--------------------------- CAN_get_ctrl ----------------------------------
 *
 *  Get address and return it in parameter of function, of controller which 
 *  index is passed as parameter
 *
 *  Parameter:  ctrl:       Index of CAN controller
 *              ptr_can:    Pointer to CAN controller in memory map
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

static CAN_ERROR CAN_get_ctrl (U32 ctrl, CAN_TypeDef **ptr_can) {

  switch (ctrl) {
    case 1: 
      *ptr_can = CAN0;
      break;
    case 2: 
      *ptr_can = CAN1;
      break;
    case 3: 
      *ptr_can = CAN2;
      break;
    default:
      return CAN_UNEXIST_CTRL_ERROR;
  }

  return CAN_OK;
}

/*--------------------------- CAN_set_baudrate ------------------------------
 *
 *  Setup the requested baudrate
 *
 *  Parameter:  ctrl:       Index of CAN controller
 *              baudrate:   Baudrate
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

static CAN_ERROR CAN_hw_set_baudrate (U32 ctrl, U32 baudrate)  {
  CAN_TypeDef *CANx;
  U32 brp = PRCCU_GetFrequencyValue(PRCCU_CLOCK_MCLK);

  CAN_get_ctrl(ctrl, &CANx);          /* Get pointer to CAN controller       */

  /* Determine which nominal time to use for requested baudrate and set
     appropriate bit timing                                                  */
  if (baudrate <= 500000)  {
    brp  = (brp / 16) / baudrate;
                                                                          
    /* Load the baudrate registers BTR and BRPR registers                    */
    CAN_SetTiming(CANx, 11, 4, 4, brp);
  }  else if (baudrate <= 1000000)  {
    brp  = (brp / 8 ) / baudrate;

    /* Load the baudrate registers BTR and BRPR registers                    */
    CAN_SetTiming(CANx,  4, 3, 3, brp);
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
 *  Parameter:  none
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_setup (U32 ctrl)  {
  GPIO_InitTypeDef  GPIO_InitStruct;    /* Declare GPIO  structure           */

  switch (ctrl) {
    case 1: 
      #if USE_CAN_CTRL1 == 1
        CFG_PeripheralClockConfig(CFG_CLK_CAN0, ENABLE);    /* Enable CAN clk*/
        CFG_PeripheralClockConfig(CFG_CLK_GPIO1, ENABLE);   /* Enable GPIO   */
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_TRI_TTL;   /* Setup Rx pin  */
        GPIO_InitStruct.GPIO_Pins = GPIO_PIN_14;
        GPIO_Init(GPIO1, &GPIO_InitStruct);
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;        /* Setup Tx pin  */
        GPIO_InitStruct.GPIO_Pins = GPIO_PIN_15;
        GPIO_Init(GPIO1, &GPIO_InitStruct);
    
        /* Setup IRQ vector                                                  */
        EIC->SIRn[CAN0_IRQChannel] = (((U32)CAN0_IRQHandler) << 16);
        EIC_IRQChannelPriorityConfig (CAN0_IRQChannel, 1);
        EIC_IRQChannelConfig (CAN0_IRQChannel, ENABLE);
      #endif
      break;
    case 2: 
      #if USE_CAN_CTRL2 == 1
        CFG_PeripheralClockConfig(CFG_CLK_CAN1, ENABLE);    /* Enable CAN clk*/
        CFG_PeripheralClockConfig(CFG_CLK_GPIO2, ENABLE);   /* Enable GPIO   */
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_TRI_TTL;   /* Setup Rx pin  */
        GPIO_InitStruct.GPIO_Pins = GPIO_PIN_1;
        GPIO_Init(GPIO2, &GPIO_InitStruct);
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;        /* Setup Tx pin  */
        GPIO_InitStruct.GPIO_Pins = GPIO_PIN_2;
        GPIO_Init(GPIO2, &GPIO_InitStruct);
    
        /* Setup IRQ vector                                                  */
        EIC->SIRn[CAN1_IRQChannel] = (((U32)CAN1_IRQHandler) << 16);
        EIC_IRQChannelPriorityConfig (CAN1_IRQChannel, 1);
        EIC_IRQChannelConfig (CAN1_IRQChannel, ENABLE);
      #endif
      break;
    case 3: 
      #if USE_CAN_CTRL3 == 1
        CFG_PeripheralClockConfig(CFG_CLK_CAN2, ENABLE);    /* Enable CAN clk*/
        CFG_PeripheralClockConfig(CFG_CLK_GPIO4, ENABLE);   /* Enable GPIO   */
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_TRI_TTL;   /* Setup Rx pin  */
        GPIO_InitStruct.GPIO_Pins = GPIO_PIN_5;
        GPIO_Init(GPIO4, &GPIO_InitStruct);
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;        /* Setup Tx pin  */
        GPIO_InitStruct.GPIO_Pins = GPIO_PIN_4;
        GPIO_Init(GPIO4, &GPIO_InitStruct);
    
        /* Setup IRQ vector                                                  */
        EIC->SIRn[CAN2_IRQChannel] = (((U32)CAN2_IRQHandler) << 16);
        EIC_IRQChannelPriorityConfig (CAN2_IRQChannel, 1);
        EIC_IRQChannelConfig (CAN2_IRQChannel, ENABLE);
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
 *  Parameter:  ctrl:       Index of CAN controller
 *              baudrate:   Baudrate
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_init (U32 ctrl, U32 baudrate)  {
  CAN_TypeDef *CANx;

  CAN_get_ctrl(ctrl, &CANx);          /* Get pointer to CAN controller       */

  /* Enable interrrupt, enter init mode, enable automatic retransmission     */
  CAN_EnterInitMode(CANx, CAN_CR_CCE | CAN_CR_IE /*| CAN_CR_DAR*/);  

  if (CAN_hw_set_baudrate(ctrl, baudrate) != CAN_OK)    /* Set baudrate      */
    return CAN_BAUDRATE_ERROR;

  CAN_InvalidateAllMsgObj(CANx);      /* Initialize all objects as unused    */

  return CAN_OK;
}

/*--------------------------- CAN_hw_start ----------------------------------
 *
 *  Enable the CAN interrupts
 *
 *  Parameter:  ctrl:       Index of CAN controller
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_start (U32 ctrl)  {
  CAN_TypeDef *CANx;

  CAN_get_ctrl(ctrl, &CANx);          /* Get pointer to CAN controller       */

  CAN_LeaveInitMode (CANx);           /* Enter normal operating mode         */ 

  return CAN_OK;
}

/*--------------------------- CAN_hw_tx_empty -------------------------------
 *
 *  Check if interface register group 0 (IF0) is available for usage
 *
 *  Parameter:  ctrl:       Index of CAN controller
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_tx_empty (U32 ctrl)  {
  CAN_TypeDef *CANx;

  CAN_get_ctrl(ctrl, &CANx);            /* Get pointer to CAN controller     */

  if ((os_sem_wait (wr_sem[ctrl-1], 0) != OS_R_TMO)){ /* If semaphore is free*/
  if ((CANx->sMsgObj[0].CRR & CAN_CRR_BUSY) == 0) /* Interface 0 is free for */
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
 *  Parameter:  ctrl:       Index of CAN controller
 *              msg:        Pointer to CAN message to be written to hardware
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_wr (U32 ctrl, CAN_msg *msg)  {
  CAN_TypeDef *CANx;

  CAN_get_ctrl(ctrl, &CANx);            /* Get pointer to CAN controller     */

  /* Set flags for informations that will be changed                         */
  /*  =    CAN_CMR_WRRD
         | CAN_CMR_ARB
         | CAN_CMR_CONTROL
         | CAN_CMR_DATAA
         | CAN_CMR_DATAB      = 0x00B3                                       */
  CANx->sMsgObj[0].CMR = 0x00B3;       

  /* Reset the MSGVAL bit to enable ID, DIR and DLC change                   */
  CANx->sMsgObj[0].A2R = 0;

  /* Setup the identifier information                                        */
  if (msg->format == STANDARD_FORMAT)  {  /* Standard ID                     */
    CANx->sMsgObj[0].A1R =    0;
    CANx->sMsgObj[0].A2R =   (U16) (msg->id << 2);
  }  else  {                              /* Extended ID                     */
    CANx->sMsgObj[0].A1R =   (U16) (msg->id);
    CANx->sMsgObj[0].A2R =   (U16)((msg->id) >> 16) | CAN_A2R_XTD;
  }

  /* Setup type information                                                  */
  if (msg->type == DATA_FRAME)  {         /* DATA FRAME                      */
    CANx->sMsgObj[0].A2R |=  CAN_A2R_DIR;
  }

  /* Setup data bytes                                                        */
  CANx->sMsgObj[0].DA1R = ((U16) msg->data[1]<<8) | msg->data[0];
  CANx->sMsgObj[0].DA2R = ((U16) msg->data[3]<<8) | msg->data[2];
  CANx->sMsgObj[0].DB1R = ((U16) msg->data[5]<<8) | msg->data[4];
  CANx->sMsgObj[0].DB2R = ((U16) msg->data[7]<<8) | msg->data[6];

  /* Setup length and message control bits                                   */
  /*  =    CAN_MCR_NEWDAT 
         | CAN_MCR_TXIE
         | CAN_MCR_TXRQST 
         | CAN_MCR_EOB        = 0x8980                                       */
  CANx->sMsgObj[0].MCR = 0x8980 | (msg->len & 0x000F);

  /* Set the message valid bit                                               */
  CANx->sMsgObj[0].A2R |= CAN_A2R_MSGVAL;

  /* Update informations from interface registers                            */
  CANx->sMsgObj[0].CRR = msg->ch;

  return CAN_OK;
}

/*--------------------------- CAN_hw_rd -------------------------------------
 *
 *  Read CAN_msg from the hardware registers of the requested controller
 *
 *  Parameter:  ctrl:       Index of CAN controller
 *              ch:         Index of object used for reception (1..32)
 *              msg:        Pointer where CAN message will be read
 *
 *  Return:     none
 *---------------------------------------------------------------------------*/

static void CAN_hw_rd (U32 ctrl, U32 ch, CAN_msg *msg)  {
  CAN_TypeDef *CANx;

  CAN_get_ctrl(ctrl, &CANx);            /* Get pointer to CAN controller     */

  /* Read the message contents to interface registers                        */
  /* =     CAN_CMR_ARB
         | CAN_CMR_CONTROL
         | CAN_CMR_CLRINTPND
         | CAN_CMR_DATAA
         | CAN_CMR_DATAB      = 0x003B                                       */
  CANx->sMsgObj[1].CMR = 0x003B;

  /* Request read to interface registers                                     */
  CANx->sMsgObj[1].CRR = ch;

  /* Read identifier information                                             */
  if ((CANx->sMsgObj[1].A2R & CAN_A2R_XTD) == 0)  { /* Standard ID           */
    msg->format = STANDARD_FORMAT;
    msg->id     = (CANx->sMsgObj[1].A2R >> 2) & 0x07FF;
  }  else  {                                        /* Extended ID           */
    msg->format = EXTENDED_FORMAT;
    msg->id     =   ((U32)(CANx->sMsgObj[1].A2R) >> 16)
                  | ((U32)(CANx->sMsgObj[1].A1R) & 0x1FFF);
  }

  /* Read type information                                                   */
  if ((CANx->sMsgObj[1].A2R & 0x2000) == 0x2000)  {         /* DATA   FRAME  */
    msg->type =   DATA_FRAME;
  }  else  {                                                /* REMOTE FRAME  */
    msg->type = REMOTE_FRAME;
  }

  /* Read length (number of received bytes)                                  */
  msg->len = CANx->sMsgObj[1].MCR & 0x0F;

  /* Read data bytes                                                         */
  msg->data[0] = (U8) CANx->sMsgObj[1].DA1R;
  msg->data[1] = (U8)(CANx->sMsgObj[1].DA1R >> 8);
  msg->data[2] = (U8) CANx->sMsgObj[1].DA2R;
  msg->data[3] = (U8)(CANx->sMsgObj[1].DA2R >> 8);
  msg->data[4] = (U8) CANx->sMsgObj[1].DB1R;
  msg->data[5] = (U8)(CANx->sMsgObj[1].DB1R >> 8);
  msg->data[6] = (U8) CANx->sMsgObj[1].DB2R;
  msg->data[7] = (U8)(CANx->sMsgObj[1].DB2R >> 8);
}

/*--------------------------- CAN_hw_set ------------------------------------
 *  Set a message that will automatically be sent as an answer to the REMOTE
 *  FRAME message
 *
 *  Parameter:  ctrl:       Index of CAN controller
 *              msg:        Pointer to CAN message to be set
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_set (U32 ctrl, CAN_msg *msg)  {
  CAN_TypeDef *CANx;

  CAN_get_ctrl(ctrl, &CANx);            /* Get pointer to CAN controller     */

  /* Update the contents needed for reply on REMOTE FRAME                    */
  /* =     CAN_CMR_WRRD
         | CAN_CMR_ARB
         | CAN_CMR_CONTROL
         | CAN_CMR_DATAA
         | CAN_CMR_DATAB      = 0x00B3                                       */
  CANx->sMsgObj[0].CMR = 0x00B3;

  /* Reset the message valid bit to enable ID, DIR and DLC change            */
  CANx->sMsgObj[0].A2R = 0;

  /* Setup the identifier information                                        */
  if (msg->format == STANDARD_FORMAT)  {  /* Standard ID                     */   
    CANx->sMsgObj[0].A1R = 0;
    CANx->sMsgObj[0].A2R = (U16) (msg->id << 2);
  }  else  {                              /* Extended ID                     */
    CANx->sMsgObj[0].A1R = (U16) (msg->id);
    CANx->sMsgObj[0].A2R = (U16)((msg->id) >> 16) | CAN_A2R_XTD;
  }

  /* Setup type information                                                  */
  if (msg->type == DATA_FRAME)  {     /* DATA FRAME                          */
    CANx->sMsgObj[0].A2R |= CAN_A2R_DIR;
  }

  /* Setup data bytes                                                        */
  CANx->sMsgObj[0].DA1R = ((U16) msg->data[1]<<8) | msg->data[0];
  CANx->sMsgObj[0].DA2R = ((U16) msg->data[3]<<8) | msg->data[2];
  CANx->sMsgObj[0].DB1R = ((U16) msg->data[5]<<8) | msg->data[4];
  CANx->sMsgObj[0].DB2R = ((U16) msg->data[7]<<8) | msg->data[6];

  /* Setup length and message control bits                                   */
  /* =     CAN_MCR_NEWDAT
         | CAN_MCR_RMTEN 
         | CAN_MCR_EOB        = 0x8280                                       */
  CANx->sMsgObj[0].MCR = 0x8280 | (msg->len & 0x000F);

  /* Set the message valid bit                                               */
  CANx->sMsgObj[0].A2R |= CAN_A2R_MSGVAL;

  /* Update from interface registers                                         */
  CANx->sMsgObj[0].CRR = msg->ch;

  return CAN_OK;
}

/*--------------------------- CAN_hw_rx_object ------------------------------
 *
 *  This function setups object that is going to be used for the message 
 *  reception
 *
 *  Parameter:  ctrl:       Index of CAN controller
 *              ch:         Index of object used for reception
 *              id:         Identifier of receiving messages
 *              object_para:Object parameters (standard or extended format, 
 *                          data or remote frame)
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_rx_object (U32 ctrl, U32 ch, U32 id, U32 object_para)  {
  U32 free_if;
  CAN_TypeDef *CANx;

  CAN_get_ctrl(ctrl, &CANx);            /* Get pointer to CAN controller     */

  /* Wait for the first free interface register                              */
  do  {
    if ((CANx->sMsgObj[0].CRR & CAN_CRR_BUSY) == 0)  {
      free_if = 0;
      break;
    }  else if ((CANx->sMsgObj[1].CRR & CAN_CRR_BUSY) == 0)  {
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
  CANx->sMsgObj[free_if].CMR = 0x00F3;

  /* Setup the identifier information                                        */
  if ((object_para & FORMAT_TYPE) == STANDARD_TYPE)  {  /* Standard ID       */
    CANx->sMsgObj[free_if].M1R = 0;
    CANx->sMsgObj[free_if].M2R = (U16)(0x7FF << 2);

    CANx->sMsgObj[free_if].A1R = 0;
    CANx->sMsgObj[free_if].A2R = CAN_A2R_MSGVAL | (U16)(id << 2);
  }  else  {                                            /* Extended ID       */
    CANx->sMsgObj[free_if].M1R = (U16)(0xFFFF);
    CANx->sMsgObj[free_if].M2R = CAN_M2R_MXTD | (U16)(0x1FFF);

    CANx->sMsgObj[free_if].A1R = (U16)(id);
    CANx->sMsgObj[free_if].A2R = CAN_A2R_MSGVAL | CAN_A2R_XTD | (U16)(id >> 16);
  }

  /* Setup length and message control bits                                   */
  /* =     CAN_MCR_UMASK
         | CAN_MCR_RXIE  
         | CAN_MCR_EOB       = 0x1480                                        */
  CANx->sMsgObj[free_if].MCR = 0x1480;

  /* Setup data bytes                                                        */
  CANx->sMsgObj[free_if].DA1R = 0;
  CANx->sMsgObj[free_if].DA2R = 0;
  CANx->sMsgObj[free_if].DB1R = 0;
  CANx->sMsgObj[free_if].DB2R = 0;

  /* Update from interface registers                                         */
  CANx->sMsgObj[free_if].CRR = ch;

  return CAN_OK;
}

/*--------------------------- CAN_hw_tx_object ------------------------------
 *
 *  This function setups object that is going to be used for the message 
 *  transmission, the setup of transmission object is not necessery so this 
 *  function is not implemented
 *
 *  Parameter:  ctrl:       Index of CAN controller
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

#if USE_CAN_CTRL1 == 1

/*--------------------------- CAN0_IRQHandler -------------------------------
 *
 *  CAN interrupt function 
 *  If transmit interrupt occured and there are messages in mailbox for 
 *  transmit it writes it to hardware and starts the transmission
 *  If receive interrupt occured it reads message from hardware registers 
 *  and puts it into receive mailbox
 *---------------------------------------------------------------------------*/

#define CTRL0                 0
#define CTRL                  1
#define CANc                  CAN0
#define CAN_INT_CLR           (1 << (CAN0_IRQChannel-32))

static void CAN0_IRQHandler (void) __irq  {
  U32 ch;
  CAN_msg *ptrmsg;

  ch = CANc->IDR;                     /* Index of object that caused         */
                                      /* interrupt                           */
  if ((ch-1)<32)  {                   /* Check if index in range 1 - 32      */

    /* Wait for IF[1] to become free                                         */
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

  EIC->IPR1 = CAN_INT_CLR;
}

#endif


#if USE_CAN_CTRL2 == 1

/*--------------------------- CAN1_IRQHandler -------------------------------
 *
 *  CAN interrupt function 
 *  If transmit interrupt occured and there are messages in mailbox for 
 *  transmit it writes it to hardware and starts the transmission
 *  If receive interrupt occured it reads message from hardware registers 
 *  and puts it into receive mailbox
 *---------------------------------------------------------------------------*/

#undef  CTRL0
#define CTRL0                 1
#undef  CTRL
#define CTRL                  2
#undef  CANc
#define CANc                  CAN1
#undef  CAN_INT_CLR
#define CAN_INT_CLR           (1 << (CAN1_IRQChannel-32))

static void CAN1_IRQHandler (void) __irq  {
  U32 ch;
  CAN_msg *ptrmsg;

  ch = CANc->IDR;                     /* Index of object that caused         */
                                      /* interrupt                           */
  if ((ch-1)<32)  {                   /* Check if index in range 1 - 32      */

    /* Wait for IF[1] to become free                                         */
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

  EIC->IPR1 = CAN_INT_CLR;
}

#endif


#if USE_CAN_CTRL3 == 1

/*--------------------------- CAN2_IRQHandler -------------------------------
 *
 *  CAN interrupt function 
 *  If transmit interrupt occured and there are messages in mailbox for 
 *  transmit it writes it to hardware and starts the transmission
 *  If receive interrupt occured it reads message from hardware registers 
 *  and puts it into receive mailbox
 *---------------------------------------------------------------------------*/

#undef  CTRL0
#define CTRL0                 2
#undef  CTRL
#define CTRL                  3
#undef  CANc
#define CANc                  CAN2
#undef  CAN_INT_CLR
#define CAN_INT_CLR           (1 << (CAN2_IRQChannel-32))

static void CAN2_IRQHandler (void) __irq  {
  U32 ch;
  CAN_msg *ptrmsg;

  ch = CANc->IDR;                     /* Index of object that caused         */
                                      /* interrupt                           */
  if ((ch-1)<32)  {                   /* Check if index in range 1 - 32      */

    /* Wait for IF[1] to become free                                         */
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

  EIC->IPR1 = CAN_INT_CLR;
}

#endif

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/

