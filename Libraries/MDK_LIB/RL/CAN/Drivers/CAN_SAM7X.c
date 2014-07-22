/*----------------------------------------------------------------------------
 *      RL-ARM - CAN
 *----------------------------------------------------------------------------
 *      Name:    CAN_SAM7X.c
 *      Purpose: CAN Driver, Hardware specific module for Atmel AT91SAM7X
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <RTL.h>                      /* RTX kernel functions & defines      */
#include <RTX_CAN.h>                  /* CAN Generic functions & defines     */
#include <AT91SAM7X256.H>             /* AT91SAM7X256 definitions            */
#include "AT91SAM7X-EK.h"             /* AT91SAM7X-EK Eval Board definitions */

#define MAX_OBJ       8               /* Number of hardware available objects*/

/* Values of bit time register for different baudrates
   SP = Sample point     = ((1+PROPAG+1+PHASE1+1)/(1+PROPAG+1+PHASE1+1+PHASE2+1)) * 100%
                            SMP | SJW | PROPAG | PHASE1 | PHASE2 | TQs |  SP */
const U32 CAN_BIT_TIME[] = {
  /* [ 0] */           0, /*    |     |        |        |        |     |     */
  /* [ 1] */           0, /*    |     |        |        |        |     |     */
  /* [ 2] */           0, /*    |     |        |        |        |     |     */
  /* [ 3] */           0, /*    |     |        |        |        |     |     */
  /* [ 4] */           0, /*    |     |        |        |        |     |     */
  /* [ 5] */           0, /*    |     |        |        |        |     |     */
  /* [ 6] */           0, /*    |     |        |        |        |     |     */
  /* [ 7] */           0, /*    |     |        |        |        |     |     */
  /* [ 8] */  0x00002211, /*  0 | 2+1 |   2+1  |   1+1  |   1+1  |  8  | 75% */
  /* [ 9] */  0x00001122, /*  0 | 1+1 |   1+1  |   2+1  |   2+1  |  9  | 67% */
  /* [10] */  0x00002222, /*  0 | 2+1 |   2+1  |   2+1  |   2+1  | 10  | 70% */
  /* [11] */  0x00003322, /*  0 | 3+1 |   3+1  |   2+1  |   2+1  | 11  | 72% */
  /* [12] */  0x00002233, /*  0 | 2+1 |   2+1  |   3+1  |   3+1  | 12  | 67% */
  /* [13] */  0x00003333, /*  0 | 3+1 |   3+1  |   3+1  |   3+1  | 13  | 77% */
  /* [14] */  0x00003334, /*  0 | 3+1 |   3+1  |   3+1  |   4+1  | 14  | 64% */
  /* [15] */  0x00003344, /*  0 | 3+1 |   3+1  |   4+1  |   4+1  | 15  | 67% */
  /* [16] */  0x00003444, /*  0 | 3+1 |   4+1  |   4+1  |   4+1  | 16  | 69% */
  /* [17] */  0x00003544, /*  0 | 3+1 |   5+1  |   4+1  |   4+1  | 17  | 71% */
  /* [18] */  0x00003455, /*  0 | 3+1 |   4+1  |   5+1  |   5+1  | 18  | 67% */
  /* [19] */  0x00003555, /*  0 | 3+1 |   5+1  |   5+1  |   5+1  | 19  | 68% */
  /* [20] */  0x00003655, /*  0 | 3+1 |   6+1  |   5+1  |   5+1  | 20  | 70% */
  /* [21] */  0x00003755, /*  0 | 3+1 |   7+1  |   5+1  |   5+1  | 21  | 71% */
  /* [22] */  0x00003766, /*  0 | 3+1 |   7+1  |   6+1  |   6+1  | 22  | 68% */
  /* [23] */  0x00003776, /*  0 | 3+1 |   7+1  |   7+1  |   6+1  | 23  | 70% */
  /* [24] */  0x00003677, /*  0 | 3+1 |   6+1  |   7+1  |   7+1  | 24  | 67% */
  /* [25] */  0x00003777, /*  0 | 3+1 |   7+1  |   7+1  |   7+1  | 25  | 68% */};

/* Mailbox object addresses                                                  */
static struct _AT91S_CAN_MB *table_mailbox_addr[MAX_OBJ] = { 
  AT91C_BASE_CAN_MB0 , AT91C_BASE_CAN_MB1,  AT91C_BASE_CAN_MB2,  AT91C_BASE_CAN_MB3,  
  AT91C_BASE_CAN_MB4,  AT91C_BASE_CAN_MB5,  AT91C_BASE_CAN_MB6,  AT91C_BASE_CAN_MB7  };

/* Mailbox object indexes (masks)                                            */
static U32 table_mailbox_index[MAX_OBJ] = { 
 (1 <<  0), (1 <<  1), (1 <<  2), (1 <<  3), (1 <<  4), (1 <<  5), (1 <<  6), (1 <<  7) };

/************************* CAN Hardware Configuration ************************/

// *** <<< Use Configuration Wizard in Context Menu >>> ***

// <o> Master Clock Frequency (MCK) <1-160000000>
//     <i> Peripheral clock frequency with which CAN controller is clocked
//     <i> (same as Master clock frequency).
//     <i> Default: 47923200
#define MCK_FREQ         47923200

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
 *           CAN_ERROR CAN_hw_wr           (U32 ctrl,         CAN_msg *msg)
 *    static      void CAN_hw_rd           (U32 ctrl, U32 ch, CAN_msg *msg)
 *           CAN_ERROR CAN_hw_set          (U32 ctrl,         CAN_msg *msg)
 *           CAN_ERROR CAN_hw_rx_object    (U32 ctrl, U32 ch, U32 id, U32 object_para)
 *           CAN_ERROR CAN_hw_tx_object    (U32 ctrl, U32 ch,         U32 object_para)
 *    Interrupt fuction
 *---------------------------------------------------------------------------*/

/* Static functions used only in this module                                 */
static void CAN_IRQHandler (void) __irq;


/* Global variable with information on which mailboxes are used for receive  */
static U32 rx_mailboxes = 0;
/* Global variable with information on which mailboxes are used for transmit */
static U32 tx_mailboxes = 0;


/************************* Auxiliary Functions *******************************/

/*--------------------------- CAN_set_baudrate ------------------------------
 *
 *  Setup the requested baudrate
 *
 *  Parameter:  ctrl:       Index of the CAN controller            
 *              baudrate:   Baudrate
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

static CAN_ERROR CAN_hw_set_baudrate (U32 ctrl, U32 baudrate)  {
  U32 i, val1, val2, min_index;

  /* Find the total time quanta that makes best fit for baudrate and 
     CAN clock frequency (=MCK)                                              */
  min_index = 8;
  val1 = (MCK_FREQ % (baudrate*8))/ 8;
  for (i = 9; i < 26; i++) {
    val2 = (MCK_FREQ % (baudrate*i))/ i;
    if (val1 > val2) {
      val1 = val2;
      min_index = i;
    }
  }

  /* BAUDRATE PRESCALER - with rounding                                      */
  val1 = min_index*baudrate;
  val2 = ((MCK_FREQ+(val1>>1))/val1) - 1;
  AT91C_BASE_CAN->CAN_BR = ((val2 & 0x7F) << 16) | CAN_BIT_TIME[min_index];

  if (val2 > 0x7F) 
    return CAN_BAUDRATE_ERROR;

  return CAN_OK;
}


/*************************** Module Functions ********************************/

/*--------------------------- CAN_hw_setup ----------------------------------
 *
 *  Setup CAN transmit and receive PINs and interrupt vectors
 *
 *  Parameter:  ctrl:       Index of the CAN controller            
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_setup (U32 ctrl)  {

  /* Enable the peripheral clock for: PIOA port and CAN controller           */
  AT91C_BASE_PMC->PMC_PCER  = 1 << AT91C_ID_PIOA;
  AT91C_BASE_PMC->PMC_PCER  = 1 << AT91C_ID_CAN;

  /* Setup RX and TX pins to Peripheral A Mode                               */
	AT91C_BASE_PIOA->PIO_ASR  = AT91C_PA20_CANTX | AT91C_PA19_CANRX;
	AT91C_BASE_PIOA->PIO_PDR  = AT91C_PA20_CANTX | AT91C_PA19_CANRX;

  /* Setup PIOA PA2 pin as output for S pin of TJA1050T CAN transceiver      */
	AT91C_BASE_PIOA->PIO_PER  = AT91B_CAN_TRANSCEIVER_RS;
	AT91C_BASE_PIOA->PIO_OER  = AT91B_CAN_TRANSCEIVER_RS;

  /* Pull pin S of TJA1050T low for High-speed mode                          */
	AT91C_BASE_PIOA->PIO_CODR = AT91B_CAN_TRANSCEIVER_RS;

  /* Setup AIC for CAN Interrupt                                             */
  AT91C_BASE_AIC->AIC_SVR[AT91C_ID_CAN] = (unsigned int)CAN_IRQHandler;
  AT91C_BASE_AIC->AIC_SMR[AT91C_ID_CAN] = AT91C_AIC_SRCTYPE_INT_HIGH_LEVEL | 1;

  /* Enable interrupt generation by CAN controller                           */
  AT91C_BASE_AIC->AIC_IECR  = 1 << AT91C_ID_CAN;

  return CAN_OK;
}

/*--------------------------- CAN_hw_init -----------------------------------
 *
 *  Initialize the CAN hardware
 *
 *  Parameter:  ctrl:       Index of the CAN controller            
 *              baudrate:   Baudrate
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_init (U32 ctrl, U32 baudrate)  {
  struct _AT91S_CAN_MB *MailBox;
  U32 i;

  if (CAN_hw_set_baudrate(ctrl, baudrate) != CAN_OK)  /* Set baudrate        */
    return CAN_BAUDRATE_ERROR;

  for (i = 1; i < MAX_OBJ+1; i++) {       /* Disable all message objects     */
    MailBox = table_mailbox_addr[i-1];    /* Get mailbox address             */
    MailBox->CAN_MB_MMR=AT91C_CAN_MOT_DIS;/* Disable all mailboxes           */
  }

  return CAN_OK;
}

/*--------------------------- CAN_hw_start ----------------------------------
 *
 *  Enable the CAN interrupts
 *
 *  Parameter:  ctrl:       Index of the CAN controller            
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_start (U32 ctrl)  {

  AT91C_BASE_CAN->CAN_MR = AT91C_CAN_CANEN;

  return CAN_OK;
}

/*--------------------------- CAN_hw_tx_empty -------------------------------
 *
 *  Check if interface register group 0 (IF0) is available for usage
 *
 *  Parameter:  ctrl:       Index of the CAN controller            
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_tx_empty (U32 ctrl)  { 

  if ((os_sem_wait (wr_sem[ctrl-1], 0) != OS_R_TMO)){ /* If semaphore is free*/
    if ((AT91C_BASE_CAN->CAN_SR & AT91C_CAN_TBSY) == 0) /* Transmitter ready */
                                                  /* for transmission        */
      return CAN_OK; 
    else 
      os_sem_send(wr_sem[ctrl-1]);    /* Return a token back to semaphore    */
  }

  return CAN_TX_BUSY_ERROR;
}

/*--------------------------- CAN_hw_rd -------------------------------------
 *
 *  Read CAN_msg from the hardware registers of the requested controller
 *
 *  Parameter:  ctrl:       Index of the CAN controller            
 *              ch:         Index of object used for reception
 *              msg:        Pointer where CAN message will be read
 *
 *  Return:     none
 *---------------------------------------------------------------------------*/

static void CAN_hw_rd (U32 ctrl, U32 ch, CAN_msg *msg)  {
  struct _AT91S_CAN_MB *MailBox = table_mailbox_addr[ch-1]; /* Mailbox adres */
  U32 CANInfo;
  U32 *CANAddr;

  /* Read frame informations                                                 */
  CANInfo = MailBox->CAN_MB_MSR;
  msg->type   = (CANInfo & 0x00100000) == 0x00100000;
  msg->len    = (U8)((CANInfo >> 16) & 0x0F);

  CANInfo = MailBox->CAN_MB_MID;
  msg->format = (CANInfo & 0x20000000) == 0x20000000;

  msg->id     =  CANInfo & 0x1FFFFFFF;        /* Extended ID                 */
  if (msg->format == STANDARD_FORMAT)         /* Standard message received   */
    msg->id >>= 18;                           /* Standard ID                 */

  /* Read the data if received message was DATA FRAME                        */
  if (msg->type == DATA_FRAME) {     
    CANAddr = (U32 *) &msg->data[0];          /* Read  first 4 data bytes    */
    *CANAddr++ = MailBox->CAN_MB_MDL;
    *CANAddr   = MailBox->CAN_MB_MDH;         /* Read second 4 data bytes    */
  }
}

/*--------------------------- CAN_hw_wr -------------------------------------
 *
 *  Write CAN_msg to the hardware registers of the requested controller
 *
 *  Parameter:  ctrl:       Index of the CAN controller            
 *              msg:        Pointer to CAN message to be written to hardware
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_wr (U32 ctrl, CAN_msg *msg)  {
  struct _AT91S_CAN_MB *MailBox = table_mailbox_addr[msg->ch-1];/* Mbx adres */
  U32 index = table_mailbox_index[msg->ch-1];               /* Mailbox mask  */
  U32 *CANAddr;

  MailBox->CAN_MB_MCR   = 0x0;
  MailBox->CAN_MB_MAM   = 0;                  /* Acceptance filter           */

  /* Setup the identifier information                                        */
  if (msg->format == STANDARD_FORMAT)         /* Standard ID                 */
    MailBox->CAN_MB_MID = (msg->id & 0x000007FF) << 18;
  else                                        /* Extended ID                 */
    MailBox->CAN_MB_MID = (msg->id & 0x1FFFFFFF) | 0x20000000;

  MailBox->CAN_MB_MMR   = AT91C_CAN_MOT_TX    /* Transmit and priority       */
                        | ((msg->ch << 16) & AT91C_CAN_PRIOR);

  CANAddr = (U32 *) &msg->data[0];            /* Data  low bytes             */
  MailBox->CAN_MB_MDL   = *CANAddr++;
  MailBox->CAN_MB_MDH   = *CANAddr;           /* Data high bytes             */

  /* Setup length information and enable the transmission                    */
  if (msg->format == REMOTE_FRAME)            /* For remote frame            */
    MailBox->CAN_MB_MCR = AT91C_CAN_MTCR 
                        | AT91C_CAN_MRTR
                        | ((msg->len & 0x0F) << 16);	
  else                                        /* For   data frame            */
    MailBox->CAN_MB_MCR = AT91C_CAN_MTCR 
                        | ((msg->len & 0x0F) << 16);	

  tx_mailboxes |= index;                /* Mailbox used for transmission     */

  AT91C_BASE_CAN->CAN_IER = index;      /* Enable int for current mailbox    */


  return CAN_OK;
}

/*--------------------------- CAN_hw_set ------------------------------------
 *  Set a message that will automatically be sent as an answer to the REMOTE
 *  FRAME message
 *
 *  Parameter:  ctrl:       Index of the CAN controller            
 *              msg:        Pointer to CAN message to be set
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_set (U32 ctrl, CAN_msg *msg)  {
  struct _AT91S_CAN_MB *MailBox = table_mailbox_addr[msg->ch-1];/* Mbx adres */
  U32 index = table_mailbox_index[msg->ch-1];               /* Mailbox mask  */
  U32 *CANAddr;

  MailBox->CAN_MB_MCR   = 0x0;
  MailBox->CAN_MB_MMR   = AT91C_CAN_MOT_PRODUCER      /* Tx on RTR receive   */
                        | ((msg->ch << 16) & AT91C_CAN_PRIOR);

  /* Setup the identifier information                                        */
  if (msg->format == STANDARD_FORMAT) {       /* Standard ID                 */
    MailBox->CAN_MB_MAM = AT91C_CAN_MIDvA;    /* Acceptance filter           */
    MailBox->CAN_MB_MID = (msg->id & 0x000007FF) << 18;
  }
  else {                                      /* Extended ID                 */
    MailBox->CAN_MB_MAM = AT91C_CAN_MIDvB     /* Acceptance filter           */
                        | AT91C_CAN_MIDE;
    MailBox->CAN_MB_MID = (msg->id & 0x1FFFFFFF) | 0x20000000;
  }

  CANAddr = (U32 *) &msg->data[0];            /* Data  low bytes             */
  MailBox->CAN_MB_MDL   = *CANAddr++;
  MailBox->CAN_MB_MDH   = *CANAddr;           /* Data high bytes             */

  /* Setup length information and enable the transmission                    */
  MailBox->CAN_MB_MCR   = AT91C_CAN_MTCR
                        | ((msg->len & 0x0F) << 16);

  tx_mailboxes |= index;                      /* Mailbox for transmission    */

  MailBox->CAN_MB_MCR = AT91C_CAN_MTCR;       /* Start the transmission      */

  AT91C_BASE_CAN->CAN_IER = index;            /* Enable interrupt for mailbox*/

  return CAN_OK;
}

/*--------------------------- CAN_hw_rx_object ------------------------------
 *
 *  This function setups object that is going to be used for the message 
 *  reception
 *
 *  Parameter:  ctrl:       Index of the CAN controller            
 *              ch:         Index of object used for reception
 *              id:         Identifier of receiving messages
 *              object_para:Object parameters (standard or extended format, 
 *                          data or remote frame)
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_rx_object (U32 ctrl, U32 ch, U32 id, U32 object_para)  {
  struct _AT91S_CAN_MB *MailBox = table_mailbox_addr[ch-1]; /* Mailbox adres */
  U32 index = table_mailbox_index[ch-1];                    /* Mailbox mask  */
  
  MailBox->CAN_MB_MCR = 0x0;

  if ((object_para & FORMAT_TYPE) == STANDARD_TYPE)  { /* Standard ID        */
    MailBox->CAN_MB_MAM = AT91C_CAN_MIDvA;    /* Acceptance filter           */
    MailBox->CAN_MB_MID = (id & 0x7FF) << 18; /* Setup STANDARD ID           */
  }  
  else  {                                     /* Extended ID                 */
    MailBox->CAN_MB_MAM = AT91C_CAN_MIDvB     /* Acceptance filter           */
                        | AT91C_CAN_MIDE;
    MailBox->CAN_MB_MID = (id & 0x1FFFFFFF)   /* Setup EXTENDED ID           */
                        | AT91C_CAN_MIDE;
  }

  MailBox->CAN_MB_MDL = 0;                    /* Data  low bytes             */
  MailBox->CAN_MB_MDH = 0;                    /* Data high bytes             */
  MailBox->CAN_MB_MCR = 0;	                  /* Control register            */

  MailBox->CAN_MB_MMR = AT91C_CAN_MOT_RX;     /* Enable Receive on mailbox   */

  rx_mailboxes |= index;                      /* Mailbox used for reception  */

  AT91C_BASE_CAN->CAN_IER = index;            /* Enable interrupt on mailbox */

  return CAN_OK;
}

/*--------------------------- CAN_hw_tx_object ------------------------------
 *
 *  This function setups object that is going to be used for the message 
 *  transmission, the setup of transmission object is not necessery so this 
 *  function is not implemented
 *
 *  Parameter:  ctrl:       Index of the CAN controller            
 *              ch:         Index of object used for transmission
 *              object_para:Object parameters (standard or extended format, 
 *                          data or remote frame)
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_tx_object (U32 ctrl, U32 ch, U32 object_para)  {

  return CAN_OK;
}


/************************* Interrupt Function ********************************/

/*-------------------------- CAN_IRQHandler ---------------------------------
 *
 *  CAN interrupt function 
 *  If transmit interrupt occured and there are messages in mailbox for 
 *  transmit it writes it to hardware and starts the transmission
 *  If receive interrupt occured it reads message from hardware registers 
 *  and puts it into receive mailbox
 *---------------------------------------------------------------------------*/

#define CTRL0                 0
#define CTRL                  CTRL0+1
#define CAN_PTR               AT91C_BASE_CAN
#define CAN_INT_CLR           (1 << AT91C_ID_CAN)

static void CAN_IRQHandler (void) __irq  {
  U32 ch;
  CAN_msg *ptrmsg;
  struct _AT91S_CAN_MB *MailBox;
  U32 i, int_status, int_ch, int_mask;

  int_mask   = CAN_PTR->CAN_IMR;
  CAN_PTR->CAN_IDR = 0xFF;                /* Disable CAN interrupts          */
  int_status = CAN_PTR->CAN_SR & int_mask;

  int_ch = int_status & tx_mailboxes;
  if (int_ch) {                         /* Transmission interrupt occured    */
    tx_mailboxes &= ~int_ch;              /* Tx on channel is inactive       */
    int_mask     &= ~int_ch;              /* Clear interrupt mask for Tx     */

    i = 1;
    for (ch = 1; ch < MAX_OBJ+1; ch++) {  /* Find mailbox that caused Tx int */
      if (int_ch & i) break;
      i <<= 1;
    }

    MailBox = table_mailbox_addr[ch-1];   /* Mailbox address                 */
    MailBox->CAN_MB_MMR=AT91C_CAN_MOT_DIS;/* Disable mailbox until next write*/
      /* If there is a message in the mailbox ready for send, read the 
         message from the mailbox and send it                                */
      if (isr_mbx_receive (MBX_tx_ctrl[CTRL0], (void **)&ptrmsg) != OS_R_OK) {
        CAN_hw_wr (CTRL, ptrmsg);
        _free_box(CAN_mpool, ptrmsg);
    } else {
      isr_sem_send(wr_sem[CTRL0]);        /* Return a token back to semaphore*/
    }
  }

  int_ch = int_status & rx_mailboxes;
  if (int_ch) {                         /* Reception    interrupt occured    */
    i = 1;
    for (ch = 1; ch < MAX_OBJ+1; ch++) {  /* Read all pending received msgs  */
      if (int_ch & i) {
        MailBox = table_mailbox_addr[ch-1];   /* Mailbox address             */

        /* If the mailbox isn't full read the message from the hardware and
           send it to the message queue                                      */
        if (os_mbx_check (MBX_rx_ctrl[CTRL0]) > 0)  {
          ptrmsg = _alloc_box (CAN_mpool);
          if (ptrmsg) CAN_hw_rd (CTRL, ch, ptrmsg);   /* Read received msg   */
          MailBox->CAN_MB_MCR = AT91C_CAN_MTCR;       /* release mailbox     */
          if (ptrmsg) isr_mbx_send (MBX_rx_ctrl[CTRL0], ptrmsg);
        }
      }
      i <<= 1;
    }
  } 

  CAN_PTR->CAN_IER          = int_mask;               /* Enable CAN interrupt*/
  AT91C_BASE_AIC->AIC_ICCR  = CAN_INT_CLR;            /* Clear interrupt flag*/
  AT91C_BASE_AIC->AIC_EOICR = AT91C_BASE_AIC->AIC_EOICR;  /* End of interrupt*/
}

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/

