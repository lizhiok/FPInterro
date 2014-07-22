/*----------------------------------------------------------------------------
 *      RL-ARM - CAN
 *----------------------------------------------------------------------------
 *      Name:    CAN_TMPM36x.c
 *      Purpose: CAN Driver, Hardware specific module for Toshiba TMPM36x
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <RTL.h>                      /* RTX kernel functions & defines      */
#include <RTX_CAN.h>                  /* CAN Generic functions & defines     */
#include <TMPM364.h>                  /* TMPM364 definitions                 */

#define MAX_OBJ            32         /* Number of hardware available objects*/

#define sCANMB            (TSB_CANMB_TypeDef *)

/* Values of bit time register for different baudrates
   SP = Sample point     = ((1+TSEG1+1)/(1+TSEG1+1+TSEG2+1)) * 100%
                                       SJW | SAM | TSEG1 | TSEG2 | TQs |  SP */
const U32 CAN_BIT_TIME[] = {/*        -----|-----|-------|-------|-----|-----*/
                           0x00000000,/*             unsupported             */
                           0x00000000,/*             unsupported             */
                           0x00000000,/*             unsupported             */
                           0x00000000,/*             unsupported             */
                           0x00000000,/*             unsupported             */
                           0x00000000,/*             unsupported             */
                           0x00000000,/*             unsupported             */
                           0x00000000,/*             unsupported             */
                           0x00000114,/* 1 |  0  |   4   |   1   |  8  | 75% */
                           0x00000224,/* 2 |  0  |   4   |   2   |  9  | 67% */
                           0x00000225,/* 2 |  0  |   5   |   2   | 10  | 70% */
                           0x00000226,/* 2 |  0  |   6   |   2   | 11  | 72% */
                           0x00000336,/* 3 |  0  |   6   |   3   | 12  | 67% */
                           0x00000337,/* 3 |  0  |   7   |   3   | 13  | 69% */
                           0x00000338,/* 3 |  0  |   8   |   3   | 14  | 71% */
                           0x00000348,/* 3 |  0  |   8   |   4   | 15  | 67% */
                           0x00000349,/* 3 |  0  |   9   |   4   | 16  | 69% */
                           0x0000034A,/* 3 |  0  |  10   |   4   | 17  | 71% */
                           0x0000035A,/* 3 |  0  |  10   |   5   | 18  | 67% */
                           0x0000035B,/* 3 |  0  |  11   |   5   | 19  | 68% */
                           0x0000035C,/* 3 |  0  |  12   |   5   | 20  | 70% */
                           0x0000035D,/* 3 |  0  |  13   |   5   | 21  | 71% */
                           0x0000036D,/* 3 |  0  |  13   |   6   | 22  | 68% */
                           0x0000036E,/* 3 |  0  |  14   |   6   | 23  | 70% */
                           0x0000037E,/* 3 |  0  |  14   |   7   | 24  | 67% */
                           0x0000037F,/* 3 |  0  |  15   |   7   | 25  | 68% */
                           };

/* Mailbox object addresses                                                  */
static TSB_CANMB_TypeDef *table_mailbox_addr[MAX_OBJ] = { 
  sCANMB TSB_CANMB0_BASE,  sCANMB TSB_CANMB1_BASE,  sCANMB TSB_CANMB2_BASE,  sCANMB TSB_CANMB3_BASE,  
  sCANMB TSB_CANMB4_BASE,  sCANMB TSB_CANMB5_BASE,  sCANMB TSB_CANMB6_BASE,  sCANMB TSB_CANMB7_BASE,  
  sCANMB TSB_CANMB8_BASE,  sCANMB TSB_CANMB9_BASE,  sCANMB TSB_CANMB10_BASE, sCANMB TSB_CANMB11_BASE, 
  sCANMB TSB_CANMB12_BASE, sCANMB TSB_CANMB13_BASE, sCANMB TSB_CANMB14_BASE, sCANMB TSB_CANMB15_BASE, 
  sCANMB TSB_CANMB16_BASE, sCANMB TSB_CANMB17_BASE, sCANMB TSB_CANMB18_BASE, sCANMB TSB_CANMB19_BASE, 
  sCANMB TSB_CANMB20_BASE, sCANMB TSB_CANMB21_BASE, sCANMB TSB_CANMB22_BASE, sCANMB TSB_CANMB23_BASE, 
  sCANMB TSB_CANMB24_BASE, sCANMB TSB_CANMB25_BASE, sCANMB TSB_CANMB26_BASE, sCANMB TSB_CANMB27_BASE, 
  sCANMB TSB_CANMB28_BASE, sCANMB TSB_CANMB29_BASE, sCANMB TSB_CANMB30_BASE, sCANMB TSB_CANMB31_BASE 
};

/************************* CAN Hardware Configuration ************************/

// *** <<< Use Configuration Wizard in Context Menu >>> ***

// <o> CAN Peripheral Clock Frequency <1-64000000>
//     <i> Peripheral clock frequency with which CAN controller is clocked
//     <i> It is equal to Fsys / 4
//     <i> Default: 12000000
#define CAN_PERI_FREQ         12000000

// *** <<< End of Configuration section             >>> ***


/*----------------------------------------------------------------------------
 *      CAN RTX Hardware Specific Driver Functions
 *----------------------------------------------------------------------------
 *  Functions implemented in this module:
 *    static CAN_ERROR CAN_hw_set_baudrate (U32 ctrl, U32 baudrate)
 *           CAN_ERROR CAN_hw_testmode     (U32 ctrl, U32 testmode)
 *           CAN_ERROR CAN_hw_setup        (U32 ctrl)
 *           CAN_ERROR CAN_hw_init         (U32 ctrl, U32 baudrate)
 *           CAN_ERROR CAN_hw_start        (U32 ctrl)
 *           CAN_ERROR CAN_hw_tx_empty     (U32 ctrl)
 *    static      void CAN_hw_rd           (U32 ctrl, U32 ch, CAN_msg *msg)
 *           CAN_ERROR CAN_hw_wr           (U32 ctrl,         CAN_msg *msg)
 *           CAN_ERROR CAN_hw_set          (U32 ctrl,         CAN_msg *msg)
 *           CAN_ERROR CAN_hw_rx_object    (U32 ctrl, U32 ch, U32 id, U32 object_para)
 *           CAN_ERROR CAN_hw_tx_object    (U32 ctrl, U32 ch,         U32 object_para)
 *    Interrupt fuctions:
 *           void      INTCANRX_IRQHandler (void)
 *           void      INTCANTX_IRQHandler (void)
 *---------------------------------------------------------------------------*/


/************************* Auxiliary Functions *******************************/

/*--------------------------- CAN_set_baudrate ------------------------------
 *
 *  Setup the requested baudrate
 *
 *  Parameter:  ctrl:       Index of the hardware CAN controller (1 .. x)
 *              baudrate:   Baudrate
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

static CAN_ERROR CAN_hw_set_baudrate (U32 ctrl, U32 baudrate)  {
  U32 i, val1, val2, min_index;

  /* Find the total time quanta that makes best fit for baudrate and 
     CAN clock frequency                                                     */
  min_index = 8;
  val1 = (CAN_PERI_FREQ % (baudrate*8))/ 8;
  for (i = 9; i < 26; i++) {
    val2 = (CAN_PERI_FREQ % (baudrate*i))/ i;
    if (val1 > val2) {
      val1 = val2;
      min_index = i;
    }
  }

  /* BAUDRATE PRESCALER - with rounding                                      */
  val1 = min_index*baudrate;
  val2 = ((CAN_PERI_FREQ+(val1>>1))/val1) - 1;
  TSB_CAN->BCR1 = val2 & 0xFF;
  TSB_CAN->BCR2 = CAN_BIT_TIME[min_index];

  if (val2 > 0xFF) 
    return CAN_BAUDRATE_ERROR;

  return CAN_OK;
}


/*************************** Module Functions ********************************/

/*--------------------------- CAN_hw_testmode -------------------------------
 *
 *  Setup CAN controller in test mode (controller in loopback mode)
 *
 *  Parameter:  ctrl:       Index of the hardware CAN controller (1 .. x)
 *              testmode:   Testmode (0 = off, 1 = on)
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_testmode (U32 ctrl, U32 testmode)  {

  if (!(TSB_CAN->GSR & 0x80)) {           /* If CCE inactive                 */
    TSB_CAN->MCR |= 0x800;                /* Activate SUR (suspend mode req) */
    while (!(TSB_CAN->GSR & 0x100));      /* While !SUA (wait suspend mode)  */
  }

  if (testmode)
    TSB_CAN->MCR |=  0x600;               /* Set   INTLB and TSTLB bits      */
  else
    TSB_CAN->MCR &= ~0x600;               /* Clear INTLB and TSTLB bits      */

  if (!(TSB_CAN->GSR & 0x80)) {           /* If CCE inactive                 */
    TSB_CAN->MCR |= 0x800;                /* Deactivate SUR (normal mode req)*/
    while (TSB_CAN->GSR & 0x100);         /* While SUA  (wait normal mode)   */
  }

  return CAN_OK;
}

/*--------------------------- CAN_hw_setup ----------------------------------
 *
 *  Setup CAN transmit and receive pins and interrupts
 *
 *  Parameter:  ctrl:       Index of the hardware CAN controller (1 .. x)
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_setup (U32 ctrl)  {

  /* Enable CAN pins                                                         */
  TSB_PE->CR  |=  0x10;                   /* Set TX pin as output            */
  TSB_PE->FR3 |=  0x30;                   /* Enable TX and RX pin functions  */
  TSB_PE->OD  &= ~0x10;                   /* Set TX pin as CMOS output       */
  TSB_PE->PUP &= ~0x30;                   /* Disable pull-ups for CAN pins   */
  TSB_PE->IE  |=  0x20;                   /* Set RX pin as input             */

  /* Enable CAN interrupts                                                   */
  NVIC_EnableIRQ(INTCANRX_IRQn);          /* Enable RX CAN interrupt         */
  NVIC_EnableIRQ(INTCANTX_IRQn);          /* Enable TX CAN interrupt         */

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

  if (CAN_hw_set_baudrate(ctrl, baudrate) != CAN_OK)  /* Set baudrate        */
    return CAN_BAUDRATE_ERROR;

  TSB_CAN->MC   = 0;                      /* Disable all mailboxes           */

  TSB_CAN->MCR |= (1 << 3);               /* Set MTOS (tx depends on ID prio */

  return CAN_OK;
}

/*--------------------------- CAN_hw_start ----------------------------------
 *
 *  Enable the CAN interrupts
 *
 *  Parameter:  ctrl:       Index of the hardware CAN controller (1 .. x)
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_start (U32 ctrl)  {

  TSB_CAN->MCR &= ~0x80;                  /* Clear CCR (req normal operation */
  while (TSB_CAN->GSR & 0x80);            /* While CCE (chg config enabled)  */

  return CAN_OK;
}

/*--------------------------- CAN_hw_tx_empty -------------------------------
 *
 *  Check if hardware is ready for transfer
 *
 *  Parameter:  ctrl:       Index of the hardware CAN controller (1 .. x)
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_tx_empty (U32 ctrl)  { 

  return CAN_OK; 
}

/*--------------------------- CAN_hw_rd -------------------------------------
 *
 *  Read CAN_msg from the hardware registers of the requested controller
 *
 *  Parameter:  ctrl:       Index of the hardware CAN controller (1 .. x)
 *              ch:         Index of object used for reception (mailbox)
 *              msg:        Pointer where CAN message will be read
 *
 *  Return:     none
 *---------------------------------------------------------------------------*/

static void CAN_hw_rd (U32 ctrl, U32 ch, CAN_msg *msg)  {
  TSB_CANMB_TypeDef *MailBox = table_mailbox_addr[ch-1];    /* Mailbox adres */
  U32                 reg_ID, reg_TSVMCF;
  U32                *msg_data;

  msg->ch       =  ch;                    /* Channel information             */

  /* Read message identifier                                                 */
  reg_ID        =  MailBox->ID;
  msg->format   = (reg_ID & (1UL << 31)) == (1UL << 31);

  /* Read time stamp value / message control field                           */
  reg_TSVMCF    =  MailBox->TSVMCF;
  msg->type     = (reg_TSVMCF & (1   <<  4)) == (1   <<  4);
  msg->len      = (U8)(reg_TSVMCF & 0x0F);

  msg->id       =  reg_ID & 0x1FFFFFFF;   /* Extended ID                     */
  if (msg->format == STANDARD_FORMAT)     /* Standard message received       */
    msg->id   >>= 18;                     /* Standard ID                     */

  /* Read the data if received message was DATA FRAME                        */
  if (msg->type == DATA_FRAME) {     
    msg_data    = (U32 *) &msg->data[0];
    *msg_data++ =  MailBox->DL;           /* Read  first 4 data bytes        */
    *msg_data   =  MailBox->DH;           /* Read second 4 data bytes        */
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
  TSB_CANMB_TypeDef *MailBox = table_mailbox_addr[msg->ch-1];   /* Mbx adres */
  U32                 mbx_mask = (1 << (msg->ch-1));
  U32                *msg_data;

  /* Mailbox 31 can only be used for receive                                 */
  if (msg->ch >= 32) 
    return CAN_UNEXIST_CH_ERROR;

  TSB_CAN->MC     &= ~mbx_mask;           /* Disable mailbox                 */
  TSB_CAN->MD     &= ~mbx_mask;           /* Set mailbox direction (tx)      */

  /* Setup the identifier information                                        */
  if (msg->format == STANDARD_FORMAT)     /* Standard ID                     */
    MailBox->ID     = (msg->id & 0x000007FF) << 18;
  else                                    /* Extended ID                     */
    MailBox->ID     = (msg->id & 0x1FFFFFFF) | (1UL << 31);

  TSB_CAN->MBIM   |=  mbx_mask;           /* Enable mailbox interrupt        */
  TSB_CAN->MC     |=  mbx_mask;           /* Enable mailbox                  */

  /* Setup data and length information                                       */
  if (msg->type == REMOTE_FRAME) {        /* For remote frame                */
    MailBox->DL     =  0;                 /* Data  low bytes                 */
    MailBox->DH     =  0;                 /* Data high bytes                 */
    MailBox->TSVMCF = (1 << 4) |          /* Set RTR bit                     */
                      (msg->len & 0x0F);  /* Set length reqested             */

  } else {                                /* For   data frame                */
    msg_data        = (U32 *) &msg->data[0];
    MailBox->DL     = *msg_data++;        /* Data  low bytes                 */
    MailBox->DH     = *msg_data;          /* Data high bytes                 */
    MailBox->TSVMCF = (msg->len & 0x0F);  /* Set length to transmit          */
  }

  TSB_CAN->TRS    |=  mbx_mask;           /* Set mailbox transmission request*/

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
  TSB_CANMB_TypeDef *MailBox = table_mailbox_addr[msg->ch-1];   /* Mbx adres */
  U32                 mbx_mask = (1 << (msg->ch-1));
  U32                *msg_data;

  /* Mailbox 31 can only be used for receive                                 */
  if (msg->ch >= 32) 
    return CAN_UNEXIST_CH_ERROR;

  TSB_CAN->MC    &= ~mbx_mask;            /* Disable mailbox                 */
  TSB_CAN->MD    &= ~mbx_mask;            /* Set mailbox direction (tx)      */

  /* Setup the identifier information                                        */
  if (msg->format == STANDARD_FORMAT)     /* Standard ID                     */
    MailBox->ID    = ((msg->id & 0x000007FF) << 18)         | (1 << 29);
  else                                    /* Extended ID                     */
    MailBox->ID    = ((msg->id & 0x1FFFFFFF) | (1UL << 31)) | (1 << 29);

  TSB_CAN->MBIM  &= ~mbx_mask;            /* Disable mailbox interrupt       */
  TSB_CAN->MC    |=  mbx_mask;            /* Enable mailbox                  */

  TSB_CAN->CDR   |=  mbx_mask;            /* Set CDR (change data request)   */

  /* Setup data and length information                                       */
  msg_data         = (U32 *) &msg->data[0];
  MailBox->DL      = *msg_data++;         /* Data  low bytes                 */
  MailBox->DH      = *msg_data;           /* Data high bytes                 */
  MailBox->TSVMCF  = (msg->len & 0x0F);   /* Set length to transmit          */

  TSB_CAN->CDR   &= ~mbx_mask;            /* Clear CDR (change data request) */

  return CAN_OK;
}

/*--------------------------- CAN_hw_rx_object ------------------------------
 *
 *  This function setups object that is going to be used for the message 
 *  reception
 *
 *  Parameter:  ctrl:       Index of the CAN controller            
 *              ch:         Index of object used for reception (mailbox)
 *              id:         Identifier of receiving messages
 *              object_para:Object parameters (standard or extended format, 
 *                          data or remote frame)
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_rx_object (U32 ctrl, U32 ch, U32 id, U32 object_para)  {
  TSB_CANMB_TypeDef *MailBox = table_mailbox_addr[ch-1];        /* Mbx adres */
  U32                 mbx_mask = (1 << (ch-1));

  TSB_CAN->MC     &= ~mbx_mask;           /* Disable mailbox                 */
  TSB_CAN->MD     |=  mbx_mask;           /* Set mailbox direction (rx)      */

  if ((object_para & FORMAT_TYPE) == STANDARD_TYPE)  { /* Standard identifier*/
    MailBox->ID     = ((id & 0x000007FF) << 18);
  } else {                                             /* Extended identifier*/
    MailBox->ID     = ((id & 0x1FFFFFFF) | (1UL << 31));
  }

  TSB_CAN->MBIM   |=  mbx_mask;           /* Enable mailbox interrupt        */
  TSB_CAN->MC     |=  mbx_mask;           /* Enable mailbox                  */

  return CAN_OK;
}

/*--------------------------- CAN_hw_tx_object ------------------------------
 *
 *  This function setups object that is going to be used for the message 
 *  transmission, the setup of transmission object is not necessery so this 
 *  function is not implemented
 *
 *  Parameter:  ctrl:       Index of the CAN controller            
 *              ch:         Index of object used for transmission (mailbox)
 *              object_para:Object parameters (standard or extended format, 
 *                          data or remote frame)
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_tx_object (U32 ctrl, U32 ch, U32 object_para)  {

  return CAN_OK;
}


/************************* Interrupt Function ********************************/

/*------------------------ INTCANRX_IRQHandler ------------------------------
 *
 *  CAN receive interrupt function 
 *  Received messages are read from hardware registers and puts it into 
 *  receive mailbox
 *---------------------------------------------------------------------------*/

#if USE_CAN_CTRL1 == 1
void INTCANRX_IRQHandler (void) { 
  CAN_msg *ptrmsg;
  U32      rmp_flg, rfp_flg, flg, flg_xor, i;

  rmp_flg    = TSB_CAN->RMP;            /* Read Receive Message Pending Reg  */
  rfp_flg    = TSB_CAN->RFP;            /* Read Remote Frame Pending Register*/
  flg        = rmp_flg | rfp_flg;       /* Union of receive message flags    */
  flg_xor    = flg;                     /* Used for optimization             */

  for (i = 0; i < 32; i++) {
    if (flg_xor & (1 << i)) {           /* If a message in mbx was received  */
      flg_xor ^= (1 << i);
      /* If the mailbox isn't full read the message from the hardware and
         send it to the message queue                                        */
      if (isr_mbx_check (MBX_rx_ctrl[0]) > 0)  {

        ptrmsg = _alloc_box (CAN_mpool);
        if (ptrmsg) {
          CAN_hw_rd (0, i+1, ptrmsg);   /* Read received message             */
          isr_mbx_send (MBX_rx_ctrl[0], ptrmsg);
        }
      }
    }
    if (!flg_xor)                       /* If no more rece msgs to process   */
      break;
  }
  TSB_CAN->RMP   = rmp_flg;             /* Clear Receive Message Pending Flg */
  TSB_CAN->RFP   = rfp_flg;             /* Clear Remote Frame Pending Flags  */
  TSB_CAN->MBRIF = flg;                 /* Clear Mbx Receive Interrupt Flags */
}
#endif

/*------------------------ INTCANTX_IRQHandler ------------------------------
 *
 *  CAN transmit interrupt function 
 *  Written messages are acknowledged
 *---------------------------------------------------------------------------*/

#if USE_CAN_CTRL1 == 1
void INTCANTX_IRQHandler (void) { 
  U32 flg;

  flg             = TSB_CAN->TA;        /* Read Transmission Acknowlegde Reg */
  TSB_CAN->TA    = flg;                 /* Clear Transmission Acknowlegde Flg*/
  TSB_CAN->MBTIF = flg;                 /* Clear Mbx Transmit Interrupt Flags*/
}
#endif


/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/

