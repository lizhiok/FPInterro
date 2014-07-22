/*------------------------------------------------------------------------------
 *      RL-ARM - CAN
 *------------------------------------------------------------------------------
 *      Name:    CAN_XMC4500.c
 *      Purpose: CAN Driver, Hardware specific module for Infineon XMC4500
 *      Rev.:    V4.70
 *------------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include <RTL.h>                        /* RTX kernel functions & defines     */
#include <RTX_CAN.h>                    /* CAN Generic functions & defines    */
#include <XMC4500.h>


/************************* CAN Hardware Configuration *************************/

// *** <<< Use Configuration Wizard in Context Menu >>> ***

// <o0> MultiCAN Module Clock (in Hz) <1-1000000000>
//     <i> System Core Clock divided by 12
#define CAN_CLK               10000000

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

/* Definitions                                                                */
#define MAX_CTRL   3
#define MAX_CH    64

/* Pointers to CAN controllers structures                                     */
CAN_NODE_TypeDef *CAN_NODE[] = { CAN_NODE0, CAN_NODE1, CAN_NODE2 };
CAN_MO_TypeDef   *CAN_MO  [] = { CAN_MO0,  CAN_MO1,  CAN_MO2,  CAN_MO3, 
                                 CAN_MO4,  CAN_MO5,  CAN_MO6,  CAN_MO7, 
                                 CAN_MO8,  CAN_MO9,  CAN_MO10, CAN_MO11, 
                                 CAN_MO12, CAN_MO13, CAN_MO14, CAN_MO15, 
                                 CAN_MO16, CAN_MO17, CAN_MO18, CAN_MO19, 
                                 CAN_MO20, CAN_MO21, CAN_MO22, CAN_MO23, 
                                 CAN_MO24, CAN_MO25, CAN_MO26, CAN_MO27, 
                                 CAN_MO28, CAN_MO29, CAN_MO30, CAN_MO31, 
                                 CAN_MO32, CAN_MO33, CAN_MO34, CAN_MO35, 
                                 CAN_MO36, CAN_MO37, CAN_MO38, CAN_MO39, 
                                 CAN_MO40, CAN_MO41, CAN_MO42, CAN_MO43, 
                                 CAN_MO44, CAN_MO45, CAN_MO46, CAN_MO47, 
                                 CAN_MO48, CAN_MO49, CAN_MO50, CAN_MO51, 
                                 CAN_MO52, CAN_MO53, CAN_MO54, CAN_MO55, 
                                 CAN_MO56, CAN_MO57, CAN_MO58, CAN_MO59, 
                                 CAN_MO60, CAN_MO61, CAN_MO62, CAN_MO63 };

/* Global variables                                                           */
U32 tx_mo  [3][2] = { 0 };              /* Object used for tramsnitting       */
U32 set_mo [3][2] = { 0 };              /* Objects handling rcv RTR send DATA */
U32 get_mo [3][2] = { 0 };              /* Objects handling send RTR get DATA */


/************************* Auxiliary Functions ********************************/

/*--------------------------- CAN_set_timing -----------------------------------
 *
 *  Setup the CAN timing with specific parameters
 *  Bit time looks like below:
 *  /--------------|--------------------------------------|-------------------\
 *  | sync seg = 1 |                tseg1                 |       tseg2       | 
 *  \--------------|--------------------------------------|-------------------/
 *  <-------------------- around 70 % ------------------->|<-- around 30 % -->
 *
 *  Parameter:  ctrl:       Index of the hardware CAN controller (1 .. x)
 *              brp:        Baud Rate Prescaler
 *              tseg1:      Specifies Time Segment, including propagation 
 *                          segment, before the sample point (in time quanta) 
 *              tseg2:      Time Segment after the sample point (in time quanta)
 *
 *  Return:                 __TRUE = success, __FALSE = fail
 *----------------------------------------------------------------------------*/

static BOOL CAN_set_timing (U32 ctrl, U32 brp, U32 tseg1, U32 tseg2) {
  CAN_NODE_TypeDef *CAN_NODEx = CAN_NODE[ctrl-1];
  U32 div8 = 0;
  U32 sjw;

  /* Check input parameters are in expected boundaries                        */
  if ((!brp)      || (brp     > 512)  || 
      (tseg1 < 2) || (tseg1   >  15)  || 
      (tseg2 < 1) || (tseg2   >   7))  {
    return (__FALSE);
  }

  /* Calculate other needed values                                            */
  sjw = ((1 + tseg1) < tseg2) ? (1 + tseg1) : (tseg2);
  if (sjw > 4)  sjw = 4;
  if (!sjw)     sjw = 1;

  if (brp > 64) {
    div8 = 1;
    brp /= 8;
  }

  CAN_NODEx->NBTR  =  0;                /* Clear Bit Timing Register          */
  CAN_NODEx->NBTR |= ( div8     << 15) |/* Configure Bit Timing Register      */
                     ((tseg2-1) << 12) |
                     ((tseg1-1) <<  8) |
                     ((sjw  -1) <<  6) |
                     ((brp  -1) <<  0) ;

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
     TSEG1, TSEG2 are values that should be written into register, 
     these values are used with + 1 for timing values
     TSEG1    = 5 (tseg1    = TSEG1    + 1),  \
     TSEG2    = 2 (tseg2    = TSEG2    + 1),  - => 1 CAN bit = 10 TQ, sample at 70%
     sample point = ((1+tseg1)/(1+tseg1+tseg2)) * 100%        

    /--------------|--------------------------------------|-------------------\
    | sync seg = 1 |             tseg1 = 6                |    tseg2 = 3      | 
    \--------------|--------------------------------------|-------------------/
    <------------------------ 70 % ---------------------->|<------ 30 % ----->
  */

  if (!CAN_set_timing (ctrl, CAN_CLK / 10 / baudrate, 6, 3)) 
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
  CAN_NODE_TypeDef *CAN_NODEx = CAN_NODE[ctrl-1];
  static U32 clk_set = 0;

  if (ctrl > MAX_CTRL) 
    return CAN_UNEXIST_CTRL_ERROR;

  if (!clk_set) {                       /* Set clock to module only once      */
    SCU_RESET->PRCLR1 = 1 << 4;         /* MultiCAN Reset Clear               */
    CAN->CLC &= ~1;                     /* Enable Module                      */
    while (CAN->CLC & 2);               /* Wait until module is enabled       */
    CAN->FDR  =  0;                     /* Clear FDR                          */
    CAN->FDR |= (1 << 14) | (1012);     /* fCAN = fSYS / (1024 -1012) = 10 MHz*/
    clk_set   =  1;
  }

  /* It is necessary to set CCE bit prior selecting transmit/receive ports    */
  CAN_NODEx->NCR |=  (1 << 6);          /* Configuration Change Enable, CCE=1 */

  switch (ctrl) {
    case 1:
      #if USE_CAN_CTRL1 == 1

        /* Setup CAN0 pins                                                    */
        /* Possible pins: 
           CAN0_TXDC0  - P0.0, P1.4, P3.2, P3.10 
           CAN0_RXDC0A - P1.5
           CAN0_RXDC0B - P14.3
           CAN0_RXDC0C - P3.12                                                */

        /* Enable CAN0 interrupt                                              */
        NVIC_SetPriority (CAN0_1_IRQn, 1);
        NVIC_EnableIRQ   (CAN0_1_IRQn);
      #endif
      break;

    case 2: 
      #if USE_CAN_CTRL2 == 1
        /* Setup CAN1 pins                                                    */
        /* Possible pins: 
           CAN1_TXDC1  - P3.9, P2.7, P1.12, P1.5
           CAN1_RXDC1A - P2.6
           CAN1_RXDC1B - P3.11
           CAN1_RXDC1C - P1.13
           CAN1_RXDC1D - P1.4
           CAN1_RXDC1F - CAN0INS                                              */

        /* Enable CAN1 interrupt                                              */
        NVIC_SetPriority (CAN0_2_IRQn, 1);
        NVIC_EnableIRQ   (CAN0_2_IRQn);
      #endif
      break;

    case 3: 
      #if USE_CAN_CTRL3 == 1
        /* Setup CAN2 pins                                                    */
        /* Possible pins: 
           CAN2_TXDC2  - P3.7, P1.9, P4.7
           CAN2_RXDC2A - P1.8
           CAN2_RXDC2B - P3.8
           CAN2_RXDC2C - P4.6
           CAN2_RXDC2F - CAN1INS                                              */

        /* Setup P1.9 as CAN_TX pin                                           */
        PORT1->IOCR8  &=  ~(0xFFUL <<  8);  /* Clear P1.9 settings            */
        PORT1->IOCR8  |=   (0x12UL << 11);  /* P1.9 output ALT2 push-pull     */
        PORT1->PDR1   &=  ~(0x07UL << 4);   /* Clear P1.9 output settings     */
        PORT1->PDR1   |=   (0x02UL << 4);   /* P1.9 output strong             */

        /* Setup P1.8 as CAN_RX pin                                           */
        PORT1->IOCR8  &=  ~(0xFFUL << 0);   /* Clear P1.8 settings            */
        PORT1->IOCR8  |=   (0x00UL << 0);   /* P1.8 input no pull device      */
        CAN_NODEx->NPCR &= ~7;              /* Select CAN2_RXDC2A as CAN2_RX  */

        /* Enable CAN2 interrupt                                              */
        NVIC_SetPriority (CAN0_3_IRQn, 1);
        NVIC_EnableIRQ   (CAN0_3_IRQn);
      #endif
      break;
  }

  CAN_NODEx->NCR &= ~(1 << 6);          /* Configuration Change Disable, CCE=0*/

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
  CAN_NODE_TypeDef *CAN_NODEx = CAN_NODE[ctrl-1];

  if (ctrl > MAX_CTRL) 
    return CAN_UNEXIST_CTRL_ERROR;

  CAN_NODEx->NCR  |=  ((1 << 0) |       /* Node initialization, INIT = 1      */
                       (1 << 6));       /* Configuration Change Enable, CCE=1 */
  CAN_NODEx->NIPR &=  ~(7 << 8) ;       /* Clear TRINP field                  */
  CAN_NODEx->NIPR |=   (ctrl << 8);     /* Select INT_0(ctrl) for transfer int*/

  if (CAN_hw_set_baudrate(ctrl, baudrate) != CAN_OK)         /* Set baudrate  */
    return CAN_BAUDRATE_ERROR;

                                        /* Clear all message object bit masks */
  tx_mo [ctrl-1][0] = 0;
  tx_mo [ctrl-1][1] = 0;
  set_mo[ctrl-1][0] = 0;
  set_mo[ctrl-1][1] = 0;
  get_mo[ctrl-1][0] = 0;
  get_mo[ctrl-1][1] = 0;

  while (CAN->PANCTR & (1 << 8));       /* Wait for RAM init to finish        */

  CAN->MCR    &= ~(0x0F << 12);         /* MPSEL = 0000B - allocation case 1  */
  CAN->MSIMASK =   0xFFFFFFFF;          /* Enable all messages                */
  
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
  CAN_NODE_TypeDef *CAN_NODEx = CAN_NODE[ctrl-1];

  if (ctrl > MAX_CTRL) 
    return CAN_UNEXIST_CTRL_ERROR;

  CAN_NODEx->NCR &= ~((1 << 0) |        /* Node not in init, INIT = 0         */
                      (1 << 6));        /* Configuration Change Disable, CCE=0*/
  CAN_NODEx->NCR |=   (1 << 1) ;        /* Transmit Interrupt Enable, TRIE = 1*/

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
  CAN_NODE_TypeDef *CAN_NODEx = CAN_NODE[ctrl-1];

  /* NOTE: in this case test mode does not work on one node, so 1 node can not 
           send and receive it's own message only test mode could work between 
           different nodes in same conttroller                                */

  if (ctrl > MAX_CTRL) 
    return CAN_UNEXIST_CTRL_ERROR;

  CAN_NODEx->NPCR &= ~(            1  << 8);
  CAN_NODEx->NPCR |=  ((testmode & 1) << 8);

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

  if (ctrl > MAX_CTRL) 
    return CAN_UNEXIST_CTRL_ERROR;

  if ((os_sem_wait (wr_sem[ctrl-1], 0) != OS_R_TMO)){ /* If semaphore is free */
    return CAN_OK;
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
  CAN_MO_TypeDef *CAN_MOx = CAN_MO[msg->ch-1];
  BOOL rmt;
  BOOL std;
  U8   mo_idx, msk;

  if (ctrl > MAX_CTRL) 
    return CAN_UNEXIST_CTRL_ERROR;

  if (msg->ch > MAX_CH) 
    return CAN_UNEXIST_CH_ERROR;

  rmt = (msg->type   == REMOTE_FRAME);
  std = (msg->format == STANDARD_FORMAT);

  /* Setup Message Object for transmission                                    */

  /* Setup the identifier information                                         */
  if (std)  {                                              /* Standard ID     */
    CAN_MOx->MOAMR = (2UL << 30) |             (U32)(msg->id << 18);
    CAN_MOx->MOAR  = (2UL << 30) |             (U32)(msg->id << 18);
  }  else  {                                               /* Extended ID     */
    CAN_MOx->MOAMR = (2UL << 30) | (1 << 29) | (U32)(msg->id);
    CAN_MOx->MOAR  = (2UL << 30) | (1 << 29) | (U32)(msg->id);
  }

  /* Setup data bytes                                                         */
  CAN_MOx->MODATAL = ((U32)msg->data[3] << 24) | 
                     ((U32)msg->data[2] << 16) | 
                     ((U32)msg->data[1] <<  8) | 
                     ((U32)msg->data[0]      ) ;
  CAN_MOx->MODATAH = ((U32)msg->data[7] << 24) | 
                     ((U32)msg->data[6] << 16) | 
                     ((U32)msg->data[5] <<  8) | 
                     ((U32)msg->data[4]      ) ;

  CAN_MOx->MOFCR   = (msg->len << 24) | /* DLC  = Data Length                 */
                    (( rmt) << 21) |    /* RMM  = 1 for Remote Monitoring     */
                    (( rmt) << 16) |    /* RXIE = 1 for DATA answer receive   */
                     (   1  << 17) |    /* TXIE = 1 -> Transmit Interrupt En  */
                     (   0  <<  0) ;    /* MMC  = 0 -> Standard Message Object*/

                                        /* Update object bit mask             */
  mo_idx = ((msg->ch-1) >> 5) & 1;
  msk    = (1 << ((msg->ch-1) & 0x1F));
  set_mo[ctrl-1][mo_idx] &=~msk;        /* It is not SET object               */
  if (rmt) 
    get_mo[ctrl-1][mo_idx] |=  msk;     /* If RTR tx requested then it's GET  */

  /* Activate transmission                                                    */
  CAN_MOx->MOCTR   = (    1  <<  6) |   /* Clear RTSEL                        */
                     ((!rmt) <<  7) |   /* Clear RXEN for DATA sending        */
                     (( rmt) << 11) |   /* Clear DIR for RTR sending          */
                     (    1  << 19) |   /* Set NEWDAT                         */
                     (    1  << 21) |   /* Set MSGVAL                         */
                     (( rmt) << 23) |   /* Set RXEN for RTR sending           */
                     (    1  << 24) |   /* Set TXRQ                           */
                     (    1  << 25) |   /* Set TXEN0                          */
                     (    1  << 26) |   /* Set TXEN1                          */
                     ((!rmt) << 27);    /* Set DIR for DATA sending           */

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
  CAN_MO_TypeDef *CAN_MOx = CAN_MO[ch-1];

  /* Read identifier information                                              */
  if (!(CAN_MOx->MOAR & (1 << 29))) {                       /* Standard ID    */
    msg->format =  STANDARD_FORMAT;
    msg->id     =  0x000007FFUL & (CAN_MOx->MOAR >> 18);
  }  else  {                                                /* Extended ID    */
    msg->format =  EXTENDED_FORMAT;
    msg->id     =  0x1FFFFFFFUL & CAN_MOx->MOAR;
  }

  /* Read type information                                                    */
  msg->type     =  DATA_FRAME;                              /* DATA   FRAME   */

  /* Read length (number of received bytes)                                   */
  msg->len      = (CAN_MOx->MOFCR >> 24) & 0x0F;

  /* Read data bytes                                                          */
  msg->data[0]  = (CAN_MOx->MODATAL      );
  msg->data[1]  = (CAN_MOx->MODATAL >>  8);
  msg->data[2]  = (CAN_MOx->MODATAL >> 16);
  msg->data[3]  = (CAN_MOx->MODATAL >> 24);
  msg->data[4]  = (CAN_MOx->MODATAH      );
  msg->data[5]  = (CAN_MOx->MODATAH >>  8);
  msg->data[6]  = (CAN_MOx->MODATAH >> 16);
  msg->data[7]  = (CAN_MOx->MODATAH >> 24);

  CAN_MOx->MOCTR = (1 <<  3);           /* Reset NEWDAT                       */
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
  CAN_MO_TypeDef *CAN_MOx = CAN_MO[msg->ch-1];
  BOOL std;
  U8   mo_idx, msk;

  if (ctrl > MAX_CTRL) 
    return CAN_UNEXIST_CTRL_ERROR;

  if (msg->ch > MAX_CH) 
    return CAN_UNEXIST_CH_ERROR;

  std = (msg->format == STANDARD_FORMAT);

  /* Setup Message Object for transmission                                    */

  /* Setup the identifier information                                         */
  if (std)  {                                              /* Standard ID     */
    CAN_MOx->MOAMR = (2UL << 30) |             (U32)(msg->id << 18);
    CAN_MOx->MOAR  = (2UL << 30) |             (U32)(msg->id << 18);
  }  else  {                                               /* Extended ID     */
    CAN_MOx->MOAMR = (2UL << 30) | (1 << 29) | (U32)(msg->id);
    CAN_MOx->MOAR  = (2UL << 30) | (1 << 29) | (U32)(msg->id);
  }

  /* Setup data bytes                                                         */
  CAN_MOx->MODATAL = ((U32)msg->data[3] << 24) | 
                     ((U32)msg->data[2] << 16) | 
                     ((U32)msg->data[1] <<  8) | 
                     ((U32)msg->data[0]      ) ;
  CAN_MOx->MODATAH = ((U32)msg->data[7] << 24) | 
                     ((U32)msg->data[6] << 16) | 
                     ((U32)msg->data[5] <<  8) | 
                     ((U32)msg->data[4]      ) ;

  CAN_MOx->MOFCR   = (msg->len << 24) | /* DLC  = Data Length                 */
                     (1 << 21) |        /* RMM  = 1 for Remote Monitoring     */
                     (1 << 16) |        /* RXIE = 1 for DATA answer receive   */
                     (1 << 17) |        /* TXIE = 1 for DATA answer send      */
                     (0 <<  0) ;        /* MMC  = 0 -> Standard Message Object*/

                                        /* Update object bit mask             */
  mo_idx = ((msg->ch-1) >> 5) & 1;
  msk    = (1 << ((msg->ch-1) & 0x1F));
  get_mo[ctrl-1][mo_idx] &= ~msk;       /* It is not GET object               */
  set_mo[ctrl-1][mo_idx] |=  msk;       /* It is SET object                   */

  /* Activate reception of RTR                                                */
  CAN_MOx->MOCTR   = (1 <<  6) |        /* Clear RTSEL                        */
                     (1 <<  8) |        /* Clear TXRQ, auto set on RTR rcv    */
                     (1 << 19) |        /* Set NEWDAT                         */
                     (1 << 21) |        /* Set MSGVAL                         */
                     (1 << 23) |        /* Set RXEN                           */
                     (1 << 25) |        /* Set TXEN0                          */
                     (1 << 26) |        /* Set TXEN1                          */
                     (1 << 27) ;        /* Set DIR for RTR reception          */

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
  CAN_MO_TypeDef *CAN_MOx = CAN_MO[ch-1];
  BOOL std;
  U8   mo_idx, msk;

  if (ctrl > MAX_CTRL) 
    return CAN_UNEXIST_CTRL_ERROR;

  if (ch > MAX_CH) 
    return CAN_UNEXIST_CH_ERROR;

  std = (object_para & FORMAT_TYPE) == STANDARD_TYPE;

  while (CAN->PANCTR & (3 << 8));       /* While BUSY or RBUSY are active     */

  CAN->PANCTR  = ((ctrl) << 24) |       /* Statically allocate obj to list    */
                 ((ch-1) << 16) |
                 ((0x02) <<  0) ;

  while (CAN->PANCTR & (3 << 8));       /* While BUSY or RBUSY are active     */

  /* Setup Message Object for reception                                       */

  CAN_MOx->MOFCR &= ~0x0F;              /* Message Mode Control = Std Object  */

  /* Setup the identifier information                                         */
  if (std)  {                                              /* Standard ID     */
    CAN_MOx->MOAMR =             (U32)(id << 18);
    CAN_MOx->MOAR  =             (U32)(id << 18);
  }  else  {                                               /* Extended ID     */
    CAN_MOx->MOAMR = (1 << 29) | (U32)(id);
    CAN_MOx->MOAR  = (1 << 29) | (U32)(id);
  }

                                        /* Update object bit mask             */
  mo_idx = ((ch-1) >> 5) & 1;
  msk    = (1 << ((ch-1) & 0x1F));
  tx_mo [ctrl-1][mo_idx] &= ~msk;       /* Clear TX so it is RX object        */
  set_mo[ctrl-1][mo_idx] &= ~msk;       /* It is not SET object               */
  get_mo[ctrl-1][mo_idx] &= ~msk;       /* It is not GET object               */

                                        /* Enable RX object to trigger INT_0n */
                                        /* where n is controller index        */
  CAN_MOx->MOIPR  &= ~0xFFFFFFFF;       /* Clear MOIPR                        */
  CAN_MOx->MOIPR  |= ((ctrl - 1)<< 14)| /* Bits 15..14 of MPN   in MOIPR      */
                     ((ch >  32)<< 13)| /* Bit  13     of MPN   in MOIPR      */
                     (((ch-1)&0x1F)<<8)|/* Bits 12.. 8 of MPN   in MOIPR      */
                     ((ctrl)         ); /* Bits 3 .. 0 of RXINP in MOIPR      */
  CAN_MOx->MOFCR  |=  1 << 16;          /* Receive Interrupt Enable           */

  /* Activate reception                                                       */
  CAN_MOx->MOCTR   = (1 <<  3) |        /* Clear NEWDAT                       */
                     (1 <<  6) |        /* Clear RTSEL                        */
                     (7 <<  8) |        /* Clear TXRQ, TXEN0, TXEN1           */
                     (1 << 11) |        /* Clear DIR                          */
                     (1 << 21) |        /* Set MSGVAL                         */
                     (1 << 23) ;        /* Set RXEN                           */

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
  CAN_MO_TypeDef *CAN_MOx = CAN_MO[ch-1];
  U8   mo_idx, msk;

  if (ctrl > MAX_CTRL) 
    return CAN_UNEXIST_CTRL_ERROR;

  if (ch > MAX_CH) 
    return CAN_UNEXIST_CH_ERROR;

  while (CAN->PANCTR & (3 << 8));       /* While BUSY or RBUSY are active     */

  CAN->PANCTR  = ((ctrl) << 24) |       /* Statically allocate obj to list    */
                 ((ch-1) << 16) |
                 ((0x02) <<  0) ;

  while (CAN->PANCTR & (3 << 8));       /* While BUSY or RBUSY are active     */

  /* Setup Message Object for transmission                                    */
                                        /* Update object bit mask             */
  mo_idx = ((ch-1) >> 5) & 1;
  msk    = (1 << ((ch-1) & 0x1F));
  tx_mo [ctrl-1][mo_idx] |=  msk;       /* It is TX object                    */
  set_mo[ctrl-1][mo_idx] &= ~msk;       /* It is not SET object               */
  get_mo[ctrl-1][mo_idx] &= ~msk;       /* It is not GET object               */

                                        /* Enable TX object to trigger INT_0n */
                                        /* where n is controller index        */
  CAN_MOx->MOIPR  &= ~0xFFFFFFFF;       /* Clear MOIPR                        */
  CAN_MOx->MOIPR  |= ((ctrl - 1)<< 14)| /* Bits 15..14 of MPN   in MOIPR      */
                     ((ch >  32)<< 13)| /* Bit  13     of MPN   in MOIPR      */
                     (((ch-1)&0x1F)<<8)|/* Bits 12.. 8 of MPN   in MOIPR      */
                     ((ctrl)    <<  4); /* Bits 6 .. 4 of TXINP in MOIPR      */

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

#define CTRL                  1
#define K                     0

void CAN0_1_IRQHandler (void) {
  CAN_msg *ptrmsg;
  U32      mspndk;
  U32      mspndk1;
  U32      mostat;
  U8       k, idx, mo_grp, mo_idx, tx, set, get;

  mspndk  = CAN->MSPND[K];
  mspndk1 = CAN->MSPND[K+1];
  while (mspndk | mspndk1) {
    k  = K;
    tx = 0;
    if (mspndk) {
      idx    = CAN->MSID[K];
      mo_grp = 0;
    } else {
      k      = K+1;
      idx    = CAN->MSID[K+1];
      mo_grp = 1;
    }
    tx     = (tx_mo [CTRL-1][mo_grp] >> idx) & 1;
    set    = (set_mo[CTRL-1][mo_grp] >> idx) & 1;
    get    = (get_mo[CTRL-1][mo_grp] >> idx) & 1;
    mo_idx = (mo_grp << 5) + idx;
    mostat = CAN_MO[mo_idx]->MOSTAT;

    if ((tx || get || set) && (mostat & 2)) {     /* If tx interrupt          */
      CAN_MO[mo_idx]->MOCTR = (1 <<  1);/* Clear TXPND                        */

      /* If there is a message in the mailbox ready for send, read the 
         message from the mailbox and send it                                 */
      if (isr_mbx_receive (MBX_tx_ctrl[CTRL-1], (void **)&ptrmsg) != OS_R_OK) {
        CAN_hw_wr (CTRL, ptrmsg);
        _free_box(CAN_mpool, ptrmsg);
      } else {
        isr_sem_send(wr_sem[CTRL-1]);   /* Return a token back to semaphore   */
      }
    } 
    if ((!tx || get || set) && (mostat & 1)) {    /* If rx interrupt          */
      if (!set) {
        /* If the mailbox isn't full read the message from the hardware and
           send it to the message queue otherwise dump the rceived message    */
        if (isr_mbx_check (MBX_rx_ctrl[CTRL-1]) > 0)  {
          ptrmsg = _alloc_box (CAN_mpool);
          if (ptrmsg) {
            CAN_hw_rd (CTRL, mo_idx+1, ptrmsg);   /* Read received message    */
            isr_mbx_send (MBX_rx_ctrl[CTRL-1], ptrmsg);
          }
        }
      }
      CAN_MO[mo_idx]->MOCTR = (1 <<  0);/* Clear RXPND                        */
    }

    CAN->MSPND[k] &= ~(1 << idx);       /* Clear pending interrupt            */
    mspndk  = CAN->MSPND[K];
    mspndk1 = CAN->MSPND[K+1];
  }
}


#undef  CTRL
#define CTRL                  2
#undef  K
#define K                     2

void CAN0_2_IRQHandler (void) {
  CAN_msg *ptrmsg;
  U32      mspndk;
  U32      mspndk1;
  U32      mostat;
  U8       k, idx, mo_grp, mo_idx, tx, set, get;

  mspndk  = CAN->MSPND[K];
  mspndk1 = CAN->MSPND[K+1];
  while (mspndk | mspndk1) {
    k  = K;
    tx = 0;
    if (mspndk) {
      idx    = CAN->MSID[K];
      mo_grp = 0;
    } else {
      k      = K+1;
      idx    = CAN->MSID[K+1];
      mo_grp = 1;
    }
    tx     = (tx_mo [CTRL-1][mo_grp] >> idx) & 1;
    set    = (set_mo[CTRL-1][mo_grp] >> idx) & 1;
    get    = (get_mo[CTRL-1][mo_grp] >> idx) & 1;
    mo_idx = (mo_grp << 5) + idx;
    mostat = CAN_MO[mo_idx]->MOSTAT;

    if ((tx || get || set) && (mostat & 2)) {     /* If tx interrupt          */
      CAN_MO[mo_idx]->MOCTR = (1 <<  1);/* Clear TXPND                        */

      /* If there is a message in the mailbox ready for send, read the 
         message from the mailbox and send it                                 */
      if (isr_mbx_receive (MBX_tx_ctrl[CTRL-1], (void **)&ptrmsg) != OS_R_OK) {
        CAN_hw_wr (CTRL, ptrmsg);
        _free_box(CAN_mpool, ptrmsg);
      } else {
        isr_sem_send(wr_sem[CTRL-1]);   /* Return a token back to semaphore   */
      }
    } 
    if ((!tx || get || set) && (mostat & 1)) {    /* If rx interrupt          */
      if (!set) {
        /* If the mailbox isn't full read the message from the hardware and
           send it to the message queue otherwise dump the rceived message    */
        if (isr_mbx_check (MBX_rx_ctrl[CTRL-1]) > 0)  {
          ptrmsg = _alloc_box (CAN_mpool);
          if (ptrmsg) {
            CAN_hw_rd (CTRL, mo_idx+1, ptrmsg);   /* Read received message    */
            isr_mbx_send (MBX_rx_ctrl[CTRL-1], ptrmsg);
          }
        }
      }
      CAN_MO[mo_idx]->MOCTR = (1 <<  0);/* Clear RXPND                        */
    }

    CAN->MSPND[k] &= ~(1 << idx);       /* Clear pending interrupt            */
    mspndk  = CAN->MSPND[K];
    mspndk1 = CAN->MSPND[K+1];
  }
}


#undef  CTRL
#define CTRL                  3
#undef  K
#define K                     4

void CAN0_3_IRQHandler (void) {
  CAN_msg *ptrmsg;
  U32      mspndk;
  U32      mspndk1;
  U32      mostat;
  U8       k, idx, mo_grp, mo_idx, tx, set, get;

  mspndk  = CAN->MSPND[K];
  mspndk1 = CAN->MSPND[K+1];
  while (mspndk | mspndk1) {
    k  = K;
    tx = 0;
    if (mspndk) {
      idx    = CAN->MSID[K];
      mo_grp = 0;
    } else {
      k      = K+1;
      idx    = CAN->MSID[K+1];
      mo_grp = 1;
    }
    tx     = (tx_mo [CTRL-1][mo_grp] >> idx) & 1;
    set    = (set_mo[CTRL-1][mo_grp] >> idx) & 1;
    get    = (get_mo[CTRL-1][mo_grp] >> idx) & 1;
    mo_idx = (mo_grp << 5) + idx;
    mostat = CAN_MO[mo_idx]->MOSTAT;

    if ((tx || get || set) && (mostat & 2)) {     /* If tx interrupt          */
      CAN_MO[mo_idx]->MOCTR = (1 <<  1);/* Clear TXPND                        */

      /* If there is a message in the mailbox ready for send, read the 
         message from the mailbox and send it                                 */
      if (isr_mbx_receive (MBX_tx_ctrl[CTRL-1], (void **)&ptrmsg) != OS_R_OK) {
        CAN_hw_wr (CTRL, ptrmsg);
        _free_box(CAN_mpool, ptrmsg);
      } else {
        isr_sem_send(wr_sem[CTRL-1]);   /* Return a token back to semaphore   */
      }
    } 
    if ((!tx || get || set) && (mostat & 1)) {    /* If rx interrupt          */
      if (!set) {
        /* If the mailbox isn't full read the message from the hardware and
           send it to the message queue otherwise dump the rceived message    */
        if (isr_mbx_check (MBX_rx_ctrl[CTRL-1]) > 0)  {
          ptrmsg = _alloc_box (CAN_mpool);
          if (ptrmsg) {
            CAN_hw_rd (CTRL, mo_idx+1, ptrmsg);   /* Read received message    */
            isr_mbx_send (MBX_rx_ctrl[CTRL-1], ptrmsg);
          }
        }
      }
      CAN_MO[mo_idx]->MOCTR = (1 <<  0);/* Clear RXPND                        */
    }

    CAN->MSPND[k] &= ~(1 << idx);       /* Clear pending interrupt            */
    mspndk  = CAN->MSPND[K];
    mspndk1 = CAN->MSPND[K+1];
  }
}


/*------------------------------------------------------------------------------
 * end of file
 *----------------------------------------------------------------------------*/

