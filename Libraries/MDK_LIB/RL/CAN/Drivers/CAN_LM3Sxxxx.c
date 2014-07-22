/*----------------------------------------------------------------------------
 *      RL-ARM - CAN
 *----------------------------------------------------------------------------
 *      Name:    CAN_LM3Sxxxx.c
 *      Purpose: CAN Driver, Hardware specific module for Luminary LM3Sxxxx
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <RTL.h>                      /* RTX kernel functions & defines      */
#include <RTX_CAN.h>                  /* CAN Generic functions & defines     */
#include <LM3Sxxxx.h>                 /* LM3Sxxxx library definitions        */

#pragma diag_suppress 177

/************************* CAN Hardware Configuration ************************/

// *** <<< Use Configuration Wizard in Context Menu >>> ***

// <o> CAN Module Clock (in Hz) <1000000-50000000>
//     <i> Default: 8000000
#define CAN_CLOCK         8000000

// *** <<< End of Configuration section             >>> ***


/*----------------------------------------------------------------------------
 *      CAN RTX Hardware Specific Driver Functions
 *----------------------------------------------------------------------------
 *  Functions implemented in this module:
 *    static U32       Rd                  (U32 reg_adr)
 *    static void      Wr                  (U32 reg_adr, U32 val)
 *    static CAN_ERROR CAN_get_ctrl        (U32 ctrl, U32 *ptr_can) 
 *    static CAN_ERROR CAN_hw_set_baudrate (U32 ctrl, U32 baudrate)
 *           CAN_ERROR CAN_hw_loopback     (U32 ctrl)
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
  void CAN0_IRQHandler               (void) __irq;
#endif
#if USE_CAN_CTRL2 == 1
  void CAN1_IRQHandler               (void) __irq;
#endif
#if USE_CAN_CTRL3 == 1
  void CAN2_IRQHandler               (void) __irq;
#endif

/* Static functions used only in this module (only used because errata)      */
static U32  Rd                       (U32 reg_adr);
static void Wr                       (U32 reg_adr, U32 val);

/* Global variables used in this module                                      */
static U32  clk_ratio = 1;            /* SYS clk / CAN clk ratio             */


/************************* Auxiliary Functions *******************************/

/*--------------------------- Rd --------------------------------------------
 *
 *  Read CAN register with software delay to workaround hardware problem    
 *
 *  Parameter:  reg_adr:    Address of CAN register
 *
 *  Return:     U32:        Read register value
 *---------------------------------------------------------------------------*/

static U32 Rd (U32 reg_adr)
{
  U32 i;
  U32 ret_val;
  U32 int_reenable;
  U32 int_msk;

  switch (reg_adr & 0xFFFFF000) {     /* Get CAN controller interrupt mask   */
    case CAN0_BASE:
      int_msk = (1 << (INT_CAN0 - 48));
      break;
    case CAN1_BASE:
      int_msk = (1 << (INT_CAN1 - 48));
      break;
    default:
      int_msk = (1 << (INT_CAN0 - 48));
      break;
  }

  /* Remember interrupt enable state                                         */
  int_reenable = HWREG(NVIC_EN1) & int_msk;

  /* Disable current CAN controller interrupt if enabled                     */
  if (int_reenable)
    HWREG (NVIC_DIS1) = int_msk;

  HWREG (reg_adr);                    /* Dummy read                          */

  for(i = 0; i < clk_ratio; i++);     /* Wait for at least SYSclk/CANclk     */

  ret_val = HWREG (reg_adr);          /* Real register read value            */

  /* Reenable current CAN controller interrupt if it was enabled before      */
  if (int_reenable)
    HWREG (NVIC_EN1) = int_msk;

  return (ret_val);                   /* Return read register value          */
}

/*--------------------------- Wr --------------------------------------------
 *
 *  Write CAN register with software delay to workaround hardware problem    
 *
 *  Parameter:  reg_adr:    Address of CAN register
 *              val:        Value to be written
 *
 *  Return:     none
 *---------------------------------------------------------------------------*/

static void Wr (U32 reg_adr, U32 val)
{
  U32 i;

  HWREG (reg_adr) = val;              /* Write value                         */

  for(i = 0; i < clk_ratio; i++);     /* Wait for at least SYSclk/CANclk     */
}

/*--------------------------- CAN_get_ctrl ----------------------------------
 *
 *  Get address and return it in parameter of function, of controller which 
 *  index is passed as parameter
 *
 *  Parameter:  ctrl:       Index of CAN controller
 *              ptr_can:    Address of CAN controller in memory map (returned)
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

static CAN_ERROR CAN_get_ctrl (U32 ctrl, U32 *ptr_can) {

  switch (ctrl) {
    case 1: 
      *ptr_can = CAN0_BASE;
      break;
    case 2: 
      *ptr_can = CAN1_BASE;
      break;
    case 3: 
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
  U32 CANx;
  U32 brp = CAN_CLOCK;                /* CAN module clock                    */

  CAN_get_ctrl(ctrl, &CANx);          /* Get address of CAN controller       */

  /* Determine which nominal time to use for requested baudrate and set
     appropriate bit timing                                                  */
  if (baudrate <= 500000)  {
    brp  = (brp / 16) / baudrate;
                                                                          
    /* Load the baudrate registers CANBIT and CANBRPE registers
       so that sample point is at about 70% bit time from bit start          */
    /* TSEG1 = 11, TSEG2 = 4, SJW = 4 => 1 CAN bit = 16 TQ, sample at 75%    */
    Wr(CANx + CAN_O_BIT , (3 << 12) | (10 << 8) | (3 << 6) | ((brp-1) & 0x3F));
    Wr(CANx + CAN_O_BRPE, (((brp-1) >> 6) & 0x0F));
  }  else if (baudrate <= 1000000)  {
    brp  = (brp / 8 ) / baudrate;

    /* Load the baudrate registers CANBIT and CANBRPE registers
       so that sample point is at about 70% bit time from bit start          */
    /* TSEG1 = 4, TSEG2 = 3, SJW = 3 => 1 CAN bit = 8 TQ, sample at 62.5%    */
    Wr(CANx + CAN_O_BIT , (2 << 12) | (3 << 8) | (2 << 6) | ((brp-1) & 0x3F));
    Wr(CANx + CAN_O_BRPE, (((brp-1) >> 6) & 0x0F));
  }  else  {
    return CAN_BAUDRATE_ERROR;
  }  

  return CAN_OK;
}


/*************************** Module Functions ********************************/

/*--------------------------- CAN_hw_loopback -------------------------------
 *
 *  Setup CAN controller in loopback mode
 *
 *  Parameter:  ctrl:       Index of CAN controller
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_loopback (U32 ctrl)  {
  U32 CANx;

  CAN_get_ctrl(ctrl, &CANx);          /* Get address of CAN controller       */

  /* Enable Test mode                                                        */
  Wr(CANx + CAN_O_CTL, Rd(CANx + CAN_O_CTL) | CAN_CTL_TEST);

  /* Activate Loopback                                                       */
  Wr(CANx + CAN_O_TST, Rd(CANx + CAN_O_TST) | CAN_TST_LBACK /*| CAN_TST_SILENT*/);

  return CAN_OK;
}

/*--------------------------- CAN_hw_setup ----------------------------------
 *
 *  Setup CAN transmit and receive PINs and interrupt vectors
 *
 *  Parameter:  ctrl:       Index of CAN controller
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_setup (U32 ctrl)  {
  switch (ctrl) {
    case 1: 
      #if USE_CAN_CTRL1 == 1
        SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);         /* Enable CAN clk*/
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);        /* Enable PIO clk*/
        GPIOPinTypeCAN(GPIO_PORTD_BASE, (1<<0) | (1<<1));   /* Setup pins    */

        /* Set priority and enable interrupt                                 */
        HWREG(NVIC_PRI9)  |= (0xFEUL << 24);
        HWREG(NVIC_EN1)   |= 1 << (INT_CAN0 - 48);
      #endif
      break;
    case 2: 
      #if USE_CAN_CTRL2 == 1
        SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN1);         /* Enable CAN clk*/
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);        /* Enable PIO clk*/
        GPIOPinTypeCAN(GPIO_PORTF_BASE, (1<<0) | (1<<1));   /* Setup pins    */

        /* Set priority and enable interrupt                                 */
        HWREG(NVIC_PRI10) |= (0xFDUL << 0);
        HWREG(NVIC_EN1)   |= 1 << (INT_CAN1 - 48);
      #endif
      break;
    case 3: 
      #if USE_CAN_CTRL3 == 1
        return CAN_UNEXIST_CTRL_ERROR;
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
  U32 i;
  U32 CANx;

  CAN_get_ctrl(ctrl, &CANx);          /* Get address of CAN controller       */

  /* Calculate SYS clk / CAN clk ratio used for software delay because of 
     hardware chip problems                                                  */
  clk_ratio = (U32)((SysCtlClockGet() / CAN_CLOCK) + 1);

  /* Enter init mode, enabled automatic retransmission                       */
  Wr(CANx + CAN_O_CTL, CAN_CTL_CCE | /*CAN_CTL_DAR |/* CAN_CTL_INIT);

  if (CAN_hw_set_baudrate(ctrl, baudrate) != CAN_OK)    /* Set baudrate      */
    return CAN_BAUDRATE_ERROR;

  /* Initialize all objects as unused                                        */
  while (Rd(CANx + CAN_O_IF1CRQ) & CAN_IFCRQ_BUSY);     /* Wait while busy   */
  Wr(CANx + CAN_O_IF1CMSK, CAN_IFCMSK_WRNRD | CAN_IFCMSK_ARB | CAN_IFCMSK_CONTROL);
  Wr(CANx + CAN_O_IF1ARB2, 0);
  Wr(CANx + CAN_O_IF1MCTL, 0);
  for (i = 0; i < 32; i++) {
    while (Rd(CANx + CAN_O_IF1CRQ) & CAN_IFCRQ_BUSY);   /* Wait while busy   */
    Wr(CANx + CAN_O_IF1CRQ, i);
  }
  while (Rd(CANx + CAN_O_IF1CRQ) & CAN_IFCRQ_BUSY);     /* Wait while busy   */
  Wr(CANx + CAN_O_IF1CMSK, CAN_IFCMSK_NEWDAT | CAN_IFCMSK_CLRINTPND);
  for (i = 0; i < 32; i++) {
    while (Rd(CANx + CAN_O_IF1CRQ) & CAN_IFCRQ_BUSY);   /* Wait while busy   */
    Wr(CANx + CAN_O_IF1CRQ, i);
  }
  Rd(CANx + CAN_O_STS);               /* Acknowledge pending interrupts      */

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
  U32 CANx;

  CAN_get_ctrl(ctrl, &CANx);          /* Get address of CAN controller       */

  /* Enable interrrupt, exit init mode, enabled automatic retransmission     */
  Wr(CANx + CAN_O_CTL, /*CAN_CTL_DAR |*/ CAN_CTL_IE);

  return CAN_OK;
}

/*--------------------------- CAN_hw_tx_empty -------------------------------
 *
 *  Check if interface register group 0 (IF1) is available for usage
 *
 *  Parameter:  ctrl:       Index of CAN controller
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_tx_empty (U32 ctrl)  {
  U32 CANx;

  CAN_get_ctrl(ctrl, &CANx);            /* Get address ofo CAN controller     */

  if ((os_sem_wait (wr_sem[ctrl-1], 0) != OS_R_TMO)){ /* If semaphore is free*/
  if (!(Rd(CANx + CAN_O_IF1CRQ) & CAN_IFCRQ_BUSY))    /* Interface 0 is free */
                                                      /* for transmission    */
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
  U32 CANx;

  CAN_get_ctrl(ctrl, &CANx);            /* Get address of CAN controller     */

  /* Set flags for informations that will be changed                         */
  Wr(CANx + CAN_O_IF1CMSK,   CAN_IFCMSK_WRNRD   | 
                             CAN_IFCMSK_ARB     | 
                             CAN_IFCMSK_CONTROL |
                             CAN_IFCMSK_DATAA   |
                             CAN_IFCMSK_DATAB);

  /* Setup the identifier information                                        */
  if (msg->format == STANDARD_FORMAT)  {  /* Standard ID                     */
    Wr(CANx + CAN_O_IF1ARB1, 0);
    Wr(CANx + CAN_O_IF1ARB2, (U16) (msg->id << 2));
  }  else  {                              /* Extended ID                     */
    Wr(CANx + CAN_O_IF1ARB1, (U16) (msg->id));
    Wr(CANx + CAN_O_IF1ARB2, (U16)((msg->id) >> 16) | CAN_IFARB2_XTD);
  }

  /* Setup type information                                                  */
  if (msg->type == DATA_FRAME)  {         /* DATA FRAME                      */
    Wr(CANx + CAN_O_IF1ARB2, Rd(CANx + CAN_O_IF1ARB2) | CAN_IFARB2_DIR);
  }

  /* Setup data bytes                                                        */
  Wr(CANx + CAN_O_IF1DA1, ((U16) msg->data[1]<<8) | msg->data[0]);
  Wr(CANx + CAN_O_IF1DA2, ((U16) msg->data[3]<<8) | msg->data[2]);
  Wr(CANx + CAN_O_IF1DB1, ((U16) msg->data[5]<<8) | msg->data[4]);
  Wr(CANx + CAN_O_IF1DB2, ((U16) msg->data[7]<<8) | msg->data[6]);

  /* Setup length and message control bits                                   */
  Wr(CANx + CAN_O_IF1MCTL, CAN_IFMCTL_NEWDAT | 
                           CAN_IFMCTL_TXIE   |
                           CAN_IFMCTL_TXRQST |
                           CAN_IFMCTL_EOB    |
                           (msg->len & 0x000F));

  /* Set the message valid bit                                               */
  Wr(CANx + CAN_O_IF1ARB2, Rd(CANx + CAN_O_IF1ARB2) | CAN_IFARB2_MSGVAL);

  /* Update informations from interface registers                            */
  Wr(CANx + CAN_O_IF1CRQ,  msg->ch);

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
  U32 CANx;
  U32 arb1, arb2;

  CAN_get_ctrl(ctrl, &CANx);            /* Get address of CAN controller     */

  /* Read the message contents to interface registers                        */
  Wr(CANx + CAN_O_IF2CMSK, CAN_IFCMSK_ARB       | 
                           CAN_IFCMSK_CONTROL   |
                           CAN_IFCMSK_CLRINTPND |
                           CAN_IFCMSK_DATAA     |
                           CAN_IFCMSK_DATAB);

  /* Request read to interface registers                                     */
  Wr(CANx + CAN_O_IF2CRQ, ch);

  arb1 = Rd(CANx + CAN_O_IF2ARB1);      /* Read ARB1 value                   */
  arb2 = Rd(CANx + CAN_O_IF2ARB2);      /* Read ARB2 value                   */

  /* Read identifier information                                             */
  if (!(arb2 & CAN_IFARB2_XTD)) {                       /* Standard ID       */
    msg->format = STANDARD_FORMAT;
    msg->id     = (arb2 >> 2) & 0x07FF;
  }  else  {                                            /* Extended ID       */
    msg->format = EXTENDED_FORMAT;
    msg->id     =  (((U32)(arb2) << 16) | 0x1FFF)
                  | ((U32)(arb1) & 0xFFFF);
  }

  /* Read type information                                                   */
  if (arb2 & CAN_IFARB2_DIR) {                              /* DATA   FRAME  */
    msg->type = REMOTE_FRAME;
  }  else  {                                                /* REMOTE FRAME  */
    msg->type =   DATA_FRAME;
  }

  msg->ch      = ch;                    /* Reception channel                 */

  /* Read length (number of received bytes)                                  */
  msg->len     = Rd(CANx + CAN_O_IF2MCTL) & 0x0F;

  /* Read data bytes                                                         */
  msg->data[0] = (U8) Rd(CANx + CAN_O_IF2DA1);
  msg->data[1] = (U8)(Rd(CANx + CAN_O_IF2DA1) >> 8);
  msg->data[2] = (U8) Rd(CANx + CAN_O_IF2DA2);
  msg->data[3] = (U8)(Rd(CANx + CAN_O_IF2DA2) >> 8);
  msg->data[4] = (U8) Rd(CANx + CAN_O_IF2DB1);
  msg->data[5] = (U8)(Rd(CANx + CAN_O_IF2DB1) >> 8);
  msg->data[6] = (U8) Rd(CANx + CAN_O_IF2DB2);
  msg->data[7] = (U8)(Rd(CANx + CAN_O_IF2DB2) >> 8);
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
  U32 CANx;

  CAN_get_ctrl(ctrl, &CANx);            /* Get address of CAN controller     */

  /* Set flags for informations that will be changed                         */
  Wr(CANx + CAN_O_IF1CMSK,  CAN_IFCMSK_WRNRD   | 
                            CAN_IFCMSK_ARB     | 
                            CAN_IFCMSK_CONTROL |
                            CAN_IFCMSK_DATAA   |
                            CAN_IFCMSK_DATAB);

  /* Setup the identifier information                                        */
  if (msg->format == STANDARD_FORMAT)  {  /* Standard ID                     */
    Wr(CANx + CAN_O_IF1ARB1, 0);
    Wr(CANx + CAN_O_IF1ARB2, (U16) (msg->id << 2));
  }  else  {                              /* Extended ID                     */
    Wr(CANx + CAN_O_IF1ARB1, (U16) (msg->id));
    Wr(CANx + CAN_O_IF1ARB2, (U16)((msg->id) >> 16) | CAN_IFARB2_XTD);
  }

  /* Setup type information                                                  */
  if (msg->type == DATA_FRAME)  {         /* DATA FRAME                      */
    Wr(CANx + CAN_O_IF1ARB2, Rd(CANx + CAN_O_IF1ARB2) | CAN_IFARB2_DIR);
  }

  /* Setup data bytes                                                        */
  Wr(CANx + CAN_O_IF1DA1, ((U16) msg->data[1]<<8) | msg->data[0]);
  Wr(CANx + CAN_O_IF1DA2, ((U16) msg->data[3]<<8) | msg->data[2]);
  Wr(CANx + CAN_O_IF1DB1, ((U16) msg->data[5]<<8) | msg->data[4]);
  Wr(CANx + CAN_O_IF1DB2, ((U16) msg->data[7]<<8) | msg->data[6]);

  /* Setup length and message control bits                                   */
  Wr(CANx + CAN_O_IF1MCTL, CAN_IFMCTL_NEWDAT | 
                           CAN_IFMCTL_RMTEN  |
                           CAN_IFMCTL_EOB    |
                           (msg->len & 0x000F));

  /* Set the message valid bit                                               */
  Wr(CANx + CAN_O_IF1ARB2, Rd(CANx + CAN_O_IF1ARB2) | CAN_IFARB2_MSGVAL);

  /* Update informations from interface registers                            */
  Wr(CANx + CAN_O_IF1CRQ, msg->ch);

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
  U32 ofs;
  U32 CANx;

  CAN_get_ctrl(ctrl, &CANx);            /* Get address of CAN controller     */

  /* Wait for the first free interface register                              */
  do  {
    if (!(Rd(CANx + CAN_O_IF1CRQ) & CAN_IFCRQ_BUSY))  {
      ofs = 0;
      break;
    }  else if (!(Rd(CANx + CAN_O_IF2CRQ) & CAN_IFCRQ_BUSY))  {
      ofs = (CAN_O_IF2CRQ - CAN_O_IF1CRQ);
      break;
    }
  }  while (1);

  /* Set flags for informations that will be changed                         */
  Wr(CANx + CAN_O_IF1CMSK + ofs,  CAN_IFCMSK_WRNRD   | 
                                  CAN_IFCMSK_MASK    |
                                  CAN_IFCMSK_ARB     | 
                                  CAN_IFCMSK_CONTROL |
                                  CAN_IFCMSK_DATAA   |
                                  CAN_IFCMSK_DATAB);

  /* Setup the identifier information                                        */
  if ((object_para & FORMAT_TYPE) == STANDARD_TYPE)  { /* Standard ID        */
    Wr(CANx + CAN_O_IF1MSK1 + ofs, 0);
    Wr(CANx + CAN_O_IF1MSK2 + ofs, (U16)(0x7FF << 2));

    Wr(CANx + CAN_O_IF1ARB1 + ofs, 0);
    Wr(CANx + CAN_O_IF1ARB2 + ofs, CAN_IFARB2_MSGVAL | (U16)(id << 2));
  }  else  {                              /* Extended ID                     */
    Wr(CANx + CAN_O_IF1MSK1 + ofs, (U16)(0xFFFF));
    Wr(CANx + CAN_O_IF1MSK2 + ofs, CAN_IFMSK2_MXTD | (U16)(0x1FFF));

    Wr(CANx + CAN_O_IF1ARB1 + ofs, (U16)(id));
    Wr(CANx + CAN_O_IF1ARB2 + ofs, CAN_IFARB2_MSGVAL | CAN_IFARB2_XTD | (U16)(id >> 16));
  }

  /* Setup length and message control bits                                   */
  Wr(CANx + CAN_O_IF1MCTL + ofs,  CAN_IFMCTL_UMASK  | 
                                  CAN_IFMCTL_RXIE   |
                                  CAN_IFMCTL_EOB    );

  /* Setup data bytes                                                        */
  Wr(CANx + CAN_O_IF1DA1 + ofs, 0);
  Wr(CANx + CAN_O_IF1DA2 + ofs, 0);
  Wr(CANx + CAN_O_IF1DB1 + ofs, 0);
  Wr(CANx + CAN_O_IF1DB2 + ofs, 0);

  /* Update informations from interface registers                            */
  Wr(CANx + CAN_O_IF1CRQ + ofs, ch);

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

  return CAN_OK;
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
#define CANc                  CAN0_BASE

void CAN0_IRQHandler (void) __irq  {
  U32 ch, status;
  CAN_msg *ptrmsg;

  ch = Rd(CANc + CAN_O_INT);          /* Index of object that caused         */
                                      /* interrupt                           */
  status = Rd(CANc + CAN_O_STS);      /* Read status register                */

  if ((ch-1)<32)  {                   /* Check if index in range 1 - 32      */

    /* Wait until Interface Register 2 becomes free                          */
    while (Rd(CANc + CAN_O_IF2CRQ) & CAN_IFCRQ_BUSY);

    if (status & CAN_STS_TXOK)  {                       /* Tx interrupt      */
      /* Clear Tx Ok flag                                                    */
      Wr(CANc + CAN_O_STS, Rd(CANc + CAN_O_STS) & ~CAN_STS_TXOK);

      Wr(CANc + CAN_O_IF2CMSK, CAN_IFCMSK_CLRINTPND);   /* Release tx object */
      Wr(CANc + CAN_O_IF2CRQ , ch);

      /* Wait until Interface Register 2 becomes free                        */
      while (Rd(CANc + CAN_O_IF2CRQ) & CAN_IFCRQ_BUSY);

        /* If there is a message in the mailbox ready for send, read the 
           message from the mailbox and send it                              */
        if (isr_mbx_receive (MBX_tx_ctrl[CTRL0], (void **)&ptrmsg) != OS_R_OK) {
          /* Wait until Interface Register 1 becomes free                    */
          while (Rd(CANc + CAN_O_IF1CRQ) & CAN_IFCRQ_BUSY);

          CAN_hw_wr (CTRL, ptrmsg);
          _free_box(CAN_mpool, ptrmsg);
      } else {
        isr_sem_send(wr_sem[CTRL0]);  /* Return a token back to semaphore    */
      }
    }  else {                                           /* Rx interrupt      */
                                      /* It should have been checked like
                                         "if (status & CAN_STS_RXOK)"
                                         but isn't because in TEST mode RxOk 
                                         flag in Status Register isn't 
                                         activated upon reception of message */
      /* Clear Rx Ok flag                                                    */
      Wr(CANc + CAN_O_STS, Rd(CANc + CAN_O_STS) & ~CAN_STS_RXOK);

      /* If the mailbox isn't full read the message from the hardware and
         send it to the message queue                                        */
      if (isr_mbx_check (MBX_rx_ctrl[CTRL0]) > 0)  {
        /* Wait until Interface Register 2 becomes free                      */
        while (Rd(CANc + CAN_O_IF2CRQ) & CAN_IFCRQ_BUSY);

        ptrmsg = _alloc_box (CAN_mpool);
        if (ptrmsg) {
          CAN_hw_rd (CTRL, ch, ptrmsg); /* Read received message             */
          isr_mbx_send (MBX_rx_ctrl[CTRL0], ptrmsg);
        }
      }

      Wr(CANc + CAN_O_IF2CMSK, CAN_IFCMSK_CLRINTPND);   /* Release rx object */
      Wr(CANc + CAN_O_IF2CRQ,  ch);
    }
  }
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
#define CANc                  CAN1_BASE

void CAN1_IRQHandler (void) __irq  {
  U32 ch, status;
  CAN_msg *ptrmsg;

  ch = Rd(CANc + CAN_O_INT);          /* Index of object that caused         */
                                      /* interrupt                           */
  status = Rd(CANc + CAN_O_STS);      /* Read status register                */

  if ((ch-1)<32)  {                   /* Check if index in range 1 - 32      */

    /* Wait until Interface Register 2 becomes free                          */
    while (Rd(CANc + CAN_O_IF2CRQ) & CAN_IFCRQ_BUSY);

    if (status & CAN_STS_TXOK)  {                       /* Tx interrupt      */
      /* Clear Tx Ok flag                                                    */
      Wr(CANc + CAN_O_STS, Rd(CANc + CAN_O_STS) & ~CAN_STS_TXOK);

      Wr(CANc + CAN_O_IF2CMSK, CAN_IFCMSK_CLRINTPND);   /* Release tx object */
      Wr(CANc + CAN_O_IF2CRQ , ch);

      /* Wait until Interface Register 2 becomes free                        */
      while (Rd(CANc + CAN_O_IF2CRQ) & CAN_IFCRQ_BUSY);

        /* If there is a message in the mailbox ready for send, read the 
           message from the mailbox and send it                              */
        if (isr_mbx_receive (MBX_tx_ctrl[CTRL0], (void **)&ptrmsg) != OS_R_OK) {
          /* Wait until Interface Register 1 becomes free                    */
          while (Rd(CANc + CAN_O_IF1CRQ) & CAN_IFCRQ_BUSY);

          CAN_hw_wr (CTRL, ptrmsg);
          _free_box(CAN_mpool, ptrmsg);
      } else {
        isr_sem_send(wr_sem[CTRL0]);  /* Return a token back to semaphore    */
      }
    }  else {                                           /* Rx interrupt      */
                                      /* It should have been checked like
                                         "if (status & CAN_STS_RXOK)"
                                         but isn't because in TEST mode RxOk 
                                         flag in Status Register isn't 
                                         activated upon reception of message */
      /* Clear Rx Ok flag                                                    */
      Wr(CANc + CAN_O_STS, Rd(CANc + CAN_O_STS) & ~CAN_STS_RXOK);

      /* If the mailbox isn't full read the message from the hardware and
         send it to the message queue                                        */
      if (isr_mbx_check (MBX_rx_ctrl[CTRL0]) > 0)  {
        /* Wait until Interface Register 2 becomes free                      */
        while (Rd(CANc + CAN_O_IF2CRQ) & CAN_IFCRQ_BUSY);

        ptrmsg = _alloc_box (CAN_mpool);
        if (ptrmsg) {
          CAN_hw_rd (CTRL, ch, ptrmsg); /* Read received message             */
          isr_mbx_send (MBX_rx_ctrl[CTRL0], ptrmsg);
        }
      }

      Wr(CANc + CAN_O_IF2CMSK, CAN_IFCMSK_CLRINTPND);   /* Release rx object */
      Wr(CANc + CAN_O_IF2CRQ,  ch);
    }
  }
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
#define CANc                  CAN2_BASE

void CAN2_IRQHandler (void) __irq  {
  U32 ch, status;
  CAN_msg *ptrmsg;

  ch = Rd(CANc + CAN_O_INT);          /* Index of object that caused         */
                                      /* interrupt                           */
  status = Rd(CANc + CAN_O_STS);      /* Read status register                */

  if ((ch-1)<32)  {                   /* Check if index in range 1 - 32      */

    /* Wait until Interface Register 2 becomes free                          */
    while (Rd(CANc + CAN_O_IF2CRQ) & CAN_IFCRQ_BUSY);

    if (status & CAN_STS_TXOK)  {                       /* Tx interrupt      */
      /* Clear Tx Ok flag                                                    */
      Wr(CANc + CAN_O_STS, Rd(CANc + CAN_O_STS) & ~CAN_STS_TXOK);

      Wr(CANc + CAN_O_IF2CMSK, CAN_IFCMSK_CLRINTPND);   /* Release tx object */
      Wr(CANc + CAN_O_IF2CRQ , ch);

      /* Wait until Interface Register 2 becomes free                        */
      while (Rd(CANc + CAN_O_IF2CRQ) & CAN_IFCRQ_BUSY);

        /* If there is a message in the mailbox ready for send, read the 
           message from the mailbox and send it                              */
        if (isr_mbx_receive (MBX_tx_ctrl[CTRL0], (void **)&ptrmsg) != OS_R_OK) {
          /* Wait until Interface Register 1 becomes free                    */
          while (Rd(CANc + CAN_O_IF1CRQ) & CAN_IFCRQ_BUSY);

          CAN_hw_wr (CTRL, ptrmsg);
          _free_box(CAN_mpool, ptrmsg);
      } else {
        isr_sem_send(wr_sem[CTRL0]);  /* Return a token back to semaphore    */
      }
    }  else {                                           /* Rx interrupt      */
                                      /* It should have been checked like
                                         "if (status & CAN_STS_RXOK)"
                                         but isn't because in TEST mode RxOk 
                                         flag in Status Register isn't 
                                         activated upon reception of message */
      /* Clear Rx Ok flag                                                    */
      Wr(CANc + CAN_O_STS, Rd(CANc + CAN_O_STS) & ~CAN_STS_RXOK);

      /* If the mailbox isn't full read the message from the hardware and
         send it to the message queue                                        */
      if (isr_mbx_check (MBX_rx_ctrl[CTRL0]) > 0)  {
        /* Wait until Interface Register 2 becomes free                      */
        while (Rd(CANc + CAN_O_IF2CRQ) & CAN_IFCRQ_BUSY);

        ptrmsg = _alloc_box (CAN_mpool);
        if (ptrmsg) {
          CAN_hw_rd (CTRL, ch, ptrmsg); /* Read received message             */
          isr_mbx_send (MBX_rx_ctrl[CTRL0], ptrmsg);
        }
      }

      Wr(CANc + CAN_O_IF2CMSK, CAN_IFCMSK_CLRINTPND);   /* Release rx object */
      Wr(CANc + CAN_O_IF2CRQ,  ch);
    }
  }
}

#endif


/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/

