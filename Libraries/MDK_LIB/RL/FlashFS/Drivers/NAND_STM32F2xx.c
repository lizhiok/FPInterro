/*-----------------------------------------------------------------------------
 *      RL-ARM - FlashFS
 *-----------------------------------------------------------------------------
 *      Name:    NAND_STM32F2xx.c
 *      Purpose: NAND Flash Driver for ST STM32F2xx
 *      Rev.:    V4.72
 *-----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include <File_Config.h>
#include <stm32f2xx.h>                  /* STM32F2xxx Definitions             */
#include "NAND_STM32F2xx.h"


/* #define FSMC_BANK1_CS1 */            /* OneNAND Driver Template */
#define FSMC_BANK3                      /* NAND Driver Template    */

/*-----------------------------------------------------------------------------
 *      NAND driver prototypes
 *----------------------------------------------------------------------------*/
static U32 Init         (NAND_DRV_CFG *cfg);
static U32 UnInit       (NAND_DRV_CFG *cfg);
static U32 PageRead     (U32 row, U8 *buf, NAND_DRV_CFG *cfg);
static U32 PageWrite    (U32 row, U8 *buf, NAND_DRV_CFG *cfg);
static U32 BlockErase   (U32 row, NAND_DRV_CFG *cfg);

/*-----------------------------------------------------------------------------
  NAND Device Driver Control Block
 *----------------------------------------------------------------------------*/
const NAND_DRV nand0_drv = {
  Init,
  UnInit,
  PageRead,
  PageWrite,
  BlockErase,
};


/*-----------------------------------------------------------------------------
 *      Initialise Flexible Static Memory Controler (FSMC)
 * Configure: - Data Lines    [D0 ÷ D16]
 *            - Address Lines [A0 ÷ A17]
 *            - Signal Lines NWE/NOE/CLK/NL/NWAIT
 *
 * cs = FSMC Chip Select (NOR/PSRAM, NAND, PC Card chip select)
 *
 *  Return: RTV_NOERR - Init successful
 *----------------------------------------------------------------------------*/
U32 FSMC_Init (U32 cs) {

  /* Enable system configuration controller clock */
  RCC->APB2ENR |= 1 << 14;

  /* Enable FSMC clock */
  RCC->AHB3ENR |= 1;

  /* Enable clocks and configure GPIO */
  RCC->AHB1ENR |= (1 << 6) |        /* GPIO Port G clock enable */   
                  (1 << 5) |        /* GPIO Port F clock enable */ 
                  (1 << 4) |        /* GPIO Port E clock enable */ 
                  (1 << 3) |        /* GPIO Port D clock enable */
                  (1 << 1) |        /* GPIO Port B clock enable */
                  (1 << 0) ;        /* GPIO Port A clock enable */

  /* PD.0, 1, 3, 4, 5, 8, 9, 10, 11, 12, 14, 15 */
  GPIOD->MODER   &= ~0xF3FF0FCF;
  GPIOD->MODER   |=  0xA2AA0A8A;    /* Pins to alternate function    */  
  GPIOD->OTYPER  &= ~0x0000DF3B;    /* Configure as push-pull pins   */
  GPIOD->PUPDR   &= ~0xF3FF0FCF;
  GPIOD->PUPDR   |=  0x51550545;    /* Pull-ups on pins              */
  GPIOD->OSPEEDR |=  0xF3FF0FCF;    /* Pins output speed to 100MHz   */
  GPIOD->AFR[0]  &= ~0x00FFF0FF;
  GPIOD->AFR[0]  |=  0x00CCC0CC;
  GPIOD->AFR[1]  &= ~0xFF0FFFFF;
  GPIOD->AFR[1]  |=  0xCC0CCCCC;    /* Pins assigned to AF12 (FSMC)  */


  /* PE.7, 8, 9, 10, 11, 12, 13, 14, 15 */
  GPIOE->MODER   &= ~0xFFFFC000;
  GPIOE->MODER   |=  0xAAAA8000;    /* Pins to alternate function    */  
  GPIOE->OTYPER  &= ~0x0000FF80;    /* Configure as push-pull pins   */
  GPIOE->PUPDR   &= ~0xFFFFC000;
  GPIOE->PUPDR   |=  0x55554000;    /* Pull-ups on pins              */
  GPIOE->OSPEEDR |=  0xFFFFC000;    /* Pins output speed to 100MHz   */
  GPIOE->AFR[0]  &= ~0xF0000000;
  GPIOE->AFR[0]  |=  0xC0000000;
  GPIOE->AFR[1]   =  0xCCCCCCCC;    /* Pins assigned to AF12 (FSMC)  */


  /* PF.0, 1, 2, 3, 4, 5, 12, 13, 14, 15 */
  GPIOF->MODER   &= ~0xFF000FFF;
  GPIOF->MODER   |=  0xAA000AAA;    /* Pins to alternate function    */  
  GPIOF->OTYPER  &= ~0x0000F03F;    /* Configure as push-pull pins   */
  GPIOF->PUPDR   &= ~0xFF000FFF;
  GPIOF->PUPDR   |=  0x55000555;    /* Pull-ups on pins              */
  GPIOF->OSPEEDR |=  0xFF000FFF;    /* Pins output speed to 100MHz   */
  GPIOF->AFR[0]  &= ~0x00FFFFFF;
  GPIOF->AFR[0]  |=  0x00CCCCCC;
  GPIOF->AFR[1]  &= ~0xFFFF0000;
  GPIOF->AFR[1]  |=  0xCCCC0000;    /* Pins assigned to AF12 (FSMC)  */


  /* PG.0, 1, 2, 3, 4, 5 */
  GPIOG->MODER   &= ~0x00000FFF;
  GPIOG->MODER   |=  0x00000AAA;    /* Pins to alternate function    */  
  GPIOG->OTYPER  &= ~0x0000003F;    /* Configure as push-pull pins   */
  GPIOG->PUPDR   &= ~0x00000FFF;
  GPIOG->PUPDR   |=  0x00000555;    /* Pull-ups on pins              */
  GPIOG->OSPEEDR |=  0x00000FFF;    /* Pins output speed to 100MHz   */
  GPIOG->AFR[0]  &= ~0x00FFFFFF;
  GPIOG->AFR[0]  |=  0x00CCCCCC;    /* Pins assigned to AF12 (FSMC)  */

  
  switch (cs) {
    #ifdef FSMC_BANK1_CS1
    case FSMC_NE1: /* OneNAND */
      /* PD.7 = Chip Select (NE1), PD.6 = nWAIT */
      GPIOD->MODER   &= ~0x0000F000;
      GPIOD->MODER   |=  0x0000A000;    /* Pins to alternate function    */
      GPIOD->OTYPER  &= ~0x000000C0;    /* Configure as push-pull pins   */
      GPIOD->OSPEEDR |=  0x0000F000;    /* Pins output speed to 100MHz   */
      GPIOD->AFR[0]  &= ~0xFF000000;
      GPIOD->AFR[0]  |=  0xCC000000;    /* Pins assigned to AF12 (FSMC)  */
      
      /* PB.7 = NL (nAVD), PB.15 = OneNAND_INT */
      GPIOB->MODER   &= ~0xC000C000;
      GPIOB->MODER   |=  0x00008000;    /* PB.7 to alternate, PB.15 input */
      GPIOB->OTYPER  &= ~0x00000080;    /* PB.7 is push-pull              */
      GPIOB->PUPDR   &=  0xC000C000;
      GPIOB->PUPDR   |=  0x00004000;    /* PB.7 has pull-up               */
      GPIOB->OSPEEDR |=  0xC000C000;    /* Pins output speed to 100MHz    */
      GPIOB->AFR[0]  &= ~(0xFU << 28);
      GPIOB->AFR[0]  |=  (0xCU << 28);  /* PB.7 to AF12 (FSMC)            */
      GPIOB->AFR[1]  &= ~(0xFU << 28);  /* PB.15 to GPIO                  */
    
      /* CS 1 -> BCR 1 (Mode C Access - OneNAND) */
      FSMC_Bank1->BTCR[0] = (1 << 15) |    /* Async Wait                   */
                            (1 << 14) |    /* Extended mode enable         */
                            (1 << 12) |    /* Bank Write enable            */
                            (0 <<  9) |    /* Wait signal polarity (High)  */
                            (1 <<  6) |    /* Flash (NOR) access enable    */
                            (1 <<  4) |    /* Memory databus width         */
                            (2 <<  2) |    /* Memory type (NOR/OneNAND)    */
                            (1 <<  0) ;    /* Memory bank enable bit       */
      /* CS 1 -> BTR 1 */
      FSMC_Bank1->BTCR[1] = (0x02 << 28) | /* Access mode (C)              */
                            (0x00 << 24) | /* Data latency                 */
                            (0x00 << 20) | /* Clock divide ratio (CLK Sig) */
                            (0x0F << 16) | /* Bus turnaround phase duration*/
                            (0x0F <<  8) | /* Data phase duration          */
                            (0x0F <<  4) | /* Address hold  phase duration */ 
                            (0x01 <<  0) ; /* Address setup phase duration */
    
      /* CS 1 -> BWTR */
      FSMC_Bank1E->BWTR[0] =(0x02 << 28)| /* Access mode (C)              */
                            (0x00 << 24)| /* Data latency                 */
                            (0x00 << 20)| /* Clock divide ratio (CLK Sig) */
                            (0x0F << 16)| /* Bus turnaround phase duration*/
                            (0x0F << 8) | /* Data phase duration          */
                            (0x0F << 4) | /* Address hold  phase duration */ 
                            (0x01 << 0) ; /* Address setup phase duration */
      break;
    #endif
    /* case FSMC_NE2: */
    /* case FSMC_NE3: */
    /* case FSMC_NE4: */

    /* BANK 2 */
    /* case FSMC_NCE2: */

    /* BANK 3 */
    #ifdef FSMC_BANK3
    case FSMC_NCE3: /* NAND */
      /* PG9 = Chip Enable (NCE3) */
      GPIOG->MODER   &= ~GPIO_MODER_MODER9;
      GPIOG->MODER   |=  GPIO_MODER_MODER9_1; /* Pin to alternate function    */
      GPIOG->OTYPER  &= ~GPIO_OTYPER_OT_9;    /* Configure as push-pull pin   */
      GPIOG->OSPEEDR |=  GPIO_OSPEEDER_OSPEEDR9_0 |
                         GPIO_OSPEEDER_OSPEEDR9_1 ;  /* Pin speed is 100MHz   */
      GPIOG->AFR[1]  &= ~0x000000F0;
      GPIOG->AFR[1]  |=  0x000000C0;          /* Pin assigned to AF12 (FSMC)  */

      /* PD6 = Ready/Busy Input */
      GPIOD->MODER   &= ~GPIO_MODER_MODER6;
      GPIOD->MODER   |=  GPIO_MODER_MODER6_1; /* Pin to alternate function    */
      GPIOD->OTYPER  &= ~GPIO_OTYPER_OT_6;    /* Configure as push-pull pin   */
      GPIOD->OSPEEDR |=  GPIO_OSPEEDER_OSPEEDR6_0 |
                         GPIO_OSPEEDER_OSPEEDR6_1 ;  /* Pin speed is 100MHz   */
      GPIOG->AFR[0]  &= ~0x0F000000;
      GPIOG->AFR[0]  |=  0x0C000000;          /* Pin assigned to AF12 (FSMC)  */

      /* Configure ECC to 512 bytes */
      FSMC_Bank3->PCR3 = FSMC_PCR3_ECCPS_1 | /* ECC page size 512 bytes        */
                         FSMC_PCR3_TAR_0   | /* ALE to RE delay: x HCLK cycles */
                         FSMC_PCR3_TCLR_0  | /* CLE to RE delay: x HCLK cycles */
                         FSMC_PCR3_ECCEN   | /* ECC logic is enabled           */
                         FSMC_PCR3_PTYP    | /* Memory type is NAND Flash      */
                         FSMC_PCR3_PBKEN   ; /* Memory bank is enabled         */
      
      /* Configure common memory space timing */
      FSMC_Bank3->PMEM3 = NAND_CO_HIZ_TIME   |    /* Set HiZ time             */
                          NAND_CO_HOLD_TIME  |    /* Set hold time            */
                          NAND_CO_WAIT_TIME  |    /* Set wait time            */
                          NAND_CO_SETUP_TIME ;    /* Set setup time           */

      /* Configure attribute memory space timing */
      FSMC_Bank3->PATT3 = NAND_AT_HIZ_TIME   |    /* Set HiZ time             */
                          NAND_AT_HOLD_TIME  |    /* Set hold time            */
                          NAND_AT_WAIT_TIME  |    /* Set wait time            */
                          NAND_AT_SETUP_TIME ;    /* Set setup time           */
      break;
      #endif      

    /* BANK 4 */
    /* case FSMC_NCE4_1: */
    /* case FSMC_NCE4_2: */    
    default: return ERR_INVALID_PARAM;
  }
  return RTV_NOERR;
}


/*-----------------------------------------------------------------------------
 *      Flexible Static Memory Controler (FSMC) uninitialization
 *
 * cs = FSMC Chip Select (NOR/PSRAM, NAND, PC Card chip select)
 *
 *  Return: RTV_NOERR - UnInit successful
 *----------------------------------------------------------------------------*/
U32 FSMC_UnInit (U32 cs) {
  switch (cs) {
    #ifdef FSMC_BANK1_CS1
    case FSMC_NE1:
      /* Uninit PB15 and PD7 */
      GPIOB->MODER   &= ~0xC0000000;
      GPIOB->OTYPER  &= ~0x00008000;
      GPIOB->OSPEEDR &= ~0xC0000000;
      GPIOB->PUPDR   &= ~0xC0000000;
      GPIOB->AFR[1]  &= ~0xF0000000;

      GPIOD->MODER   &= ~0x0000C000;
      GPIOD->OTYPER  &= ~0x00000080;
      GPIOD->OSPEEDR &= ~0x0000C000;
      GPIOD->PUPDR   &= ~0x0000C000;
      GPIOD->AFR[0]  &= ~0xF0000000;
      
      /* FSMC Registers to reset values */
      FSMC_Bank1->BTCR[0]  = 0x0000305A;
      FSMC_Bank1->BTCR[1]  = 0x0FFFFFFF;
      FSMC_Bank1E->BWTR[0] = 0x0FFFFFFF; 
      break;
    #endif
    /* case FSMC_NE2: */
    /* case FSMC_NE3: */
    /* case FSMC_NE4: */

    /* BANK 2 */
    /* case FSMC_NCE2: */

    /* BANK 3 */
    #ifdef FSMC_BANK3
    case FSMC_NCE3:
      /* Uninit PG9 and PD6 */
      GPIOG->MODER   &= ~GPIO_MODER_MODER9;
      GPIOG->OTYPER  &= ~GPIO_OTYPER_OT_9;
      GPIOG->OSPEEDR &= ~GPIO_OSPEEDER_OSPEEDR6;
      GPIOG->PUPDR   &= ~GPIO_PUPDR_PUPDR9;
      GPIOG->AFR[1]  &= ~0x000000F0;

      GPIOD->MODER   &= ~GPIO_MODER_MODER6;
      GPIOD->OTYPER  &= ~GPIO_OTYPER_OT_6;
      GPIOD->OSPEEDR &= ~GPIO_OSPEEDER_OSPEEDR6;
      GPIOD->PUPDR   &= ~GPIO_PUPDR_PUPDR6;
      GPIOD->AFR[0]  &= ~0x0F000000;

      /* FSMC Registers to reset values */
      FSMC_Bank3->PCR3  = 0x00000018;
      FSMC_Bank3->PMEM3 = 0xFCFCFCFC;
      FSMC_Bank3->PATT3 = 0xFCFCFCFC;
      break;
    #endif

    /* BANK 4 */
    /* case FSMC_NCE4_1: */
    /* case FSMC_NCE4_2: */
    default: return ERR_INVALID_PARAM;
  }
  return RTV_NOERR;
}


#ifdef FSMC_BANK1_CS1
/*-----------------------------------------------------------------------------
 *      Wait until OneNAND is busy (INT pin is low)
 *
 *  Return: __TRUE            - OneNAND Ready before timeout
 *          __FALSE           - OneNAND Busy, timeout expired
 *----------------------------------------------------------------------------*/
__inline static BOOL WaitINT (void) {
  U32 i;

  for (i = TIMEOUT_OneNAND; i; i--) {
#if 1 /* Pool on INT bit (Interrupt Status Register) */
    if (FSMC_AHB_W16 (FSMC_BANK1_BASE, ON_REG_INT) & (1 << 15)) {
      return __TRUE;
    }
#else /* Pool on INT pin (PB.15)  */
    if (GPIOB->IDR & (1 << 15)) {
      return __TRUE;
    }
#endif
  }
  return __FALSE;
}

/*-----------------------------------------------------------------------------
 *      Unlock Block in OneNAND Flash Array
 *
 * pbn = physical block number
 *
 *  Return: __TRUE            - OneNAND block unlocked before timeout
 *          __FALSE           - OneNAND Busy, timeout expired
 *----------------------------------------------------------------------------*/
static BOOL OneNAND_Unlock (U32 pbn) {
  /* Set block address */
  FSMC_AHB_W16 (FSMC_BANK1_BASE, ON_REG_SBA) = (U16)pbn;
  
  /* Clear interrupt flags */
  FSMC_AHB_W16 (FSMC_BANK1_BASE, ON_REG_INT) = 0x0000;

  /* Write unlock command */
  FSMC_AHB_W16 (FSMC_BANK1_BASE, ON_REG_CMD) = ON_CMD_UNLOCK;
  
  /* Wait until operation complete */
  if (WaitINT() == __TRUE) {
    if (FSMC_AHB_W16 (FSMC_BANK1_BASE, ON_REG_WPS) == 0x0004) {
      return __TRUE;
    }
  }
  return __FALSE;
}


/*-----------------------------------------------------------------------------
 *      OneNAND initialization
 * Required page layout:
 * | Sect 0 | Sect 1 | Sect 2 | Sect 3 | Spare 0 | Spare 1 | Spare 2 | Spare 3 |
 *
 *  *cfg = Pointer to configuration structure
 *
 *  Return: RTV_NOERR        - Init successful
 *          ERR_NAND_HW_TOUT - Reset failed / NAND Unlock failed 
 *----------------------------------------------------------------------------*/
static U32 FSMC_OneNAND_Init (NAND_DRV_CFG *cfg) {
  U32 pbn;

  /* Setup OneNAND Page Layout */
  cfg->PgLay->Pos_LSN  = ON_POS_LSN;
  cfg->PgLay->Pos_COR  = ON_POS_COR;
  cfg->PgLay->Pos_BBM  = ON_POS_BBM;
  cfg->PgLay->Pos_ECC  = ON_POS_ECC;
  cfg->PgLay->SectInc  = ON_SECT_INC;
  cfg->PgLay->SpareOfs = ON_SPARE_OFS;
  cfg->PgLay->SpareInc = ON_SPARE_INC;
  
  /* Clear interrupts */
  FSMC_AHB_W16 (FSMC_BANK1_BASE, ON_REG_INT) = 0x0000;

  /* Flash Hot Reset */
  FSMC_AHB_W16 (FSMC_BANK1_BASE, ON_REG_CMD) = ON_CMD_RST_HOT;

  if (WaitINT() == __FALSE) {
    return ERR_NAND_HW_TOUT;
  }

  /* Set configuration */
  FSMC_AHB_W16 (FSMC_BANK1_BASE, ON_REG_CFG_1) = (0 << 15) | /* Async mode */
                                                 (4 << 12) | /* Latency    */
                                                 (0 <<  9) | /* Burst Len  */
                                                 (0 <<  8) | /* ECC On     */
                                                 (1 <<  7) | /* Ready high */
                                                 (1 <<  6) | /* INT high   */
                                                 (1 <<  5) | /* IOB Enable */
                                                 (0 <<  4) ; /* Rdy 1clock */
  /* Check if controller ready */
  if (FSMC_AHB_W16 (FSMC_BANK1_BASE, ON_REG_STAT) != 0) {
    return ERR_NAND_HW_TOUT;
  }

  /* UnLock NAND Flash Array */
  for (pbn = 0; pbn < cfg->NumBlocks; pbn++) {
    if (OneNAND_Unlock (pbn) == __FALSE) {
      /* Block unlock failed */
      return ERR_NAND_HW_TOUT;
    }
  }
  return RTV_NOERR;
}


/*-----------------------------------------------------------------------------
 *      Read data from OneNAND
 *  row  = Page address
 *  *buf = Pointer to data buffer
 *  *cfg = Pointer to configuration structure
 *
 *  Return: RTV_NOERR         - Page read successful
 *          ERR_NAND_HW_TOUT  - Hardware transfer timeout
 *          ERR_ECC_COR       - ECC corrected the data within page
 *          ERR_ECC_UNCOR     - ECC was not able to correct the data
 *----------------------------------------------------------------------------*/
static U32 FSMC_OneNAND_Read (U32 row, U8 *buf, NAND_DRV_CFG *cfg) {
  U32 i, cnt, ba, dat, ecc;
  
  /* Set block and page address */
  FSMC_AHB_W16 (FSMC_BANK1_BASE, ON_REG_SA_1) = (row / cfg->NumPages);
  FSMC_AHB_W16 (FSMC_BANK1_BASE, ON_REG_SA_8) = (row % cfg->NumPages)<<2;
  
  /* Set start buffer register: DataRAM0, 4 sectors */
  FSMC_AHB_W16 (FSMC_BANK1_BASE, ON_REG_ST_BUF) = (U16)(8 << 8);

  /* Clear interrupt flags */
  FSMC_AHB_W16 (FSMC_BANK1_BASE, ON_REG_INT) = 0x0000;

  /* Send load command */
  FSMC_AHB_W16 (FSMC_BANK1_BASE, ON_REG_CMD) = ON_CMD_LDM_BUF;
  
  /* Wait until operation complete */
  if (WaitINT() == __FALSE) {
    return ERR_NAND_HW_TOUT;
  }

  /* Read user data from DATARAM0 buffer */
  cnt = (cfg->PageSize/512) * 512;
  for (ba = ON_DBUF_ADDR, i = 0; i < cnt; i += 2, ba += 2) {
    dat = FSMC_AHB_W16 (FSMC_BANK1_BASE, ba);
    buf[i]   = (U8)dat;
    buf[i+1] = (U8)(dat >> 8);
  }
  
  /* Read spare area data from DataRAM0 buffer */
  cnt += (cfg->PageSize/512) * 16;
  for (ba = ON_SBUF_ADDR; i < cnt; i += 2, ba += 2) {
    dat = FSMC_AHB_W16 (FSMC_BANK1_BASE, ba);
    buf[i]   = (U8)dat;
    buf[i+1] = (U8)(dat >> 8);
  }

  /* Read ECC status */
  ecc = FSMC_AHB_W16 (FSMC_BANK1_BASE, ON_REG_ECC_ST);
  if (ecc & 0xAAAA) {
    return ERR_ECC_UNCOR;
  }
  if (ecc & 0x5555) {
    return ERR_ECC_COR;
  }
  
  return RTV_NOERR;
}


/*-----------------------------------------------------------------------------
 *      Write data to OneNAND
 *  row  = Page address
 *  *buf = Pointer to data to write
 *  *cfg = Pointer to configuration structure
 *
 *  Return: RTV_NOERR         - Buffer written successfuly
 *          ERR_NAND_PROG     - Programing failed
 *          ERR_NAND_HW_TOUT  - Hardware request timeout
 *----------------------------------------------------------------------------*/
static U32 FSMC_OneNAND_Write (U32 row, U8 *buf, NAND_DRV_CFG *cfg) {
  U16 dat;
  U32 i, cnt, ba;

  /* Copy user data to DataRAM0 buffer */
  ba  = ON_DBUF_ADDR;
  cnt = cfg->SectorsPerPage * 512;
  for (i = 0; i < cnt; i += 2) {
    dat = (U16)(buf[i+1] << 8) | (U16)buf[i];
    FSMC_AHB_W16 (FSMC_BANK1_BASE, ba) = dat;
    ba += 2;
  }

  /* Copy spare area data to DataRAM0 buffer */
  ba = ON_SBUF_ADDR;
  cnt += cfg->SectorsPerPage * 16;
  for (; i < cnt; i += 2) {
    dat = (U16)(buf[i+1] << 8) | (U16)buf[i];
    FSMC_AHB_W16 (FSMC_BANK1_BASE, ba) = dat;
    ba += 2;
  }

  /* Set block and page address */
  FSMC_AHB_W16 (FSMC_BANK1_BASE, ON_REG_SA_1) = (row / cfg->NumPages);
  FSMC_AHB_W16 (FSMC_BANK1_BASE, ON_REG_SA_8) = (row % cfg->NumPages)<<2;
  
  /* Set start buffer register: DataRAM0, 4 sectors */
  FSMC_AHB_W16 (FSMC_BANK1_BASE, ON_REG_ST_BUF) = (U16)(8 << 8);

  /* Clear interrupt flags */
  FSMC_AHB_W16 (FSMC_BANK1_BASE, ON_REG_INT) = 0x0000;

  /* Send program command */
  FSMC_AHB_W16 (FSMC_BANK1_BASE, ON_REG_CMD) = ON_CMD_PRG_BUF;

  /* Wait until operation complete */
  if (WaitINT() == __FALSE) {
    return ERR_NAND_HW_TOUT;
  }

  /* Read status */
  if (FSMC_AHB_W16 (FSMC_BANK1_BASE, ON_REG_STAT) & ON_FL_PROG) {
    /* Programing failed */
    return ERR_NAND_PROG;
  }

  return RTV_NOERR;
}


/*-----------------------------------------------------------------------------
 *      Erase block of memory on NOR/SRAM
 *  row  = Block address
 *  *cfg = Pointer to configuration structure
 *
 *  Return: RTV_NOERR        - Erase operation successful
 *          ERR_NAND_ERASE   - Erase operation failed
 *          ERR_NAND_HW_TOUT - Hardware request timeout
 *----------------------------------------------------------------------------*/
static U32 FSMC_OneNAND_Erase (U32 row, NAND_DRV_CFG *cfg) {
  U32 fba;

  /* Write block address */
  fba = row / cfg->NumPages;
  FSMC_AHB_W16 (FSMC_BANK1_BASE, ON_REG_SA_1) = (U16)fba;
    
  /* Write 0 to interrupt register */
  FSMC_AHB_W16 (FSMC_BANK1_BASE, ON_REG_INT) = (U16)0x0000;

  /* Write erase command */
  FSMC_AHB_W16 (FSMC_BANK1_BASE, ON_REG_CMD) = ON_CMD_ERASE;

  /* Wait until operation complete */
  if (WaitINT() == __FALSE) {
    return ERR_NAND_HW_TOUT;
  }

  /* Read status */
  if (FSMC_AHB_W16 (FSMC_BANK1_BASE, ON_REG_STAT) & ON_FL_ERASE) {
    /* Erase failed */
    return ERR_NAND_ERASE;
  }
  
  return RTV_NOERR;
}
#endif


#ifdef FSMC_BANK3
/*-----------------------------------------------------------------------------
        Write command
 *----------------------------------------------------------------------------*/
__inline static void NAND_WrCmd (U32 cmd) {
  NAND_CMD_W8 = (U8)cmd;
}


/*-----------------------------------------------------------------------------
        Set page address
 *----------------------------------------------------------------------------*/
static void NAND_SetPgAddr (U32 numCyc, U32 row, U32 pgSz) {  
  U32 i;

  NAND_ADR_W8 = 0;
  numCyc--;
  if (pgSz > 528) {    
    NAND_ADR_W8 = 0;
    numCyc--;
  }

  i = 0;
  while (numCyc--) {
    NAND_ADR_W8 = (U8)(row >> i);
    i += 8;
  }
}


/*-----------------------------------------------------------------------------
        Set block address
 *----------------------------------------------------------------------------*/
__inline void NAND_SetBlAddr (U32 numCyc, U32 row, U32 pgSz) {
  U32 i;

  numCyc = (pgSz > 528) ? (numCyc - 2) : (numCyc - 1);

  i = 0;
  while (numCyc--) {
    NAND_ADR_W8 = (U8)(row >> i);
    i += 8;
  }
}


/*-----------------------------------------------------------------------------
 *      Wait until NAND is busy (Ready bit in status register is 0)
 *
 *  Return: __TRUE            - NAND Ready before timeout
 *          __FALSE           - NAND Busy, timeout expired
 *----------------------------------------------------------------------------*/
__inline static BOOL NAND_WaitReady (void) {
  U32 i;

  for (i = NAND_TIMEOUT; i; i--) {
    /* Read Status Register */
    NAND_CMD_W8 = NAND_CMD_STATUS;
    if (NAND_DATA_W8 & NAND_STAT_RDY) {
      return (__TRUE);
    }
  }
  return (__FALSE);
}


/*-----------------------------------------------------------------------------
        Check status register if specified flag is set

  flag = NAND status flag

  Return: NAND_FLAG_SET  - NAND flag is set
          NAND_FLAG_CLR  - NAND flag is cleared
 *----------------------------------------------------------------------------*/
__inline static U32 NAND_ChkStatus (U32 flag) {

  NAND_CMD_W8 = NAND_CMD_STATUS;  
  return (NAND_DATA_W8 & flag) ? NAND_FLAG_SET : NAND_FLAG_CLR; 
}


/*-----------------------------------------------------------------------------
 *      NAND initialization
 * Default page layout is used.
 *
 *  *cfg = Pointer to configuration structure
 *
 *  Return: RTV_NOERR        - Init successful
 *          ERR_NAND_HW_TOUT - Reset failed / NAND Unlock failed 
 *----------------------------------------------------------------------------*/
static U32 FSMC_NAND_Init (NAND_DRV_CFG *cfg) {  
  
  /* Reset NAND chip */
  NAND_WrCmd (NAND_CMD_RESET);

  /* Wait until ready */
  if (NAND_WaitReady() == __FALSE) {
    /* Reset failed */
    return ERR_NAND_HW_TOUT;
  }
  
  /* NAND is ready */
  return RTV_NOERR;
}


/*-----------------------------------------------------------------------------
 *      Read data from NAND
 *  row  = Page address
 *  *buf = Pointer to data buffer
 *  *cfg = Pointer to configuration structure
 *
 *  Return: RTV_NOERR         - Page read successful
 *          ERR_NAND_HW_TOUT  - Hardware transfer timeout
 *          ERR_ECC_COR       - ECC corrected the data within page
 *          ERR_ECC_UNCOR     - ECC was not able to correct the data
 *----------------------------------------------------------------------------*/
static U32 FSMC_NAND_Read (U32 row, U8 *buf, NAND_DRV_CFG *cfg) {
  U32 i, sz;

  /* Write command 1 */
  NAND_WrCmd (NAND_CMD_READ1ST);

  /* Set address */
  NAND_SetPgAddr (cfg->AddrCycles, row, cfg->PageSize);

  /* Write command 2 */
  NAND_WrCmd (NAND_CMD_READ2ND);

  /* Wait until NAND ready */
  if (NAND_WaitReady() == __FALSE) {
    return ERR_NAND_HW_TOUT;
  }

  /* Switch back to read: Write command 1 */
  NAND_WrCmd (NAND_CMD_READ1ST);

  /* Read page from NAND chip */
  sz = cfg->PageSize;
  for (i = 0; i < sz; i += 8) {
    buf[i]   = NAND_DATA_W8;
    buf[i+1] = NAND_DATA_W8;
    buf[i+2] = NAND_DATA_W8;
    buf[i+3] = NAND_DATA_W8;
    buf[i+4] = NAND_DATA_W8;
    buf[i+5] = NAND_DATA_W8;
    buf[i+6] = NAND_DATA_W8;
    buf[i+7] = NAND_DATA_W8;
  }

  return RTV_NOERR;
}


/*-----------------------------------------------------------------------------
 *      Write data to NAND
 *  row  = Page address
 *  *buf = Pointer to data to write
 *  *cfg = Pointer to configuration structure
 *
 *  Return: RTV_NOERR         - Buffer written successfuly
 *          ERR_NAND_PROG     - Programing failed
 *          ERR_NAND_HW_TOUT  - Hardware request timeout
 *----------------------------------------------------------------------------*/
static U32 FSMC_NAND_Write (U32 row, U8 *buf, NAND_DRV_CFG *cfg) {
  U32 i, sz;

  /* Write command 1 */
  NAND_WrCmd (NAND_CMD_PROG1ST);

  /* Set address */
  NAND_SetPgAddr (cfg->AddrCycles, row, cfg->PageSize);
  
  /* Write data to NAND chip */
  sz = cfg->PageSize;
  for (i = 0; i < sz; i += 8) {
    NAND_DATA_W8 = buf[i];
    NAND_DATA_W8 = buf[i+1];
    NAND_DATA_W8 = buf[i+2];
    NAND_DATA_W8 = buf[i+3];
    NAND_DATA_W8 = buf[i+4];
    NAND_DATA_W8 = buf[i+5];
    NAND_DATA_W8 = buf[i+6];
    NAND_DATA_W8 = buf[i+7];
  }

  /* Write command 2 */
  NAND_WrCmd (NAND_CMD_PROG2ND);

  /* Wait until NAND ready */
  if (NAND_WaitReady() == __FALSE) {
    return ERR_NAND_HW_TOUT;
  }

  /* Check status */
  switch (NAND_ChkStatus (NAND_STAT_FAIL)) {
    case NAND_FLAG_TOUT:
      return ERR_NAND_HW_TOUT;
    case NAND_FLAG_SET:
      return ERR_NAND_PROG;
    default:
      return RTV_NOERR;
  }
}

/*-----------------------------------------------------------------------------
 *      Erase block of memory on NAND
 *  row  = Block address
 *  *cfg = Pointer to configuration structure
 *
 *  Return: RTV_NOERR        - Erase operation successful
 *          ERR_NAND_ERASE   - Erase operation failed
 *          ERR_NAND_HW_TOUT - Hardware request timeout
 *----------------------------------------------------------------------------*/
static U32 FSMC_NAND_Erase (U32 row, NAND_DRV_CFG *cfg) {
  /* Write command 1 */
  NAND_WrCmd (NAND_CMD_ERASE1ST);
  
  /* Set address */
  NAND_SetBlAddr (cfg->AddrCycles, row, cfg->PageSize);
  
  /* Write command 2 */
  NAND_WrCmd (NAND_CMD_ERASE2ND);

  /* Wait until NAND ready */
  if (NAND_WaitReady() == __FALSE) {
    return ERR_NAND_HW_TOUT;
  }
  
  /* Check status */
  switch (NAND_ChkStatus (NAND_STAT_FAIL)) {
    case NAND_FLAG_TOUT:
      return ERR_NAND_HW_TOUT;
    case NAND_FLAG_SET:
      return ERR_NAND_ERASE;
    default:
      return RTV_NOERR;
  }
}
#endif

/*-----------------------------------------------------------------------------
 *      Select Chip Select
 *  DrvInst = Instance Number
 *----------------------------------------------------------------------------*/
static U32 SetCs (U32 DrvInst) {
  if (DrvInst == 0) {
    #ifdef FSMC_BANK1_CS1
    return FSMC_NE1;                                   /* Select OneNand      */
    #endif
    
    #ifdef FSMC_BANK3
    return FSMC_NCE3;                                  /* Select NAND Flash   */
    #endif  
  }
  else {
    return FSMC_CS_INV;                                /* Invalid Chip Select */ 
  }
}


/*-----------------------------------------------------------------------------
 *      Initialise NAND flash driver
 *  *cfg = Pointer to configuration structure
 *
 *  Return: RTV_NOERR         - NAND Flash Initialisation successful
 *          ERR_NAND_HW_TOUT  - NAND Flash Reset Command failed
 *          ERR_INVALID_PARAM - Invalid parameter
 *----------------------------------------------------------------------------*/
static U32 Init (NAND_DRV_CFG *cfg) {
  U32 cs;

  /* Determine Chip Select from instance number */
  cs = SetCs (cfg->DrvInst);

  switch (cs) {
    /* BANK 1 */
    #ifdef FSMC_BANK1_CS1
    case FSMC_NE1:
      FSMC_Init(FSMC_NE1);                              /* Init FSMC Bank 1_1 */
      return FSMC_OneNAND_Init (cfg);                   /* Init OneNAND       */
    #endif
    /* case FSMC_NE2: */
    /* case FSMC_NE3: */
    /* case FSMC_NE4: */

    /* BANK 2 */
    /* case FSMC_NCE2: */

    /* BANK 3 */
    #ifdef FSMC_BANK3
    case FSMC_NCE3:                                     
      FSMC_Init (FSMC_NCE3);                            /* Init FSMC Bank 3   */
      return FSMC_NAND_Init (cfg);                      /* Init NAND Flash    */   
    #endif

    /* BANK 4 */
    /* case FSMC_NCE4_1: */
    /* case FSMC_NCE4_2: */
    default: return ERR_INVALID_PARAM;
  }
}

/*-----------------------------------------------------------------------------
 *      Uninitialise NAND flash driver
 *  *cfg = Pointer to configuration structure
 *
 *  Return: RTV_NOERR         - UnInit successful
 *----------------------------------------------------------------------------*/
static U32 UnInit (NAND_DRV_CFG *cfg) {
  U32 cs;

  /* Determine Chip Select from instance number */
  cs = SetCs (cfg->DrvInst);

  switch (cs) {
    /* BANK 1 */
    #ifdef FSMC_BANK1_CS1
    case FSMC_NE1:  return FSMC_UnInit (FSMC_NE1);
    #endif
    /* case FSMC_NE2: */
    /* case FSMC_NE3: */
    /* case FSMC_NE4: */

    /* BANK 2 */
    /* case FSMC_NCE2: */

    /* BANK 3 */
    #ifdef FSMC_BANK3
    case FSMC_NCE3: return FSMC_UnInit (FSMC_NCE3);
    #endif

    /* BANK 4 */
    /* case FSMC_NCE4_1: */
    /* case FSMC_NCE4_2: */
    default:        return ERR_INVALID_PARAM;
  }
}

/*-----------------------------------------------------------------------------
 *      Read page
 *  row  = Page address
 *  *buf = Pointer to data buffer
 *  *cfg = Pointer to configuration structure
 *
 *  Return: RTV_NOERR         - Page read successful
 *          ERR_NAND_HW_TOUT  - Hardware transfer timeout
 *          ERR_ECC_COR       - ECC corrected the data within page
 *          ERR_ECC_UNCOR     - ECC was not able to correct the data
 *----------------------------------------------------------------------------*/
static U32 PageRead (U32 row, U8 *buf, NAND_DRV_CFG *cfg) {
  U32 cs;

  /* Determine Chip Select from instance number */
  cs = SetCs (cfg->DrvInst);

  switch (cs) {
    /* BANK 1 */
    #ifdef FSMC_BANK1_CS1
    case FSMC_NE1:  return FSMC_OneNAND_Read (row, buf, cfg);
    #endif
    /* case FSMC_NE2: */
    /* case FSMC_NE3: */
    /* case FSMC_NE4: */

    /* BANK 2 */
    /* case FSMC_NCE2: */

    /* BANK 3 */
    #ifdef FSMC_BANK3
    case FSMC_NCE3: return FSMC_NAND_Read (row, buf, cfg);
    #endif

    /* BANK 4 */
    /* case FSMC_NCE4_1: */
    /* case FSMC_NCE4_2: */
    default: return ERR_INVALID_PARAM;
  }
}

/*-----------------------------------------------------------------------------
 *      Write page
 *  row  = Page address
 *  *buf = Pointer to data buffer
 *  *cfg = Pointer to configuration structure
 *
 *  Return: RTV_NOERR         - Page write successful
 *          ERR_NAND_PROG     - Page write failed
 *          ERR_NAND_HW_TOUT  - Hardware transfer timeout
 *----------------------------------------------------------------------------*/
static U32 PageWrite (U32 row, U8 *buf, NAND_DRV_CFG *cfg) {
  U32 cs;

  /* Determine Chip Select from instance number */
  cs = SetCs (cfg->DrvInst);

  switch (cs) {
    /* BANK 1 */
    #ifdef FSMC_BANK1_CS1
    case FSMC_NE1:  return FSMC_OneNAND_Write (row, buf, cfg);
    #endif
    /* case FSMC_NE2: */
    /* case FSMC_NE3: */
    /* case FSMC_NE4: */

    /* BANK 2 */
    /* case FSMC_NCE2: */

    /* BANK 3 */
    #ifdef FSMC_BANK3
    case FSMC_NCE3: return FSMC_NAND_Write (row, buf, cfg);
    #endif

    /* BANK 4 */
    /* case FSMC_NCE4_1: */
    /* case FSMC_NCE4_2: */
    default: return ERR_INVALID_PARAM;
  }
}

/*-----------------------------------------------------------------------------
 *      Erase block
 *  row  = Block address
 *  *cfg = Pointer to configuration structure
 *
 *  Return: RTV_NOERR         - Block erase successful
 *          ERR_NAND_ERASE    - Block erase failed
 *          ERR_NAND_HW_TOUT  - Hardware transfer timeout
 *----------------------------------------------------------------------------*/
static U32 BlockErase (U32 row, NAND_DRV_CFG *cfg) {
  U32 cs;

  /* Determine Chip Select from instance number */
  cs = SetCs (cfg->DrvInst);
  
  switch (cs) {
    /* BANK 1 */
    #ifdef FSMC_BANK1_CS1
    case FSMC_NE1:  return FSMC_OneNAND_Erase (row, cfg);
    #endif
    /* case FSMC_NE2: */
    /* case FSMC_NE3: */
    /* case FSMC_NE4: */

    /* BANK 2 */
    /* case FSMC_NCE2: */

    /* BANK 3 */
    #ifdef FSMC_BANK3
    case FSMC_NCE3: return FSMC_NAND_Erase (row, cfg);
    #endif

    /* BANK 4 */
    /* case FSMC_NCE4_1: */
    /* case FSMC_NCE4_2: */
    default: return ERR_INVALID_PARAM;
  }
}

/*-----------------------------------------------------------------------------
 * end of file
 *----------------------------------------------------------------------------*/
