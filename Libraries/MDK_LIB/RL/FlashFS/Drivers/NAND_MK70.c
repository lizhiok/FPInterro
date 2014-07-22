/*------------------------------------------------------------------------------
 *      RL-ARM - FlashFS
 *------------------------------------------------------------------------------
 *      Name:    NAND_MK70.c
 *      Purpose: NAND Flash Interface Driver for Freescale MK70
 *      Rev.:    V4.60
 *------------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2012 KEIL - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include <File_Config.h>
#include <MK70F12.H>                    /* MK70F12 Definitions                */
#include "NAND_MKxx.h"

/* Select which chip select will be used */
#define NAND0_HW_CS  NAND_CS0
//#define NAND0_HW_CS  NAND_CS1

/* Select bus width */
//#define NAND_BUS_W  NAND_BUS_W8
#define NAND_BUS_W  NAND_BUS_W16


/*----------------------------------------------------------------------------
 *      NAND driver prototypes
 *---------------------------------------------------------------------------*/
static uint32_t Init         (NAND_DRV_CFG *cfg);
static uint32_t UnInit       (NAND_DRV_CFG *cfg);
static uint32_t PageRead     (uint32_t row, uint8_t *buf, NAND_DRV_CFG *cfg);
static uint32_t PageWrite    (uint32_t row, uint8_t *buf, NAND_DRV_CFG *cfg);
static uint32_t BlockErase   (uint32_t row, NAND_DRV_CFG *cfg);


/*------------------------------------------------------------------------------
  NAND Device Driver Control Block
 *----------------------------------------------------------------------------*/
const NAND_DRV nand0_drv = {
  Init,
  UnInit,
  PageRead,
  PageWrite,
  BlockErase,
};


/*------------------------------------------------------------------------------
 *      Setup NAND page layout
 *
 *  Inputs: cfg - NAND Flash Geometry and Layout configuration structure
 *
 *  Return: (none)
 *----------------------------------------------------------------------------*/
static void SetupPageLay (NAND_DRV_CFG *cfg) {
  NAND_PG_LAY *pgLay = cfg->PgLay;
  
  if (cfg->PageSize == 2112) {
    /* Setup page layout */
    pgLay->Pos_LSN  = MK_POS_LSN;
    pgLay->Pos_COR  = MK_POS_COR;
    pgLay->Pos_BBM  = MK_POS_BBM;
    pgLay->Pos_ECC  = MK_POS_ECC;
    pgLay->SectInc  = MK_SECT_INC;
    pgLay->SpareOfs = MK_SPARE_OFS;
    pgLay->SpareInc = MK_SPARE_INC;
  }
}


/*------------------------------------------------------------------------------
 *      Reset NAND Flash
 *
 *  Inputs: (none)
 *
 *  Return: RTV_NOERR         - Reset successful, NAND is ready
 *          ERR_NAND_HW_TOUT  - Reset Command failed
 *----------------------------------------------------------------------------*/
static uint32_t NandReset (void) {
  uint32_t i;

  /* Clear interrupt flags */
  NFC->ISR  |= NFC_ISR_WERRCLR_MASK | NFC_ISR_DONECLR_MASK | NFC_ISR_IDLECLR_MASK;
  /* Set reset code */
  NFC->CMD2  = NFC_CMD2_BYTE1(NAND_CMD_RESET) | NFC_CMD2_CODE (NFC_CMD_RESET);
  NFC->CMD1  = NFC_CMD1_BYTE3(NAND_CMD_STATUS);
  
  /* Execute command */
  NFC->CMD2 |= NFC_CMD2_BUSY_START_MASK;
  /* Wait until flash controller busy */
  for (i = NAND_TIMEOUT; i; i--) {
    if ((NFC->CMD2 & NFC_CMD2_BUSY_START_MASK) == 0) {
      if ((NFC->SR2 & NAND_STAT_FAIL) == 0) {
        /* Reset successful, NAND is ready */
        return (RTV_NOERR);
      }
      else break;
    }
  }
  /* Reset Command failed */
  return (ERR_NAND_HW_TOUT);
}


/*------------------------------------------------------------------------------
 *      Read page
 *  row  = Page address
 *  *buf = Pointer to data buffer
 *  *cfg = Pointer to configuration structure
 *
 *  Return: RTV_NOERR             - Page read successful
 *          ERR_NAND_HW_TOUT      - Hardware transfer timeout
 *          ERR_NAND_UNSUPPORTED  - Chip select not supported
 *          ERR_ECC_COR           - ECC corrected the data within page
 *          ERR_ECC_UNCOR         - ECC was not able to correct the data
 *----------------------------------------------------------------------------*/
static uint32_t PageRead (uint32_t row, uint8_t *buf, NAND_DRV_CFG *cfg) {
  uint8_t  *ep;
  uint32_t *bp, *dp;
  uint32_t i, sz;

  /* Clear interrupt flags */
  NFC->ISR |= NFC_ISR_WERRCLR_MASK | NFC_ISR_DONECLR_MASK | NFC_ISR_IDLECLR_MASK;
  
  /* Select chip select to use */
  switch (cfg->DrvInst) {
    case 0: NFC->RAR = NFC_RAR_CS0_MASK | NFC_RAR_RB0_MASK; break;
    case 1: NFC->RAR = NFC_RAR_CS1_MASK | NFC_RAR_RB1_MASK; break;
    default:
      return (ERR_NAND_UNSUPPORTED);
  }

  /* Set row address */
  NFC->RAR |= row;

  /* Prepare page write and status read commands */
  NFC->CMD2 = NFC_CMD2_CODE  (NFC_CMD_READ)     |
              NFC_CMD2_BYTE1 (NAND_CMD_READ1ST) ;
  NFC->CMD1 = NFC_CMD1_BYTE2 (NAND_CMD_READ2ND) ;

  /* Execute command */
  NFC->CMD2 |= NFC_CMD2_BUSY_START_MASK;
  
  /* Wait until command done */
  for (i = NAND_TIMEOUT; i; i--) {
    if ((NFC->CMD2 & NFC_CMD2_BUSY_START_MASK) == 0) {
      /* Copy data from NFC SRAM into page buffer */
      dp = (uint32_t *)NFC_BASE;            /* NFC SRAM pointer                   */
      bp = (uint32_t *)buf;                 /* Page buffer pointer                */
      sz = cfg->PageSize;
      for (i = 0; i < sz; i += 4) {
        *bp++ = *dp++;
      }

      /* Check ECC result */
      ep  = (uint8_t *)(NFC_BASE + (MK_ECC_AD & ~0x07) + 4);
      if (*ep == 0x81 || *ep == 0x00) {
        /* Page is empty or is consistent */
        return (RTV_NOERR);             /* Page read successful               */
      }
      else {
        if (*ep & 0x80) {
          return (ERR_ECC_UNCOR);
        }
        return (ERR_ECC_COR);
      }
    }
  }
  return (ERR_NAND_HW_TOUT);            /* Busy asserted or no response       */
}


/*------------------------------------------------------------------------------
 *      Write page
 *  row  = Page address
 *  *buf = Pointer to data buffer
 *  *cfg = Pointer to configuration structure
 *
 *  Return: RTV_NOERR             - Page write successful
 *          ERR_NAND_PROG         - Page write failed
 *          ERR_NAND_HW_TOUT      - Hardware transfer timeout
 *          ERR_NAND_UNSUPPORTED  - Chip select not supported
 *----------------------------------------------------------------------------*/
static uint32_t PageWrite (uint32_t row, uint8_t *buf, NAND_DRV_CFG *cfg) {
  uint32_t *bp, *dp;
  uint32_t  i, sz;
  
  /* Clear interrupt flags */
  NFC->ISR |= NFC_ISR_WERRCLR_MASK | NFC_ISR_DONECLR_MASK | NFC_ISR_IDLECLR_MASK;
  
  /* Select chip select to use */
  switch (cfg->DrvInst) {
    case 0: NFC->RAR = NFC_RAR_CS0_MASK | NFC_RAR_RB0_MASK; break;
    case 1: NFC->RAR = NFC_RAR_CS1_MASK | NFC_RAR_RB1_MASK; break;
    default:
      return (ERR_NAND_UNSUPPORTED);
  }

  /* Set DMA1 address */
  NFC->DMA1 = (uint32_t)buf;

  /* Set row address */
  NFC->RAR |= row;

  /* Prepare page write and status read commands */
  NFC->CMD2 = NFC_CMD2_CODE  (NFC_CMD_WRITE)    |
              NFC_CMD2_BYTE1 (NAND_CMD_PROG1ST) ;
  NFC->CMD1 = NFC_CMD1_BYTE2 (NAND_CMD_PROG2ND) |
              NFC_CMD1_BYTE3 (NAND_CMD_STATUS)  ;

  /* Copy data from buffer to NFC SRAM */
  dp = (uint32_t *)NFC_BASE;            /* NFC SRAM pointer                   */
  bp = (uint32_t *)buf;                 /* Page buffer pointer                */
  sz = cfg->PageSize;
  for (i = 0; i < sz; i += 4) {
    *dp++ = *bp++;
  }

  /* Execute command */
  NFC->CMD2 |= NFC_CMD2_BUSY_START_MASK;
  
  /* Wait until command done */
  for (i = NAND_TIMEOUT; i; i--) {
    if ((NFC->CMD2 & NFC_CMD2_BUSY_START_MASK) == 0) {
      /* Check status */
      if (NFC->SR2 & NAND_STAT_FAIL) {
        return (ERR_NAND_PROG);         /* Program operation failed           */
      }
      return (RTV_NOERR);               /* Page write successful              */
    }
  }
  return (ERR_NAND_HW_TOUT);            /* Busy asserted or no response       */
}


/*------------------------------------------------------------------------------
 *      Erase block
 *  row  = Block address
 *  *cfg = Pointer to configuration structure
 *
 *  Return: RTV_NOERR             - Block erase successful
 *          ERR_NAND_ERASE        - Block erase failed
 *          ERR_NAND_HW_TOUT      - Hardware transfer timeout
 *          ERR_NAND_UNSUPPORTED  - Chip select not supported
 *----------------------------------------------------------------------------*/
static uint32_t BlockErase (uint32_t row, NAND_DRV_CFG *cfg) {
  uint32_t i;

    /* Clear interrupt flags */
  NFC->ISR |= NFC_ISR_WERRCLR_MASK | NFC_ISR_DONECLR_MASK | NFC_ISR_IDLECLR_MASK;

  /* Select chip select to use */
  switch (cfg->DrvInst) {
    case 0: NFC->RAR = NFC_RAR_CS0_MASK | NFC_RAR_RB0_MASK; break;
    case 1: NFC->RAR = NFC_RAR_CS1_MASK | NFC_RAR_RB1_MASK; break;
    default:
      return (ERR_NAND_UNSUPPORTED);
  }

  /* Set row address */
  NFC->RAR |= row;

  /* Prepare block erase command */
  NFC->CMD2 = NFC_CMD2_CODE  (NFC_CMD_ERASE)     |
              NFC_CMD2_BYTE1 (NAND_CMD_ERASE1ST) ;
  NFC->CMD1 = NFC_CMD1_BYTE2 (NAND_CMD_ERASE2ND) |
              NFC_CMD1_BYTE3 (NAND_CMD_STATUS)   ;

  /* Execute command */
  NFC->CMD2 |= NFC_CMD2_BUSY_START_MASK;

  /* Wait until command done */
  for (i = NAND_TIMEOUT; i; i--) {
    if ((NFC->CMD2 & NFC_CMD2_BUSY_START_MASK) == 0) {
      if (NFC->SR2 & NAND_STAT_FAIL) {
        return (ERR_NAND_ERASE);        /* Erase operation failed             */
      }
      return (RTV_NOERR);               /* Block erased successfuly           */
    }
  }
  return (ERR_NAND_HW_TOUT);            /* Busy asserted or no response       */
}


/*------------------------------------------------------------------------------
 *      Uninitialise NAND flash driver
 *  *cfg = Pointer to configuration structure
 *
 *  Return: RTV_NOERR         - UnInit successful
 *----------------------------------------------------------------------------*/
static uint32_t UnInit (NAND_DRV_CFG *cfg) {

  /* Disable NFC gate clocking */
  SIM->SCGC3 &= ~SIM_SCGC3_NFC_MASK;

  /* Set NAND peripheral lines to reset state */
  #if NAND0_HW_CS == NAND_CS0
  PORTC->PCR[17] = PORT_PCR_MUX (0);
  #elif NAND_HW_CS == NAND_CS1
  PORTC->PCR[18] = PORT_PCR_MUX (0);
  #endif
  PORTD->PCR[10] = PORT_PCR_MUX (0);
  PORTC->PCR[11] = PORT_PCR_MUX (0);
  PORTD->PCR[8]  = PORT_PCR_MUX (0);
  PORTD->PCR[9]  = PORT_PCR_MUX (0);

  PORTC->PCR[16] = PORT_PCR_MUX (0);

  PORTD->PCR[5]  = PORT_PCR_MUX (0);
  PORTD->PCR[4]  = PORT_PCR_MUX (0);
  PORTC->PCR[10] = PORT_PCR_MUX (0);
  PORTC->PCR[9]  = PORT_PCR_MUX (0);
  PORTC->PCR[8]  = PORT_PCR_MUX (0);
  PORTC->PCR[7]  = PORT_PCR_MUX (0);
  PORTC->PCR[6]  = PORT_PCR_MUX (0);
  PORTC->PCR[5]  = PORT_PCR_MUX (0);
  #if (NAND_BUS_W == NAND_BUS_W16)
  PORTC->PCR[4]  = PORT_PCR_MUX (0);
  PORTC->PCR[2]  = PORT_PCR_MUX (0);
  PORTC->PCR[1]  = PORT_PCR_MUX (0);
  PORTC->PCR[0]  = PORT_PCR_MUX (0);

  PORTB->PCR[23] = PORT_PCR_MUX (0);
  PORTB->PCR[22] = PORT_PCR_MUX (0);
  PORTB->PCR[21] = PORT_PCR_MUX (0);
  PORTB->PCR[20] = PORT_PCR_MUX (0);
  #endif
  return (RTV_NOERR);
}


/*------------------------------------------------------------------------------
 *      Initialize NAND flash driver
 *
 *  *cfg = Pointer to configuration structure
 *
 *  Return: RTV_NOERR         - NAND Flash Initialisation successful
 *          ERR_NAND_HW_TOUT  - NAND Flash Reset Command failed
 *          ERR_NAND_DMA_TOUT - DMA Configuration failed
 *----------------------------------------------------------------------------*/
static uint32_t Init (NAND_DRV_CFG *cfg) {
  uint32_t mode, pos;
  
  /* Setup page layout */
  SetupPageLay (cfg);
  
  /* Configure clocking: take PLL0 output clock, divide it and use for NFC */
  SIM->CLKDIV4 &= ~(SIM_CLKDIV4_NFCDIV_MASK | SIM_CLKDIV4_NFCFRAC_MASK);
  /* NFCDIV = 2, NFCFRAC = 0 -> Divider output == Divider input/3 */
  SIM->CLKDIV4 |= SIM_CLKDIV4_NFCDIV (5);
  
  /* Set PLL0 output as NFC divider input */
  SIM->SOPT2 &= ~(SIM_SOPT2_NFCSRC_MASK | SIM_SOPT2_NFC_CLKSEL_MASK);
  SIM->SOPT2 |=   SIM_SOPT2_NFCSRC (1);
  
  /* Enable PLL0 clock to peripherals */
  MCG->C5    |= MCG_C5_PLLCLKEN0_MASK;
  
  /* Clock to NFC is PLL0 / 3, enable gate clocking */
  SIM->SCGC3 |= SIM_SCGC3_NFC_MASK;     /* Enable NFC gate clocking           */
  SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK |  /* Enable PORTB gate clocking         */
                SIM_SCGC5_PORTC_MASK |  /* Enable PORTC gate clocking         */
                SIM_SCGC5_PORTD_MASK ;  /* Enable PORTD gate clocking         */

  /* Init NAND peripheral lines */
  #if NAND0_HW_CS == NAND_CS0
  PORTC->PCR[17] = PORT_PCR_MUX (6) | PORT_PCR_DSE_MASK;  /* PTC17 - nCE 0 - ALT6 */
  #elif NAND_HW_CS == NAND_CS1
  PORTC->PCR[18] = PORT_PCR_MUX (6) | PORT_PCR_DSE_MASK;  /* PTC18 - nCE 1 - ALT6 */
  #endif
  PORTD->PCR[10] = PORT_PCR_MUX (6) | PORT_PCR_DSE_MASK;  /* PTD10 - nRE   - ALT6 */
  PORTC->PCR[11] = PORT_PCR_MUX (5) | PORT_PCR_DSE_MASK;  /* PTC11 - nWE   - ALT5 */
  PORTD->PCR[8]  = PORT_PCR_MUX (6) | PORT_PCR_DSE_MASK;  /* PTD8  - CLE   - ALT6 */
  PORTD->PCR[9]  = PORT_PCR_MUX (6) | PORT_PCR_DSE_MASK;  /* PTD9  - ALE   - ALT6 */

  PORTC->PCR[16] = PORT_PCR_MUX (6) | PORT_PCR_DSE_MASK;  /* PTC16 - R/B   - ALT6 */

  PORTD->PCR[5]  = PORT_PCR_MUX (5) | PORT_PCR_DSE_MASK;  /* PTD5  - D0    - ALT5 */
  PORTD->PCR[4]  = PORT_PCR_MUX (5) | PORT_PCR_DSE_MASK;  /* PTD4  - D1    - ALT5 */
  PORTC->PCR[10] = PORT_PCR_MUX (5) | PORT_PCR_DSE_MASK;  /* PTC10 - D2    - ALT5 */
  PORTC->PCR[9]  = PORT_PCR_MUX (5) | PORT_PCR_DSE_MASK;  /* PTC9  - D3    - ALT5 */
  PORTC->PCR[8]  = PORT_PCR_MUX (5) | PORT_PCR_DSE_MASK;  /* PTC8  - D4    - ALT5 */
  PORTC->PCR[7]  = PORT_PCR_MUX (5) | PORT_PCR_DSE_MASK;  /* PTC7  - D5    - ALT5 */
  PORTC->PCR[6]  = PORT_PCR_MUX (5) | PORT_PCR_DSE_MASK;  /* PTC6  - D6    - ALT5 */
  PORTC->PCR[5]  = PORT_PCR_MUX (5) | PORT_PCR_DSE_MASK;  /* PTC5  - D7    - ALT5 */
  #if (NAND_BUS_W == NAND_BUS_W16)
  PORTC->PCR[4]  = PORT_PCR_MUX (5) | PORT_PCR_DSE_MASK;  /* PTC4  - D8    - ALT5 */
  PORTC->PCR[2]  = PORT_PCR_MUX (5) | PORT_PCR_DSE_MASK;  /* PTC2  - D9    - ALT5 */
  PORTC->PCR[1]  = PORT_PCR_MUX (5) | PORT_PCR_DSE_MASK;  /* PTC1  - D10   - ALT5 */
  PORTC->PCR[0]  = PORT_PCR_MUX (5) | PORT_PCR_DSE_MASK;  /* PTC0  - D11   - ALT5 */

  PORTB->PCR[23] = PORT_PCR_MUX (5) | PORT_PCR_DSE_MASK;  /* PTB23 - D12   - ALT5 */
  PORTB->PCR[22] = PORT_PCR_MUX (5) | PORT_PCR_DSE_MASK;  /* PTB22 - D13   - ALT5 */
  PORTB->PCR[21] = PORT_PCR_MUX (5) | PORT_PCR_DSE_MASK;  /* PTB21 - D14   - ALT5 */
  PORTB->PCR[20] = PORT_PCR_MUX (5) | PORT_PCR_DSE_MASK;  /* PTB20 - D15   - ALT5 */
  #endif

  /* Set configuration */
  switch (cfg->PageSize) {
    case 528:  mode = 1; break;         /* 4-bit error correction per page    */
    case 2112: mode = 5; break;         /* 16-bits error correction per page  */

    default:
      /* Page size is not supported */
      return (ERR_NAND_UNSUPPORTED);
  }

  /* Position of ECC result byte, generated by BCH hardware */
  pos = MK_ECC_AD >> 3;

  /* Init NAND Flash Controller */
  NFC->CFG = NFC_CFG_STOPWERR_MASK |    /* Stop on write error                */
             NFC_CFG_ECCAD (pos)   |    /* SRAM byte address for ECC          */
             NFC_CFG_ECCSRAM_MASK  |    /* Write ECC status to SRAM           */
             NFC_CFG_ECCMODE (mode)|    /* ECC correction mode                */
             NFC_CFG_FAST_MASK     |    /* Fast flash timing                  */
             NFC_CFG_IDCNT (2)     |    /* Read two bytes on read-id command  */
             NFC_CFG_TIMEOUT (10)  |    /* No. cycles WE high to R/B sampling */
             #if (NAND_BUS_W == NAND_BUS_W16)
             NFC_CFG_BITWIDTH_MASK |    /* 0 = 8bit bus, 1 = 16bit bus        */
             #endif
             NFC_CFG_PAGECNT(1);        /* No. of virtual pages               */

  NFC->SECSZ  = NFC_SECSZ_SIZE(cfg->PageSize);
  #if (NAND_BUS_W == NAND_BUS_W16)
  NFC->SECSZ += 1;                      /* Only odd size supported if 16-bits */
  #endif

  NFC->CAR  = 0;                        /* Column address is never needed     */
  NFC->RAI  = 0;                        /* Row address increment disabled     */

  /* Reset NAND to check if NAND is ready */
  return (NandReset());
}
/*------------------------------------------------------------------------------
 * End of file
 *----------------------------------------------------------------------------*/
