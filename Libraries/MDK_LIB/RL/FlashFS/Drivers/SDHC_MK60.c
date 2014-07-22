/*-----------------------------------------------------------------------------
 *      RL-ARM - FlashFS
 *-----------------------------------------------------------------------------
 *      Name:    SDHC_MK60.c
 *      Purpose: SD/SDIO MMC Interface Driver for Freescale MK60
 *      Rev.:    V4.54
 *-----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2012 KEIL - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include <File_Config.h>
#include <MK60F12.H>
#include "SDHC_MKxx.h"

/*-----------------------------------------------------------------------------
  Memory Card FAT Driver instance definition
   mci0_drv: First SD/MMC drive [M0:]
   mci1_drv: Second SD/MMC drive [M1:]
 *----------------------------------------------------------------------------*/

#define __DRV_ID   mci0_drv
#define __SDIOCLK  60000000
#define __CPUCLK   120000000

/* MCI Driver Interface functions */
static BOOL Init (void);
static BOOL UnInit (void);
static void Delay (U32 us);
static BOOL BusMode (U32 mode);
static BOOL BusWidth (U32 width);
static BOOL BusSpeed (U32 kbaud);
static BOOL Command (U8 cmd, U32 arg, U32 resp_type, U32 *rp);
static BOOL ReadBlock (U32 bl, U8 *buf, U32 cnt);
static BOOL WriteBlock (U32 bl, U8 *buf, U32 cnt);
static BOOL SetDma (U32 mode, U8 *buf, U32 cnt);
static U32  CheckMedia (void);        /* Optional function for SD card check  */

/* MCI Device Driver Control Block */
MCI_DRV __DRV_ID = {
  Init,
  UnInit,
  Delay,
  BusMode,
  BusWidth,
  BusSpeed,
  Command,
  ReadBlock,
  WriteBlock,
  SetDma,
  CheckMedia                            /* Can be NULL if not existing        */
};


/* Wait time in for loop cycles */
#define DMA_TOUT  10000000

/*--------------------------- Init -------------------------------------------*/

static BOOL Init (void) {
  /* Initialize SDHC interface. */

  /* Enable external reference clock and select it as clock source for SDHC   */
  OSC0->CR   |= OSC_CR_ERCLKEN_MASK;
  SIM->SOPT2  = (SIM->SOPT2 & ~SIM_SOPT2_ESDHCSRC_MASK) | SIM_SOPT2_ESDHCSRC(2);

  /* Enable PortE and SDHC gate clocking */
  SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
  SIM->SCGC3 |= SIM_SCGC3_ESDHC_MASK;

  /* Reset SDHC module */
  SDHC->SYSCTL =  SDHC_SYSCTL_RSTA_MASK | SDHC_SYSCTL_SDCLKFS(0x80);
  while (SDHC->SYSCTL & SDHC_SYSCTL_RSTA_MASK);

  /* Configure memory card related PortE pins as Alternative 4 (SDHC pins)    */
  /* with high drive strength enabled                                         */ 
  PORTE->PCR[0]  = (PORTE->PCR[0] & ~PORT_PCR_MUX_MASK)  | PORT_PCR_MUX(4) |
                    PORT_PCR_DSE_MASK | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;

  PORTE->PCR[1]  = (PORTE->PCR[1] & ~PORT_PCR_MUX_MASK)  | PORT_PCR_MUX(4) |
                    PORT_PCR_DSE_MASK | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;

  PORTE->PCR[2]  = (PORTE->PCR[2] & ~PORT_PCR_MUX_MASK)  | PORT_PCR_MUX(4) |
                    PORT_PCR_DSE_MASK;

  PORTE->PCR[3]  = (PORTE->PCR[3] & ~PORT_PCR_MUX_MASK)  | PORT_PCR_MUX(4) |
                    PORT_PCR_DSE_MASK | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;

  PORTE->PCR[4]  = (PORTE->PCR[4] & ~PORT_PCR_MUX_MASK)  | PORT_PCR_MUX(4) |
                    PORT_PCR_DSE_MASK | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;

  PORTE->PCR[5]  = (PORTE->PCR[5] & ~PORT_PCR_MUX_MASK)  | PORT_PCR_MUX(4) |
                    PORT_PCR_DSE_MASK | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;

  /* Configure card detect and write protect pin as Alternative 1 (GPIO pins) */
  /* input, using pull up resistors                                           */
  PORTE->PCR[27] = (PORTE->PCR[27]& ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(1) |
                    PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
  PORTE->PCR[28] = (PORTE->PCR[28]& ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(1) |
                    PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
  PTE->PDDR &= ~(1 << 27 | 1 << 28);

  /* Configure SDHC module */
  SDHC->VENDOR  = 0;                    /* Disable External DMA Request       */
  SDHC->PROCTL  = SDHC_PROCTL_EMODE(2) |/* Little endian mode                 */
                  SDHC_PROCTL_D3CD_MASK;/* DAT3 as Card Detection Pin         */

  /* Enable interrupt flags */
  SDHC->IRQSTATEN = SDHC_IRQSTATEN_DMAESEN_MASK |

                    SDHC_IRQSTATEN_DEBESEN_MASK |
                    SDHC_IRQSTATEN_DCESEN_MASK  |
                    SDHC_IRQSTATEN_DTOESEN_MASK |

                    SDHC_IRQSTATEN_CIESEN_MASK  |
                    SDHC_IRQSTATEN_CEBESEN_MASK |
                    SDHC_IRQSTATEN_CCESEN_MASK  |
                    SDHC_IRQSTATEN_CTOESEN_MASK |

                    SDHC_IRQSTATEN_CRMSEN_MASK  |
                    SDHC_IRQSTATEN_BRRSEN_MASK  |
                    SDHC_IRQSTATEN_BWRSEN_MASK  |

                    SDHC_IRQSTATEN_TCSEN_MASK   |
                    SDHC_IRQSTATEN_CCSEN_MASK   ;

  SDHC->IRQSTAT = SDHC_IRQSTAT_ALL;     /* Clear all interrupt flags          */

    /* Set Watermark Level Register to 128 words == 512 bytes */
  SDHC->WML = SDHC_WML_WRWML(128) | SDHC_WML_RDWML(128);

  /* Set transfer block size to 512 bytes */
  SDHC->BLKATTR = SDHC_BLKATTR_BLKCNT(1) | SDHC_BLKATTR_BLKSIZE(0x200);
  
  /* Send 80 SD-clocks */
  SDHC->SYSCTL |= SDHC_SYSCTL_INITA_MASK;
  while (SDHC->SYSCTL & SDHC_SYSCTL_INITA_MASK);

  /* Success, SDHC initialized. */
  return (__TRUE);
}


/*--------------------------- UnInit -----------------------------------------*/

static BOOL UnInit (void) {
  /* Set memory card related PortE pins to reset state                        */
  PORTE->PCR[0]  = 0;
  PORTE->PCR[1]  = 0;
  PORTE->PCR[2]  = 0;
  PORTE->PCR[3]  = 0;
  PORTE->PCR[4]  = 0;
  PORTE->PCR[5]  = 0;
  PORTE->PCR[27] = 0;
  PORTE->PCR[28] = 0;
  
  /* Set SDHC related clock settings to reset state                           */
  SIM->SOPT2 &= ~SIM_SOPT2_ESDHCSRC_MASK;
  SIM->SCGC3 &= ~SIM_SCGC3_ESDHC_MASK;
  return (__TRUE);
}


/*--------------------------- Delay ------------------------------------------*/

static void Delay (U32 us) {
  /* Approximate delay in micro seconds. */
  U32 i;

  for (i = WAIT_CNT(__CPUCLK, us); i; i--);
}


/*--------------------------- BusMode ----------------------------------------*/

static BOOL BusMode (U32 mode) {
  /* Set SDIO Bus mode to Open Drain or Push Pull. */

  switch (mode) {
    case BUS_OPEN_DRAIN:
    case BUS_PUSH_PULL:
      /* Not configurable. */
      return (__TRUE);
    
    default:
      return (__FALSE);
  }
}


/*--------------------------- BusWidth ---------------------------------------*/

static BOOL BusWidth (U32 width) {
  /* Set Data Transfer width. */

  switch (width) {
    case 1:
      SDHC->PROCTL &= ~SDHC_PROCTL_DTW_MASK;  /* 1-bit mode                   */
      return (__TRUE);

    case 4:
      SDHC->PROCTL |=  SDHC_PROCTL_DTW(1);    /* 4-bit mode                   */
      return (__TRUE);
    
    default:
      return (__FALSE);
  }
}


/*--------------------------- BusSpeed ---------------------------------------*/

static BOOL BusSpeed (U32 kbaud) {
  /* Set card clock speed to desired value. */
  uint32_t pr, div, set_pr, set_div, low;
  int32_t  cl;

  /* Disable card clock */
  SDHC->SYSCTL &= ~SDHC_SYSCTL_SDCLKEN_MASK;

  /* Find prescaler and divider */
  low     = __SDIOCLK/1000;             /* Closest setting has high init val  */
  set_pr  = 0x80;                       /* Maximum prescaler value            */
  set_div = 0x0F;                       /* Maximum divider value              */
  for (pr = 2; pr <= 256; pr <<= 1) {
    for (div = 1; div <= 16; div++) {
      
      cl = pr * div * kbaud - __SDIOCLK/1000;
      if (cl >= 0) {
        if (low > cl) {
          low     = cl;
          set_pr  = pr;
          set_div = div;
        }
      }
    }
  }  

  /* Set new clock */
  SDHC->SYSCTL &= ~(SDHC_SYSCTL_DTOCV_MASK   |
                    SDHC_SYSCTL_SDCLKFS_MASK |
                    SDHC_SYSCTL_DVS_MASK    );
  
  SDHC->SYSCTL |= SDHC_SYSCTL_DTOCV (0x0E)          |
                  SDHC_SYSCTL_SDCLKFS (set_pr >> 1) |
                  SDHC_SYSCTL_DVS     (set_div - 1) ;

  /* Wait until internal card clock is stable */
  while (!(SDHC->PRSSTAT & SDHC_PRSSTAT_SDSTB_MASK));
  
  /* Enable card clock */  
  SDHC->SYSCTL |=  SDHC_SYSCTL_SDCLKEN_MASK;

  /* Wait until able to issue command using DAT or CMD line */
  while (SDHC->PRSSTAT & (SDHC_PRSSTAT_CIHB_MASK | SDHC_PRSSTAT_CDIHB_MASK));
  return (__TRUE);
}


/*--------------------------- Command ----------------------------------------*/

static BOOL Command (U8 cmd, U32 arg, U32 resp_type, U32 *rp) {
  /* Send a Command to Flash card and get a Response. */
  U32 cmdval, stat;

  while (SDHC->PRSSTAT & SDHC_PRSSTAT_CDIHB_MASK);
  while (SDHC->PRSSTAT & SDHC_PRSSTAT_CIHB_MASK);

  SDHC->IRQSTAT = SDHC_IRQSTAT_ALL;     /* Clear all interrupt flags          */

  /* Setup command */
  cmdval  = SDHC_XFERTYP_DMAEN_MASK           |
            SDHC_XFERTYP_CMDINX (cmd & 0x3F)  |
            SDHC_XFERTYP_CICEN_MASK           |
            SDHC_XFERTYP_CCCEN_MASK           ;

  switch (resp_type) {
    case RESP_SHORT: cmdval |= SDHC_XFERTYP_RSPTYP(2); break;
    case RESP_LONG:  cmdval |= SDHC_XFERTYP_RSPTYP(1); break;
  }

  /* Prepare Read/Write commands */
  if (cmd == READ_BLOCK  || cmd == READ_MULT_BLOCK) {
    cmdval   |= SDHC_XFERTYP_BCEN_MASK   |
                SDHC_XFERTYP_DTDSEL_MASK |
                SDHC_XFERTYP_AC12EN_MASK |
                SDHC_XFERTYP_DPSEL_MASK  ;
    if (arg > 1) {
      cmdval |= SDHC_XFERTYP_MSBSEL_MASK;
    }    
  }
  if (cmd == WRITE_BLOCK || cmd == WRITE_MULT_BLOCK) {
    cmdval   |= SDHC_XFERTYP_BCEN_MASK   |
                SDHC_XFERTYP_AC12EN_MASK |
                SDHC_XFERTYP_DPSEL_MASK  ;
    if (arg > 1) {
      cmdval |= SDHC_XFERTYP_MSBSEL_MASK;
    }
  }

  /* Send the command. */
  SDHC->CMDARG  = arg;
  SDHC->XFERTYP = cmdval;

  if (resp_type == RESP_NONE) {
    /* No need to wait until command finished */
    return (__TRUE);
  }

  /* Wait for response */
  for (;;) {
    stat = SDHC->IRQSTAT;

    if (stat & SDHC_IRQSTAT_CTOE_MASK) {
      /* Response timeout */
      return (__FALSE);
    }

    if ((stat & (SDHC_IRQSTAT_CTOE_MASK|SDHC_IRQSTAT_CCE_MASK)) == SDHC_IRQSTAT_CTOE_MASK) {
      /* Response timeout error */
      return (__FALSE);
    }
    if ((stat & (SDHC_IRQSTAT_CTOE_MASK|SDHC_IRQSTAT_CCE_MASK)) == SDHC_IRQSTAT_CCE_MASK) {
      /* Response CRC error */
      if ((cmd == SEND_OP_COND)      ||
          (cmd == SEND_APP_OP_COND)  ||
          (cmd == STOP_TRANS)) {
        break;
      }
    }
    if ((stat & (SDHC_IRQSTAT_CTOE_MASK|SDHC_IRQSTAT_CCE_MASK)) == (SDHC_IRQSTAT_CCE_MASK|SDHC_IRQSTAT_CTOE_MASK)) {
      /* CMD line conflict */
      return (__FALSE);
    }

    if ((stat & (SDHC_IRQSTAT_CC_MASK)) && !(stat & SDHC_IRQSTAT_CTOE_MASK)) {
      /* Response received */
      break;
    }
  }

  /* Read MCI response registers */
  if (resp_type == RESP_LONG) {
    rp[0] = (SDHC->CMDRSP[3] << 8) | (SDHC->CMDRSP[2] >> 24);
    rp[1] = (SDHC->CMDRSP[2] << 8) | (SDHC->CMDRSP[1] >> 24);
    rp[2] = (SDHC->CMDRSP[1] << 8) | (SDHC->CMDRSP[0] >> 24);
    rp[3] =  SDHC->CMDRSP[0] << 8;
  }
  else rp[0] = SDHC->CMDRSP[0];
  SDHC->IRQSTAT = SDHC_IRQSTAT_CMD;     /* Clear command interrupt flags      */
  return (__TRUE);
}


/*--------------------------- ReadBlock --------------------------------------*/

static BOOL ReadBlock (U32 bl, U8 *buf, U32 cnt) {
  /* Read one or more 512 byte blocks from Flash Card */
  U32 i, stat;

  for (i = DMA_TOUT; i; i--) {
    stat = SDHC->IRQSTAT;
    if (stat & SDHC_IRQSTAT_DMAE_MASK) {
      /* DMA error */
      return (__FALSE);
    }
    if (stat & SDHC_IRQSTAT_TC_MASK) {
      /* Data transfer finished */
      return (__TRUE);    
    }
    if (!(stat & SDHC_IRQSTAT_TC_MASK) && (stat & SDHC_IRQSTAT_DTOE_MASK)) {
      /* Data transfer timeout */
      return (__FALSE);
    }
  }
  /* DMA Transfer timeout */
  return (__FALSE);
}


/*--------------------------- WriteBlock -------------------------------------*/

static BOOL WriteBlock (U32 bl, U8 *buf, U32 cnt) {
  /* Write a cnt number of 512 byte blocks to Flash Card. */
  U32 i, stat;

  for (i = DMA_TOUT; i; i--) {
    stat = SDHC->IRQSTAT;
    if (stat & SDHC_IRQSTAT_DMAE_MASK) {
      /* DMA error */
      return (__FALSE);
    }
    if (stat & SDHC_IRQSTAT_TC_MASK) {
      /* Data transfer finished */
      return (__TRUE);
    }
    if (!(stat & SDHC_IRQSTAT_TC_MASK) && (stat & SDHC_IRQSTAT_DTOE_MASK)) {
      /* Data transfer timeout */
      return (__FALSE);
    }
  }
  /* DMA Transfer timeout */
  return (__FALSE);
}


/*--------------------------- DmaStart ---------------------------------------*/

static BOOL SetDma (U32 mode, U8 *buf, U32 cnt) {
  /* Set DMA destination/source address */
  SDHC->DSADDR   = (U32)buf;

  /* Set Transfer block count */
  SDHC->BLKATTR = (SDHC->BLKATTR & ~SDHC_BLKATTR_BLKCNT_MASK) | SDHC_BLKATTR_BLKCNT(cnt);
  return __TRUE;
}


/*--------------------------- CheckMedia -------------------------------------*/

static U32 CheckMedia (void) {
  /* Read CardDetect and WriteProtect SD card socket pins. */
  U32 stat = 0;

  if (!(PTE->PDIR & (1 << 28))) {
    /* Card is inserted (CD=0). */
    stat |= M_INSERTED;
  }
  if (!PTE->PDIR & (1 << 27)) {
    /* Write protect switch is active */
    stat |= M_PROTECTED;
  }
  return (stat);
}


/*-----------------------------------------------------------------------------
 * End of file
 *----------------------------------------------------------------------------*/
