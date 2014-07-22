/*----------------------------------------------------------------------------
 *      RL-ARM - FlashFS
 *----------------------------------------------------------------------------
 *      Name:    FS_FLASHPRG.C 
 *      Purpose: Flash Programming Functions - LPC2xxx 128kB Flash
 *      Rev.:    V4.20
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2011 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <File_Config.h>

/*----------------------------------------------------------------------------
  Embedded Flash Driver instance definition
   fl0_drv: First  Embedded Flash drive [F0:]
   fl1_drv: Second Embedded Flash drive [F1:]
 *----------------------------------------------------------------------------*/

#define __DRV_ID  fl0_drv

/* Embedded Flash Driver Interface functions */
static BOOL Init        (U32 adr, U32 clk);
static BOOL UnInit      (void);
static BOOL ProgramPage (U32 adr, U32 sz, U8 *buf);
static BOOL EraseSector (U32 adr);

/* Embedded Flash Device Driver Control Block */
EFS_DRV __DRV_ID = {
  Init,
  UnInit,
  NULL,                                 /* =NULL, use FFS internal ReadData  */
  ProgramPage,
  EraseSector,
  NULL
};


/* Local definitions */
#define PAGE_SZ     1024

/* Local Variables */
static U32 Page[PAGE_SZ/4];
static U32 CCLK;
static struct sIAP {           // IAP Structure
  U32 cmd;                     // Command
  U32 par[4];                  // Parameters
  U32 stat;                    // Status
} IAP;

/* External functions */
extern void IAP_Execute (struct sIAP *pIAP);

/* Local functions */
static U32 GetSecNum (U32 adr);

/*--------------------------- Init ------------------------------------------*/

static BOOL Init (U32 adr, U32 clk)  {
  /* Initialize flash programming functions. */

//CCLK /=  1000;                               // Clock Frequency in kHz
  CCLK  = (1049*(clk >> 10)) >> 10;            // Approximate (no Library Code)
  return (__TRUE);
}


/*--------------------------- UnInit ----------------------------------------*/

static BOOL UnInit (void)  {
  /* Uninitialize flash programming functions. */

  CCLK = 0;
  return (__TRUE);
}


/*--------------------------- ProgramPage -----------------------------------*/

static BOOL ProgramPage (U32 adr, U32 sz, U8 *buf) {
 /* Program Page in Flash Memory. */ 
  U32 padr, ofs, cnt, n;

  IAP.cmd    = 50;                             // Prepare Sector for Write
  IAP.par[0] = GetSecNum(adr);                 // Start Sector
  IAP.par[1] = GetSecNum(adr + sz - 1);        // End Sector
  IAP_Execute (&IAP);                          // Execute IAP Command
  if (IAP.stat) return (__FALSE);              // Command Failed

  while (sz) {                                 // Go through all Data
    padr = adr & ~(PAGE_SZ - 1);               // Page Address
    ofs  = adr - padr;                         // Data Offset
    cnt  = PAGE_SZ - ofs;                      // Data Count
    if (cnt > sz) cnt = sz;                    // Adjust Data Count
    if (cnt != PAGE_SZ) {                      // Incomplete Page being written
      for (n = 0; n < PAGE_SZ/4; n++) {        // Go through complete Page
        Page[n] = *((U32 *)padr + n);          // Read Page Data from Flash
      }
    }
    for (n = 0; n < cnt; n++) {                // Go through Page Data
      *((U8 *)Page + ofs++) = *buf++;          // Copy & Align to Page Buffer
    }

    IAP.cmd    = 51;                           // Copy RAM to Flash
    IAP.par[0] = padr;                         // Destination Flash Address
    IAP.par[1] = (U32)Page;                    // Source RAM Address
    IAP.par[2] = PAGE_SZ;                      // Page Size
    IAP.par[3] = CCLK;                         // CCLK in kHz
    IAP_Execute (&IAP);                        // Execute IAP Command
    if (IAP.stat) return (__FALSE);            // Command Failed

    adr += cnt;                                // Next Address
    sz  -= cnt;                                // Next Size
  }

  return (__TRUE);                             // Finished without Errors
}


/*--------------------------- EraseSector -----------------------------------*/

static BOOL EraseSector (U32 adr) {
  /*  Erase Sector in Flash Memory. */
  U32 n;

  n = GetSecNum(adr);                          // Get Sector Number

  IAP.cmd    = 50;                             // Prepare Sector for Erase
  IAP.par[0] = n;                              // Start Sector
  IAP.par[1] = n;                              // End Sector
  IAP_Execute (&IAP);                          // Execute IAP Command
  if (IAP.stat) return (__FALSE);              // Command Failed

  IAP.cmd    = 52;                             // Erase Sector
  IAP.par[0] = n;                              // Start Sector
  IAP.par[1] = n;                              // End Sector
  IAP.par[2] = CCLK;                           // CCLK in kHz
  IAP_Execute (&IAP);                          // Execute IAP Command
  if (IAP.stat) return (__FALSE);              // Command Failed

  return (__TRUE);                             // Finished without Errors
}


/*--------------------------- GetSecNum -------------------------------------*/

static U32 GetSecNum (U32 adr) {
  U32 n;

  n = (adr >> 13) & 0x1F;                      // Sector Number
  return (n);
}

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/

