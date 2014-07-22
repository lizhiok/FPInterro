/*----------------------------------------------------------------------------
 *      RL-ARM - FlashFS
 *----------------------------------------------------------------------------
 *      Name:    FS_FLASHPRG.C 
 *      Purpose: Flash Programming Functions - STR91x Flash
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
static BOOL EraseChip   (void);         /* Optional function if supported    */ 

/* Embedded Flash Device Driver Control Block */
EFS_DRV __DRV_ID = {
  Init,
  UnInit,
  NULL,                                 /* =NULL, use FFS internal ReadData  */
  ProgramPage,
  EraseSector,
  EraseChip
};


#define M8(adr)  (*((volatile U8  *) (adr)))
#define M16(adr) (*((volatile U16 *) (adr)))
#define M32(adr) (*((volatile U32 *) (adr)))


#define BANK0_ADR       0               // Bank0 Address

#ifdef STR91xFxx2
 #define BANK0_SZ       3               // Bank0 Size = 256kB
 #define BANK0_SEC_CNT  4               // Bank0 Sector Count
#endif
#ifdef STR91xFxx4
 #define BANK0_SZ       4               // Bank0 Size = 512kB
 #define BANK0_SEC_CNT  8               // Bank0 Sector Count
#endif

#define BANK0_SEC_SZ    0x10000         // Bank0 Sector Size

#define BANK1_ADR       0x400000        // Bank1 Address
#define BANK1_SZ        2               // Bank1 Size =  32kB
#define BANK1_SEC_CNT   4               // Bank1 Sector Count
#define BANK1_SEC_SZ    0x2000          // Bank1 Sector Size

#ifdef BANK0
 #define BANK_ADR       BANK0_ADR       // Bank Address
 #define BANK_SEC_CNT   BANK0_SEC_CNT   // Bank Sector Count
 #define BANK_SEC_SZ    BANK0_SEC_SZ    // Bank Sector Size
 #define BANK_SEC_MASK  ((1 << BANK0_SEC_CNT) - 1)
#endif

#ifdef BANK1
 #define BANK_ADR       BANK1_ADR       // Bank Address
 #define BANK_SEC_CNT   BANK1_SEC_CNT   // Bank Sector Count
 #define BANK_SEC_SZ    BANK1_SEC_SZ    // Bank Sector Size
 #define BANK_SEC_MASK  (((0x100 << BANK1_SEC_CNT) - 1) & ~0xFF)
#endif


/* System Control Unit Registers */
#define SCU_CLKCNTR     (*((volatile U32 *) 0x5C002000))
#define SCU_SCR0        (*((volatile U32 *) 0x5C002034))

/* Flash Memory Interface Registers */
#define FMI_BBSR        (*((volatile U32 *) 0x54000000))
#define FMI_BBADR       (*((volatile U32 *) 0x5400000C))
#define FMI_NBBSR       (*((volatile U32 *) 0x54000004))
#define FMI_NBBADR      (*((volatile U32 *) 0x54000010))
#define FMI_CR          (*((volatile U32 *) 0x54000018))
#define FMI_SR          (*((volatile U32 *) 0x5400001C))

/* Flash Commands */
#define CMD_PROT1CFM    0x01            // Protect Level 1 Confirm
#define CMD_SERS        0x20            // Sector Erase Set-up
#define CMD_PRGS        0x40            // Program Set-up
#define CMD_CLRSTAT     0x50            // Clear Status Register
#define CMD_PROT1S      0x60            // Protect Level 1 Set-up
#define CMD_RDSTAT      0x70            // Read Status Register
#define CMD_BNKERS      0x80            // Bank Erase Set-up
#define CMD_RSIG        0x90            // Read Electronic Signature
#define CMD_RDOTP       0x98            // Read OTP Sector
#define CMD_CFM         0xD0            // Prog/Ers Resume, Ers Confirm
                                        // Level 1 Unprotect Confirm
#define CMD_RDARR       0xFF            // Read Array

/* Status register bits */
#define PECS            0x80            // Prog/Ers Controller Status
#define ESS             0x40            // Erase Suspend Status
#define ES              0x20            // Erase Status
#define PS              0x10            // Program Status
#define PSS             0x04            // Program Suspend Status
#define SP              0x02            // Sector Protection Status


/* Local Variables */
static U32 base_adr;

/* Local Function Prototypes */
static void Delay (void);
static BOOL WaitStatus (U32 adr, U16 stat);


/*--------------------------- Init ------------------------------------------*/

static BOOL Init (U32 adr, U32 clk)  {
  /* Initialize flash programming functions. */
  int i, j;

  base_adr = adr;

  // Setup System Control Unit

//SCU_CLKCNTR = 0x00020002;             // Main Clock Source is Oscillator
//SCU_SCR0   &=~0x00000001;             // PFQBC Unit Disable

  // Setup Flash Memory Interface
  FMI_BBSR    = BANK0_SZ;
  FMI_BBADR   = BANK0_ADR >> 2;
  FMI_NBBSR   = BANK1_SZ;
  FMI_NBBADR  = BANK1_ADR >> 2;
  FMI_CR      = 0x00000018;             // Enable Bank 0 & 1

  // Clear Level 1 Protection (unprotect all sectors)
  for (i = 0, j = BANK_ADR; i < BANK_SEC_CNT; i++, j += BANK_SEC_SZ) {
    M16(j) = CMD_PROT1S;
    M16(j) = CMD_CFM;
    M16(j) = CMD_RDARR;
    Delay();
  }

  // Check if all sectors are unprotected
  M16(BANK1_ADR)  = CMD_RSIG;
  Delay();
  i = M16(BANK1_ADR + 0x10) & BANK_SEC_MASK;
  M16(BANK1_ADR)  = CMD_RDARR;          // Leave RSIG mode

  if (i) {
    return (__FALSE);                   // Not unprotected
  }
  return (__TRUE);
}


/*--------------------------- UnInit ----------------------------------------*/

static BOOL UnInit (void)  {
  /* Uninitialize flash programming functions. */
  base_adr = 0;
  return (__TRUE);
}


/*--------------------------- ProgramPage -----------------------------------*/

static BOOL ProgramPage (U32 adr, U32 sz, U8 *buf) {
 /* Program Page in Flash Memory. */ 
  U32 i;

  for (i = 0; i < ((sz+1)/2); i++)  {
    M16(adr & ~3) = CMD_PRGS;           // Write Program Set-up Command
    M16(adr) = *((__packed U16 *)buf);  // Write 2 byte data
    if (!(WaitStatus(adr & ~3, PS | SP))) {
      return (__FALSE);                 // Program unsuccessful
    }
    buf += 2;
    adr += 2;
  }

  return (__TRUE);                      // Done successfully
}


/*--------------------------- EraseSector -----------------------------------*/

static BOOL EraseSector (U32 adr) {
  /* Erase Sector in Flash Memory. */

  M16(adr) = CMD_SERS;                  // Issue Erase Sector procedure
  M16(adr) = CMD_CFM;

  if (!(WaitStatus(adr, ES | SP))) {
    return (__FALSE);                   // Erase unsuccessful
  }

  return (__TRUE);                      // Done
}


/*--------------------------- EraseChip -------------------------------------*/

static BOOL EraseChip (void) {
 /* Global Erase complete Flash Memory. */

  M16(base_adr) = CMD_BNKERS;           // Issue Erase Bank procedure
  M16(base_adr) = CMD_CFM;

  if (!(WaitStatus(base_adr, ES | SP))) {
    return (__FALSE);                   // Erase unsuccessful
  }

  return (__TRUE);                      // Done
}


/*--------------------------- Delay -----------------------------------------*/

void Delay (void) {
  int i;

  for (i = 0; i < 10; i++);
}


/*--------------------------- WaitStatus ------------------------------------*/

static BOOL WaitStatus (U32 adr, U16 stat) {
  /* Wait for last operation to finish and check status. */

  while (!(M16(adr) & PECS));           // Wait for operation finish

  stat    &= M16(adr);                  // Read status

  M16(adr) = CMD_CLRSTAT;               // Clear Status Register
  M16(adr) = CMD_RDARR;                 // exit Read Status Register mode

  if (stat == 0) {
    return (__TRUE);
  }
  return (__FALSE);
}

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/


