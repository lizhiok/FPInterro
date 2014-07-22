/*----------------------------------------------------------------------------
 *      RL-ARM - FlashFS
 *----------------------------------------------------------------------------
 *      Name:    MCI_SAM3U.H 
 *      Purpose: Multimedia Card Interface Driver for Atmel AT91SAM3U Defs
 *      Rev.:    V4.20
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#ifndef __MCI_SAM3U_H
#define __MCI_SAM3U_H

/* PMC Write Protect Mode Keys */
#define PMC_WPEN_KEY        0x504D4301
#define PMC_WPDIS_KEY       0x504D4300

/* MCI definitions */
#define HSMCI_FIFO_ADR        ((U32)HSMCI + 0x200)

/* MCI Command register bit information */
#define MCI_RSPTYP_NO        (0x0 <<  6)  /* (MCI) No response */
#define MCI_RSPTYP_48        (0x1 <<  6)  /* (MCI) 48-bit response */
#define MCI_RSPTYP_136       (0x2 <<  6)  /* (MCI) 136-bit response */
#define MCI_RSPTYP_R1B       (0x3 <<  6)  /* (MCI) R1b response */
#define MCI_SPCMD_INIT       (0x1 <<  8)  /* (MCI) Special Command - Initialization Command */
#define MCI_TRCMD_NO         (0x0 << 16)  /* (MCI) No transfer */
#define MCI_TRCMD_START      (0x1 << 16)  /* (MCI) Start transfer */
#define MCI_TRCMD_STOP       (0x2 << 16)  /* (MCI) Stop transfer */
#define MCI_TRDIR_WRITE      (0x0 << 18)  /* (MCI) Write */
#define MCI_TRDIR_READ       (0x1 << 18)  /* (MCI) Read */
#define MCI_TRTYP_BLOCK      (0x0 << 19)  /* (MCI) MMC/SDCard Single Block transfer type */
#define MCI_TRTYP_MULTIPLE   (0x1 << 19)  /* (MCI) MMC/SDCard Multiple Block transfer type */
#define MCI_TRTYP_STREAM     (0x2 << 19)  /* (MCI) MMC Stream transfer type */
#define MCI_TRTYP_SDIO_BYTE  (0x4 << 19)  /* (MCI) SDIO Byte transfer type */
#define MCI_TRTYP_SDIO_BLOCK (0x5 << 19)  /* (MCI) SDIO Block transfer type */

#endif

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
