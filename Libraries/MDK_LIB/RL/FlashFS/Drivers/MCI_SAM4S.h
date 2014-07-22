/*----------------------------------------------------------------------------
 *      RL-ARM - FlashFS
 *----------------------------------------------------------------------------
 *      Name:    MCI_SAM4S.H 
 *      Purpose: Multimedia Card Interface Driver for Atmel AT91SAM4S Defs
 *      Rev.:    V4.60
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2012 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#ifndef __MCI_SAM4S_H
#define __MCI_SAM4S_H

#define HSMCI_TOUT          100000

/* PMC Write Protect Mode Keys */
#define PMC_WPEN_KEY        0x504D4301
#define PMC_WPDIS_KEY       0x504D4300

/* PIO Write Protect Mode Keys */
#define PIO_WPEN_KEY        0x50494F01
#define PIO_WPDIS_KEY       0x50494F00

/* HSMCI Write Protect Mode Keys */
#define HSMCI_WPEN_KEY      0x4D434901
#define HSMCI_WPDIS_KEY     0x4D434900

/* PIO registers pin masks */
#define PIO_PER_PIN_MSK   (PIO_PER_P6  | PIO_PER_P26 | PIO_PER_P27 | PIO_PER_P28 | PIO_PER_P29 | PIO_PER_P30 | PIO_PER_P31)
#define PIO_IDR_PIN_MSK   (PIO_IDR_P6  | PIO_IDR_P26 | PIO_IDR_P27 | PIO_IDR_P28 | PIO_IDR_P29 | PIO_IDR_P30 | PIO_IDR_P31)
#define PIO_PDR_PIN_MSK   (PIO_PDR_P26 | PIO_PDR_P27 | PIO_PDR_P28 | PIO_PDR_P29 | PIO_PDR_P30 | PIO_PDR_P31)
#define PIO_PUER_PIN_MSK  (PIO_PUER_P6 | PIO_PUER_P26 | PIO_PUER_P27 | PIO_PUER_P28 | PIO_PUER_P29 | PIO_PUER_P30 | PIO_PUER_P31)
#define PIO_ABCDSR_PIN_MSK  (PIO_ABCDSR_P26 | PIO_ABCDSR_P27 | PIO_ABCDSR_P28 | PIO_ABCDSR_P29 | PIO_ABCDSR_P30 | PIO_ABCDSR_P31)

#endif

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
