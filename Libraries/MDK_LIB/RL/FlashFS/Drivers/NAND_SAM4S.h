/*----------------------------------------------------------------------------
 *      RL-ARM - FlashFS
 *----------------------------------------------------------------------------
 *      Name:    NAND_SAM4S.h 
 *      Purpose: NAND Flash Interface Driver for Atmel ATSAM4S Defs
 *      Rev.:    V4.60
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2012 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#ifndef __NAND_SAM4S_H_
#define __NAND_SAM4S_H_

/*----------------------------------------------------------------------------
 *      SAM4S Peripheral Write Protection Keys
 *---------------------------------------------------------------------------*/

/* PMC Write Protect Mode Keys */
#define PMC_WPEN_KEY        0x504D4301
#define PMC_WPDIS_KEY       0x504D4300

/* PIO Write Protect Mode Keys */
#define PIO_WPEN_KEY        0x50494F01
#define PIO_WPDIS_KEY       0x50494F00

/* SMC Write Protection Control Keys */
#define SMC_WPEN_KEY        0x534D4301
#define SMC_WPDIS_KEY       0x534D4300


/*----------------------------------------------------------------------------
 *      SAM4S SMC I/O pin masks
 *---------------------------------------------------------------------------*/
#define SMC_PIO_PER_PIN_MSK    (PIO_PER_P0 | PIO_PER_P1  | PIO_PER_P2  | PIO_PER_P3 | \
                                PIO_PER_P4 | PIO_PER_P5  | PIO_PER_P6  | PIO_PER_P7 | \
                                PIO_PER_P9 | PIO_PER_P10 | PIO_PER_P16 | PIO_PER_P17)

#define SMC_PIO_PDR_PIN_MSK    (PIO_PDR_P0 | PIO_PDR_P1  | PIO_PDR_P2  | PIO_PDR_P3 | \
                                PIO_PDR_P4 | PIO_PDR_P5  | PIO_PDR_P6  | PIO_PDR_P7 | \
                                PIO_PDR_P9 | PIO_PDR_P10 | PIO_PDR_P16 | PIO_PDR_P17)

#define SMC_PIO_IDR_PIN_MSK    (PIO_IDR_P0 | PIO_IDR_P1  | PIO_IDR_P2  | PIO_IDR_P3 | \
                                PIO_IDR_P4 | PIO_IDR_P5  | PIO_IDR_P6  | PIO_IDR_P7 | \
                                PIO_IDR_P9 | PIO_IDR_P10 | PIO_IDR_P16 | PIO_IDR_P17)
//#define SMC_PIO_PUER_PIN_MSK   ()
#define SMC_PIO_ABCDSR_PIN_MSK (PIO_ABCDSR_P0 | PIO_ABCDSR_P1  | PIO_ABCDSR_P2  | PIO_ABCDSR_P3 | \
                                PIO_ABCDSR_P4 | PIO_ABCDSR_P5  | PIO_ABCDSR_P6  | PIO_ABCDSR_P7 | \
                                PIO_ABCDSR_P9 | PIO_ABCDSR_P10 | PIO_ABCDSR_P16 | PIO_ABCDSR_P17)

/*----------------------------------------------------------------------------
 *      SAM4S NAND Driver Defines
 *---------------------------------------------------------------------------*/

/* Hardware timeout */
#define NAND_TIMEOUT        150000     /* ~10ms @ 100MHz should be enough    */

/* Bus width */
#define NAND_BUS_W8         0x00
#define NAND_BUS_W16        0x01

/* Ready/Busy */
#define NAND_BUSY           0x00
#define NAND_READY          0x01

/* Flag Set/Cleared */
#define NAND_FLAG_CLR       0x00
#define NAND_FLAG_SET       0x01
#define NAND_FLAG_TOUT      0x02

#endif /* __NAND_SAM4S_H_ */

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
