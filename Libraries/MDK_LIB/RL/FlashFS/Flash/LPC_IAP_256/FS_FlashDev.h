/***********************************************************************/
/*  This file is part of the ARM Toolchain package                     */
/*  Copyright KEIL ELEKTRONIK GmbH 2003 - 2007                         */
/***********************************************************************/
/*                                                                     */
/*  FlashDev.H:  Device Description for Philips LPC2xxx 256kB Flash    */
/*               using Flash Boot Loader with IAP                      */
/*                                                                     */
/***********************************************************************/

#define FLASH_DEVICE                                 \
  DFB(0x02000, 0x000000),     /* Sector Size  8kB */ \
  DFB(0x02000, 0x002000),     /* Sector Size  8kB */ \
  DFB(0x02000, 0x004000),     /* Sector Size  8kB */ \
  DFB(0x02000, 0x006000),     /* Sector Size  8kB */ \
  DFB(0x02000, 0x008000),     /* Sector Size  8kB */ \
  DFB(0x02000, 0x00A000),     /* Sector Size  8kB */ \
  DFB(0x02000, 0x00C000),     /* Sector Size  8kB */ \
  DFB(0x02000, 0x00E000),     /* Sector Size  8kB */ \
  DFB(0x10000, 0x010000),     /* Sector Size 64kB */ \
  DFB(0x10000, 0x020000),     /* Sector Size 64kB */ \
  DFB(0x02000, 0x030000),     /* Sector Size  8kB */ \
  DFB(0x02000, 0x032000),     /* Sector Size  8kB */ \
  DFB(0x02000, 0x034000),     /* Sector Size  8kB */ \
  DFB(0x02000, 0x036000),     /* Sector Size  8kB */ \
  DFB(0x02000, 0x038000),     /* Sector Size  8kB */ \
  DFB(0x02000, 0x03A000),     /* Sector Size  8kB */ \
  DFB(0x02000, 0x03C000),     /* Sector Size  8kB */ \

#define FL_NSECT 17
