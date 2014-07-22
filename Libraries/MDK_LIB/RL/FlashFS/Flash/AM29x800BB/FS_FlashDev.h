/***********************************************************************/
/*  This file is part of the ARM Toolchain package                     */
/*  Copyright KEIL ELEKTRONIK GmbH 2003 - 2007                         */
/***********************************************************************/
/*                                                                     */
/*  FlashDev.H:  Device Description for AM29x800BB (16-bit Bus)        */
/*                                                                     */
/***********************************************************************/

#define FLASH_DEVICE                                 \
  DFB(0x04000, 0x000000),     /* Sector Size 16kB */ \
  DFB(0x08000, 0x004000),     /* Sector Size 32kB */ \
  DFB(0x02000, 0x00C000),     /* Sector Size  8kB */ \
  DFB(0x02000, 0x00E000),     /* Sector Size  8kB */ \
  DFB(0x02000, 0x010000),     /* Sector Size  8kB */ \
  DFB(0x02000, 0x012000),     /* Sector Size  8kB */ \
  DFB(0x08000, 0x014000),     /* Sector Size 32kB */ \
  DFB(0x04000, 0x01C000),     /* Sector Size 16kB */ \
  DFB(0x10000, 0x020000),     /* Sector Size 64kB */ \
  DFB(0x10000, 0x030000),     /* Sector Size 64kB */ \
  DFB(0x10000, 0x040000),     /* Sector Size 64kB */ \
  DFB(0x10000, 0x050000),     /* Sector Size 64kB */ \
  DFB(0x10000, 0x060000),     /* Sector Size 64kB */ \
  DFB(0x10000, 0x070000),     /* Sector Size 64kB */ \
  DFB(0x10000, 0x080000),     /* Sector Size 64kB */ \
  DFB(0x10000, 0x090000),     /* Sector Size 64kB */ \
  DFB(0x10000, 0x0A0000),     /* Sector Size 64kB */ \
  DFB(0x10000, 0x0B0000),     /* Sector Size 64kB */ \
  DFB(0x10000, 0x0C0000),     /* Sector Size 64kB */ \
  DFB(0x10000, 0x0D0000),     /* Sector Size 64kB */ \
  DFB(0x10000, 0x0E0000),     /* Sector Size 64kB */ \
  DFB(0x10000, 0x0F0000),     /* Sector Size 64kB */ \

#define FL_NSECT    22
