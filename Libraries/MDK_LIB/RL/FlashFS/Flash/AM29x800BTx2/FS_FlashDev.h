/***********************************************************************/
/*  This file is part of the ARM Toolchain package                     */
/*  Copyright KEIL ELEKTRONIK GmbH 2003 - 2004                         */
/***********************************************************************/
/*                                                                     */
/*  FlashDev.H:  Device Description for Dual AM29x800BT (32-bit Bus)   */
/*                                                                     */
/***********************************************************************/

#define FLASH_DEVICE                                     \
  DFB(0x20000, 0x000000),    /* Sector Size Dual 64kB */ \
  DFB(0x20000, 0x020000),    /* Sector Size Dual 64kB */ \
  DFB(0x20000, 0x040000),    /* Sector Size Dual 64kB */ \
  DFB(0x20000, 0x060000),    /* Sector Size Dual 64kB */ \
  DFB(0x20000, 0x080000),    /* Sector Size Dual 64kB */ \
  DFB(0x20000, 0x0A0000),    /* Sector Size Dual 64kB */ \
  DFB(0x20000, 0x0C0000),    /* Sector Size Dual 64kB */ \
  DFB(0x20000, 0x0E0000),    /* Sector Size Dual 64kB */ \
  DFB(0x20000, 0x100000),    /* Sector Size Dual 64kB */ \
  DFB(0x20000, 0x120000),    /* Sector Size Dual 64kB */ \
  DFB(0x20000, 0x140000),    /* Sector Size Dual 64kB */ \
  DFB(0x20000, 0x160000),    /* Sector Size Dual 64kB */ \
  DFB(0x20000, 0x180000),    /* Sector Size Dual 64kB */ \
  DFB(0x20000, 0x1A0000),    /* Sector Size Dual 64kB */ \
  DFB(0x08000, 0x1C0000),    /* Sector Size Dual 16kB */ \
  DFB(0x10000, 0x1C8000),    /* Sector Size Dual 32kB */ \
  DFB(0x04000, 0x1D8000),    /* Sector Size Dual  8kB */ \
  DFB(0x04000, 0x1DC000),    /* Sector Size Dual  8kB */ \
  DFB(0x04000, 0x1E0000),    /* Sector Size Dual  8kB */ \
  DFB(0x04000, 0x1E4000),    /* Sector Size Dual  8kB */ \
  DFB(0x10000, 0x1E8000),    /* Sector Size Dual 32kB */ \
  DFB(0x08000, 0x1F8000),    /* Sector Size Dual 16kB */ \

#define FL_NSECT    22
