/***********************************************************************/
/*  This file is part of the ARM Toolchain package                     */
/*  Copyright KEIL ELEKTRONIK GmbH 2003 - 2008                         */
/***********************************************************************/
/*                                                                     */
/*  FlashDev.H:  Device Description for SST39x320x (16-bit Bus)        */
/*                                                                     */
/* Note: Device fragmented to 64K blocks                               */
/***********************************************************************/

#define FLASH_DEVICE                               \
  DFB(0x10000, 0x000000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x010000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x020000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x030000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x040000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x050000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x060000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x070000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x080000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x090000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x0A0000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x0B0000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x0C0000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x0D0000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x0E0000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x0F0000),    /* Block Size 64kB */ \
                                                   \
  DFB(0x10000, 0x100000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x110000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x120000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x130000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x140000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x150000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x160000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x170000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x180000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x190000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x1A0000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x1B0000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x1C0000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x1D0000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x1E0000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x1F0000),    /* Block Size 64kB */ \
                                                   \
  DFB(0x10000, 0x200000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x210000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x220000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x230000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x240000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x250000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x260000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x270000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x280000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x290000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x2A0000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x2B0000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x2C0000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x2D0000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x2E0000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x2F0000),    /* Block Size 64kB */ \
                                                   \
  DFB(0x10000, 0x300000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x310000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x320000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x330000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x340000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x350000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x360000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x370000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x380000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x390000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x3A0000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x3B0000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x3C0000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x3D0000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x3E0000),    /* Block Size 64kB */ \
  DFB(0x10000, 0x3F0000),    /* Block Size 64kB */ \

#define FL_NSECT    64
