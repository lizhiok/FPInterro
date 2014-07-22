/***********************************************************************/
/*  This file is part of the ARM Toolchain package                     */
/*  Copyright KEIL ELEKTRONIK GmbH 2003 - 2008                         */
/***********************************************************************/
/*                                                                     */
/*  FlashDev.H:  Device Description for Atmel 32MBit Serial Flash      */
/*                                            Memory (AT45DB642D)      */
/*                                                                     */
/* Note: Device fragmented to 132K virtual sectors for the FlashFS.    */
/***********************************************************************/

#define SPI_FLASH_DEVICE                                 \
  DSB(0x21000, 0x000000),     /* Sector Size 132kB  0 */ \
  DSB(0x21000, 0x021000),     /* Sector Size 132kB  1 */ \
  DSB(0x21000, 0x042000),     /* Sector Size 132kB  2 */ \
  DSB(0x21000, 0x063000),     /* Sector Size 132kB  3 */ \
  DSB(0x21000, 0x084000),     /* Sector Size 132kB  4 */ \
  DSB(0x21000, 0x0A5000),     /* Sector Size 132kB  5 */ \
  DSB(0x21000, 0x0C6000),     /* Sector Size 132kB  6 */ \
  DSB(0x21000, 0x0E7000),     /* Sector Size 132kB  7 */ \
  DSB(0x21000, 0x108000),     /* Sector Size 132kB  8 */ \
  DSB(0x21000, 0x129000),     /* Sector Size 132kB  9 */ \
  DSB(0x21000, 0x14A000),     /* Sector Size 132kB 10 */ \
  DSB(0x21000, 0x16B000),     /* Sector Size 132kB 11 */ \
  DSB(0x21000, 0x18C000),     /* Sector Size 132kB 12 */ \
  DSB(0x21000, 0x1AD000),     /* Sector Size 132kB 13 */ \
  DSB(0x21000, 0x1CE000),     /* Sector Size 132kB 14 */ \
  DSB(0x21000, 0x1EF000),     /* Sector Size 132kB 15 */ \
  DSB(0x21000, 0x210000),     /* Sector Size 132kB 16 */ \
  DSB(0x21000, 0x231000),     /* Sector Size 132kB 17 */ \
  DSB(0x21000, 0x252000),     /* Sector Size 132kB 18 */ \
  DSB(0x21000, 0x273000),     /* Sector Size 132kB 19 */ \
  DSB(0x21000, 0x294000),     /* Sector Size 132kB 20 */ \
  DSB(0x21000, 0x2B5000),     /* Sector Size 132kB 21 */ \
  DSB(0x21000, 0x2D6000),     /* Sector Size 132kB 22 */ \
  DSB(0x21000, 0x2F7000),     /* Sector Size 132kB 23 */ \
  DSB(0x21000, 0x318000),     /* Sector Size 132kB 24 */ \
  DSB(0x21000, 0x339000),     /* Sector Size 132kB 25 */ \
  DSB(0x21000, 0x35A000),     /* Sector Size 132kB 26 */ \
  DSB(0x21000, 0x37B000),     /* Sector Size 132kB 27 */ \
  DSB(0x21000, 0x39C000),     /* Sector Size 132kB 28 */ \
  DSB(0x21000, 0x3BD000),     /* Sector Size 132kB 29 */ \
  DSB(0x21000, 0x3DE000),     /* Sector Size 132kB 30 */ \
  DSB(0x21000, 0x3FF000),     /* Sector Size 132kB 31 */ \
  DSB(0x21000, 0x420000),     /* Sector Size 132kB 32 */ \
  DSB(0x21000, 0x441000),     /* Sector Size 132kB 33 */ \
  DSB(0x21000, 0x462000),     /* Sector Size 132kB 34 */ \
  DSB(0x21000, 0x483000),     /* Sector Size 132kB 35 */ \
  DSB(0x21000, 0x4A4000),     /* Sector Size 132kB 36 */ \
  DSB(0x21000, 0x4C5000),     /* Sector Size 132kB 37 */ \
  DSB(0x21000, 0x4E6000),     /* Sector Size 132kB 38 */ \
  DSB(0x21000, 0x507000),     /* Sector Size 132kB 39 */ \
  DSB(0x21000, 0x528000),     /* Sector Size 132kB 40 */ \
  DSB(0x21000, 0x549000),     /* Sector Size 132kB 41 */ \
  DSB(0x21000, 0x56A000),     /* Sector Size 132kB 42 */ \
  DSB(0x21000, 0x58B000),     /* Sector Size 132kB 43 */ \
  DSB(0x21000, 0x5AC000),     /* Sector Size 132kB 44 */ \
  DSB(0x21000, 0x5CD000),     /* Sector Size 132kB 45 */ \
  DSB(0x21000, 0x5EE000),     /* Sector Size 132kB 46 */ \
  DSB(0x21000, 0x60F000),     /* Sector Size 132kB 47 */ \
  DSB(0x21000, 0x630000),     /* Sector Size 132kB 48 */ \
  DSB(0x21000, 0x651000),     /* Sector Size 132kB 49 */ \
  DSB(0x21000, 0x672000),     /* Sector Size 132kB 50 */ \
  DSB(0x21000, 0x693000),     /* Sector Size 132kB 51 */ \
  DSB(0x21000, 0x6B4000),     /* Sector Size 132kB 52 */ \
  DSB(0x21000, 0x6D5000),     /* Sector Size 132kB 53 */ \
  DSB(0x21000, 0x6F6000),     /* Sector Size 132kB 54 */ \
  DSB(0x21000, 0x717000),     /* Sector Size 132kB 55 */ \
  DSB(0x21000, 0x738000),     /* Sector Size 132kB 56 */ \
  DSB(0x21000, 0x759000),     /* Sector Size 132kB 57 */ \
  DSB(0x21000, 0x77A000),     /* Sector Size 132kB 58 */ \
  DSB(0x21000, 0x79B000),     /* Sector Size 132kB 59 */ \
  DSB(0x21000, 0x7BC000),     /* Sector Size 132kB 60 */ \
  DSB(0x21000, 0x7DD000),     /* Sector Size 132kB 61 */ \
  DSB(0x21000, 0x7FE000),     /* Sector Size 132kB 62 */ \
  DSB(0x21000, 0x81F000),     /* Sector Size 132kB 63 */ \

#define SF_NSECT    64
