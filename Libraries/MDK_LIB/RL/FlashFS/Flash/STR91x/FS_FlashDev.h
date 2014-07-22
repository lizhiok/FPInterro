/***********************************************************************/
/*  This file is part of the ARM Toolchain package                     */
/*  Copyright KEIL ELEKTRONIK GmbH 2003 - 2008                         */
/***********************************************************************/
/*                                                                     */
/*  FlashDev.C:  Device Description for ST Microelectronics STR91X     */
/*                   Flash Bank 1     Base address= 0x00400000         */
/*                                                                     */
/***********************************************************************/

#define FLASH_DEVICE                                  \
  DFB(0x002000, 0x000000),     /* Sector Size  8kB */ \
  DFB(0x002000, 0x002000),     /* Sector Size  8kB */ \
  DFB(0x002000, 0x004000),     /* Sector Size  8kB */ \
  DFB(0x002000, 0x006000),     /* Sector Size  8kB */ \

#define FL_NSECT 4
