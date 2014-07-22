/*----------------------------------------------------------------------------
 *      RL-ARM - FlashFS
 *----------------------------------------------------------------------------
 *      Name:    SERIAL.C
 *      Purpose: Serial Input Output
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2011 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <LPC213x.H>                    /* LPC21xx definitions               */

#define CR     0x0D

/*----------------------------------------------------------------------------
 *       init_serial:  Initialize Serial Interface
 *---------------------------------------------------------------------------*/
void init_serial (void) {
  PINSEL0 = 0x00050000;               /* Enable RxD1 and TxD1                */
  U1LCR = 0x83;                       /* 8 bits, no Parity, 1 Stop bit       */
  U1DLL = 0x04;                       /* 234 kBaud @ 15MHz VPB Clock         */
//U1DLL = 97;                         /* 9600 Baud Rate @ 15MHz VPB Clock    */
  U1LCR = 0x03;                       /* DLAB = 0                            */
}

/*----------------------------------------------------------------------------
 *       sendchar:  Write a character to Serial Port
 *---------------------------------------------------------------------------*/
int sendchar (int ch) {
  if (ch == '\n') {
    while (!(U1LSR & 0x20));
    U1THR = CR;                          /* output CR                        */
  }
  while (!(U1LSR & 0x20));
  return (U1THR = ch);
}

/*----------------------------------------------------------------------------
 *       getkey:  Read a character from Serial Port
 *---------------------------------------------------------------------------*/
int getkey (void) {
  while (!(U1LSR & 0x01));
  return (U1RBR);
}

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
