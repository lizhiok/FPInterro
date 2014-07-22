/*----------------------------------------------------------------------------
 *      RL-ARM - FlashFS
 *----------------------------------------------------------------------------
 *      Name:    GETLINE.C
 *      Purpose: Line Edited Character Input
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2011 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <RTL.h>
#include <stdio.h>

#define CNTLQ      0x11
#define CNTLS      0x13
#define DEL        0x7F
#define BACKSPACE  0x08
#define CR         0x0D
#define LF         0x0A
#define ESC        0x1B

extern int getkey (void);

/*----------------------------------------------------------------------------
 *      Line Editor
 *---------------------------------------------------------------------------*/
BOOL getline (char *lp, U32 n) {
  U32 cnt = 0;
  char c;

  do {
    c = getkey ();
    switch (c) {
      case CNTLQ:                          /* ignore Control S/Q             */
      case CNTLS:
        break;
      case BACKSPACE:
      case DEL:
        if (cnt == 0) {
          break;
        }
        cnt--;                             /* decrement count                */
        lp--;                              /* and line pointer               */
        putchar (0x08);                    /* echo backspace                 */
        putchar (' ');
        putchar (0x08);
        fflush (stdout);
        break;
      case ESC:
        *lp = 0;                           /* ESC - stop editing line        */
        return (__FALSE);
      case CR:
        c = LF;
      default:
        putchar (*lp = c);                 /* echo and store character       */
        fflush (stdout);
        lp++;                              /* increment line pointer         */
        cnt++;                             /* and count                      */
        break;
    }
  } while (cnt < n - 1  &&  c != LF);      /* check limit and line feed      */
  *lp = 0;                                 /* mark end of string             */
  return (__TRUE);
}

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
