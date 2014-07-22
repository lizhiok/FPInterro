/*----------------------------------------------------------------------------
 *      RL-ARM - RTX
 *----------------------------------------------------------------------------
 *      Name:    RTX_SWI.C
 *      Purpose: RTX example program
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <RTL.h>                      /* RTX kernel functions & defines      */

OS_TID tsk1;
OS_TID tsk2;

U32 counter;                          /* 5-bit counter for tasks             */

__task void job1 (void);
__task void job2 (void);


/*----------------------------------------------------------------------------
 * Software Interrupt Functions accept parameters, may return values
 * and run in Supervisor Mode (Interrupt protected)
 *---------------------------------------------------------------------------*/

void __swi(8)  inc_5bit (U32 *cp);
void __SWI_8            (U32 *cp) {
  /* A protected function to increment a 5-bit counter. */
  *cp = (*cp + 1) & 0x1F;
}


void __swi(9)  add_5bit (U32 *cp, U32 val);
void __SWI_9            (U32 *cp, U32 val) {
  /* A protected function to add a value to 5-bit counter. */
  *cp = (*cp + val) & 0x1F;
}


/*----------------------------------------------------------------------------
 *   Task 1
 *---------------------------------------------------------------------------*/
__task void job1 (void) {
  tsk1 = os_tsk_self ();
  tsk2 = os_tsk_create (job2,2);      /* start task 2                        */

  while (1) {
    inc_5bit (&counter);              /* increment 5-bit counter             */
  }
}

/*----------------------------------------------------------------------------
 *   Task 2
 *---------------------------------------------------------------------------*/
__task void job2 (void) {
  while (1) {
    add_5bit (&counter, 7);           /* add 7 to 5-bit counter              */
    os_dly_wait (1);
  }
}

/*----------------------------------------------------------------------------
 *   Initialize and start RTX Kernel
 *---------------------------------------------------------------------------*/
int main (void) {
  os_sys_init (job1);
}

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/


