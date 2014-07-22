/*----------------------------------------------------------------------------
 *      RL-ARM - RTX
 *----------------------------------------------------------------------------
 *      Name:    RTX_EX1.C
 *      Purpose: Your First RTX example program
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <RTL.h>                      /* RTX kernel functions & defines      */

/* id1, id2 will contain task identifications at run-time */
OS_TID id1, id2;

/* Forward reference */
__task void task1 (void);
__task void task2 (void);

/*----------------------------------------------------------------------------
 *   Task 1:  RTX Kernel starts this task with os_sys_init (task1)
 *---------------------------------------------------------------------------*/
__task void task1 (void) {
  /* Obtain own system task identification number */
  id1 = os_tsk_self ();
  /* Assign system identification number of task2 to id2 */
  id2 = os_tsk_create (task2, 1);
  for (;;) {    /* do-this */
    /* Indicate to task2 completion of do-this */
    os_evt_set (0x0004, id2);
    /* Wait for completion of do-that (0xffff means no time-out)*/
    os_evt_wait_or (0x0004, 0xffff);
    /* Wait now for 50 ms */
    os_dly_wait (5);
  }
}

/*----------------------------------------------------------------------------
 *   Task 2:  RTX Kernel starts this task with os_tsk_create (task2, 1)
 *---------------------------------------------------------------------------*/
__task void task2 (void) {
  for (;;) {
    /* Wait for completion of do-this (0xffff means no time-out) */
    os_evt_wait_or (0x0004, 0xffff); /* do-that */
    /* Pause for 20 ms until signaling event to task1 */
    os_dly_wait (2);
    /* Indicate to task1 completion of do-that */
    os_evt_set (0x0004, id1);
  }
}

/*----------------------------------------------------------------------------
 *        Main: Initialize and start RTX Kernel
 *---------------------------------------------------------------------------*/
int main (void) {
  os_sys_init (task1);
}

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/


