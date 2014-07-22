/*----------------------------------------------------------------------------
 *      RL-ARM - RTX
 *----------------------------------------------------------------------------
 *      Name:    RTX_EX2.C
 *      Purpose: RTX example program
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <RTL.h>                      /* RTX kernel functions & defines      */

OS_TID tsk1;                          /* assigne identification for task 1   */
OS_TID tsk2;                          /* assigne identification for task 2   */
OS_TID tsk3;                          /* assigne identification for task 3   */
OS_TID tsk4;                          /* assigne identification for task 4   */

U16 counter1;                         /* counter for task 1                  */
U16 counter2;                         /* counter for task 2                  */
U16 counter3;                         /* counter for task 3                  */
U16 counter4;                         /* counter for task 4                  */

__task void job1 (void);
__task void job2 (void);
__task void job3 (void);
__task void job4 (void);

/*----------------------------------------------------------------------------
 *   Task 1:  RTX Kernel starts this task with os_sys_init (job1)
 *---------------------------------------------------------------------------*/
__task void job1 (void) {
  os_tsk_prio_self (2);               /* higher priority to preempt job3     */
  tsk1 = os_tsk_self ();              /* get own task identification number  */
  tsk2 = os_tsk_create (job2,2);      /* start task 2                        */
  tsk3 = os_tsk_create (job3,1);      /* start task 3                        */
  tsk4 = os_tsk_create (job4,1);      /* start task 4                        */

  while (1) {                         /* endless loop                        */
    counter1++;                       /* increment counter 1                 */
    os_dly_wait (5);                  /* wait for timeout: 5 ticks           */
  }
}

/*----------------------------------------------------------------------------
 *   Task 2 'job2':  RTX Kernel starts this task with os_tsk_create (job2,2)
 *---------------------------------------------------------------------------*/
__task void job2 (void) {             /* higher priority to preempt job3     */
  while (1) {                         /* endless loop                        */
    counter2++;                       /* increment counter 2                 */
    os_dly_wait (10);                 /* wait for timeout: 10 ticks          */
  }
}

/*----------------------------------------------------------------------------
 *   Task 3 'job3':  RTX Kernel starts this task with os_tsk_create (job3,1)
 *---------------------------------------------------------------------------*/
__task void job3 (void) {
  while (1)  {                        /* endless loop                        */
    counter3++;                       /* increment counter 3                 */
    if (counter3 == 0) {              /* signal overflow of counter 3        */
      os_evt_set (0x0001,tsk4);       /* to task 4                           */
      os_tsk_pass ();
    }
  }
}

/*----------------------------------------------------------------------------
 *   Task 4 'job4':  RTX Kernel starts this task with os_tsk_create (job4,1)
 *---------------------------------------------------------------------------*/
__task void job4 (void) {
  while (1) {                         /* endless loop                        */
    os_evt_wait_or (0x0001, 0xffff);  /* wait for signal event               */
    counter4++;                       /* process overflow from counter 3     */
  }
}

/*----------------------------------------------------------------------------
 *        Main: Initialize and start RTX Kernel
 *---------------------------------------------------------------------------*/
int main (void) {                     /* program execution starts here       */
  os_sys_init (job1);                 /* initialize and start task 1         */
}

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/


