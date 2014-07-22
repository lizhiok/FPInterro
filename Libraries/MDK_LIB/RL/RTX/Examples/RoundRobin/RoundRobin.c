/*----------------------------------------------------------------------------
 *      RL-ARM - RTX
 *----------------------------------------------------------------------------
 *      Name:    RoundRobin.C
 *      Purpose: Demonstration of RoundRobin Task switching
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <RTL.h>                      /* RTX kernel functions & defines      */

/* id1, id2, id3, id4 will contain task identifications at run-time */
OS_TID id1, id2, id3, id4;

U32 counter1;                         /* counter for task 1                  */
U32 counter2;                         /* counter for task 2                  */
U32 counter3;                         /* counter for task 3                  */
U32 counter4;                         /* counter for task 3                  */

/* Function prototypes */
__task void task1 (void);
__task void task2 (void);
__task void task3 (void);
__task void task4 (void);

/*----------------------------------------------------------------------------
 *   Task 1:  RTX Kernel starts this task with os_sys_init (task1)
 *---------------------------------------------------------------------------*/
__task void task1 (void) {
  /* Obtain own system task identification number */
  id1 = os_tsk_self ();
  os_tsk_prio_self (2);

  /* Assign system identification numbers of task2, task3 and task4 */
  id2 = os_tsk_create (task2, 2);
  id3 = os_tsk_create (task3, 4);
  id4 = os_tsk_create (task4, 1);
  while (1)  {                        /* endless loop                        */
    counter1++;                       /* increment counter 1                 */
  }
}
 
/*----------------------------------------------------------------------------
 *   Task 2:  RTX Kernel starts this task with os_tsk_create (task2,2)
 *---------------------------------------------------------------------------*/
__task void task2 (void) {
  while (1)  {                        /* endless loop                        */
    counter2++;                       /* increment counter 2                 */
    if ((counter2 & 0xFFFF) == 0) {
      os_evt_set (0x0004, id3);
    }
  }
}

/*----------------------------------------------------------------------------
 *   Task 3:  RTX Kernel starts this task with os_tsk_create (task3,4)
 *---------------------------------------------------------------------------*/
__task void task3 (void) {
  while (1) {                         /* endless loop                        */
    os_evt_wait_or (0x0004, 0xffff);
    counter3++;                       /* increment counter 3                 */
  }
}
 
/*----------------------------------------------------------------------------
 *   Task 4:  RTX Kernel starts this task with os_tsk_create (task4,1)
 *---------------------------------------------------------------------------*/
__task void task4 (void) {
  /* This task is never executed. After starting it stays ready forever. */
  while (1)  {                        /* endless loop                        */
    counter4++;                       /* increment counter 4                 */
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


