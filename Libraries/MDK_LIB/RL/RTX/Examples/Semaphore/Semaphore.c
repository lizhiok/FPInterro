/*----------------------------------------------------------------------------
 *      RL-ARM - RTX
 *----------------------------------------------------------------------------
 *      Name:    Semaphore.c
 *      Purpose: RTX example program
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <RTL.h>                      /* RTX kernel functions & defines      */
#include <LPC21xx.H>                  /* LPC21xx definitions                 */
#include <stdio.h>

extern void init_serial (void);       /* Initialize Serial Interface         */

OS_TID tsk1, tsk2;
OS_SEM semaphore1;

/*----------------------------------------------------------------------------
 *    Task 1 - High Priority - Active every 3 ticks
 *---------------------------------------------------------------------------*/
__task void task1 (void) {
  OS_RESULT ret;

  while (1) {
    /* Pass control to other tasks for 3 OS ticks */
    os_dly_wait(3);
    /* Wait 1 ticks for the free semaphore */
    ret = os_sem_wait (semaphore1, 1);
    if (ret != OS_R_TMO) {
      /* If there was no time-out the semaphore was aquired */
      printf ("Task 1\n");
      /* Return a token back to a semaphore */
      os_sem_send (semaphore1);
    }
  }
}

/*----------------------------------------------------------------------------
 *    Task 2 - Low Priority - looks for a free semaphore and uses the resource
 *                            whenever it is available
 *---------------------------------------------------------------------------*/
__task void task2 (void) {
  while (1) {
    /* Wait indefinetly for a free semaphore */
    os_sem_wait (semaphore1, 0xFFFF);
    /* OK, the serial interface is free now, use it. */
    printf ("Task 2 \n");
    /* Return a token back to a semaphore. */
    os_sem_send (semaphore1);
  }
}

/*----------------------------------------------------------------------------
 *    Task 3 'init' 
 *---------------------------------------------------------------------------*/
__task void init (void) {
  /* Initialize the UART */
  init_serial ();
  /* Initialize the Semaphore before the first use */
  os_sem_init (semaphore1, 1);
  /* Create an instance of task1 with priority 10 */
  tsk1 = os_tsk_create (task1, 10);
  /* Create an instance of task2 with default priority 1 */
  tsk2 = os_tsk_create (task2, 0);
  /* Delete the init task */  
  os_tsk_delete_self ();
}

/*----------------------------------------------------------------------------
 *    Main: Initialize and start RTX Kernel
 *---------------------------------------------------------------------------*/
int main (void) {
  /* Initialize RTX and start init   */
  os_sys_init (init);
}

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/

