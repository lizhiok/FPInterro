/*----------------------------------------------------------------------------
 *      RL-ARM - RTX
 *----------------------------------------------------------------------------
 *      Name:    RT_TIME.C
 *      Purpose: Delay and interval wait functions
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include "rt_TypeDef.h"
#include "RTX_Config.h"
#include "rt_Task.h"
#include "rt_Time.h"

/*----------------------------------------------------------------------------
 *      Global Variables
 *---------------------------------------------------------------------------*/

/* Free running system tick counter */
U32 os_time;


/*----------------------------------------------------------------------------
 *      Functions
 *---------------------------------------------------------------------------*/


/*--------------------------- os_time_get -----------------------------------*/

U32 os_time_get (void) {
  /* Get system time tick */
  return (os_time);
}


/*--------------------------- os_dly_wait -----------------------------------*/

void os_dly_wait (U16 delay_time) {
  /* Delay task by "delay_time" */
  tsk_lock();
  os_block (delay_time, WAIT_DLY);
  tsk_unlock();
}


/*--------------------------- os_itv_set ------------------------------------*/

void os_itv_set (U16 interval_time) {
  /* Set interval length and define start of first interval */
  os_runtask->interval_time = interval_time;
  os_runtask->delta_time    = interval_time + (U16)os_time;
}


/*--------------------------- os_itv_wait -----------------------------------*/

void os_itv_wait (void) {
  /* Wait for interval end and define start of next one */
  U16 delta;

  tsk_lock();
  delta = os_runtask->delta_time -(U16)os_time;
  os_runtask->delta_time += os_runtask->interval_time;
  if ((delta & 0x8000) == 0) {
    os_block (delta, WAIT_ITV);
  }
  tsk_unlock();
}

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/

