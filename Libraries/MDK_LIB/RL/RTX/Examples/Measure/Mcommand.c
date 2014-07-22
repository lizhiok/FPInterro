/*----------------------------------------------------------------------------
 *      RL-ARM - RTX
 *----------------------------------------------------------------------------
 *      Name:    MCOMMAND.C
 *      Purpose: Time Set Commands for the Remote Measurement Recorder
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <stdio.h>                      /* standard ANSI I/O .h-file         */
#include <RTL.h>
#include "measure.h"                    /* global project definition file    */


/*----------------------------------------------------------------------------
 *                           Display measurements
 *---------------------------------------------------------------------------*/
void measure_display (struct mrec display) {    
  int i;                                /* index count for AIN0 - AIN3       */

  printf ("\rTime: %2d:%02d:%02d.%03d  P0:%08X",
           display.time.hour,           /* display hour                      */
           display.time.min,            /* display minute                    */
           display.time.sec,            /* display second                    */
           display.time.msec,           /* display millisecond               */
           (int)display.port0);         /* display port 0 value              */

  for (i = 0; i < 4; i++) {             /* display AN0 through AN3           */
    printf (" AIN%d:%4.2fV", i, (float)display.analog[i] * 3.3f / 1024.0f);
  }
}

/*----------------------------------------------------------------------------
 *                           Set Current Time
 *---------------------------------------------------------------------------*/
void set_time (char *buffer) {
  int hour, min, sec;                   /* temporary time values             */
  int args;                             /* number of arguments               */

  args = sscanf (buffer, "%d:%d:%d",    /* scan input line for               */
                 &hour, &min, &sec);    /* hour, minute and second           */
  if (hour > 23  ||  min > 59  ||       /* check for valid inputs            */
      sec > 59   ||  args < 2  ||  args == EOF)  {
    printf (ERROR, "INVALID TIME FORMAT");
  }
  else {                                /* if inputs valid then              */
    os_tsk_prio_self (5);               /* raise the priority above 'CLOCK'  */
    current.time.hour = hour;           /* setting the new time: hours       */
    current.time.min  = min;            /* setting the new time: minutes     */
    current.time.sec  = sec;            /* setting the new time: seconds     */
    current.time.msec = 0;              /* setting the new time: miliseconds */
    interval = 0;                       /* force new interval                */
    os_tsk_prio_self (1);               /* enable task preemption again      */
  }
}
  
/*----------------------------------------------------------------------------
 *                            Set Interval Time
 *---------------------------------------------------------------------------*/
void set_interval (char * buffer) {
  int min;                              /* temporary interval record         */
  int args;                             /* number of arguments               */
  unsigned long itime;
  float second;                         /* float sec for ss.mmm format       */

  args = sscanf (buffer, "%d:%f",       /* scan input line for               */
                 &min, &second);        /* minute, second and milliseconds   */
                                        /* check valid inputs                */
  if (second >= 60.0 ||  args < 2  || args == EOF)  {
    printf (ERROR, "INVALID INTERVAL FORMAT");
  }
  else {                                /* if inputs are valid then          */
    second += (float)min*60;            /* calculate second and millisecond  */
    itime = second * 1000;
    itime--;                            /* correct the interval time         */
    os_tsk_prio_self (5);               /* raise task priority               */
    setinterval = itime;                /* of the new setinterval time       */
    interval = 0;                       /* force new interval                */
    os_tsk_prio_self (1);               /* enable task preemption again      */
  }
}

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
