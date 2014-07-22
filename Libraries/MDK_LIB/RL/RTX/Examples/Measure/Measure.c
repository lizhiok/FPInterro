/*----------------------------------------------------------------------------
 *      RL-ARM - RTX
 *----------------------------------------------------------------------------
 *      Name:    MEASURE.C
 *      Purpose: Remote Measurement Recorder example program
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <RTL.h>                        /* RTX kernel functions & defines    */
#include <stdio.h>                      /* standard I/O .h-file              */
#include <stdlib.h>                     /* standard library .h-file          */
#include <ctype.h>                      /* character functions               */
#include "measure.h"                    /* global project definition file    */
#include <LPC21xx.H>                    /* LPC21xx definitions               */

static const char menu[] = 
   "\n"
   "+******* REMOTE MEASUREMENT RECORDER using MDK and RTX ********+\n"
   "| This program is a simple Measurement Recorder. It is based   |\n"
   "| on the LPC21xx CPU and records the state of Port 0 and the   |\n"
   "| voltage on the four analog inputs AIN0 through AIN3.         |\n"
   "+ command -+ syntax -----+ function ---------------------------+\n"
   "| Read     | R [n]       | read <n> recorded measurements      |\n"
   "| Display  | D           | display current measurement values  |\n"
   "| Time     | T hh:mm:ss  | set time                            |\n"
   "| Interval | I mm:ss.ttt | set interval time                   |\n"
   "| Clear    | C           | clear measurement records           |\n"
   "| Quit     | Q           | quit measurement recording          |\n"
   "| Start    | S           | start measurement recording         |\n"
   "+----------+-------------+-------------------------------------+\n";


OS_TID t_command;                          /* assigned id of task: command   */
OS_TID t_clock;                            /* assigned id of task: clock     */
OS_TID t_measure;                          /* assigned id of task: lights    */
OS_TID t_interval;                         /* assigned id of task: keyread   */
OS_TID t_getesc;                           /* assigned id of task: get_esc   */

unsigned long setinterval;                 /* interval setting values        */
unsigned long interval;                    /* interval counter               */

static char startflag;                     /* start measurement recording    */

struct mrec current;                       /* current measurements           */

#define RAM    8192                        /* size of DATA RAM is 8 KBytes   */
#define SCNT  (RAM / sizeof (current))     /* number of records in DATA RAM  */

static struct mrec save_record [SCNT];     /* buffer for measurements        */
static int sindex;                         /* save index                     */
static int savefirst;                      /* save first index               */
static char escape;                        /* flag: mark ESCAPE character    */
#define ESC  0x1B                          /* ESCAPE character code          */

const char ERROR [] = "\n*** ERROR: %s\n"; /* ERROR message string in code   */

#define WRONGINDEX 0xffff                  /* error signal for wrong index   */

static U64 cmd_stack[800/8];               /* A bigger stack for command_task*/
                                           /* Must be 8-byte aligned!        */

__task void init_task (void);              /* Task Function prototypes       */
__task void clock_task (void);
__task void get_escape (void);
__task void command_task (void);
__task void measure_task (void);
__task void interval_task (void);

/*----------------------------------------------------------------------------
 *               Save current measurements in save_record
 *---------------------------------------------------------------------------*/
static void save_measurements (void) {
  save_record[sindex++] = current;            /* copy current measurements   */
  if (sindex == SCNT) sindex = 0;             /* check bounds of sindex      */
  if (sindex == savefirst) {                  /* check circular buffer limits*/
    if (++savefirst == SCNT)  savefirst = 0;  /* check bounds of savefirst   */
  }
}

/*----------------------------------------------------------------------------
 *                       Calculate first Read Index
 *---------------------------------------------------------------------------*/
static unsigned int read_index (char *buffer) {
  int index = 0;
  int args;

  if (setinterval < 1000 && startflag)  {     /* check if setinterval is     */
    printf (ERROR, "QUIT MEASUREMENTS BEFORE READ");   /* below 1 second     */
    return (WRONGINDEX);                      /* no display on the fly if    */
  }                                           /* interval time < 1 second    */

  args = sscanf (buffer, "%d", &index);       /* scan input for read count   */
  if (args == 0  ||  index == 0  || args == EOF)  index = SCNT-1;
  index = sindex - index;                     /* calculate first read index  */
  if (index < 0) index += SCNT;               /* from read count             */
  return (index);
}
  
/*----------------------------------------------------------------------------
 *                         Clear Measurement Records
 *---------------------------------------------------------------------------*/
static void clear_records (void) {
  int idx;                                    /* index for loop              */

  startflag = 0;                              /* stop measurement collecting */
  sindex = savefirst = 0;                     /* reset circular buffer index */
  for (idx = 0; idx != SCNT; idx++) {         /* mark all records unused     */
    save_record[idx].time.hour = 0xff;        /* unused flag: hour = 0xff    */
  }     
}

/*----------------------------------------------------------------------------
 *        Task 'clock'
 *---------------------------------------------------------------------------*/
__task void clock_task (void) {
  os_itv_set (1);                             /* set wait interval:  1 ms    */
  while (1) {                                 /* clock is an endless loop    */
    if (++current.time.msec == 1000) {        /* update millisecond cnt      */
      current.time.msec = 0;

      if (++current.time.sec == 60) {         /* calculate the second        */
        current.time.sec = 0;

        if (++current.time.min == 60) {       /* calculate the minute        */
          current.time.min = 0;

          if (++current.time.hour == 24) {    /* calculate the hour          */
            current.time.hour = 0;
          }
        }
      }
    }
    os_itv_wait ();                           /* wait interval 1ms           */
  }
}

/*----------------------------------------------------------------------------
 *        Task 'measure'
 *---------------------------------------------------------------------------*/
__task void measure_task (void) {
  unsigned int val;
  unsigned int crval;
  int i;

  while (1) {
    os_evt_wait_or (0x0001, 0xffff);
    current.port0 = IOPIN0;                   /* read port 0                 */
    for (i = 0; i != 4; i++) {                /* loop for 4 A/D inputs       */
      crval = 0x01000000 | (1<<i);
      ADCR |= crval;                          /* Start A/D Conversion        */
      do {
        val = ADDR;                           /* Read A/D Data Register      */
      } while ((val & 0x80000000) == 0);      /* Wait for end of A/D Convers.*/
      ADCR &= ~crval;                         /* Stop A/D Conversion         */
      val = (val >> 6) & 0x03FF;              /* Extract AIN Value           */
      current.analog[i] = val;                /* result of A/D process       */ 
    }
    os_evt_set (0x0001, t_interval);
    os_evt_set (0x0001, t_command);
  }
}

/*----------------------------------------------------------------------------
 *        Task 'interval'
 *---------------------------------------------------------------------------*/
__task void interval_task (void) {
  int dly;

  while (1) {                                 /* interval is an endless loop */
    if (!startflag) {
      os_evt_wait_or (0x0010, 0xffff);        /* wait for signal to start    */
      startflag = 1;                          /* measurement recording       */
    }
           
    if (interval == 0) {    
      interval = setinterval;
      os_evt_set (0x0001, t_measure);         /* signal 'measure' task to    */
      os_evt_wait_or (0x0001, 0xffff);        /* take measurements, wait for */
      save_measurements ();                   /* results and save them       */
    }
    if (interval > 100) dly = 100;
    else                dly = interval;
    interval -= dly;
    os_dly_wait (dly);
  }    
}

/*----------------------------------------------------------------------------
 *        Task 'get_escape': check if ESC (escape character) was entered
 *---------------------------------------------------------------------------*/
__task void get_escape (void) {
  while (1) {                                 /* endless loop                */
    if (getkey () == ESC) {                   /* If ESC entered, set flag    */ 
      escape = 1;                             /* 'escape', set event flag of */
      os_evt_set (0x0002, t_command);         /* task 'command' and          */
    }
  }
}

/*----------------------------------------------------------------------------
 *        Task 'command': command processor
 *---------------------------------------------------------------------------*/
__task void command_task (void) {
  char cmdbuf [15];                           /* command input buffer        */
  int i;                                      /* index for command buffer    */
  int idx;                                    /* index for circular buffer   */

  clear_records ();                           /* initialize circular buffer  */
  printf (menu);                              /* display command menu        */
  while (1) {                                 /* loop forever                */
    printf ("\nCommand: ");                    
    getline (&cmdbuf[0], sizeof (cmdbuf));    /* input command line          */

    for (i = 0; cmdbuf[i] != 0; i++)  {       /* convert to upper characters */
      cmdbuf[i] = (char) toupper(cmdbuf[i]);
    }

    for (i = 0; cmdbuf[i] == ' '; i++) { ; }  /* skip blanks                 */

    switch (cmdbuf[i])  {                     /* proceed to command function */

      case 'R':                               /* Read circular Buffer        */
        if ((idx = read_index (&cmdbuf[i+1])) == WRONGINDEX)  break;

        t_getesc = os_tsk_create (get_escape,1);/* ESC check in display loop */
        escape = 0;                           /* clear escape flag           */

        while (idx != sindex)  {              /* check end of table          */
          if (escape) break;                  /* check if ESC key pressed    */  
          if (save_record[idx].time.hour != 0xff)  {
            measure_display (save_record[idx]);  /* display record           */
            printf ("\n");
          }
          if (++idx == SCNT) idx = 0;         /* next circular buffer entry  */
        }
        os_tsk_delete (t_getesc);             /* terminate 'get_escape' task.*/
        break;

      case 'T':                               /* Enter Current Time          */
        set_time (&cmdbuf[i+1]);
        break;

      case 'I':                               /* Enter Interval Time         */
        set_interval (&cmdbuf[i+1]);
        break;

      case 'D':                               /* Display Command             */
        printf ("\nDisplay current Measurements: (ESC to abort)\n");

        t_getesc = os_tsk_create (get_escape, 1);/* ESC check in display loop*/
        escape = 0;                           /* clear escape flag           */
        os_evt_clr (0x0003, t_command);       /* clear pending signals       */

        while (!escape)  {                    /* while no ESC entered        */
          os_evt_set (0x0001, t_measure);
          os_evt_wait_or (0x0003,0xffff);     /* wait for time change or ESC */
          measure_display (current);          /* display values              */
        }

        os_tsk_delete (t_getesc);             /* terminate 'get_escape' task.*/
        printf ("\n\n");
        break;

      case 'S':                               /* Start Command               */
        printf ("\nStart Measurement Recording\n");
        os_evt_set (0x0010, t_interval);      /* signal interval task to     */ 
        break;                                /* start measurement recording */

      case 'Q':                               /* Quit Command                */
        printf ("\nQuit Measurement Recording\n");
        startflag = 0;
        break;

      case 'C':                               /* Clear Command               */
        printf ("\nClear Measurement Records\n");
        clear_records ();
        break;

      default:                                /* Error Handling              */
        printf (ERROR, "UNKNOWN COMMAND");
        printf (menu);                        /* display command menu        */
        break;
    }
  }
}

/*----------------------------------------------------------------------------
 *        Task 'init': Initialize
 *---------------------------------------------------------------------------*/
__task void init_task (void) {

  PINSEL1 = 0x15400000;                       /* Select AIN0..AIN3           */
  IODIR1  = 0xFF0000;                         /* P1.16..23 defined as Outputs*/
  ADCR    = 0x002E0401;                       /* Setup A/D: 10-bit @ 3MHz    */

  serial_init ();                             /* init. the serial interface  */
  t_clock   = os_tsk_create (clock_task, 3);  /* start clock task            */
                                              /* start command task          */
  t_command = os_tsk_create_user (command_task, 1, &cmd_stack, sizeof(cmd_stack));
  t_measure = os_tsk_create (measure_task, 2);/* start measure task          */
  t_interval= os_tsk_create (interval_task,2);/* start interval task         */
  setinterval = 100;
  os_tsk_delete_self ();                      /* stop init task (done)       */
}

/*----------------------------------------------------------------------------
 *        Main: Initialize and start RTX Kernel
 *---------------------------------------------------------------------------*/
int main (void) {                             /* program exec. starts here   */
  os_sys_init_prio (init_task,50);            /* init and start task 'init'  */
}

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
