/*----------------------------------------------------------------------------
 *      RL-ARM - RTX
 *----------------------------------------------------------------------------
 *      Name:    TRAFFIC.C
 *      Purpose: Traffic Light Controller example program
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <stdio.h>                    /* standard I/O .h-file                */
#include <ctype.h>                    /* character functions                 */
#include <string.h>                   /* string and memory functions         */
#include <RTL.h>                      /* RTX kernel functions & defines      */
#include <LPC21xx.H>                  /* LPC21xx definitions                 */

const char menu[] = 
  "\n"
  "+***** TRAFFIC LIGHT CONTROLLER using MDK and RTX kernel *****+\n"
  "| This program is a simple Traffic Light Controller. Between  |\n"
  "| start time and end time the system controls a traffic light |\n"
  "| with pedestrian self-service.  Outside of this time range   |\n"
  "| the yellow caution lamp is blinking.                        |\n"
  "+ command -+ syntax -----+ function --------------------------+\n"
  "| Display  | D           | display times                      |\n"
  "| Time     | T hh:mm:ss  | set clock time                     |\n"
  "| Start    | S hh:mm:ss  | set start time                     |\n"
  "| End      | E hh:mm:ss  | set end time                       |\n"
  "+----------+-------------+------------------------------------+\n";


extern void getline (char *, int);
                                      /* external function:  input line      */
extern void serial_init (void);       /* external function:  init serial UART*/
extern int  getkey (void);            /* external function:  input character */

OS_TID t_command;                     /* assigned task id of task: command   */
OS_TID t_clock;                       /* assigned task id of task: clock     */
OS_TID t_lights;                      /* assigned task id of task: lights    */
OS_TID t_keyread;                     /* assigned task id of task: keyread   */
OS_TID t_getesc;                      /* assigned task id of task: get_esc   */
  
struct time  {                        /* structure of the time record        */
  U8 hour;                            /* hour                                */
  U8 min;                             /* minute                              */
  U8 sec;                             /* second                              */
};

struct time ctime = { 12,  0,  0 };   /* storage for clock time values       */
struct time start = {  7, 30,  0 };   /* storage for start time values       */
struct time end   = { 18, 30,  0 };   /* storage for end   time values       */

#define red     0x010000              /* I/O Pin:  red    lamp output        */
#define yellow  0x020000              /* I/O Pin:  yellow lamp output        */
#define green   0x040000              /* I/O Pin:  green  lamp output        */
#define stop    0x100000              /* I/O Pin:  stop   lamp output        */
#define walk    0x200000              /* I/O Pin:  walk   lamp output        */
#define key     0x004000              /* I/O Pin:  self-service key input    */
#define SET(o,s) {        \
  if (s==0) IOCLR1 = o;   \
  else      IOSET1 = o;   \
};

char cmdline[16];                     /* storage for command input line      */
struct time rtime;                    /* temporary storage for entry time    */
BIT display_time = __FALSE;           /* flag signal cmd state display_time  */
BIT escape;                           /* flag: mark ESCAPE character entered */

#define ESC  0x1B                     /* ESCAPE character code               */

static U64 cmd_stack[800/8];          /* A bigger stack for command task     */

__task void init (void);              /* Function prototypes                 */
__task void clock (void);
__task void get_escape (void);
__task void command (void);
__task void blinking (void);
__task void lights (void);
__task void keyread (void);

/*----------------------------------------------------------------------------
 *        Main: Initialize and start RTX Kernel
 *---------------------------------------------------------------------------*/
int main (void) {                     /* program execution starts here       */
  os_sys_init (init);                 /* init and start with task 'INIT'     */
}

/*----------------------------------------------------------------------------
 *        Task 1 'init': Initialize
 *---------------------------------------------------------------------------*/
__task void init (void) {
  IODIR1 = 0xFF0000;                   /* P1.16..22 defined as Outputs       */
  serial_init ();                      /* initialize the serial interface    */
  t_clock   = os_tsk_create (clock,  0); /* start clock task                 */
                                       /* start command task               */
  t_command = os_tsk_create_user (command,0,&cmd_stack,sizeof(cmd_stack));
  t_lights  = os_tsk_create (lights, 0); /* start lights task                */
  t_keyread = os_tsk_create (keyread,0); /* start keyread task               */
  os_tsk_delete_self ();               /* stop init task (no longer needed)  */
}

/*----------------------------------------------------------------------------
 *        Task 3 'clock'
 *---------------------------------------------------------------------------*/
__task void clock (void) {
  os_itv_set (100);                    /* set wait interval:  1 second       */
  while (1) {                          /* clock is an endless loop           */
    if (++ctime.sec == 60) {           /* calculate the second               */
      ctime.sec = 0;
      if (++ctime.min == 60) {         /* calculate the minute               */
        ctime.min = 0;
        if (++ctime.hour == 24) {      /* calculate the hour                 */
          ctime.hour = 0;
        }
      }
    }
    if (display_time) {                /* if command_status == display_time  */
      os_evt_set (0x0001,t_command);   /* signal to task command time changed*/
    }
    os_itv_wait ();                    /* wait interval (already set to 1s ) */
  }
}

/*----------------------------------------------------------------------------
 *        readtime: convert line input to time values & store in rtime
 *---------------------------------------------------------------------------*/
char readtime (char *buffer) {
  int args;                                    /* number of arguments        */
  int hour,min,sec;

  rtime.sec = 0;                               /* preset second              */
  args = sscanf (buffer, "%d:%d:%d",           /* scan input line for        */
                         &hour, &min, &sec);   /* hour, minute and second    */
  
  if (hour > 23  ||  min > 59  ||              /* check for valid inputs     */
      sec > 59   ||  args < 2        ) {
     printf ("\n*** ERROR: INVALID TIME FORMAT\n");
     return (0);
  }
  rtime.hour = hour;                           /* setting a new time: hour   */
  rtime.min  = min;                            /* setting a new time: min    */
  rtime.sec  = sec;                            /* setting a new time: sec    */
  return (1);
}

/*----------------------------------------------------------------------------
 *        Task 7 'get_escape': check if ESC (escape character) was entered
 *---------------------------------------------------------------------------*/
__task void get_escape (void) {
  while (1)  {                                /* endless loop                */
    if (getkey () == ESC) {                   /* If ESC entered, set flag    */ 
      escape = __TRUE;                        /* 'escape', set event flag of */
      os_evt_set (0x0002, t_command);         /* task 'command' and          */
    }
  }
}

/*----------------------------------------------------------------------------
 *        Task 2 'command': command processor
 *---------------------------------------------------------------------------*/
__task void command (void) {
  U32 i;

  printf (menu);                              /* display command menu        */
  while (1) {                                 /* endless loop                */
    printf ("\nCommand: ");                   /* display prompt              */
    getline (cmdline, sizeof (cmdline));      /* get command line input      */

    for (i = 0; cmdline[i] != 0; i++) {       /* convert to uppercase        */
      cmdline[i] = (char) toupper(cmdline[i]);
    }

    for (i = 0; cmdline[i] == ' '; i++);      /* skip blanks                 */

    switch (cmdline[i]) {                     /* proceed to command function */
      case 'D':                               /* Display Time Command        */
        printf ("Start Time: %02d:%02d:%02d    "
                "End Time: %02d:%02d:%02d\n",
                 start.hour, start.min, start.sec,
                 end.hour,   end.min,   end.sec);
        printf ("                        type ESC to abort\r");

        t_getesc = os_tsk_create (get_escape, 0);/* ESC check in display loop*/
        escape = __FALSE;                     /* clear escape flag           */
        display_time = __TRUE;                /* set display time flag       */
        os_evt_clr (0x0003, t_command);       /* clear pending signals       */

        while (!escape) {                     /* while no ESC entered        */
          printf ("Clock Time: %02d:%02d:%02d\r",   /* display time          */
                   ctime.hour, ctime.min, ctime.sec);
          os_evt_wait_or (0x0003, 0xffff);     /* wait for time change or ESC*/
        }

        os_tsk_delete (t_getesc);             /* terminate 'get_escape' task.*/
        display_time = __FALSE;               /* clear display time flag     */
        printf ("\n\n");
        break;

      case 'T':                               /* Set Time Command            */
        if (readtime (&cmdline[i+1])) {       /* read time input and         */
          ctime.hour = rtime.hour;            /* store in 'ctime'            */
          ctime.min  = rtime.min;
          ctime.sec  = rtime.sec;
        }
        break;

      case 'E':                               /* Set End Time Command        */
        if (readtime (&cmdline[i+1]))  {      /* read time input and         */
          end.hour = rtime.hour;              /* store in 'end'              */
          end.min  = rtime.min;
          end.sec  = rtime.sec;
        }
        break;

      case 'S':                               /* Set Start Time Command      */
        if (readtime (&cmdline[i+1]))  {      /* read time input and         */
          start.hour = rtime.hour;            /* store in 'start'            */
          start.min  = rtime.min;
          start.sec  = rtime.sec;
        }
        break;

      default:                                /* Error Handling              */
        printf (menu);                        /* display command menu        */
        break;
    }   
  }
}

/*----------------------------------------------------------------------------
 *        signalon: check if clock time is between start and end
 *---------------------------------------------------------------------------*/
char signalon (void) {
  if (memcmp (&start, &end, sizeof (struct time)) < 0)  {
    if (memcmp (&start, &ctime, sizeof (struct time)) < 0  &&
        memcmp (&ctime, &end,   sizeof (struct time)) < 0) {
      return (1);
    }
  }
  else { 
    if (memcmp (&end,   &ctime, sizeof (start)) > 0  &&
        memcmp (&ctime, &start, sizeof (start)) > 0) {
      return (1);
    }
  }
  return (0);                                 /* signal off, blinking on     */
}

/*----------------------------------------------------------------------------
 *        Task 4 'blinking': runs if current time is outside start & end time 
 *---------------------------------------------------------------------------*/
__task void blinking (void) {                 /* blink yellow light          */
  SET (red|yellow|green|stop|walk, 0);        /* all lights off              */
  while (1) {                                 /* endless loop                */
    SET (yellow, 1);                          /* yellow light on             */
    os_dly_wait (30);                         /* wait for timeout: 30 ticks  */
    SET (yellow, 0);                          /* yellow light off            */
    os_dly_wait (30);                         /* wait for timeout: 30 ticks  */
    if (signalon ()) {                        /* if blinking time over       */
      os_tsk_create (lights, 0);              /* start lights                */
      os_tsk_delete_self ();                  /* and stop blinking           */
    }
  }
}

/*----------------------------------------------------------------------------
 *      Task 5 'lights': executes if current time is between start & end time
 *---------------------------------------------------------------------------*/
__task void lights (void) {                   /* traffic light operation     */
  SET (red|stop, 1);                          /* red & stop lights on        */
  SET (yellow|green|walk, 0);
  while (1) {                                 /* endless loop                */
    os_dly_wait (50);                         /* wait for timeout: 50 ticks  */
    if (!signalon ()) {                       /* if traffic signal time over */
      os_tsk_create (blinking, 0);            /* start blinking              */
      os_tsk_delete_self ();                  /* stop lights                 */
    }
    SET (yellow, 1);
    os_dly_wait (50);                         /* wait for timeout: 50 ticks  */
    SET (red|yellow, 0);                      /* green light for cars        */
    SET (green, 1);
    os_evt_clr (0x0010, t_lights);
    os_dly_wait (50);                         /* wait for timeout: 50 ticks  */
    os_evt_wait_and (0x0010, 750);            /* wait for event with timeout */
    SET (yellow, 1);                          /* timeout value: 750 ticks    */
    SET (green, 0);
    os_dly_wait (50);                         /* wait for timeout: 50 ticks  */
    SET (red, 1);                             /* red light for cars          */
    SET (yellow, 0);
    os_dly_wait (50);                         /* wait for timeout: 50 ticks  */
    SET (stop, 0);                            /* green light for walkers     */    
    SET (walk, 1);
    os_dly_wait (250);                        /* wait for timeout: 250 ticks */
    SET (stop, 1);                            /* red light for walkers       */        
    SET (walk, 0);
  }
}

/*----------------------------------------------------------------------------
 *        Task 6 'keyread': process key stroke from pedestrian push button
 *---------------------------------------------------------------------------*/
__task void keyread (void) {
  while (1) {                                 /* endless loop                */
    if ((IOPIN0 & key) == 0) {                /* if key pressed              */
      os_evt_set (0x0010, t_lights);          /* send signal to task lights  */
    }
    os_dly_wait (5);                          /* wait for timeout: 5 ticks   */
  }
}

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
