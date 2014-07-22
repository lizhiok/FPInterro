/*----------------------------------------------------------------------------
 *      RL-ARM - RTX
 *----------------------------------------------------------------------------
 *      Name:    MEASURE.H
 *      Purpose: Struct type and extern definitions for the MEASURE project
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/


struct clock {                          /* structure of the clock record     */
  unsigned char    hour;                /* hour                              */
  unsigned char     min;                /* minute                            */
  unsigned char     sec;                /* second                            */
  unsigned short   msec;                /* milliseconds                      */
};

struct mrec  {                          /* structure for measurement records */
  struct   clock   time;                /* time of measurement               */
  unsigned long   port0;                /* state of port 0                   */
  unsigned short analog [4];            /* voltage on analog Pins AD0 .. AD3 */
};

/* external functions: */
extern void serial_init (void);         /* initialize serial interface       */
extern void getline (char *line, int n);         /* input line               */
extern int  getkey (void);                       /* input character          */
extern void measure_display (struct mrec disp);  /* display mrec             */
extern void set_time (char *buffer);             /* set current time         */
extern void set_interval (char *buffer);         /* set interval time        */

extern const char ERROR [];             /* ERROR message string              */
extern struct mrec current;             /* current measurements              */
extern unsigned long setinterval;       /* interval setting values           */
extern unsigned long interval;          /* interval counter                  */

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/

