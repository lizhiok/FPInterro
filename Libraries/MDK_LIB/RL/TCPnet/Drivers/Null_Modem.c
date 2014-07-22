/*----------------------------------------------------------------------------
 *      RL-ARM - TCPnet
 *----------------------------------------------------------------------------
 *      Name:    NULL_MODEM.C
 *      Purpose: Standard Null-Modem Driver
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include "Net_Config.h"

/* Net_Config.c */
extern SYS_CFG  sys_config;
#define TSEC    sys_config.TickRate

#define MODEM_IDLE      0
#define MODEM_ERROR     1
#define MODEM_READY     2
#define MODEM_LISTEN    3
#define MODEM_ONLINE    4
#define MODEM_DIAL      5
#define MODEM_HANGUP    6

/* Local variables */
static U8  mbuf[32];                /* Modem Command buffer                  */
static U8  mlen;                    /* Length of data in 'mbuf'              */
static U8  modem_st;                /* Modem state                           */
static U16 delay;
static U16 mdelay;
static U8  step;
static U8  retries;
static BIT wait_for;
static BIT listen_mode;


/*----------------------------------------------------------------------------
 *      Modem Driver Functions
 *----------------------------------------------------------------------------
 *  Required functions for Modem driver module:
 *   - void modem_init ()
 *   - void modem_dial (U8 *dialnum)
 *   - void modem_listen ()
 *   - void modem_hangup ()
 *   - BOOL modem_online ()
 *   - BOOL modem_process (U8 ch)
 *   - void modem_run ()
 *---------------------------------------------------------------------------*/

/* Local Function Prototypes */
static void send_cmd (U8 *str);
static void flush_buf (void);
static void proc_dial (void);

/*--------------------------- modem_init ------------------------------------*/

void modem_init (void) {
  /* Initializes the modem variables and control signals DTR & RTS. */
  delay    = 0;
  mdelay   = 0;
  modem_st = MODEM_IDLE;
  flush_buf ();
}


/*--------------------------- modem_dial ------------------------------------*/

void modem_dial (U8 *dialnum) {
  /* Modem dial target number 'dialnum'. Because we simulate connected */
  /* modem, we only send "RING\r" twice in 'modem_run()' function.     */
  dialnum = dialnum;
  listen_mode = __FALSE;
  step = 0;
  proc_dial ();
}


/*--------------------------- modem_listen ----------------------------------*/

void modem_listen (void) {
  /* This function puts Modem into Answering Mode. */
  listen_mode = __TRUE;
}


/*--------------------------- modem_hangup ----------------------------------*/

void modem_hangup (void) {
  /* This function clears DTR to force the modem to hang up if   */
  /* it was on line and/or make the modem to go to command mode. */
  modem_st = MODEM_IDLE;
}


/*--------------------------- modem_online ----------------------------------*/

BOOL modem_online (void) {
  /* Checks if the modem is online. Return false when not. */
  /* For direct cable connection we are always online.     */
  if (modem_st == MODEM_ONLINE) {
    return (__TRUE);
  }
  return (__FALSE);
}


/*--------------------------- flush_buf -------------------------------------*/

static void flush_buf (void) {
  /* Flushes the receiving buffer. */
  mlen = 0;
  mem_set (mbuf, 0, sizeof(mbuf));
}


/*--------------------------- send_cmd --------------------------------------*/

static void send_cmd (U8 *str) {
  /* Send Command Mode strings to Modem */
  while (*str) {
    com_putchar (*str++);
  }
}


/*--------------------------- proc_dial -------------------------------------*/

static void proc_dial (void) {
  /* Modem Dial target number and connect to remote modem */

  switch (step) {
    case 0:
      /* Send Command 'CLIENT' */
      modem_st = MODEM_DIAL;
      retries  = 3;
      wait_for = __TRUE;
      step++;
client:send_cmd ("CLIENT\r");
      delay    = TSEC * 3;
      break;

    case 1:
      /* Wait for 'CLIENTSERVER' reply */
      if (wait_for == __TRUE) {
        if (--retries) {
          goto client;
        }
        modem_st = MODEM_ERROR;
        break;
      }
      modem_st = MODEM_ONLINE;
      step++;
      break;

    case 2:
      /* Connected. */
      break;
  }
}


/*--------------------------- modem_process ---------------------------------*/

BOOL modem_process (U8 ch) {
  /* Modem character process event handler. This function is called when */
  /* a new character has been received from the modem in command mode    */

  if (mlen < sizeof(mbuf)) {
    mbuf[mlen++] = ch;
  }
  mdelay = TSEC * 1;

  if (listen_mode == __TRUE) {
    if (str_scomp (mbuf,"CLIENT") == __TRUE) {
      /* For Direct Cable Connection on Win2K/XP */
      send_cmd ("CLIENTSERVER\r");
      flush_buf ();
      modem_st = MODEM_ONLINE;
      /* Modem Connected, return TRUE */
      return (__TRUE);
    }
  }
  else {
    if (str_scomp (mbuf,"CLIENTSERVER") == __TRUE) {
      flush_buf ();
      wait_for = __FALSE;
      modem_st = MODEM_ONLINE;
      /* Modem Connected, return TRUE */
      return (__TRUE);
    }
  }

  if (ch == '\r' || ch == '\n') {
    return (__FALSE);
  }

  /* Modem not connected, return FALSE */
  return (__FALSE);
}


/*--------------------------- modem_run -------------------------------------*/

void modem_run (void) {
  /* This is a main thread for MODEM Control module. It is called on every */
  /* system timer timer tick to implement delays easy. By default this is  */
  /* every 100ms. The 'sytem tick' timeout is set in 'Net_Config.c'        */

  if (mdelay) {
    /* Clear buffer timeout? */
    if (--mdelay == 0) {
      flush_buf ();
    }
  }

  if (delay) {
    if (--delay) {
      return;
    }
  }
  if (modem_st == MODEM_DIAL) {
    proc_dial ();
  }
}


/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
