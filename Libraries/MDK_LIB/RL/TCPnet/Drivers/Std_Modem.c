/*----------------------------------------------------------------------------
 *      RL-ARM - TCPnet
 *----------------------------------------------------------------------------
 *      Name:    STD_MODEM.C
 *      Purpose: Standard Modem Driver
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
static U8  *dial_num;               /* Dial Target Number                    */
static U8  *reply;                  /* Wait for 'reply'                      */
static U16 delay;
static U8  step;
static U8  retries;
static BIT wait_for;
static BIT wait_conn;
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
static void proc_listen (void);
static void proc_dial (void);
static void proc_hangup (void);
static void set_mode (void);

/*--------------------------- modem_init ------------------------------------*/

void modem_init (void) {
  /* Initializes the modem variables and control signals DTR & RTS. */
  mlen = 0;
  mem_set (mbuf, 0, sizeof(mbuf));
  wait_for  = 0;
  wait_conn = 0;
  modem_st = MODEM_IDLE;
}


/*--------------------------- modem_dial ------------------------------------*/

void modem_dial (U8 *dialnum) {
  /* Modem dial target number 'dialnum' */

  dial_num = dialnum;
  listen_mode = 0;
  step = 0;
  proc_dial ();
}


/*--------------------------- modem_listen ----------------------------------*/

void modem_listen (void) {
  /* This function puts Modem into Answering Mode. */
  step = 0;
  listen_mode = 1;
  proc_listen ();
}


/*--------------------------- modem_hangup ----------------------------------*/

void modem_hangup (void) {
  /* This function clears DTR to force the modem to hang up if   */
  /* it was on line and/or make the modem to go to command mode. */
  step = 0;
  proc_hangup ();
}


/*--------------------------- modem_online ----------------------------------*/

BOOL modem_online (void) {
  /* Checks if the modem is online. Return false when not. */
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
  U32 state;

  switch (step) {
    case 0:
      /* Send Reset Command 'ATZ' */
      state = modem_st;
      modem_st = MODEM_DIAL;
      wait_conn = __FALSE;
      if(state == MODEM_READY) {
        step = 2;
        goto atdt;
      }
      reply = "OK\r";
      retries = 3;
      step++;
atz:  send_cmd ("ATZ\r");
      delay = TSEC * 5;
      wait_for = __TRUE;
      break;

    case 1:
      /* Wait for 'OK' reply */
      if (wait_for == __TRUE) {
        if (--retries) {
          goto atz;
        }
        modem_st = MODEM_ERROR;
        break;
      }
      /* Send 'ATE0V1' command to modem */
      retries = 3;
      step++;
ate0: send_cmd ("ATE0V1\r");
      delay =  TSEC * 3;
      wait_for = __TRUE;
      break;

    case 2:
      /* Wait for 'OK' reply */
      if (wait_for == __TRUE) {
        if (--retries) {
          goto ate0;
        }
        modem_st = MODEM_ERROR;
        break;
      }
      /* Dial Target number. */
atdt: send_cmd ("ATDT");
      send_cmd (dial_num);
      send_cmd ("\r");
      reply = "CONNECT";
      delay = TSEC * 30;
      wait_for = __TRUE;
      wait_conn = __TRUE;
      step++;
      break;

    case 3:
      /* Timeout on Waiting for 'CONNECT', hangup */
      wait_conn = __FALSE;
      retries = 3;
      reply = "OK\r";
      step++;
ath:  send_cmd ("ATH\r");
      delay = TSEC * 2;
      wait_for = __TRUE;
      break;

    case 4:
      /* Cancel the call */
      if (wait_for == __TRUE) {
        if (--retries) {
          goto ath;
        }
        modem_st = MODEM_ERROR;
        break;
      }
      /* Timeout, hangup modem. */
      modem_st = MODEM_READY;
      break;
  }
}


/*--------------------------- proc_listen -----------------------------------*/

static void proc_listen (void) {
  /* Handle Modem listening mode (waiting for incomming connection) */

  switch (step) {
    case 0:
      /* Send Reset Command 'ATZ' */
      modem_st = MODEM_LISTEN;
      wait_conn = __FALSE;
      reply = "OK\r";
      retries = 3;
      step++;
atz:  send_cmd ("ATZ\r");
      delay = TSEC * 5;
      wait_for = __TRUE;
      break;

    case 1:
      /* Wait for 'OK' reply */
      if (wait_for == __TRUE) {
        if (--retries) {
          goto atz;
        }
        modem_st = MODEM_ERROR;
        break;
      }
      /* Send 'ATE0V1' command to modem */
      retries = 3;
      step++;
ate0: send_cmd ("ATE0V1\r");
      delay =  TSEC * 3;
      wait_for = __TRUE;
      break;

    case 2:
      /* Wait for 'OK' reply */
      if (wait_for == __TRUE) {
        if (--retries) {
          goto ate0;
        }
        modem_st = MODEM_ERROR;
        break;
      }
      /* Wait for incomming call. */
ring: reply = "RING\r";
      delay = 0;
      wait_for = __TRUE;
      step++;
      break;

    case 3:
      /* Waiting for the call */
      if (wait_for == __TRUE) {
        break;
      }
      send_cmd ("ATA\r");
      reply = "CONNECT";
      delay = TSEC * 30;
      wait_for = __TRUE;
      wait_conn = __TRUE;
      step++;
      break;

    case 4:
      /* Timeout on Waiting for 'CONNECT', hangup */
      wait_conn = __FALSE;
      retries = 3;
      reply = "OK\r";
      step++;
ath:  send_cmd ("ATH\r");
      delay = TSEC * 2;
      wait_for = __TRUE;
      break;

    case 5:
      if (wait_for == __TRUE) {
        if (--retries) {
          goto ath;
        }
        /* Failed, send reset to modem. */
        step = 0;
        goto atz;
      }
      step = 2;
      goto ring;
  }
}


/*--------------------------- set_mode --------------------------------------*/

static void set_mode (void) {
  /* Set Modem mode when 'NO CARRIER' received */
  if (listen_mode == __TRUE) {
    modem_st = MODEM_LISTEN;
    wait_for = __FALSE;
    step = 2;
  }
  else {
    modem_st = MODEM_READY;
  }
}


/*--------------------------- proc_hangup -----------------------------------*/

static void proc_hangup (void) {
  /* Hangup Modem if connected. */

  switch (step) {
    case 0:
      wait_conn = __FALSE;
      if (modem_st <= MODEM_READY) {
        /* Modem not connected, we are done here. */
        break;
      }
      modem_st = MODEM_HANGUP;
      delay = TSEC * 3;
      wait_for = __FALSE;
      step++;
      break;

    case 1:
      /* Send ESC sequence after delay */
      send_cmd ("+++");
      delay = TSEC * 2;
      step++;
      break;

    case 2:
      /* Now hangup the modem, send 'ATH' */
      reply = "OK\r";
      retries = 3;
      step++;
ath:  send_cmd ("ATH\r");
      delay = TSEC * 3;
      wait_for = __TRUE;
      break;

    case 3:
      /* Wait for 'OK' reply */
      if (wait_for == __TRUE) {
        if (--retries) {
          goto ath;
        }
        /* No reply, reset the modem. */
        retries = 3;
        step++;
atz:    send_cmd ("ATZ\r");
        delay = TSEC * 5;
        wait_for = __TRUE;
        break;
      }
      /* OK, disconnected, we are done. */
      set_mode ();
      break;

    case 4:
      if (wait_for == __TRUE) {
        if (--retries) {
          goto atz;
        }
        /* Modem Reset failed */
        modem_st = MODEM_ERROR;
        break;
      }
      set_mode ();
      break;
  }
}


/*--------------------------- modem_process ---------------------------------*/

BOOL modem_process (U8 ch) {
  /* Modem character process event handler. This function is called when */
  /* a new character has been received from the modem in command mode    */

  if (modem_st == MODEM_IDLE) {
    mlen = 0;
    return (__FALSE);
  }
  if (mlen < sizeof(mbuf)) {
    mbuf[mlen++] = ch;
  }
  /* Modem driver is processing a command */
  if (wait_for) {
    /* 'modem_run()' is waiting for modem reply */
    if (str_scomp (mbuf,reply) == __TRUE) {
      wait_for = 0;
      delay    = 0;
      if (wait_conn) {
        /* OK, we are online here. */
        wait_conn = 0;
        modem_st  = MODEM_ONLINE;
        /* Inform the parent process we are online now. */
        return (__TRUE);
      }
    }
  }
  /* Watch the modem disconnect because we do not use CD line */
  if (mem_comp (mbuf,"NO CARRIER",10) == __TRUE) {
    set_mode ();
  }
  if (ch == '\r' || ch == '\n') {
    flush_buf ();
  }
  /* Modem not connected, return FALSE */
  return (__FALSE);
}


/*--------------------------- modem_run -------------------------------------*/

void modem_run (void) {
  /* This is a main thread for MODEM Control module. It is called on every */
  /* system timer timer tick to implement delays easy. By default this is  */
  /* every 100ms. The 'sytem tick' timeout is set in 'Net_Config.c'        */

  if (delay) {
    if (--delay) {
      return;
    }
  }

  switch (modem_st) {
    case MODEM_IDLE:
    case MODEM_ERROR:
      /* Modem idle or in error */
      break;

    case MODEM_ONLINE:
      /* Modem is online - connected */
      break;

    case MODEM_DIAL:
      /* Dial target number */
      proc_dial ();
      break;

    case MODEM_LISTEN:
      /* Activate answering mode */
      proc_listen ();
      break;

    case MODEM_HANGUP:
      /* Hangup and reset the modem */
      proc_hangup ();
      break;
  }
}


/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
