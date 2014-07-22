/*----------------------------------------------------------------------------
 *      RL-ARM - RTX
 *----------------------------------------------------------------------------
 *      Name:    RT_SYSTEM.C
 *      Purpose: System Task Manager
 *      Rev.:    V4.73
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include "rt_TypeDef.h"
#include "RTX_Config.h"
#include "rt_Task.h"
#include "rt_System.h"
#include "rt_Event.h"
#include "rt_List.h"
#include "rt_Mailbox.h"
#include "rt_Semaphore.h"
#include "rt_Time.h"
#include "rt_Timer.h"
#include "rt_Robin.h"
#include "rt_Hal.h"


/*----------------------------------------------------------------------------
 *      Global Variables
 *---------------------------------------------------------------------------*/

/* Post service requests */
struct OS_PSH os_psh;

/*----------------------------------------------------------------------------
 *      Global Functions
 *---------------------------------------------------------------------------*/

__asm void $$RTX$$version (void) {
   /* Export a version number symbol for a version control. */

                EXPORT  __RL_RTX_VER

__RL_RTX_VER    EQU     0x473
}

const U32 RL_RTX_Version __attribute__((used)) = ((4<<16)|73);


/*--------------------------- os_suspend ------------------------------------*/

U32 os_suspend (void) {
  /* Suspend OS scheduler */
  U32 delta = 0xFFFF;

  tsk_lock ();
  if (os_dly.p_dlnk != NULL) {
    delta = os_dly.delta_time;
  }
  if ((os_tmr.next != NULL) && (os_tmr.tcnt < delta)) {
    delta = os_tmr.tcnt;
  }
  return (delta);
}


/*--------------------------- os_resume -------------------------------------*/

void os_resume (U32 sleep_time) {
  /* Resume OS scheduler after suspend */
  U32 delta;

  os_runtask->state = READY;
  os_put_rdy_first (os_runtask);
  os_robin.task = NULL;

  /* Update delays. */
  if (os_dly.p_dlnk != NULL) {
    delta = sleep_time;
    if (delta >= os_dly.delta_time) {
      delta   -= os_dly.delta_time;
      os_time += os_dly.delta_time;
      os_dly.delta_time = 1;
      while (os_dly.p_dlnk != NULL) {
        os_dec_dly ();
        if (delta == 0) break;
        delta--;
        os_time++;
      }
    }
    else {
      os_time           += delta;
      os_dly.delta_time -= delta;
    }
  }
  else {
    os_time += sleep_time;
  }

  /* Check the user timers. */
  if (os_tmr.next != NULL) {
    delta = sleep_time;
    if (delta >= os_tmr.tcnt) {
      delta   -= os_tmr.tcnt;
      os_tmr.tcnt = 1;
      while (os_tmr.next != NULL) {
        os_tmr_tick ();
        if (delta == 0) break;
        delta--;
      }
    }
    else {
      os_tmr.tcnt -= delta;
    }
  }

  /* Switch back to highest ready task */
  os_dispatch (NULL);
  tsk_unlock ();
}


/*--------------------------- os_psh_req ------------------------------------*/

void os_psh_req (void) {
  /* Initiate a post service handling request if required.                  */
  /* Modifies system globals os_psh.flag, os_psh.cnt accordingly.           */

  if (os_psh.flag || os_tmr_inspect_ovf()) {
    return;
  }

  os_psh.cnt = os_tmr_inspect_cnt();
  if (!os_tmr_inspect_ovf()) {
    os_psh.req = __TRUE;
    os_tmr_force_irq();
  }
}


/*--------------------------- os_sys_manager --------------------------------*/

__naked void os_sys_manager (U32 stack) {
  /* Check for system clock update and ISR post service requests. */
  P_XCB p_CB;
  BOOL tmr_ovfl = __FALSE;
  U32  idx;

  /* Suspend current running task. */
  os_runtask->tsk_stack= stack;
  os_runtask->full_ctx = __TRUE;
  os_runtask->state    = READY;
  os_put_rdy_first (os_runtask);

  /* Check if it is a timer overflow or only an explicit request. */
  if (!os_psh.flag || 
     (os_tmr_inspect_cnt() < os_psh.cnt) && !os_tmr_inspect_ovf()) {
    /* There was a timer overflow */
    os_tmr_reload ();
    /* Check Round Robin timeout. */
    os_chk_robin ();
    /* Update delays. */
    os_time++;
    os_dec_dly ();
    tmr_ovfl = __TRUE;
  }
  os_psh.flag = __FALSE;

  /* In any case check for post service requests. */
  idx = os_psq->last;
  while (os_psq->count) {
    p_CB = os_psq->q[idx].id;
    if (p_CB->cb_type == TCB) {
      /* Is of TCB type? */
      os_evt_psh ((P_TCB)p_CB, (U16)os_psq->q[idx].arg);
    }
    else if (p_CB->cb_type == MCB) {
      /* Is of MCB type? */
      os_mbx_psh ((P_MCB)p_CB, (void *)os_psq->q[idx].arg);
    }
    else {
      /* Must be of SCB type. */
      os_sem_psh ((P_SCB)p_CB);
    }
    /* Fifo is a circular buffer. */
    if (++idx == os_psq->size) idx = 0;
    _dec_qc (&os_psq->count);
  }
  os_psq->last = idx;

  if (tmr_ovfl == __TRUE) {
    /* Check user timers. */
    os_tmr_tick ();
  }
  /* Check for stack overflow. */
  os_stk_check (os_runtask->tsk_stack);
  os_runtask = os_get_first (&os_rdy);
  os_runtask->state = RUNNING;
  rt_post_taskswitch (os_runtask->task_id);
  os_sys_manager_ret (os_runtask);
}

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/

