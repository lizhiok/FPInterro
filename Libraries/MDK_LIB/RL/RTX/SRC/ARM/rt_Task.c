/*----------------------------------------------------------------------------
 *      RL-ARM - RTX
 *----------------------------------------------------------------------------
 *      Name:    RT_TASK.C
 *      Purpose: Task functions and system start up.
 *      Rev.:    V4.73
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include "rt_TypeDef.h"
#include "RTX_Config.h"
#include "rt_System.h"
#include "rt_Task.h"
#include "rt_List.h"
#include "rt_MemBox.h"
#include "rt_Robin.h"
#include "rt_HAL.h"

/*----------------------------------------------------------------------------
 *      Global Variables
 *---------------------------------------------------------------------------*/

/* Running task info. */
P_TCB  os_runtask;

/* Task Control Block of idle demon */
struct OS_TCB os_idle_TCB;


/*----------------------------------------------------------------------------
 *      Local Functions
 *---------------------------------------------------------------------------*/

static void os_set_TID (P_TCB task) {
  U32 tid;

  for (tid = 1; tid <= os_maxtaskrun; tid++) {
    if (os_active_TCB[tid-1] == NULL) {
      os_active_TCB[tid-1] = task;
      task->task_id        = tid;
      break;
    }
  }
}


/*--------------------------- os_init_context -------------------------------*/

static void os_init_context (P_TCB p_TCB, U8 prio, FUNCP task_body, U8 full_ctx) {
  /* Initialize general part of the Task Control Block. */
  p_TCB->cb_type   = TCB;
  p_TCB->state     = READY;
  p_TCB->prio      = prio;
  p_TCB->prio_base = prio;
  p_TCB->p_lnk     = NULL;
  p_TCB->p_rlnk    = NULL;
  p_TCB->p_dlnk    = NULL;
  p_TCB->p_blnk    = NULL;
  p_TCB->p_mlnk    = NULL;
  p_TCB->delta_time    = 0;
  p_TCB->interval_time = 0;
  p_TCB->events  = 0;
  p_TCB->waits   = 0;
  p_TCB->ret_val = OS_R_OK;
  p_TCB->full_ctx= full_ctx;

  if (p_TCB->priv_stack == 0) {
    /* Allocate the memory space for the stack. */
    p_TCB->stack = _alloc_box (mp_stk);
  }
  os_init_stack (p_TCB, task_body);
}


/*----------------------------------------------------------------------------
 *      Global Functions
 *---------------------------------------------------------------------------*/

/*--------------------------- os_dispatch -----------------------------------*/

void os_dispatch (P_TCB next_TCB) {
  /* Dispatch next task if any identified or dispatch highest ready task    */
  /* "next_TCB" identifies a task to run or has value NULL (=no next task)  */
  if (next_TCB == NULL) {
    /* Running task was blocked: continue with highest ready task */
    next_TCB = os_get_first (&os_rdy);
    os_switch_tasks (next_TCB);
  }
  else {
    /* Check which task continues */
    if (next_TCB->prio > os_runtask->prio) {
      /* preempt running task */
      os_put_rdy_first (os_runtask);
      os_runtask->state = READY;
      os_switch_tasks (next_TCB);
    }
    else {
      /* put next task into ready list, no task switch takes place */
      next_TCB->state = READY;
      os_put_prio (&os_rdy, next_TCB);
    }
  }
}


/*--------------------------- os_block --------------------------------------*/

OS_RESULT os_block (U16 timeout, U8 block_state) {
  /* Block running task and choose next ready task.                         */
  /* "timeout" sets a time-out value or is 0xffff (=no time-out).           */
  /* "block_state" defines the appropriate task state */
  P_TCB next_TCB;

  if (timeout) {
    if (timeout < 0xffff) {
      os_put_dly (os_runtask, timeout);
    }
    os_runtask->state   = block_state;
    os_runtask->ret_val = OS_R_OK;
    next_TCB = os_get_first (&os_rdy);
    os_switch_tasks (next_TCB);
    return (os_runtask->ret_val);
  }
  return (OS_R_TMO);
}


/*--------------------------- os_tsk_pass -----------------------------------*/

void os_tsk_pass (void) {
  /* Allow tasks of same priority level to run cooperatively.*/
  P_TCB p_new;

  tsk_lock();
  p_new = os_get_same_rdy_prio();
  if (p_new != NULL) {
    os_put_prio ((P_XCB)&os_rdy, os_runtask);
    os_runtask->state = READY;
    os_switch_tasks (p_new);
  }
  tsk_unlock();
}


/*--------------------------- os_tsk_self -----------------------------------*/

OS_TID os_tsk_self (void) {
  /* Return own task identifier value. */
  if (os_runtask == NULL) {
    return (0);
  }
  return (os_runtask->task_id);
}


/*--------------------------- os_tsk_prio -----------------------------------*/

OS_RESULT os_tsk_prio (OS_TID task_id, U8 new_prio) {
  /* Change execution priority of a task to "new_prio". */
  P_TCB p_task;

  tsk_lock();
  if (task_id == 0) {
    /* Change execution priority of calling task. */
    os_runtask->prio      = new_prio;
    os_runtask->prio_base = new_prio;
run:if (os_rdy_prio() > new_prio) {
      os_put_prio (&os_rdy, os_runtask);
      os_runtask->state = READY;
      os_dispatch (NULL);
    }
    tsk_unlock();
    return (OS_R_OK);
  }

  /* Find the task in the "os_active_TCB" array. */
  if (task_id > os_maxtaskrun || os_active_TCB[task_id-1] == NULL) {
    /* Task with "task_id" not found or not started. */
    tsk_unlock ();
    return (OS_R_NOK);
  }
  p_task = os_active_TCB[task_id-1];
  p_task->prio      = new_prio;
  p_task->prio_base = new_prio;
  if (p_task == os_runtask) {
    goto run;
  }
  os_resort_prio (p_task);
  if (p_task->state == READY) {
    /* Task enqueued in a ready list. */
    p_task = os_get_first (&os_rdy);
    os_dispatch (p_task);
  }
  tsk_unlock ();
  return (OS_R_OK);
}


/*--------------------------- os_tsk_create0 --------------------------------*/

OS_TID os_tsk_create0 (FUNCP task, U32 prio_stksz, void *stk, void *argv) {
  /* Start a new task declared with "task". */
  P_TCB task_context;

  tsk_lock();
  /* Priority 0 is reserved for idle task! */
  if ((prio_stksz & 0xFF) == 0) {
    prio_stksz += 1;
  }
  task_context = _alloc_box (mp_tcb);
  if (task_context == NULL) {
    tsk_unlock();
    return (0);
  }
  /* If "size != 0" use a private user provided stack. */
  task_context->stack      = stk;
  task_context->priv_stack = prio_stksz >> 8;
  /* Pass parameter 'argv' to 'os_init_context' */
  task_context->p_msg = argv;
  /* For 'size == 0' system allocates the user stack from the memory pool. */
  os_init_context (task_context, prio_stksz & 0xFF, task, __TRUE);

  /* Find a free entry in 'os_active_TCB' table. */
  os_set_TID (task_context);

  rt_notify (task_context->ptask, (__TRUE << 8) | task_context->task_id);
  os_dispatch (task_context);
  tsk_unlock();
  return ((OS_TID)task_context->task_id);
}


/*--------------------------- os_tsk_create_ex0 -----------------------------*/

OS_TID os_tsk_create_ex0 (FUNCPP task, U32 prio_stksz, void *stk, void *argv) {
  /* Create a new task with parameter 'argv'. */
  return (os_tsk_create0 ((FUNCP)task, prio_stksz, stk, argv));
}


/*--------------------------- os_tsk_delete ---------------------------------*/

OS_RESULT os_tsk_delete (OS_TID task_id) {
  /* Terminate the task identified with "task_id". */
  P_TCB  task_context;
  P_TCB  p_TCB;
  P_MUCB p_MCB, p_MCB0;

  tsk_lock();
  if (task_id == 0 || task_id == os_runtask->task_id) {
    /* Terminate itself. */
    os_runtask->state = INACTIVE;
    os_stk_check (0);
    p_MCB = os_runtask->p_mlnk;
    while (p_MCB) {
      /* Release mutexes owned by this task */
      if (p_MCB->p_lnk) {
        /* A task is waiting for mutex. */
        p_TCB = os_get_first ((P_XCB)p_MCB);
        p_TCB->ret_val = OS_R_MUT;
        os_rmv_dly (p_TCB);
        p_TCB->state = READY;
        os_put_prio (&os_rdy, p_TCB);
        /* A waiting task becomes the owner of this mutex. */
        p_MCB0 = p_MCB;
        p_MCB->level  = 1;
        p_MCB->owner  = p_TCB;
        p_MCB->p_mlnk = p_TCB->p_mlnk;
        p_TCB->p_mlnk = p_MCB; 
        p_MCB = p_MCB0->p_mlnk;
      }
      else {
        p_MCB = p_MCB->p_mlnk;
      }
    }
    os_active_TCB[os_runtask->task_id-1] = NULL;
    _free_box (mp_stk, os_runtask->stack);
    os_runtask->stack = NULL;
    rt_notify (os_runtask->ptask, (__FALSE << 8) | os_runtask->task_id);
    _free_box (mp_tcb, os_runtask);
    os_runtask = NULL;
    os_dispatch (NULL);
    /* The program should never come to this point. */
  }
  else {
    /* Find the task in the "os_active_TCB" array. */
    if (task_id > os_maxtaskrun || os_active_TCB[task_id-1] == NULL) {
      /* Task with "task_id" not found or not started. */
      tsk_unlock ();
      return (OS_R_NOK);
    }
    task_context = os_active_TCB[task_id-1];
    os_rmv_list (task_context);
    os_rmv_dly (task_context);
    p_MCB = task_context->p_mlnk;
    while (p_MCB) {
      /* Release mutexes owned by this task */
      if (p_MCB->p_lnk) {
        /* A task is waiting for mutex. */
        p_TCB = os_get_first ((P_XCB)p_MCB);
        p_TCB->ret_val = OS_R_MUT;
        os_rmv_dly (p_TCB);
        p_TCB->state = READY;
        os_put_prio (&os_rdy, p_TCB);
        /* A waiting task becomes the owner of this mutex. */
        p_MCB0 = p_MCB;
        p_MCB->level  = 1;
        p_MCB->owner  = p_TCB;
        p_MCB->p_mlnk = p_TCB->p_mlnk;
        p_TCB->p_mlnk = p_MCB; 
        p_MCB = p_MCB0->p_mlnk;
      }
      else {
        p_MCB = p_MCB->p_mlnk;
      }
    }
    os_active_TCB[task_id-1] = NULL;
    _free_box (mp_stk, task_context->stack);
    task_context->stack = NULL;
    rt_notify (task_context->ptask, (__FALSE << 8) | task_context->task_id);
    _free_box (mp_tcb, task_context);
    tsk_unlock ();
    if (os_rdy_prio() > os_runtask->prio) {
      /* Ready task has higher priority than running task. */
      os_runtask->state = READY;
      os_put_prio (&os_rdy, os_runtask);
      os_dispatch (NULL);
    }
  }
  return (OS_R_OK);
}


/*--------------------------- os_sys_init1 -----------------------------------*/

__naked void os_sys_init1 (void) {
  /* Initialize system and start up a first task. */
  U32 i;

  rt_init ();
  tsk_lock ();

  /* Initialize dynamic memory and task TCB pointers to NULL. */
  for (i = 0; i < os_maxtaskrun; i++) {
    os_active_TCB[i] = NULL;
  }
  _init_box (&mp_tcb, mp_tcb_size, sizeof(struct OS_TCB));
  _init_box (&mp_stk, mp_stk_size, BOX_ALIGN_8 | (U16)(os_stackinfo));
  _init_box ((U32 *)m_tmr, mp_tmr_size, sizeof(struct OS_TMR));

  /* Set up TCB of idle demon */
  os_idle_TCB.task_id    = 255;
  os_idle_TCB.priv_stack = 0;
  os_init_context (&os_idle_TCB, 0, os_idle_demon, __FALSE);

  /* Set up ready list: initially empty */
  os_rdy.cb_type = HCB;
  os_rdy.p_lnk   = NULL;
  /* Set up delay list: initially empty */
  os_dly.cb_type = HCB;
  os_dly.p_dlnk  = NULL;
  os_dly.p_blnk  = NULL;
  os_dly.delta_time = 0;

  /* Fix SP and system variables to assume idle task is running */
  /* Transform main program into idle task by assuming idle TCB */
  os_set_env (&os_idle_TCB);
  os_runtask = &os_idle_TCB;
  os_runtask->state = RUNNING;

  /* Initialize ps queue */
  os_psq->first = 0;
  os_psq->last  = 0;
  os_psq->size  = os_fifo_size;

  /* Initialize system clock timer */
  os_tmr_init ();
  os_init_robin ();

  /* Start up first user task before entering the endless loop */
  os_sys_run ((FUNCP)os_tsk_create0);

  /* Call body of idle task: contains an endless loop */
  os_idle_demon();

  /* This point never reached if idle task contains endless loop */
  for (;;);
}

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
