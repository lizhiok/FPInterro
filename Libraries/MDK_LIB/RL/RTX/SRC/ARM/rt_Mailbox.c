/*----------------------------------------------------------------------------
 *      RL-ARM - RTX
 *----------------------------------------------------------------------------
 *      Name:    RT_MAILBOX.C
 *      Purpose: Implements waits and wake-ups for mailbox messages
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include "rt_TypeDef.h"
#include "RTX_Config.h"
#include "rt_System.h"
#include "rt_List.h"
#include "rt_Mailbox.h"
#include "rt_Task.h"
#include "rt_HAL.h"


/*----------------------------------------------------------------------------
 *      Functions
 *---------------------------------------------------------------------------*/


/*--------------------------- os_mbx_init -----------------------------------*/

void os_mbx_init (OS_ID mailbox, U16 mbx_size) {
  /* Initialize a mailbox */
  P_MCB p_MCB = mailbox;

  tsk_lock();
  p_MCB->cb_type = MCB;
  p_MCB->isr_st  = 0;
  p_MCB->p_lnk   = NULL;
  p_MCB->first   = 0;
  p_MCB->last    = 0;
  p_MCB->count   = 0;
  p_MCB->size    = (mbx_size + sizeof(void *) - sizeof(struct OS_MCB)) /
                                                    (U32)sizeof (void *);
  tsk_unlock();
}


/*--------------------------- os_mbx_send -----------------------------------*/

OS_RESULT os_mbx_send (OS_ID mailbox, void *p_msg, U16 timeout) {
  /* Send message to a mailbox */
  P_MCB p_MCB = mailbox;
  P_TCB p_TCB;

  tsk_lock();
m:if (p_MCB->p_lnk != NULL && p_MCB->count == 0) {
    /* A task is waiting for message */
    p_TCB = os_get_first ((P_XCB)p_MCB);
    p_TCB->p_msg = p_msg;
    p_TCB->ret_val = OS_R_MBX;
    os_rmv_dly (p_TCB);
    os_dispatch (p_TCB);
  }
  else {
    /* Store message in mailbox queue */
    if (p_MCB->count == p_MCB->size) {
      /* No free message entry, wait for one. If message queue is full, */
      /* then no task is waiting for message. The 'p_MCB->p_lnk' list   */
      /* pointer can now be reused for send message waits task list.    */
      if (timeout == 0) {
        goto wto;
      }
      if (p_MCB->p_lnk != NULL) {
        os_put_prio ((P_XCB)p_MCB, os_runtask);
      }
      else {
        p_MCB->p_lnk = os_runtask;
        os_runtask->p_lnk  = NULL;
        os_runtask->p_rlnk = (P_TCB)p_MCB;
        /* Signal the 'isr_mbx_receive ()' that the task is waiting */
        /* to send a message */
        p_MCB->isr_st = 1;
      }
      if (os_block(timeout, WAIT_MBX) == OS_R_TMO) {
wto:    tsk_unlock();
        return (OS_R_TMO);
      }
      /* A message has been fetched, check the mailbox again. */
      goto m;
    }
    /* Yes, there is a free entry in a mailbox. */
    p_MCB->msg[p_MCB->first] = p_msg;
    _incw (&p_MCB->count);
    if (++p_MCB->first == p_MCB->size) {
      p_MCB->first = 0;
    }
  }
  tsk_unlock();
  return (OS_R_OK);
}


/*--------------------------- os_mbx_wait -----------------------------------*/

OS_RESULT os_mbx_wait (OS_ID mailbox, void **message, U16 timeout) {
  /* Receive a message; possibly wait for it */
  P_MCB p_MCB = mailbox;
  P_TCB p_TCB;

  tsk_lock();
  /* If a message is available in the fifo buffer */
  /* remove it from the fifo buffer and return. */
  if (p_MCB->count) {
    *message = p_MCB->msg[p_MCB->last];
    _decw (&p_MCB->count);
    if (++p_MCB->last == p_MCB->size) {
      p_MCB->last = 0;
    }
    if (p_MCB->p_lnk != NULL) {
      /* A task is waiting to send message */
      p_TCB = os_get_first ((P_XCB)p_MCB);
      os_rmv_dly (p_TCB);
      os_dispatch (p_TCB);
    }
    tsk_unlock();
    return (OS_R_OK);
  }
  /* No message available: wait for one */
  if (timeout == 0) {
    goto rto;
  }
  if (p_MCB->p_lnk != NULL) {
    os_put_prio ((P_XCB)p_MCB, os_runtask);
  }
  else {
    p_MCB->p_lnk = os_runtask;
    os_runtask->p_lnk = NULL;
    os_runtask->p_rlnk = (P_TCB)p_MCB;
  }
  if (os_block(timeout, WAIT_MBX) == OS_R_TMO) {
rto:tsk_unlock();
    *message = NULL;
    return (OS_R_TMO);
  }
  tsk_unlock();
  *message = os_runtask->p_msg;
  return (OS_R_MBX);
}


/*--------------------------- os_mbx_check ----------------------------------*/

OS_RESULT os_mbx_check (OS_ID mailbox) {
  /* Check for free space in a mailbox. Returns the number of messages     */
  /* that can be stored to a mailbox. It returns 0 when mailbox is full.   */
  P_MCB p_MCB = mailbox;

  return (p_MCB->size - p_MCB->count);
}


/*--------------------------- isr_mbx_send ----------------------------------*/

void isr_mbx_send (OS_ID mailbox, void *p_msg) {
  /* Same function as "os_mbx_send", but to be called by ISRs. */
  P_MCB p_MCB = mailbox;

  os_psq_enq (p_MCB, (U32)p_msg);
  os_psh_req ();
}


/*--------------------------- isr_mbx_receive -------------------------------*/

OS_RESULT isr_mbx_receive (OS_ID mailbox, void **message) {
  /* Receive a message in the interrupt function. The interrupt function   */
  /* should not wait for a message since this would block the rtx os.      */
  P_MCB p_MCB = mailbox;

  if (p_MCB->count) {
    /* A message is available in the fifo buffer. */
    *message = p_MCB->msg[p_MCB->last];
    if (p_MCB->isr_st == 1) {
      /* A task is locked waiting to send message */
      p_MCB->isr_st = 2;
      os_psq_enq (p_MCB, 0);
      os_psh_req ();
    }
    p_MCB->count--;
    if (++p_MCB->last == p_MCB->size) {
      p_MCB->last = 0;
    }
    return (OS_R_MBX);
  }
  return (OS_R_OK);
}


/*--------------------------- os_mbx_psh ------------------------------------*/

void os_mbx_psh (P_MCB p_CB, void *p_msg) {
  /* Store the message to the mailbox queue or pass it to task directly. */
  P_TCB p_TCB;

  /* Check if this was an 'isr_mbx_receive ()' post service request.   */
  if (p_CB->p_lnk != NULL && p_CB->isr_st == 2) {
    /* A task is waiting to send message, remove it from the waiting list. */
    p_CB->isr_st = 0;
    p_TCB = os_get_first ((P_XCB)p_CB);
    p_TCB->ret_val = OS_R_OK;
    goto rdy;
  }
  /* A task is waiting for message, pass the message to task directly. */
  if (p_CB->p_lnk != NULL && p_CB->count == 0) {
    p_TCB = os_get_first ((P_XCB)p_CB);
    p_TCB->p_msg = p_msg;
    p_TCB->ret_val = OS_R_MBX;
rdy:p_TCB->state = READY;
    os_rmv_dly (p_TCB);
    os_put_prio (&os_rdy, p_TCB);
  }
  else {
    /* No task is waiting for message, store it to the mailbox queue. */
    if (p_CB->count < p_CB->size) {
      p_CB->msg[p_CB->first] = p_msg;
      _incw (&p_CB->count);
      if (++p_CB->first == p_CB->size) {
        p_CB->first = 0;
      }
    }
    else {
      os_error (OS_ERR_MBX_OVF);
    }
  }
}

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/

