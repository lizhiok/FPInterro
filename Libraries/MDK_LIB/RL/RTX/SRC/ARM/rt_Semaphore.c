/*----------------------------------------------------------------------------
 *      RL-ARM - RTX
 *----------------------------------------------------------------------------
 *      Name:    RT_SEMAPHORE.C
 *      Purpose: Implements binary and counting semaphores
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include "rt_TypeDef.h"
#include "RTX_Config.h"
#include "rt_System.h"
#include "rt_List.h"
#include "rt_Task.h"
#include "rt_Semaphore.h"


/*----------------------------------------------------------------------------
 *      Functions
 *---------------------------------------------------------------------------*/


/*--------------------------- os_sem_init -----------------------------------*/

void os_sem_init (OS_ID semaphore, U16 token_count) {
  /* Initialize a semaphore */
  P_SCB p_SCB = semaphore;

  tsk_lock();
  p_SCB->cb_type = SCB;
  p_SCB->p_lnk  = NULL;
  p_SCB->tokens = token_count;
  tsk_unlock();
}


/*--------------------------- os_sem_send -----------------------------------*/

OS_RESULT os_sem_send (OS_ID semaphore) {
  /* Return a token to semaphore */
  P_SCB p_SCB = semaphore;
  P_TCB p_TCB;

  tsk_lock();
  if (p_SCB->p_lnk != NULL) {
    /* A task is waiting for token */
    p_TCB = os_get_first ((P_XCB)p_SCB);
    p_TCB->ret_val = OS_R_SEM;
    os_rmv_dly (p_TCB);
    os_dispatch (p_TCB);
  }
  else {
    /* Store token. */
    p_SCB->tokens++;
  }
  tsk_unlock();
  return (OS_R_OK);
}


/*--------------------------- os_sem_wait -----------------------------------*/

OS_RESULT os_sem_wait (OS_ID semaphore, U16 timeout) {
  /* Obtain a token; possibly wait for it */
  P_SCB p_SCB = semaphore;

  tsk_lock();
  if (p_SCB->tokens) {
    p_SCB->tokens--;
    tsk_unlock();
    return (OS_R_OK);
  }
  /* No token available: wait for one */
  if (timeout == 0) {
    goto rto;
  }
  if (p_SCB->p_lnk != NULL) {
    os_put_prio ((P_XCB)p_SCB, os_runtask);
  }
  else {
    p_SCB->p_lnk = os_runtask;
    os_runtask->p_lnk = NULL;
    os_runtask->p_rlnk = (P_TCB)p_SCB;
  }
  if (os_block(timeout, WAIT_SEM) == OS_R_TMO) {
rto:tsk_unlock();
    return (OS_R_TMO);
  }
  tsk_unlock();
  return (OS_R_SEM);
}


/*--------------------------- isr_sem_send ----------------------------------*/

void isr_sem_send (OS_ID semaphore) {
  /* Same function as "os_sem_send", but to be called by ISRs */
  P_SCB p_SCB = semaphore;

  os_psq_enq (p_SCB, 0);
  os_psh_req ();
}


/*--------------------------- os_sem_psh ------------------------------------*/

void os_sem_psh (P_SCB p_CB) {
  /* Check if task has to be waken up */
  P_TCB p_TCB;

  if (p_CB->p_lnk != NULL) {
    /* A task is waiting for token */
    p_TCB = os_get_first ((P_XCB)p_CB);
    os_rmv_dly (p_TCB);
    p_TCB->state   = READY;
    p_TCB->ret_val = OS_R_SEM;
    os_put_prio (&os_rdy, p_TCB);
  }
  else {
    /* Store token */
    p_CB->tokens++;
  }
}


/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/

