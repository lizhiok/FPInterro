/*----------------------------------------------------------------------------
 *      RL-ARM - RTX
 *----------------------------------------------------------------------------
 *      Name:    RT_TASK.H
 *      Purpose: Task functions and system start up.
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

/* Definitions */

/* Values for 'state'   */
#define INACTIVE        0
#define READY           1
#define RUNNING         2
#define WAIT_DLY        3
#define WAIT_ITV        4
#define WAIT_OR         5
#define WAIT_AND        6
#define WAIT_SEM        7
#define WAIT_MBX        8
#define WAIT_MUT        9

/* Return codes */
#define OS_R_TMO        0x01
#define OS_R_EVT        0x02
#define OS_R_SEM        0x03
#define OS_R_MBX        0x04
#define OS_R_MUT        0x05

#define OS_R_OK         0x00
#define OS_R_NOK        0xff

/* Variables */
extern P_TCB  os_runtask;
extern struct OS_TCB os_idle_TCB;

/* Functions */
extern void      os_dispatch (P_TCB next_TCB);
extern OS_RESULT os_block (U16 timeout, U8 block_state);
extern void      os_tsk_pass (void);
extern OS_TID    os_tsk_self (void);
extern OS_RESULT os_tsk_prio (OS_TID task_id, U8 new_prio);
extern OS_TID    os_tsk_create0 (FUNCP task, U32 prio_stksz, void *stk, 
                                 void *argv);
extern OS_TID    os_tsk_create_ex0 (FUNCPP task, U32 prio_stksz, void *stk, 
                                    void *argv);
extern OS_RESULT os_tsk_delete (OS_TID task_id);
extern void      os_sys_init1 (void);

/*----------------------------------------------------------------------------
 *      Interface functions for RT Agent task monitoring
 *---------------------------------------------------------------------------*/

__weak void rt_init (void);
__weak void rt_post_taskswitch (U32 task_id);
__weak void rt_notify (FUNCP ptask, U32 create_tid);


/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
