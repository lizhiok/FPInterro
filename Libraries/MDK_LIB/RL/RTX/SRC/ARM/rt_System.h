/*----------------------------------------------------------------------------
 *      RL-ARM - RTX
 *----------------------------------------------------------------------------
 *      Name:    RT_SYSTEM.H
 *      Purpose: System Task Manager definitions
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

/* Variables */
#define os_psq  ((P_PSQ)&os_fifo)
extern struct OS_PSH os_psh;

/* Functions */
extern void os_psh_req (void);
extern void os_sys_manager (U32 stack);
extern U32  os_suspend (void);
extern void os_resume (U32 sleep_time);

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
