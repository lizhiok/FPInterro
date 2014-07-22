/*----------------------------------------------------------------------------
 *      RL-ARM - RTX
 *----------------------------------------------------------------------------
 *      Name:    RT_EVENT.H
 *      Purpose: Implements waits and wake-ups for event flags
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

/* Functions */
extern OS_RESULT os_evt_wait (U16 wait_flags, U16 timeout, BOOL and_wait);
extern void      os_evt_set  (U16 event_flags, OS_TID task_id);
extern void      os_evt_clr  (U16 clear_flags, OS_TID task_id);
extern void      isr_evt_set (U16 event_flags, OS_TID task_id);
extern U16       os_evt_get  (void);
extern void      os_evt_psh  (P_TCB p_CB, U16 set_flags);

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/

