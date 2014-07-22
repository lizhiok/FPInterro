/*----------------------------------------------------------------------------
 *      RL-ARM - RTX
 *----------------------------------------------------------------------------
 *      Name:    RT_MUTEX.H
 *      Purpose: Implements mutex synchronization objects
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

/* Functions */
extern void      os_mut_init    (OS_ID mutex);
extern OS_RESULT os_mut_release (OS_ID mutex);
extern OS_RESULT os_mut_wait    (OS_ID mutex, U16 timeout);

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/

