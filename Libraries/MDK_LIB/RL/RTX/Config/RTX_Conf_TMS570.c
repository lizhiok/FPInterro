/*----------------------------------------------------------------------------
 *      RL-ARM - RTX
 *----------------------------------------------------------------------------
 *      Name:    RTX_CONFIG.C
 *      Purpose: Configuration of RTX Kernel for TI TMS570
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <RTL.h>
#include <TMS570.H>                       /* TMS570 definitions              */

/*----------------------------------------------------------------------------
 *      RTX User configuration part BEGIN
 *---------------------------------------------------------------------------*/

//-------- <<< Use Configuration Wizard in Context Menu >>> -----------------
//
// <h>Task Configuration
// =====================
//
//   <o>Number of concurrent running tasks <0-250>
//   <i> Define max. number of tasks that will run at the same time.
//   <i> Default: 6
#ifndef OS_TASKCNT
 #define OS_TASKCNT     7
#endif

//   <o>Number of tasks with user-provided stack <0-250>
//   <i> Define the number of tasks that will use a bigger stack.
//   <i> The memory space for the stack is provided by the user.
//   <i> Default: 0
#ifndef OS_PRIVCNT
 #define OS_PRIVCNT     0
#endif

//   <o>Task stack size [bytes] <20-4096:8><#/4>
//   <i> Set the stack size for tasks which is assigned by the system.
//   <i> Default: 500
#ifndef OS_STKSIZE
 #define OS_STKSIZE     128
#endif

// <q>Check for the stack overflow
// ===============================
// <i> Include the stack checking code for a stack overflow.
// <i> Note that additional code reduces the RTX performance.
#ifndef OS_STKCHECK
 #define OS_STKCHECK    1
#endif

// <q>Run in privileged mode
// =========================
// <i> Run all Tasks in privileged (SYS) mode.
// <i> Default: Unprivileged (USER)
#ifndef OS_RUNPRIV
 #define OS_RUNPRIV     1
#endif

// </h>
// <h>Tick Timer Configuration
// =============================
//   <o>Hardware timer <0=> RTI-Block 0 <1=> RTI-Block 1
//   <i> Define the on-chip timer used as a time-base for RTX.
//   <i> Default: RTI-Block 0
#ifndef OS_TIMER
 #define OS_TIMER       0
#endif

//   <o>Timer clock value [Hz] <1-1000000000>
//   <i> Set the timer clock value for selected timer.
//   <i> Default: 80000000  (80 MHz)
#ifndef OS_CLOCK
 #define OS_CLOCK       80000000
#endif

//   <o>Timer tick value [us] <1-1000000>
//   <i> Set the timer tick value for selected timer.
//   <i> Default: 10000  (10ms)
#ifndef OS_TICK
 #define OS_TICK        10000
#endif

// </h>

// <h>System Configuration
// =======================
// <e>Round-Robin Task switching
// =============================
// <i> Enable Round-Robin Task switching.
#ifndef OS_ROBIN
 #define OS_ROBIN       1
#endif

//   <o>Round-Robin Timeout [ticks] <1-1000>
//   <i> Define how long a task will execute before a task switch.
//   <i> Default: 5
#ifndef OS_ROBINTOUT
 #define OS_ROBINTOUT   5
#endif

// </e>

//   <o>Number of user timers <0-250>
//   <i> Define max. number of user timers that will run at the same time.
//   <i> Default: 0  (User timers disabled)
#ifndef OS_TIMERCNT
 #define OS_TIMERCNT    0
#endif

//   <o>ISR FIFO Queue size<4=>   4 entries  <8=>   8 entries
//                         <12=> 12 entries  <16=> 16 entries
//                         <24=> 24 entries  <32=> 32 entries
//                         <48=> 48 entries  <64=> 64 entries
//                         <96=> 96 entries
//   <i> ISR functions store requests to this buffer,
//   <i> when they are called from the IRQ handler.
//   <i> Default: 16 entries
#ifndef OS_FIFOSZ
 #define OS_FIFOSZ      16
#endif

// </h>


//------------- <<< end of configuration section >>> -----------------------

// Standard library system mutexes
// ===============================
//  Define max. number system mutexes that are used to protect 
//  the arm standard runtime library. For microlib they are not used.
#ifndef OS_MUTEXCNT
 #define OS_MUTEXCNT    8
#endif

/*----------------------------------------------------------------------------
 *      RTX User configuration part END
 *---------------------------------------------------------------------------*/

#define pRTI            ((RTI_ST *)RTI)
#define pVIM            ((VIM_ST *)VIM)
#define pVIM_RAM        ((VIM_RAM_ST *)VIM_RAM)
#define pSYS            (SYSTEM_1)

#if   (OS_TIMER == 0)                                 /* RTI Counter Block 0 */
 #define CB_            0
 #define RTICPUCx_UL    RTICPUC0_UL
#elif (OS_TIMER == 1)                                 /* RTI Counter Block 1 */ 
 #define CB_            1
 #define RTICPUCx_UL    RTICPUC1_UL
#else
 #error OS_TIMER invalid
#endif

#define OS_TIM_         (1<<21) | (1<<2)                /* Interrupt Mask   */
#define OS_TRV          ((U32)(((double)OS_CLOCK*(double)OS_TICK)/1E6)-1)
#define OS_TVAL         (pRTI->RTIUC0_UL)               /* Timer Value      */
                                                        /* Overflow Flag    */
#define OS_TOVF         (pRTI->RTIINTFLAG_UN.RTIINTFLAG_ST.INT0_B1)
#define OS_TFIRQ()      pSYS->SSISR1        = 0x7500;   /* Force Interrupt  */
#define OS_TIACK()      pSYS->SSIF          = 0x01;     /* Interrupt Ack    */ \
                        pRTI->RTIINTFLAG_UN.RTIINTFLAG_UL = 1;
#define OS_TINIT()      pVIM_RAM->ISR[2+1]  =           /* Initialization   */ \
                        pVIM_RAM->ISR[21+1] = (FuncPTR)os_clock_interrupt;     \
                        pRTI->RTICPUCx_UL   = OS_TRV;                          \
                        pRTI->RTICOMP0_UL   = 1;                               \
                        pRTI->RTIUDCP0_UL   = 1;                               \
                        pRTI->RTICOMPCTRL_UN.RTICOMPCTRL_ST.COMPSEL0_B1 = CB_; \
                        pRTI->RTISETINT_UN.RTISETINT_ST.SETINT0_B1 = 1;        \
                        pRTI->RTIGCTRL_UN.RTIGCTRL_UL   |= (1 << CB_);

#define OS_LOCK()       pVIM->REQMASKCLR0_UL = OS_TIM_; /* Task Lock        */
#define OS_UNLOCK()     pVIM->REQMASKSET0_UL = OS_TIM_; /* Task Unlock      */


/*----------------------------------------------------------------------------
 *      Global Functions
 *---------------------------------------------------------------------------*/

/*--------------------------- os_idle_demon ---------------------------------*/

__task void os_idle_demon (void) {
  /* The idle demon is a system task, running when no other task is ready */
  /* to run. The 'os_xxx' function calls are not allowed from this task.  */

  for (;;) {
  /* HERE: include optional user code to be executed when no task runs.*/
  }
}


/*--------------------------- os_tmr_call -----------------------------------*/

void os_tmr_call (U16 info) {
  /* This function is called when the user timer has expired. Parameter  */
  /* 'info' holds the value, defined when the timer was created.         */

  /* HERE: include optional user code to be executed on timeout. */
}


/*--------------------------- os_error --------------------------------------*/

void os_error (U32 err_code) {
  /* This function is called when a runtime error is detected. Parameter */
  /* 'err_code' holds the runtime error code (defined in RTL.H).         */

  /* HERE: include optional code to be executed on runtime error. */
  for (;;);
}


/*----------------------------------------------------------------------------
 *      RTX Configuration Functions
 *---------------------------------------------------------------------------*/


#include <RTX_lib.c>

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
