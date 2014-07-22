/*----------------------------------------------------------------------------
 *      RL-ARM - RTX
 *----------------------------------------------------------------------------
 *      Name:    RTX_CONFIG.C
 *      Purpose: Configuration of RTX Kernel for ST STR73x
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <RTL.h>
#include <73x_lib.H>

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
 #define OS_TASKCNT     6
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
//   <i> Default: 200
#ifndef OS_STKSIZE
 #define OS_STKSIZE     50
#endif

// <q>Check for the stack overflow
// ===============================
// <i> Include the stack checking code for a stack overflow.
// <i> Note that additional code reduces the RTX performance.
#ifndef OS_STKCHECK
 #define OS_STKCHECK    1
#endif

// </h>
// <h>Tick Timer Configuration
// =============================
//   <o>Hardware timer <1=> Timer 1 <2=> Timer 2 <3=> Timer 3
//                     <4=> Timer 4 <5=> Timer 5
//   <i> Define the on-chip timer used as a time-base for RTX.
//   <i> Default: Timer 1
#ifndef OS_TIMER
 #define OS_TIMER       1
#endif

//   <o>Timer clock value [Hz] <1-1000000000>
//   <i> Set the timer clock value for selected timer.
//   <i> Default: 32000000  (32MHz MCLK)
#ifndef OS_CLOCK
 #define OS_CLOCK       32000000
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

#if   (OS_TIMER == 1)                                   /* Timer 1          */
 #define OS_TID_        17                              /*  Timer ID        */
 #define PCGR_VAL       7
 #define TIMx           TIM1
 #define PCGRx          PCGR0
#elif (OS_TIMER == 2)                                   /* Timer 2          */
 #define OS_TID_        18                              /*  Timer ID        */
 #define PCGR_VAL       16
 #define TIMx           TIM2
 #define PCGRx          PCGR1
#elif (OS_TIMER == 3)                                   /* Timer 3          */
 #define OS_TID_        19                              /*  Timer ID        */
 #define PCGR_VAL       17
 #define TIMx           TIM3
 #define PCGRx          PCGR1
#elif (OS_TIMER == 4)                                   /* Timer 4          */
 #define OS_TID_        20                              /*  Timer ID        */
 #define PCGR_VAL       18
 #define TIMx           TIM4
 #define PCGRx          PCGR1
#elif (OS_TIMER == 5)                                   /* Timer 5          */
 #define OS_TID_        24                              /*  Timer ID        */
 #define PCGR_VAL       3
 #define TIMx           TIM5
 #define PCGRx          PCGR1
#else
 #error OS_TIMER invalid
#endif

#define OS_TIM_         (1 << OS_TID_)                  /*  Interrupt Mask  */
#define OS_TRV          ((U16)(((double)OS_CLOCK*(double)OS_TICK)/1E8)-5)
#define OS_TVAL         ((TIMx->CNTR + 4) & 0xFFFF)     /*  Timer Value     */
#define OS_TOVF          (TIMx->SR & TIM_FLAG_OCB)      /*  Overflow Flag   */
#define OS_TFIRQ()      TIMx->CR2 |=  TIM_FLAG_TO;      /*  Force Interrupt */
#define OS_TIACK()      TIMx->CR2 &= ~TIM_FLAG_TO;      /*  Interrupt Ack   */ \
                        TIMx->SR   = ~TIM_FLAG_OCB;                            \
                        EIC->IPR0  =  OS_TIM_;
#define OS_TINIT()      CFG->PCGRx|=  1 << PCGR_VAL;    /*  Initialization  */ \
                        TIMx->OCBR =  OS_TRV;                                  \
                        TIMx->CR2  =  TIM_FLAG_OCB | (100 - 1);                \
                        TIMx->CR1  =  TIM_PWM_Mask | TIM_ENABLE_Mask;          \
                        EIC->IVR   =  (U32)os_clock_interrupt;                 \
                        EIC->SIRn[OS_TID_] = 1 | ((U32)os_clock_interrupt<<16);\
                        EIC->IER0 |=  OS_TIM_;


#define OS_LOCK()       EIC->ICR  &= ~0x0001;           /* Task Lock        */
#define OS_UNLOCK()     EIC->ICR  |=  0x0001;           /* Task Unlock      */

/* WARNING: Using IDLE mode might cause you troubles while debugging. */
#define _idle_()        RCCU->SMR &= ~1;


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
