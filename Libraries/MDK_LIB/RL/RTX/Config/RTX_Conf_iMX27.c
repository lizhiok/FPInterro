/*----------------------------------------------------------------------------
 *      RL-ARM - RTX
 *----------------------------------------------------------------------------
 *      Name:    RTX_CONFIG.C
 *      Purpose: Configuration of RTX Kernel for Freescale i.MX27
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <RTL.h>
#include <iMX27.h>                       /* i.MX27 definitions               */

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
//   <o>RTX Kernel timer 
//     <1=> General Purpose Timer 1
//     <2=> General Purpose Timer 2
//     <3=> General Purpose Timer 3
//     <4=> General Purpose Timer 4
//     <5=> General Purpose Timer 5
//     <6=> General Purpose Timer 6
//   <i> Define the on-chip timer used as a time-base for RTX.
//   <i> Default: General Purpose Timer 6
#ifndef OS_TIMER
 #define OS_TIMER       6
#endif

//   <o>Timer clock value [Hz] <1-1000000000>
//   <i> Set the timer clock value for selected timer.
//   <i> Default: 14000000  (PERCLK1 = (266/19) = 14 MHz)
#ifndef OS_CLOCK
 #define OS_CLOCK       14000000
#endif

//   <o>Timer tick value [us] <1-1000000>
//   <i> Set the timer tick value for selected timer (in microseconds).
//   <i> Default: 10000  (10000)
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

#if   (OS_TIMER == 1)                                   /* GPT1             */
 #define TIMx_BASE     GPT1_BASE
 #define OS_TID_       26                               /*  Timer ID        */
#elif (OS_TIMER == 2)                                   /* GPT2             */
 #define TIMx_BASE     GPT2_BASE
 #define OS_TID_       25                               /*  Timer ID        */
#elif (OS_TIMER == 3)                                   /* GPT3             */
 #define TIMx_BASE     GPT3_BASE
 #define OS_TID_       24                               /*  Timer ID        */
#elif (OS_TIMER == 4)                                   /* GPT4             */
 #define TIMx_BASE     GPT4_BASE
 #define OS_TID_       4                                /*  Timer ID        */
#elif (OS_TIMER == 5)                                   /* GPT5             */
 #define TIMx_BASE     GPT5_BASE
 #define OS_TID_       3                                /*  Timer ID        */
#elif (OS_TIMER == 6)                                   /* GPT6             */
 #define TIMx_BASE     GPT6_BASE
 #define OS_TID_       2                                /*  Timer ID        */
#else
 #error OS_TIMER invalid
#endif

#define TCTL            (*(U32 *)(TIMx_BASE + 0x00))
#define TPRER           (*(U32 *)(TIMx_BASE + 0x04))
#define TCMP            (*(U32 *)(TIMx_BASE + 0x08))
#define TCR             (*(U32 *)(TIMx_BASE + 0x0C))
#define TCN             (*(U32 *)(TIMx_BASE + 0x10))
#define TSTAT           (*(U32 *)(TIMx_BASE + 0x14))

#define OS_TIM_         (1 << OS_TID_)                  /*  Interrupt Mask  */
#define OS_TRV          ((U32)(((double)OS_CLOCK*(double)OS_TICK)/1E6)-1)
#define OS_TVAL         (TCN)                           /*  Timer Value     */
#define OS_TOVF         (TSTAT & 0x01)                  /*  Reload Flag     */
#define OS_TFIRQ()      AITC_INTFRCL  |= OS_TIM_;       /*  Force Interrupt */
#define OS_TIACK()      TSTAT          = 1;             /*  Interrupt Ack   */
#define OS_TINIT()      TCTL           = (1 << 10) |    /*  Initialization  */ \
                                         (1 <<  4) |                           \
                                         (1 <<  1) ;                           \
                        TPRER          = 0;                                    \
                        TCMP           = OS_TRV;                               \
                        TCTL          |= 1;

#define OS_IACK()       ;                               /* Interrupt Ack    */

#define OS_LOCK()       AITC_INTDISNUM = OS_TID_;       /* Lock             */
#define OS_UNLOCK()     AITC_INTENNUM  = OS_TID_;       /* Unlock           */

/* WARNING: Using IDLE mode might cause you troubles while debugging. */
#define _idle_()        ;


/*----------------------------------------------------------------------------
 *      Global Functions
 *---------------------------------------------------------------------------*/

extern void os_clock_interrupt (void);
extern void os_def_interrupt   (void) __irq;

/*--------------------------- IRQ_Handler -----------------------------------*/

/* User Interrupt Functions
extern __irq void IRQx (void);                 // User IRQx Function
extern __irq void IRQy (void);                 // User IRQy Function

#define mIRQx       (1 <<  7)                  // User IRQx Mask
#define mIRQy       (1 << 10)                  // User IRQy Mask
*/

__asm void IRQ_Handler_RTX (void) {
   /* Common IRQ Handler */
        PRESERVE8
        ARM

        STMDB   SP!,{R0}                       ; Save R0
        LDR     R0,=__cpp((U32)&AITC_NIPNDL)   ; Load NIPNDL Address
        LDR     R0,[R0]                        ; Load NIPNDL Value

//      TST     R0,#mIRQx                      ; Check IRQx Flag
//      LDMNEIA SP!,{R0}                       ; Restore R0
//      LDRNE   PC,=__cpp(IRQx)                ; IRQx Function

//      TST     R0,#mIRQy                      ; Check IRQy Flag
//      LDMNEIA SP!,{R0}                       ; Restore R0
//      LDRNE   PC,=__cpp(IRQy)                ; IRQy Function

        TST     R0,#OS_TIM_                    ; Check Timer Flag
        LDMNEIA SP!,{R0}                       ; Restore R0
        LDRNE   PC,=__cpp(os_clock_interrupt)  ; OS Clock IRQ Function

        LDMIA   SP!,{R0}                       ; Restore R0
        LDR     PC,=__cpp(os_def_interrupt)    ; OS Default IRQ Function
}


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

void os_def_interrupt (void) __irq  {
  /* Default Interrupt Function: may be called when timer ISR is disabled */
  OS_IACK();
}


#include <RTX_lib.c>

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
