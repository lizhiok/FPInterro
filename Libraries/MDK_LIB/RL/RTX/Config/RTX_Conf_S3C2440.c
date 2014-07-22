/*----------------------------------------------------------------------------
 *      RL-ARM - RTX
 *----------------------------------------------------------------------------
 *      Name:    RTX_CONFIG.C
 *      Purpose: Configuration of RTX Kernel for Samsung S3C2440
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <RTL.h>
#include <S3C2440.h>                     /* S3C2440 definitions              */

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
//   <o>Hardware timer <0=> Timer 0 <1=> Timer 1 <2=> Timer 2
//                     <3=> Timer 3 <4=> Timer 4 
//   <i> Define the on-chip timer used as a time-base for RTX.
//   <i> Default: Timer 0
#ifndef OS_TIMER
 #define OS_TIMER       0
#endif

//   <o>Timer clock value [Hz] <1-1000000000>
//   <i> Set the timer clock value for selected timer.
//   <i> Default TimerX: 50000000
//   <i>   Bootloader sets PCLK (which drives timer) to 50MHz
#ifndef OS_CLOCK
 #define OS_CLOCK       50000000
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

#if   (OS_TIMER == 0)                                   /* Timer0           */
 #define TIMx_START     (1 <<  0)
 #define TIMx_MAN       (1 <<  1)
 #define TIMx_AUREL     (1 <<  3)
 #define TCNTBx         TCNTB0
 #define TCMPBx         TCMPB0
 #define TCNTOx         TCNTO0
#elif (OS_TIMER == 1)                                   /* Timer1           */
 #define TIMx_START     (1 <<  8)
 #define TIMx_MAN       (1 <<  9)
 #define TIMx_AUREL     (1 << 11)
 #define TCNTBx         TCNTB1
 #define TCMPBx         TCMPB1
 #define TCNTOx         TCNTO1
#elif (OS_TIMER == 2)                                   /* Timer2           */
 #define TIMx_START     (1 << 12)
 #define TIMx_MAN       (1 << 13)
 #define TIMx_AUREL     (1 << 15)
 #define TCNTBx         TCNTB2
 #define TCMPBx         TCMPB2
 #define TCNTOx         TCNTO2
#elif (OS_TIMER == 3)                                   /* Timer3           */
 #define TIMx_START     (1 << 16)
 #define TIMx_MAN       (1 << 17)
 #define TIMx_AUREL     (1 << 19)
 #define TCNTBx         TCNTB3
 #define TCMPBx         TCMPB3
 #define TCNTOx         TCNTO3
#elif (OS_TIMER == 4)                                   /* Timer4           */
 #define TIMx_START     (1 << 20)
 #define TIMx_MAN       (1 << 21)
 #define TIMx_AUREL     (1 << 22)
 #define TCNTBx         TCNTB4
 #define TCNTOx         TCNTO4
#else
 #error OS_TIMER invalid
#endif

#define OS_MSK          (1 << (OS_TIMER + 10))          /*  Interrupt Mask  */
#define OS_TIM_         (OS_TIMER + 10)                 /*  Interrupt Index */
#define OS_TRV          ((U32)(((double)OS_CLOCK*(double)OS_TICK)/1E8)-1)
#define OS_TVAL         (OS_TRV - TCNTOx)               /*  Timer Value     */
#define OS_TOVF         (correct_tim == 0)              /*  Reload Flag     */
#define OS_TFIRQ()      if (TCNTOx > 3) {               /*  Force Interrupt */ \
                          TCNTBx       = 1;                                    \
                          TCON        &= ~TIMx_AUREL;                          \
                          TCON        |=  TIMx_MAN;                            \
                          TCON        &= ~TIMx_MAN;                            \
                          correct_tim  = TCNTOx - 3;                           \
                        }
#define OS_TIACK()      if (correct_tim) {              /*  Interrupt Ack   */ \
                          TCNTBx       =  correct_tim;                         \
                          TCON        |=  TIMx_MAN;                            \
                          TCON        &= ~TIMx_MAN;                            \
                          TCON        |=  TIMx_AUREL;                          \
                        }                                                      \
                        SRCPND = OS_MSK;                                       \
                        INTPND = OS_MSK;

#if (OS_TIMER < 3) 
 #define OS_TINIT()     TCFG0    |=  (100 - 1);         /*  Initialization  */ \
                        TCNTBx    =  OS_TRV;                                   \
                        TCON     |=  (TIMx_MAN);                               \
                        TCON     &= ~(TIMx_MAN);                               \
                        TCON     |=  TIMx_AUREL;                               \
                        TCON     |=  TIMx_START;                               \
                        INTMSK   &= ~(OS_MSK);
#else
 #define OS_TINIT()     TCFG0    |=  ((100 - 1) << 8);  /*  Initialization  */ \
                        TCNTBx    =  OS_TRV;                                   \
                        TCON     |=  (TIMx_MAN);                               \
                        TCON     &= ~(TIMx_MAN);                               \
                        TCON     |=  TIMx_AUREL;                               \
                        TCON     |=  TIMx_START;                               \
                        INTMSK   &= ~(OS_MSK);
#endif

#define OS_IACK()       SRCPND    = SRCPND;             /*  Interrupt Ack   */ \
                        INTPND    = INTPND;

#define OS_LOCK()       INTMSK   |=  (OS_MSK);          /* Lock             */
#define OS_UNLOCK()     INTMSK   &= ~(OS_MSK);          /* Unlock           */

/* WARNING: Using IDLE mode might cause you troubles while debugging. */
#define _idle_()        CLKCON   |= (1 << 2);

static U16 correct_tim = 0;

/*----------------------------------------------------------------------------
 *      Global Functions
 *---------------------------------------------------------------------------*/

extern void os_clock_interrupt (void);
extern void os_def_interrupt   (void) __irq;

/*--------------------------- IRQ_Handler_RTX -------------------------------*/

/* User Interrupt Functions
extern __irq void IRQx (void);                 // User IRQx Function
extern __irq void IRQy (void);                 // User IRQy Function

#define mIRQx       5                          // User IRQx Index
#define mIRQy       9                          // User IRQy Index
*/

__asm void IRQ_Handler_RTX (void) {
   /* Common IRQ Handler */
        PRESERVE8
        ARM

        STMDB   SP!,{R0}                       ; Save R0
        LDR     R0,=__cpp((U32)&INTOFFSET)     ; Load INTOFFSET Address
        LDR     R0,[R0]                        ; Load INTOFFSET Value

//      TEQ     R0,#mIRQx                      ; Check IRQx Flag
//      LDMEQIA SP!,{R0}                       ; Restore R0
//      LDREQ   PC,=__cpp(IRQx)                ; IRQx Function

//      TEQ     R0,#mIRQy                      ; Check IRQy Flag
//      LDMEQIA SP!,{R0}                       ; Restore R0
//      LDREQ   PC,=__cpp(IRQy)                ; IRQy Function

        TEQ     R0,#(OS_TIM_)                  ; Check OS IRQ Flag
        LDMIA   SP!,{R0}                       ; Restore R0
        LDREQ   PC,=__cpp(os_clock_interrupt)  ; OS Clock IRQ Function

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
