/*----------------------------------------------------------------------------
 *      RL-ARM - RTX
 *----------------------------------------------------------------------------
 *      Name:    RTX_CONFIG.C
 *      Purpose: Configuration of RTX Kernel for ROHM BU1511KV2
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <RTL.h>
#include <BU1511KV2.h>                   /* BU1511KV2 definitions            */

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
//   <o>Hardware timer <0=> Timer 1 <1=> Timer 2
//   <i> Define the on-chip timer used as a time-base for RTX.
//   <i> Default: Timer 1
#ifndef OS_TIMER
 #define OS_TIMER       0
#endif

//   <o>Timer clock value [Hz] <1-1000000000>
//   <i> Set the timer clock value for selected timer.
//   <i> Default: 40500000 (40.5MHz)
#ifndef OS_CLOCK
 #define OS_CLOCK       40500000
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

#if   (OS_TIMER == 0)                                   /* Timer 1          */
 #define TMRx   ROHM_TMR1
#elif (OS_TIMER == 1)                                   /* Timer 2          */
 #define TMRx   ROHM_TMR2
#else
 #error OS_TIMER invalid
#endif

#define OS_TIM_         (1 << OS_TIMER)                 /*  Interrupt Mask  */
#define OS_TRV          ((U32)(((double)OS_CLOCK*(double)OS_TICK)/1E6)-1)
#define OS_TVAL         (OS_TRV - TMRx->CurrentValue)   /*  Timer Value     */
#define OS_TOVF         (TMRx->STAT)                    /*  Overflow Flag   */
#define OS_TFIRQ()      force_irq = __TRUE;             /*  Force Interrupt */
#define OS_TIACK()      U32 volatile eoi = TMRx->EOI;   /*  Interrupt Ack   */ \
                        force_irq = __FALSE;
#define OS_TINIT()      U32 volatile eoi;               /*  Initialization  */ \
                        TMRx->LoadCount = OS_TRV;                              \
                        TMRx->ControlReg= 0x03;                                \
                        eoi = TMRx->EOI;                                       \
                        ROHM_INTCTL->irq_inten_I  |= OS_TIM_;

#define OS_LOCK()       ROHM_INTCTL->irq_intmask_I = 0xFFFF;   /* Lock      */
#define OS_UNLOCK()     ROHM_INTCTL->irq_intmask_I = 0x0000;   /* Unlock    */ 


/*----------------------------------------------------------------------------
 *      Global Functions
 *---------------------------------------------------------------------------*/

extern void os_clock_interrupt (void);

void (*IRQ_ISR[16])(void);                 /* IRQ Interrupt Service Routines */
BIT force_irq;

/*--------------------------- IRQ_Handler -----------------------------------*/

__asm void IRQ_Handler (void) {
   /* Common IRQ Interrupt Service Routine */
        PRESERVE8
        ARM

        STMDB   SP!,{R0}                       ; Save R0
        LDR     R0,=__cpp((U32)&ROHM_INTCTL->irq_finalstatus_I)
        LDR     R0,[R0]                        ; Load IRQ active flags

        TST     R0,#OS_TIM_                    ; Check RTX Timer Flag
        LDMNEIA SP!,{R0}                       ; Restore R0
        LDRNE   PC,=__cpp(os_clock_interrupt)  ; OS Clock IRQ Function

        SWP     LR,LR,[SP]                     ; Save LR to Stack
        STMDB   SP!,{R1-R3,R12}                ; Save Workspace to Stack
        STMDB   SP!,{LR}                       ; Save Original R0 Value

        LDR     R1,=__cpp(IRQ_ISR)             ; ISR Table
        ADR     LR,irqret                      ; Setup Return Point

        LSL     R0,R0,#16                      ; IRQ Flags &= 0xFFFF
        LSRS    R0,R0,#16

irqnext LSRS    R0,R0,#1                       ; IRQ Flag -> Carry
        LDRCS   R1,[R1]                        ; Get ISR Address
        BXCS    R1                             ; Branch to ISR

        ADD     R1,R1,#4                       ; Next ISR Address
        BNE     irqnext

irqret  LDR     R1,=__cpp(&force_irq)          ; Check the force_irq
        LDRB    R0,[R1]
        CMP     R0,#__cpp(__TRUE)
        LDMIA   SP!,{R0-R3,R12,LR}             ; Restore Workspace from Stack
        LDREQ   PC,=__cpp(os_clock_interrupt)  ; run task manager if set
        SUBS    PC,LR,#4                       ; Return to program if not
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


#include <RTX_lib.c>

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
