/*----------------------------------------------------------------------------
 *      RL-ARM - RTX
 *----------------------------------------------------------------------------
 *      Name:    RTX_CONFIG.C
 *      Purpose: Configuration of RTX Kernel for OKI ML674000/ML675000
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <RTL.h>
#include <ML675001.H>

#define REG(x) (*((volatile unsigned int *)(x)))

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
//   <o>Hardware timer <0=> System Timer
//   <i> Define the on-chip timer used as a time-base for RTX.
//   <i> Default: System Timer
#ifndef OS_TIMER
 #define OS_TIMER       0
#endif

//   <o>Timer clock value [Hz] <1-1000000000>
//   <i> Set the timer clock value for selected timer.
//   <i> Default: 3750000  (3.75MHz at 60MHz CCLK and prescaler by 16)
#ifndef OS_CLOCK
 #define OS_CLOCK       3750000
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

#if   (OS_TIMER == 0)                                   /* PIT              */
 #define OS_TRV         ((U32)(((double)OS_CLOCK*(double)OS_TICK)/1E6))
 #define OS_TVAL        ((REG(TMOVF) & TMOVF_OVF) ? 0:1)/*  Timer Value     */
 #define OS_TOVF        (REG(TMOVF) & TMOVF_OVF)        /*  Overflow Flag   */
 #define OS_TFIRQ()     REG(IRQS) |=  IRQS_IRQS;        /*  Force Interrupt */
 #define OS_TIACK()     REG(IRQS) &= ~IRQS_IRQS;        /*  Interrupt Ack   */ \
                        REG(TMOVF) = TMOVF_OVF;                                \
                        REG(CILCL) = 0;
 #define OS_TINIT()     REG(TMRLR) = 0x10000 - OS_TRV;  /*  Initialization  */ \
                        REG(TMEN)  = TMEN_TCEN;
#else
 #error OS_TIMER invalid
#endif


#define OS_LOCK()       REG(ILC0) &= ~ILC0_ILR0;                /* Lock     */ \
                        REG(ILC1) &= ~ILC1_ILR8;
#define OS_UNLOCK()     REG(ILC0) |=  ILC0_ILR0 & ILC0_INT_LV1; /* Unlock   */ \
                        REG(ILC1) |=  ILC1_ILR8 & ILC1_INT_LV1;

/* WARNING: Using IDLE mode might cause you troubles while debugging. */
#define _idle_()        ;


/*----------------------------------------------------------------------------
 *      Global Functions
 *---------------------------------------------------------------------------*/

extern void os_clock_interrupt (void);

void (*IRQ_ISR[32])(void);                 /* IRQ Interrupt Service Routines */

/*--------------------------- IRQ_Handler -----------------------------------*/

__asm void IRQ_Handler (void) {
   /* Common IRQ Interrupt Service Routine */
        PRESERVE8
        ARM

        STMFD   SP!,{R0}                       ; Save Work Register
        LDR     R0,=__cpp(IRN)                 ; IRQ Number Register
        LDR     R0,[R0]                        ; Read IRQ Number
        CMP     R0,#0                          ; Check for System Timer IRQ
        BEQ     irqsys                         ; Jump to System IRQ
        CMP     R0,#8                          ; Check for Software IRQ
        BEQ     irqsys                         ; Jump to System IRQ
        SUB     LR,LR,#4                       ; Update Link Register
        STMFD   SP!,{R1-R12}                   ; Save Workspace to Stack
        MOV     R12,R0                         ; Save IRQ Number to R12
        LDR     R0,[SP,#(12*4)]                ; Restore R0 Register
        STR     LR,[SP,#(12*4)]                ; Save Link Register
        STMFD   SP!,{R0}                       ; Save R0 Register
        ADR     LR,irqret                      ; Setup Return Point
        LDR     R1,=__cpp(IRQ_ISR)             ; ISR Table
        LDR     R0,[R1,R12,LSL #2]             ; Get ISR Address
        BX      R0                             ; Branch to ISR

irqsys  LDMFD   SP!,{R0}                       ; Restore Work Register
        LDR     PC,=__cpp(os_clock_interrupt)  ; Jump to RTOS Clock IRQ

irqret  LDR     R0,=__cpp(CILCL)               ; Current IRQ Level Clear Register
        MOV     R1,#0
        STR     R1,[R0]                        ; Clear Current Interrupt Level
        LDMFD   SP!,{R0-R12,PC}^               ; Return to program
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
