/*----------------------------------------------------------------------------
 *      RL-ARM - RTX
 *----------------------------------------------------------------------------
 *      Name:    RTX_CONFIG.C
 *      Purpose: Configuration of RTX Kernel for TI TMS470R1
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <RTL.h>
#include <TMS470R1.h>                    /* TMS470R1 definitions             */

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
//   <o>Hardware timer <0=> RTI
//   <i> Define the on-chip timer used as a time-base for RTX.
//   <i> Default: RTI
#ifndef OS_TIMER
 #define OS_TIMER       0
#endif

//   <o>Timer clock value [Hz] <1-1000000000>
//   <i> Set the timer clock value for selected timer.
//   <i> Default: 60000000  (SYSCLK = XTAL*8/1 = 60.0 MHz)
#ifndef OS_CLOCK
 #define OS_CLOCK       60000000
#endif

//   <o>Timer tick value [us] <1-1000000>
//   <i> Set the timer tick value for selected timer (in microseconds).
//   <i> Default: 10000  (10ms = 10000us)
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

#if   (OS_TIMER == 0)                                   /* RTI              */
 #define OS_PRES_       (1000)                          /*  Prescale value  */
 #define OS_MOD_        ((OS_PRES_ - 1) & 0x7FF)        /*  Modulo M reload */
 #define OS_TIM_        (1 <<  1)                       /*  Interrupt Mask  */
 #define OS_FRC_        (1 << 21)                       /*  Force Irq Mask  */
 #define OS_CMP2_EN_    (1 << 4)                        /*  Enable CMP2 Irq */
 #define OS_CMP2_FLG_   (1 << 6)                        /*  CMP2 Irq Flag   */
 #define OS_TRV         ((U32)(((double)OS_CLOCK*(double)OS_TICK)/1E6))
 #define OS_TVAL        (pSM->RTICNTR >> 11)            /*  Timer Value     */
 #define OS_TOVF        (pSM->RTICINT & OS_CMP2_FLG_)   /*  Overflow Flag   */
 #define OS_TREL()      pSM->RTICNTR = 0;               /*  Timer Reload    */
 #define OS_TFIRQ()     pSM->SSIR = (0x75 << 8) | 0xFF; /*  Force Interrupt */
 #define OS_TIACK()     if ((pSM->SSIF & 1) &&          /*  Interrupt Ack   */ \
                           ((pSM->SSIR & 0xFF) == 0xFF)) {                     \
                          pSM->SSIF    =  0;                                   \
                          pSM->INTREQ &= ~OS_FRC_;                             \
                        }                                                      \
                        pSM->RTICINT  &= ~OS_CMP2_FLG_;                        \
                        pSM->INTREQ   &= ~OS_TIM_;
 #define OS_TINIT()     pSM->RTIPCTL   =  (7 << 11) |   /*  Initialization  */ \
                                          OS_MOD_;                             \
                        pSM->RTICMP2   =  (OS_TRV / OS_PRES_);                 \
                        pSM->RTICINT   =  0;                                   \
                        pSM->RTICINT   =  OS_CMP2_EN_;                         \
                        pSM->RTICNTR   =  0;                                   \
                        pSM->REQMASK  |=  OS_TIM_ | OS_FRC_;
#else
 #error OS_TIMER invalid
#endif

#define OS_IACK()       ;                               /* Interrupt Ack    */

#define OS_LOCK()       pSM->REQMASK &= ~(OS_TIM_ | OS_FRC_);   /* Lock     */
#define OS_UNLOCK()     pSM->REQMASK |=  (OS_TIM_ | OS_FRC_);   /* Unlock   */

/* WARNING: Using IDLE mode might cause you troubles while debugging. */
#define _idle_()        ;


/*----------------------------------------------------------------------------
 *      Global Functions
 *---------------------------------------------------------------------------*/

extern void os_clock_interrupt (void);
extern void os_def_interrupt   (void) __irq;

/*--------------------------- IRQ_Handler -----------------------------------*/

//extern __irq void GPIOA_IRQHandler (void);
//extern __irq void HET_IRQHandler   (void);

//#define GPIOA_IRQ       (1 << 5)
//#define HET_IRQ         (1 << 7)


__asm void IRQ_Handler (void) {
   /* Common IRQ Handler */
        PRESERVE8
        ARM

        STMDB   SP!,{R0}                       ; Save R0
        LDR     R0,=__cpp((U32)&pSM->INTREQ)   ; Load INTREQ Address
        LDR     R0,[R0]                        ; Load INTREQ Value

        TST     R0,#OS_TIM_                    ; Check RTI_COMP2_IRQ Flag
        LDMNEIA SP!,{R0}                       ; Restore R0
        LDRNE   PC,=__cpp(os_clock_interrupt)  ; OS Clock IRQ Function

//      TST     R0,#GPIOA_IRQ                  ; Check GPIOA_IRQ Flag
//      LDMNEIA SP!,{R0}                       ; Restore R0
//      LDRNE   PC,=__cpp(GPIOA_IRQHandler)    ; GPIOA_IRQ Function

//      TST     R0,#HET_IRQ                    ; Check HET_IRQ Flag
//      LDMNEIA SP!,{R0}                       ; Restore R0
//      LDRNE   PC,=__cpp(HET_IRQHandler)      ; HET_IRQ Function

        TST     R0,#OS_FRC_                    ; Check SYSTEM_IRQ Flag
        LDRNE   R0,=__cpp((U32)&pSM->SSIR)     ; Load SSIR Address
        LDRNE   R0,[R0]                        ; Load SSIR Value
        ANDNE   R0,R0, #0xFF                   ; Leave only SSDATA
        TEQ     R0,#0xFF                       ; Check if forced timer IRQ
        LDMEQIA SP!,{R0}                       ; Restore R0
        LDREQ   PC,=__cpp(os_clock_interrupt)  ; OS Clock IRQ Function

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
