/*----------------------------------------------------------------------------
 *      RL-ARM - RTX
 *----------------------------------------------------------------------------
 *      Name:    RTX_CONFIG.C
 *      Purpose: Configuration of RTX Kernel for Atmel AT91SAM9G25
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <RTL.h>
#include <AT91SAM9G25.H>                 /* AT91SAM9G25 definitions          */

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
 #define OS_STKSIZE     100
#endif

// <q>Check for the stack overflow
// ===============================
// <i> Include the stack checking code for a stack overflow.
// <i> Note that additional code reduces the RTX performance.
#ifndef OS_STKCHECK
 #define OS_STKCHECK    0
#endif

// </h>
// <h>Tick Timer Configuration
// =============================
//   <o>Hardware timer <0=> Timer 0 <1=> PIT
//   <i> Define the on-chip timer used as a time-base for RTX.
//   <i> Default: PIT
#ifndef OS_TIMER
 #define OS_TIMER       1
#endif

//   <q>Common IRQ System Handler for PIT timer
//   <i> Include a code for Common System Interrupt Handler
//   <i> when a PIT timer is used
#ifndef OS_SYSIRQ
 #define OS_SYSIRQ      0
#endif

//   <o>Timer clock value [Hz] <1-1000000000>
//   <i> Set the timer clock value for selected timer.
//   <i> Default PIT: 6000000  (4.03125 MHz at 64.5 MHz MCLK and prescaler by 16)
//   <I> Default TCx: 3000000  (2.01562 MHz at 64.5 MHz MCLK and prescaler by 32)
#ifndef OS_CLOCK
 #define OS_CLOCK       403125
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


#if   (OS_TIMER == 0)                                   /* Timer/Counter 0  */
 #define ID_TC          TC01_IRQn
 #define TCx            TC0
#elif (OS_TIMER == 1)                                   /* PIT Timer        */
 #define ID_TC          SYS_IRQn
#else
 #error OS_TIMER invalid
#endif

#define OS_TIM_         (1 << ID_TC)                    /*  Interrupt Mask  */
#define OS_TRV          ((U32)(((double)OS_CLOCK*(double)OS_TICK)/1E6)-1)

#if (OS_TIMER == 0) 
 #define OS_TVAL        (TCx->TC_CHANNEL[0].TC_CV & 0x0000FFFF)       /*  Timer Value     */
 #define OS_TOVF        ((TCx->TC_CHANNEL[0].TC_SR >> 4) & 1)         /*  Reload Flag     */
 #define OS_TFIRQ()     AIC->AIC_ISCR  = OS_TIM_;       /*  Force Interrupt */
 #define OS_TIACK()     AIC->AIC_ICCR  = OS_TIM_;       /*  Interrupt Ack   */  \
                        AIC->AIC_EOICR = TCx->TC_CHANNEL[0].TC_SR;
 #define OS_TINIT()     PMC->PMC_PCER  = OS_TIM_;       /*  Initialization  */  \
                        TCx->TC_CHANNEL[0].TC_CCR    = TC_CCR_CLKEN | TC_CCR_SWTRG;       \
                        TCx->TC_CHANNEL[0].TC_CMR    = 2 | TC_CMR_CPCTRG;                 \
                        TCx->TC_CHANNEL[0].TC_RC     = OS_TRV;                            \
                        TCx->TC_CHANNEL[0].TC_IER    = TC_IER_CPCS;                       \
                        AIC->AIC_SPU   = (U32)os_def_interrupt;                           \
                        AIC->AIC_SVR[TC01_IRQn] = (U32)os_clock_interrupt;                \
                        AIC->AIC_SMR[TC01_IRQn] =                                         \
                          AIC_SMR_SRCTYPE_INT_EDGE_TRIGGERED | 0;
#else
 #define OS_TVAL        (PIT->PIT_PIIR & 0x000FFFFF)/*  Timer Value     */
 #define OS_TOVF        (PIT->PIT_SR & 1)         /*  Overflow Flag   */
 #define OS_TFIRQ()     SET_IRQFLAG;                     /*  Force Interrupt */                   \
                        AIC->AIC_ISCR      = OS_TIM_;
 #define OS_TIACK()     CLR_IRQFLAG;                     /*  Interrupt Ack   */                   \
                        AIC->AIC_ICCR      = OS_TIM_;                                     \
                        AIC->AIC_EOICR     = PIT->PIT_PIVR;
 #define OS_TINIT()     PIT->PIT_MR = OS_TRV |   /*  Initialization  */                            \
                          PIT_MR_PITIEN | PIT_MR_PITEN;                                   \
                        AIC->AIC_SPU = (U32)os_def_interrupt;                             \
                        AIC->AIC_SVR[SYS_IRQn] = (U32)sys_ctrl_interrupt;                 \
                        AIC->AIC_SMR[SYS_IRQn] =                                          \
                        AIC_SMR_SRCTYPE_INT_EDGE_TRIGGERED | 0;
#endif

#define OS_IACK()       AIC->AIC_EOICR = 0;             /* Interrupt Ack    */

#define OS_LOCK()       AIC->AIC_IDCR  = OS_TIM_;       /* Lock             */
#define OS_UNLOCK()     AIC->AIC_IECR  = OS_TIM_;       /* Unlock           */

/* WARNING: Using IDLE mode might cause you troubles while debugging. */
#define _idle_()        SYS->PMC_SCDR = 1;

#if (OS_TIMER == 1 && OS_SYSIRQ == 1)
 BIT force_irq;
 #define SET_IRQFLAG    force_irq = __TRUE
 #define CLR_IRQFLAG    force_irq = __FALSE
#else
 #define SET_IRQFLAG
 #define CLR_IRQFLAG
#endif

/*----------------------------------------------------------------------------
 *      Global Functions
 *---------------------------------------------------------------------------*/

extern void os_clock_interrupt (void);

#if (OS_TIMER == 1 && OS_SYSIRQ == 1)
/*--------------------------- sys_irq_handler -------------------------------*/

__irq void irq_sys_handler (void) {
  /* Common System Interrupt Handler for: DBGU, RSTC, RTT, WDT and PMC  */
  /* system peripheral interrupts.                                      */

  for(;;);
}


/*--------------------------- sys_ctrl_interrupt ----------------------------*/

__asm void sys_ctrl_interrupt (void) {
   /* Common System Interrupt Handler entry. */
        PRESERVE8
        ARM

        STMDB   SP!,{R0}                        ; Save Work Register
        LDR     R0,=__cpp((U32)&PIT->PIT_SR)     ; PIT Status Register
        LDR     R0,[R0]                         ; Read PIT ISR

        TST     R0,#__cpp(PIT_SR_PITS)          ; Check for PIT interrupt
        LDMIANE SP!,{R0}                        ; Restore Work Register
        LDRNE   PC,=__cpp(os_clock_interrupt)   ; Jump to RTOS Clock IRQ

        LDR     R0,=__cpp(&force_irq)           ; Check for forced interrupt
        LDRB    R0,[R0]                         ; Read os_psh_flag

        CMP     R0,#__cpp(__TRUE)               ; Check if __TRUE
        LDMIA   SP!,{R0}                        ; Restore Work Register
        LDREQ   PC,=__cpp(os_clock_interrupt)   ; Jump to RTOS Clock IRQ

        LDR     PC,=__cpp(irq_sys_handler)      ; Jump to SYS IRQ Handler
}
#else
 #define sys_ctrl_interrupt   os_clock_interrupt
#endif


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

static void os_def_interrupt (void) __irq  {
  /* Default Interrupt Function: may be called when timer ISR is disabled */
  OS_IACK();
}


#include <RTX_lib.c>

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
