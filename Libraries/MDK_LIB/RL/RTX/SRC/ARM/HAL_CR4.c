/*----------------------------------------------------------------------------
 *      RL-ARM - RTX
 *----------------------------------------------------------------------------
 *      Name:    HAL_CR4.C
 *      Purpose: Hardware Abstraction Layer for Cortex-R4
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include "rt_TypeDef.h"
#include "RTX_Config.h"
#include "rt_System.h"
#include "rt_Task.h"
#include "rt_List.h"
#include "rt_HAL.h"


/*----------------------------------------------------------------------------
 *      Global Variables
 *---------------------------------------------------------------------------*/

U32 sys_stktop;
BIT vfp_en;

/*----------------------------------------------------------------------------
 *      Functions
 *---------------------------------------------------------------------------*/


__weak void __SWI_3 (void) {
  /* Reserved function. */
}

__weak void __SWI_4 (void) {
  /* Reserved function. */
}

__weak void __SWI_6 (void) {
  /* Reserved function. */
}

__weak void __SWI_7 (void) {
  /* Reserved function. */
}


/*--------------------------- os_set_env ------------------------------------*/

__asm void os_set_env (P_TCB p_TCB) {
  /* Fix up runtime environment to fit idle task. */

        LDR     R0,[R0,#TCB_TSTACK]     ; p_TCB in R0
        MOV     SP,R0
        BX      LR
        ALIGN
}


/*--------------------------- os_sys_init0 ----------------------------------*/

__asm void os_sys_init0 (FUNCP task, U32 prio_stksz, void *stk) {
  /* Save parameters on the USR stack for "os_tsk_create0" function. */

        MOV     R3,SP
        STMDB   SP!,{R0-R2}             ; push parameters on the stack
        LDR     R0,=__cpp(&sys_stktop)  ; save SYS stack pointer
        STR     R3,[R0]

        VMRS    R1,FPSID                ; is VFP enabled? 
        MVN     R1,R1,LSR #23           ; 1= enabled
        AND     R1,#1
        LDR     R0,=__cpp(&vfp_en)      ; save it
        STRB    R1,[R0]

        LDR     R0,=__cpp(os_sys_init1) ; jump to os_sys_init()
        BX      R0
        ALIGN
}


/*--------------------------- os_sys_run ------------------------------------*/

__asm void os_sys_run (FUNCP func) {
  /* Run the function with parameters pushed on the stack. */

        MOV     R12,R0                  ; func
        LDR     R3,=__cpp(&sys_stktop)  ; get SYS stack pointer
        LDR     R3,[R3]
        SUBS    R3,#12                  ; get pushed parameters
        LDMIA   R3!,{R0-R2}
        MOVS    R3,#0
        BX      R12                     ; jump to "func()"
        ALIGN
}

/*--------------------------- os_clock_interrupt ----------------------------*/

__asm void os_clock_interrupt (void) {
        PRESERVE8
        ARM

        EXPORT  os_sys_manager_ret

        STMDB   SP!,{R0-R1}             ; Save Full Context
        STMDB   SP,{SP}^                ; User SP
        MRS     R1,SPSR                 ; User CPSR
        LDMDB   SP,{R0}
        SUB     LR,LR,#0x4
        STMDB   R0!,{R1,LR}             ; Push PC, CPSR
        STMDB   R0,{LR}^                ; Push User LR
        SUB     R0,R0,#0x4              ; Write back problem !!
        STMDB   R0!,{R2-R12}            ; Push R2-R12
        LDMIA   SP!,{R2-R3}
        STMDB   R0!,{R2-R3}             ; Push R0-R1

        VMRS    R1,FPSID                ; is VFP enabled?
        TST     R1,#(1 << 23)
        VMRSEQ  R1,FPSCR                ; yes, stack also VFP registers
        STMDBEQ R0!,{R1,R2}             ; for 8-byte alignment
        VSTMDBEQ R0!,{S0-S31}           ; Push S0-S31

        MOV     R4,R0                   ; os_runtask->tsk_stack
        MSR     CPSR_c,#MODE_SYS :OR: CPSR_I_BIT
        LDR     R0,=__cpp(&sys_stktop)  ; enter SYS mode
        LDR     SP,[R0]                 ; RTX System stack

        LDR     R0,=__cpp(os_irq_ack_lock); os_irq_ack_lock()
        MOV     LR,PC
        BX      R0

        LDR     R0,=__cpp(&os_psh)
        LDRB    R1,[R0,#4];             ; os_psh.flag = os_psh.req
        STRB    R1,[R0,#5]
        MOV     R1,#0
        STRB    R1,[R0,#4]              ; os_psh.req = 0

        MSR     CPSR_c,#MODE_SYS        ; enter SYS mode, IRQ enabled
        MOV     R0,R4
        LDR     R1,=__cpp(os_sys_manager); os_sys_manager()
        BX      R1

os_sys_manager_ret
        MOVS    R4,R0                   ; p_new
        MSR     CPSR_c,#MODE_SVC :OR: CPSR_I_BIT
        LDR     R0,=__cpp(tsk_unlock)   ; tsk_unlock()
        LDR     LR,=Switch_task         ; return to Switch_task
        BX      R0
}


/*-------------------------- SWI_Handler -----------------------------------*/

__asm void SWI_Handler (void) {
        PRESERVE8
        ARM

        IMPORT  SWI_Count
        IMPORT  SWI_Table
        EXPORT  os_switch_tasks_ret

        MRS     R12,SPSR                ; Get SPSR
        TST     R12,#CPSR_T_BIT         ; Check Thumb Bit
        LDRNEH  R12,[LR,#-2]            ; Thumb: Load Halfword
        BICNE   R12,R12,#0xFF00         ;        Extract SWI Number
        LDREQ   R12,[LR,#-4]            ; ARM:   Load Word
        BICEQ   R12,R12,#0xFF000000     ;        Extract SWI Number

        CMP     R12,#0
        BNE     SWI_User

        /*------------------- os_switch_tasks ------------------------*/

        STMDB   SP,{SP}^                ; Save reduced context
        LDR     R3,=SWI_Table
        LDMDB   SP,{R1}                 ; User SP
        MRS     R12,SPSR                ; User CPSR
        STMDB   R1!,{R4-R11,R12,LR}     ; push R4-R11, CPSR, PC

        VMRS    R2,FPSID                ; is VFP enabled?
        TST     R2,#(1 << 23)
        VSTMDBEQ R1!,{S16-S31}          ; Push reduced VFP

        LDR     R3,[R3]                 ; Load SWI Function Address
        BX      R3                      ; Call SWI Function

os_switch_tasks_ret
        MOV     R4,R0                   ; p_new

Switch_task
        LDR     R0,[R4,#TCB_TSTACK]     ; p_new->tsk_stack
        LDRB    R1,[R4,#TCB_FCTX]       ; p_new->full_ctx
        CMP     R1,#0x0
        BNE     Full_ctx

        VMRS    R1,FPSID                ; is VFP enabled?
        TST     R1,#(1 << 23)
        VLDMIAEQ R0!,{S16-S31}          ; Restore reduced VFP
                
        LDMIA   R0!,{R4-R11}            ; Restore Reduced Context
        LDMIA   R0!,{R1,LR}
        STMDB   SP,{R0}                 ; Set User SP
        LDMDB   SP,{SP}^
        MSR     SPSR_cxsf,R1            ; Set User CPSR
        MOVS    PC,LR                   ; RETI

Full_ctx
        VMRS    R1,FPSID                ; is VFP enabled?
        TST     R1,#(1 << 23)
        VLDMIAEQ R0!,{S0-S31}           ; Restore full VFP
        LDMIAEQ R0!,{R1,R2}             ; ! 8-byte alignment
        VMSREQ  FPSCR,R1
        
        ADD     R1,R0,#64               ; Restore Full Context
        STMDB   SP,{R1}
        LDMDB   SP,{SP}^                ; Set User SP
        LDR     R1,[R0,#56]             ; CPSR
        ADD     LR,R0,#52
        LDMIA   LR,{LR}^                ; Set User LR
        MSR     SPSR_cxsf,R1            ; Set User CPSR
        LDMDB   LR,{R0-R12}             ; Restore R0-R12
        LDR     LR,[LR,#8]              ; PC
        MOVS    PC,LR                   ; RETI

        /*------------------- User SWI -------------------------------*/

SWI_User
        STMFD   SP!,{R8,LR}             ; Store R8, LR
        LDR     R8,=SWI_Count
        LDR     R8,[R8]
        CMP     R12,R8                  ; Check for overflow
SWI_Dead
        BHS     SWI_Dead                ; Loop if none Existing SWI
        LDR     R8,=SWI_Table
        LDR     R12,[R8,R12,LSL #2]     ; Load SWI Function Address
        MRS     R8,SPSR                 ; Save SPSR
        MOV     LR,PC                   ; Return Address
        BX      R12                     ; Call SWI Function 
        MSR     SPSR_cxsf,R8            ; Restore SPSR
        LDMFD   SP!,{R8,PC}^            ; Restore R8 and Return
}


/*--------------------------- os_init_context -------------------------------*/

void os_init_stack (P_TCB p_TCB, FUNCP task_body) {
  /* Prepare TCB and saved context for a first time start of a task. */
  U32 *stk,i,size;

  /* Prepare a complete interrupt frame for first task start */
  size = p_TCB->priv_stack >> 2;
  if (size == 0) {
    size = (U16)os_stackinfo >> 2;
  }
  /* Write to the top of stack. */
  stk = &p_TCB->stack[size];

  /* Auto correct to 8-byte ARM stack alignment. */
  if ((U32)stk & 0x04) {
    stk--;
  }

  /* Skip init stack for idle task. */
  if (p_TCB->full_ctx) {
    stk -= 16;
    /* Initial PC and default CPSR */
    stk[15] = (U32)task_body;
    stk[14] = (os_flags & 0x01) ? INIT_CPSR_SYS : INIT_CPSR_USER;
    /* Set T-bit if task function in Thumb mode. */
    if ((U32)task_body & 1) {
      stk[14] |= CPSR_T_BIT;
    }
    /* Assign a void pointer to R0. */
    stk[0]  = (U32)p_TCB->p_msg;
    /* Clear R1-R12,LR registers. */
    for (i = 1; i < 14; i++) {
      stk[i] = 0;
    }
    if (vfp_en == __TRUE) {
      /* Clear VFP registers S0-S31, FPSCR. */
      stk -= 34;
      for (i = 0; i < 34; i++) {
        stk[i] = 0;
      }
    }
  }

  /* Initial Task stack pointer. */
  p_TCB->tsk_stack = (U32)stk;

  /* Task entry point. */
  p_TCB->ptask = task_body;

  /* Set a magic word for checking of stack overflow. */
  p_TCB->stack[0] = MAGIC_WORD;
}


/*--------------------------- os_switch_tasks -------------------------------*/

__naked void __swi(0) os_switch_tasks (P_TCB p_new);
__naked void __SWI_0                  (P_TCB p_new, U32 stk) {
  /* Switch to next task, save old and restore new context. */

  if (os_runtask) {
    /* Do not update TCB if task has deleted itself. */
    os_runtask->tsk_stack= stk;
    os_runtask->full_ctx = __FALSE;

    /* Check for stack overflow. */
    os_stk_check (stk);
  }
  os_runtask   = p_new;
  p_new->state = RUNNING;
  rt_post_taskswitch (p_new->task_id);
  tsk_unlock ();
  os_switch_tasks_ret (p_new);
}


/*--------------------------- os_stk_check ----------------------------------*/

__weak void os_stk_check (U32 stk) {
  /* Check for stack overflow. */
  if (stk == 0) {
    stk = __current_sp ();
  }
  if ((stk < (U32)os_runtask->stack) ||
      (os_runtask->stack[0] != MAGIC_WORD)) {
    os_error (OS_ERR_STK_OVF);
  }
}

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
