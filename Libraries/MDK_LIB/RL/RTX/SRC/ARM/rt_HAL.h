/*----------------------------------------------------------------------------
 *      RL-ARM - RTX
 *----------------------------------------------------------------------------
 *      Name:    RT_HAL.H
 *      Purpose: Hardware Abstraction Layer for ARM7/9, Cortex-R4 definitions
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

/* Definitions */
#define MAGIC_WORD      0xE25A2EA5
#if (__TARGET_ARCH_7_R && __BIG_ENDIAN)
 #define INIT_CPSR_SYS  0x4000021F
 #define INIT_CPSR_USER 0x40000210
#else
 #define INIT_CPSR_SYS  0x4000001F
 #define INIT_CPSR_USER 0x40000010
#endif

#define CPSR_T_BIT      0x20
#define CPSR_I_BIT      0x80
#define CPSR_F_BIT      0x40

#define MODE_USR        0x10
#define MODE_FIQ        0x11
#define MODE_IRQ        0x12
#define MODE_SVC        0x13
#define MODE_ABT        0x17
#define MODE_UND        0x1B
#define MODE_SYS        0x1F

/* Variables */
extern U32 sys_stktop;

/* Functions */
#if (__TARGET_ARCH_7_R)
 #define _incw(p)              while(__strex((__ldrex(p)+1),p))
 #define _decw(p)              while(__strex((__ldrex(p)-1),p))
 #define _dec_qc(p)            _decw(p)
 __inline U32 _inc_qi (U32 size, U8 *count, U8 *first) {
  U32 cnt,c2;
  do {
    if ((cnt = __ldrex(count)) == size) {
      __clrex();
      return (cnt); }
  } while (__strex(cnt+1, count));
  do {
    c2 = (cnt = __ldrex(first)) + 1;
    if (c2 == size) c2 = 0;
  } while (__strex(c2, first));
  return (cnt);
 }
#else
 extern void _incw             (U16 *fptr) __swi(3);
 extern void _decw             (U16 *fptr) __swi(4);
 extern U8   _inc_qi           (U8 size, U8 *count, U8 *first);
 extern void _dec_qc           (U8 *count);
#endif
extern void os_set_env         (P_TCB p_TCB);
extern void os_sys_init0       (FUNCP task, U32 prio_stksz, void *stk);
extern void os_sys_run         (FUNCP func);
extern void os_clock_interrupt (void);
extern void os_sys_manager_ret (P_TCB p_new);
extern void SWI_Handler        (void);
extern void os_init_stack      (P_TCB p_TCB, FUNCP task_body);
extern void os_stk_check       (U32 stk);
extern void os_switch_tasks    (P_TCB p_new) __swi(0);
extern void os_switch_tasks_ret(P_TCB p_new);

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/

