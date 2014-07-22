/*----------------------------------------------------------------------------
 *      RL-ARM - RTX
 *----------------------------------------------------------------------------
 *      Name:    MAILBOX.C
 *      Purpose: RTX example program
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <RTL.h>                      /* RTX kernel functions & defines      */
#include <LPC21xx.H>                  /* LPC21xx definitions                 */
#include <stdio.h>

OS_TID tsk1;                          /* assigned identification for task 1  */
OS_TID tsk2;                          /* assigned identification for task 2  */

typedef struct {                      /* Message object structure            */
  float voltage;                      /* AD result of measured voltage       */
  float current;                      /* AD result of measured current       */
  U32   counter;                      /* A counter value                     */
} T_MEAS;

os_mbx_declare (MsgBox,16);           /* Declare an RTX mailbox              */
_declare_box (mpool,sizeof(T_MEAS),16);/* Dynamic memory pool                */

__task void send_task (void);
__task void rec_task (void);

/*----------------------------------------------------------------------------
 *        Initialize serial interface
 *---------------------------------------------------------------------------*/
void init_serial () {
  PINSEL0 = 0x00050000;               /* Enable RxD1 and TxD1                */
  U1LCR = 0x83;                       /* 8 bits, no Parity, 1 Stop bit       */
  U1DLL = 97;                         /* 9600 Baud Rate @ 15MHz VPB Clock    */
  U1LCR = 0x03;                       /* DLAB = 0                            */
}


/*----------------------------------------------------------------------------
 *  Task 1:  RTX Kernel starts this task with os_sys_init (send_task)
 *---------------------------------------------------------------------------*/
__task void send_task (void) {
  T_MEAS *mptr;

  tsk1 = os_tsk_self ();              /* get own task identification number  */
  tsk2 = os_tsk_create (rec_task, 0); /* start task 2                        */
  os_mbx_init (MsgBox, sizeof(MsgBox));/* initialize the mailbox             */
  os_dly_wait (5);                    /* Startup delay for MCB21xx           */

  mptr = _alloc_box (mpool);          /* Allocate a memory for the message   */
  mptr->voltage = 223.72;             /* Set the message content             */
  mptr->current = 17.54;
  mptr->counter = 120786;
  os_mbx_send (MsgBox, mptr, 0xffff); /* Send the message to the mailbox     */
  IOSET1 = 0x10000;
  os_dly_wait (100);

  mptr = _alloc_box (mpool);
  mptr->voltage = 227.23;             /* Prepare a 2nd message               */
  mptr->current = 12.41;
  mptr->counter = 170823;
  os_mbx_send (MsgBox, mptr, 0xffff); /* And send it.                        */
  os_tsk_pass ();                     /* Cooperative multitasking            */
  IOSET1 = 0x20000;
  os_dly_wait (100);

  mptr = _alloc_box (mpool);
  mptr->voltage = 229.44;             /* Prepare a 3rd message               */
  mptr->current = 11.89;
  mptr->counter = 237178;
  os_mbx_send (MsgBox, mptr, 0xffff); /* And send it.                        */
  IOSET1 = 0x40000;
  os_dly_wait (100);

  os_tsk_delete_self ();              /* We are done here, delete this task  */
}

/*----------------------------------------------------------------------------
 *  Task 2: RTX Kernel starts this task with os_tsk_create (rec_task, 0)
 *---------------------------------------------------------------------------*/
__task void rec_task (void) {
  T_MEAS *rptr;

  for (;;) {
    os_mbx_wait (MsgBox, (void **)&rptr, 0xffff); /* wait for the message    */
    printf ("\nVoltage: %.2f V\n",rptr->voltage);
    printf ("Current: %.2f A\n",rptr->current);
    printf ("Number of cycles: %d\n",rptr->counter);
    _free_box (mpool, rptr);           /* free memory allocated for message  */
  }
}

/*----------------------------------------------------------------------------
 *        Main: Initialize and start RTX Kernel
 *---------------------------------------------------------------------------*/
int main (void) {                     /* program execution starts here       */

  IODIR1 = 0xFF0000;                  /* P1.16..22 defined as Outputs        */
  init_serial ();                     /* initialize the serial interface     */
  _init_box (mpool, sizeof(mpool),    /* initialize the 'mpool' memory for   */
              sizeof(T_MEAS));        /* the membox dynamic allocation       */
  os_sys_init (send_task);            /* initialize and start task 1         */
}

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
