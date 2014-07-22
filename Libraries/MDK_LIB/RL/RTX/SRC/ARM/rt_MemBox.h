/*----------------------------------------------------------------------------
 *      RL-ARM - RTX
 *----------------------------------------------------------------------------
 *      Name:    RT_MEMBOX.H
 *      Purpose: Interface functions for fixed memory block management system
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

/* Functions */
extern int   _init_box   (void *box_mem, U32 box_size, U32 blk_size);
extern void *_alloc_box  (void *box_mem)  __swi (1);
extern void *_calloc_box (void *box_mem);
extern int   _free_box   (void *box_mem, void *box)  __swi (2);

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/

