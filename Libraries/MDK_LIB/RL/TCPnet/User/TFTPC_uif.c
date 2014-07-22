/*----------------------------------------------------------------------------
 *      RL-ARM - TCPnet
 *----------------------------------------------------------------------------
 *      Name:    TFTPC_UIF.C
 *      Purpose: TFTP Client User Interface Module
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <stdio.h>
#include <Net_Config.h>


/*----------------------------------------------------------------------------
 * TFTP Client File Access Functions
 *---------------------------------------------------------------------------*/

/*--------------------------- tftpc_fopen -----------------------------------*/

void *tftpc_fopen (U8 *fname, U8 *mode) {
  /* Open local file for reading or writing. */
  return (fopen ((char *)fname, (char *)mode));
}


/*--------------------------- tftpc_fclose ----------------------------------*/

void tftpc_fclose (void *file) {
  /* Close a local file. */
  fclose (file);
}


/*--------------------------- tftpc_read ------------------------------------*/

U16 tftpc_fread (void *file, U8 *buf, U16 len) {
  /* Read 'len' bytes from file to buffer 'buf'. Return number of bytes */
  /* copied. The file will be closed, when the return value is < 'len'  */
  return (fread (buf, 1, len, file));
}


/*--------------------------- tftpc_write -----------------------------------*/

U16 tftpc_fwrite (void *file, U8 *buf, U16 len) {
  /* Write data to file. Return number of bytes actually written. */
  return (fwrite (buf, 1, len, file));
}

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
