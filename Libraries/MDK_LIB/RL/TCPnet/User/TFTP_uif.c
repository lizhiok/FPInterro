/*----------------------------------------------------------------------------
 *      RL-ARM - TCPnet
 *----------------------------------------------------------------------------
 *      Name:    TFTP_UIF.C
 *      Purpose: TFTP Server User Interface Module
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <stdio.h>
#include <Net_Config.h>

/*----------------------------------------------------------------------------
 * TFTP Server File Access Functions
 *---------------------------------------------------------------------------*/

/*--------------------------- tftp_fopen ------------------------------------*/

void *tftp_fopen (U8 *fname, U8 *mode) {
  /* Open filename fname for reading or writing. */
  return (fopen((char *)fname, (char *)mode));
}


/*--------------------------- tftp_fclose -----------------------------------*/

void tftp_fclose (void *file) {
  /* Close the file, opened for reading or writing. */
  fclose (file);
}


/*--------------------------- tftp_fread ------------------------------------*/

U16 tftp_fread (void *file, U8 *buf, U16 len) {
  /* Read 'len' bytes from file to buffer 'buf'. Return number of bytes */
  /* copied. The file will be closed, when the return value is < 'len'  */
  return (fread (buf, 1, len, file));
}


/*--------------------------- tftp_fwrite -----------------------------------*/

U16 tftp_fwrite (void *file, U8 *buf, U16 len) {
  /* Write data to file. Return number of bytes actually written. */
  return (fwrite (buf, 1, len, file));
}


/*--------------------------- tftp_accept_host ------------------------------*/
#if 0
BOOL tftp_accept_host (U8 *rem_ip, U16 rem_port) {
  /* This function checks if a connection from remote host is accepted or  */
  /* not. If this function is missing, all remote hosts are accepted.      */

   if (rem_ip[0] == 192  &&
       rem_ip[1] == 168  &&
       rem_ip[2] == 1    &&
       rem_ip[3] == 1) {
      /* Accept a connection. */  
      return (__TRUE);
   }
   /* Deny a connection. */
   return (__FALSE);
}
#endif

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
