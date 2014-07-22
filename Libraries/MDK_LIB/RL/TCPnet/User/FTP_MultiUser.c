/*----------------------------------------------------------------------------
 *      RL-ARM - TCPnet
 *----------------------------------------------------------------------------
 *      Name:    FTP_MULTIUSER.C
 *      Purpose: FTP Server - Multiple user accounts demo
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <Net_Config.h>
#include <stdio.h>
#include <string.h>

/* ---------------------------------------------------------------------------
 * This module demonstrates how to extend the FTP server with additional
 * user accounts. Additional user accounts are created here in this file. 
 * The system admin account is always created. Credentials for this account
 * are defined in Net_Config.c The system admin account has no restrictions.
 *   
 * The interface functions for the ftp server are:
 *
 * U8 ftp_check_account (U8 code, U8 *id)
 *   This function checks, if the account is externally created by the user.
 *   Parameters:
 *     - 'code' identifies the code for the 'id' parameter
 *        code == 0 - 'id' is the username
 *        code > 0  - 'id' is the password for the user identified with 'code'
 *     - 'id' is the username or password entered in the ftp client
 *        (a 0-terminated string)  
 *   The function should return an user identification in range from 1 - 255
 *   This information is not used internally by the ftp server. It is only
 *   stored and used later to identify the user.
 *      
 * BOOL ftp_file_access (U8 *fname, U8 mode, U8 user_id)
 *   This function checks if a file/folder access is allowed for the user.
 *     - 'fname' parameter is a file name or folder name that the client is 
 *       trying to access (0-terminated string)
 *     - 'mode' defines the access mode (read, write, ...) 
 *     - 'user_id' parameter is a user identification number (range 1 - 255)
 *   This function shall return:
 *      __TRUE if access for 'fname' is allowed and
 *      __FALSE if access is forbidden 
 *           
 * U8 ftp_get_user_id (void)
 *   This function can be used in the uif module to identify the user that is
 *   trying to access the file or folder. You can restrict access to some
 *   files to priviliged users only in the ftp_uif.c module.
 *   This function returns the 'user_id' which identifies the user.    
 *   NOTE: This function is in the TCPnet library.
 * --------------------------------------------------------------------------*/

/* Local variables. */
static const char *users[] = {
    "Dave",
    "Michael",
    "Guest"};

static const char *passwords[] = {
    "test1",
    "test2",
    ""};

/*--------------------------- ftp_check_account -----------------------------*/

U8 ftp_check_account (U8 code, U8 *id) {
  /* This function checks externally provided user account.  */
  int i;

  if (code == 0) {
    /* Check if the username is valid. */
    for (i = 0; i < 3; i++) {
      if (strcmp ((char *)id, users[i]) == 0) {
        /* Return user index + 1. */
        return (i+1);
      }
    }
  }
  else {
    /* Check the password for user identified with 'code' */
    if (strcmp ((char *)id, passwords[code-1]) == 0) {
      /* Return user identification if ok. */
      return (code);
    }
  }
  /* User account does not exist. */
  return (0);
}

/*--------------------------- ftp_file_access -------------------------------*/

BOOL ftp_file_access (U8 *fname, U8 mode, U8 user_id) {
  /* This function checks if file access for the user is allowed. */
  /* Parameters:                                                  */
  /*   fname   - file/directory name                              */
  /*   mode    - access mode type:                                */
  /*             0 = read file                                    */
  /*             1 = write/append/delete/rename file              */
  /*             2 = create/remove directory                      */
  /*             3 = info (list directory, get file size,...)     */
  /*   user_id - user identification number                       */
  /*             (the same as returned from ftp_check_account()   */
  
  if (user_id == 3) {
    /* The user "Guest" has only read access rights. */
    switch (mode) {
      case 0:
      case 3:
        break;
      default:
        return (__FALSE);
    }
    return (__TRUE);
  }
  if (user_id == 2) {
    /* The user "Michael" is not allowed to modify/delete "Test.txt" */
    if (mode == 1 && strcmp ((char *)fname,"Test.txt") == 0) {
      return (__FALSE);
    }
  }
  return (__TRUE);
}

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
