/*----------------------------------------------------------------------------
 *      RL-ARM - TCPnet
 *----------------------------------------------------------------------------
 *      Name:    HTTP_MULTIUSER.C
 *      Purpose: HTTP Server - Multiple user accounts demo
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <Net_Config.h>
#include <stdio.h>
#include <string.h>

/* ---------------------------------------------------------------------------
 * This module demonstrates how to extend the Web server with additional
 * user accounts. Additional user accounts are created here in this file. 
 * The system admin account is always created. Credentials for this account
 * are defined in Net_Config.c The system admin account has no restrictions.
 *   
 * The interface functions for the web server are:
 *
 * U8 http_check_account (U8 *user, U8 *passw)
 *   This function checks, if the account is externally created by the user.
 *   Parameters are 0-terminated strings:
 *     - 'user' is a username entered in the browser
 *     - 'passw' is the password entered in the browser
 *   The function should return an user identification in range from 1 - 255
 *   This information is not used internally by the web server. It is only
 *   stored and used later to identify the user.
 *      
 * BOOL http_file_access (U8 *fname, U8 user_id)
 *   This function checks if a file access is allowed for the specified user.
 *     - 'fname' parameter is a file name that the client is trying to access
 *       (0-terminated string)
 *     - 'user_id' parameter is a user identification number (range 1 - 255)
 *   This function shall return:
 *      __TRUE if access for 'fname' is allowed and
 *      __FALSE if access is forbidden 
 *           
 * U8 http_get_user_id (void)
 *   This function can be used in the cgi module to identify the user that is
 *   trying to access a resource file. You can generate different page content
 *   for different users from cgi_func()
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

/*--------------------------- http_check_account ----------------------------*/

U8 http_check_account (U8 *user, U8 *passw) {
  /* This function checks externally provided user account.  */
  int i;

  for (i = 0; i < 3; i++) {
    if ((strcmp ((char *)user, users[i]) == 0) && 
        (strcmp ((char *)passw, passwords[i]) == 0)) {
      /* Return user index + 1. */
      return (i+1);
    }
  }
  /* User account does not exist. */
  return (0);
}


/*--------------------------- http_file_access ------------------------------*/

BOOL http_file_access (U8 *fname, U8 user_id) {
  /* This function checks if file access for the user is allowed. */
  /* 'user_id' is the same as returned from htp_check_account().  */

  if (user_id == 3) {
    /* Check if "Guest" is trying to access the "system.cgi" */
    if (strcmp ((char *)fname, "system.cgi") == 0) {
      /* The access to this file is not allowed for user "Guest". */ 
      /* Web server will return the error code 403 - Forbidden.   */
      return (__FALSE);
    }
  }
  return (__TRUE);
}

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
