/*----------------------------------------------------------------------------
 *      RL-ARM - TCPnet
 *----------------------------------------------------------------------------
 *      Name:    TELNET_MULTIUSER.C
 *      Purpose: Telnet Server - Multiple user accounts demo
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <Net_Config.h>
#include <stdio.h>
#include <string.h>

/* ---------------------------------------------------------------------------
 * This module demonstrates how to extend the Telnet server with additional
 * user accounts. Additional user accounts are created here in this file. 
 * The system admin account is always created. Credentials for this account
 * are defined in Net_Config.c 
 *   
 * The interface functions for the telnet server are:
 *
 * U8 tnet_check_account (U8 code, U8 *id)
 *   This function checks, if the account is externally created by the user.
 *   Parameters:
 *     - 'code' identifies the code for the 'id' parameter
 *        code == 0 - 'id' is the username
 *        code > 0  - 'id' is the password for the user identified with 'code'
 *     - 'id' is the username or password entered in the telnet client
 *       (a 0-terminated string)  
 *   The function should return an user identification in range from 1 - 255
 *   This information is not used internally by the telnet server. It is only
 *   stored and used later to identify the user.
 *      
 * U8 tnet_get_user_id (void)
 *   This function can be used in the uif module to identify the user that is
 *   trying to execute a telnet command. You can restrict execution of some
 *   commands to priviliged users only in tnet_process_cmd().
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

/*--------------------------- tnet_check_account ----------------------------*/

U8 tnet_check_account (U8 code, U8 *id) {
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

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
