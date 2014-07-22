/*----------------------------------------------------------------------------
 *      RL-ARM - FlashFS
 *----------------------------------------------------------------------------
 *      Name:    FILE_EX1.C
 *      Purpose: File manipulation example program
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2011 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <LPC22xx.h>                  /* special function registers LPC2294  */
#include <RTL.h>                      /* RTL kernel functions & defines      */
#include <stdio.h>                    /* standard I/O .h-file                */
#include <ctype.h>                    /* character functions                 */
#include <string.h>                   /* string and memory functions         */

extern void init_serial (void);

const char menu[] = 
  "\n"
  "+******** FILE manipulation example using RTL FlashFS *********+\n"
  "| This program is a file save, read, rename, delete program.   |\n"
  "| You need to specify the filename before any file operation.  |\n"
  "+ command -+ syntax -----+ function ---------------------------+\n"
  "| Name     | N <name>    | set file name                       |\n"
  "| Capture  | C           | capture serial data to file         |\n"
  "| Append   | A           | append serial data to file          |\n"
  "| Read     | R           | read file                           |\n"
  "| rEname   | E <new name>| rename file to new name             |\n"
  "| Delete   | D           | delete file                         |\n"
  "| List     | L           | list file directory                 |\n"
  "+----------+-------------+-------------------------------------+\n";


extern BOOL getline (char *, U32);    /* external function:  input line      */

char in_line[80];                     /* storage for command input line      */
char filename[32] = {"CAPTURE.TXT" }; /* current file name                   */

/*----------------------------------------------------------------------------
 *        Skip 'spacer' characters
 *---------------------------------------------------------------------------*/
char * skip_blanks (char *sp) {
  while (*sp == ' ') {
    sp++;
  }
  return (sp);
}

/*----------------------------------------------------------------------------
 *        Convert string to upper case
 *---------------------------------------------------------------------------*/
void to_upper (char *sp) {
  while ((*sp = (char)toupper (*sp)) != 0) {
    sp++;
  }
}

/*----------------------------------------------------------------------------
 *        Capture serial data to file
 *---------------------------------------------------------------------------*/
void capture_file (char mode) {
  FILE *f;
  char *fmode;
  BOOL retv;

  if (mode == 'C') {                  /* Capture file                        */
    printf("\nCapture data to file: %s",filename);
    fmode = "w";
  }
  else {                              /* Append file                         */
    printf("\nAppend data to file: %s",filename);
    fmode = "a";
  }
  printf("\nPress ESC to stop.\n");
  f = fopen (filename,fmode);         /* open a file for writing             */
  if (f == NULL) {
    printf ("\nCan not open file!\n");/* error when trying to open file      */
    return;
  } 
                                      /* read line-edited serial input       */
  do {
    retv = getline (in_line, sizeof (in_line));
    fputs (in_line, f);
  } while (retv == __TRUE);
  fclose (f);                         /* close the output file               */
  printf ("\nFile closed.\n");
}

/*----------------------------------------------------------------------------
 *        Read file and dump it to serial window
 *---------------------------------------------------------------------------*/
void read_file () {
  FILE *f;
  int ch;

  printf("\nRead data from file: %s\n",filename);
  f = fopen (filename,"r");           /* open the file for reading           */
  if (f == NULL) {                    /* error when opening the file         */
    printf ("\nFile not found!\n");   /* most likely file was not found      */
    return;
  }

  while ((ch = fgetc (f)) != EOF) {   /* read the characters from the file   */
    putchar (ch);                     /* and write them on the screen        */
  }
  fclose (f);                         /* close the input file when done      */
  printf ("\nFile closed.\n");
}


/*----------------------------------------------------------------------------
 *        Main: 
 *---------------------------------------------------------------------------*/
int main (void) {
  char temp[32], *sp;
  FINFO info;
  U32 args;

  init_serial ();

  finit (NULL);
  fformat ("");

  printf (menu);                              /* display command menu        */
  while (1) {                                 /* endless loop                */
    printf ("\nCommand: ");                   /* display prompt              */
    fflush (stdout);
                                              /* get command line input      */
    if (getline (in_line, sizeof (in_line)) == __FALSE) {
      continue;
    }
    sp = &in_line[0];
    to_upper (sp);                            /* convert to uppercase        */
    sp = skip_blanks (sp);                    /* skip blanks                 */

    switch (*sp) {                            /* proceed to command function */
      case 'N':                               /* change filename command     */
        sp = skip_blanks (sp+1);
        args = sscanf (sp, "%s", (S8 *)&temp);/* scan input line for name*/
        if (args == 1) {                      /* check if new name entered   */
          strcpy (filename, temp);
        }
        printf ("\nFile Name: %s\n", filename);
        break;

      case 'C':                               /* capture file command        */
      case 'A':                               /* append file command         */
        capture_file (*sp);
        printf (menu);
        break;

      case 'R':                               /* read the 'filename' file    */
        read_file ();
        printf (menu);
        break;

      case 'E':                               /* rename the file command     */
        sp = skip_blanks (sp+1);
        args = sscanf (sp, "%s", (S8 *)&temp);/* scan input line for name*/
        if (args == 1) {                      /* check if new name entered   */
          if (frename (filename, temp) == 0) {
            printf ("\nFile successfully renamed.\n");  
            strcpy (filename, temp);          /* set also the filename       */
            printf ("\nFile Name: %s\n", filename);
          }
          else {
            printf ("\nFile rename error.\n");
          }
        }
        printf (menu);
        break;

      case 'D':                               /* delete the file command     */
        if (fdelete (filename) == 0) {
          printf ("\nFile successfully deleted.\n");
        }
        else {
          printf ("\nFile not found error.\n");
        }
        printf (menu);
        break;

      case 'L':                               /* list file directory command */
        printf ("\nFile System Directory...");
        info.fileID = 0;
        while (ffind ("*.*",&info) == 0) {
          printf ("\n%-32s %5ld bytes, ID: %04d",info.name, info.size, info.fileID);
        }
        if (info.fileID == 0) {
          printf ("\nNo files...");
        }
        printf ("\nFree: %ld bytes.\n",(U32)ffree(""));
        break;

      default:                                /* Error Handling              */
        printf (menu);                        /* display command menu        */
        break;
    }
  }
}


/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
