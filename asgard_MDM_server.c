/* =========================================================================
 * Asgard Multi DM control server - Frantz Martinache
 * 
 * The control server is a shell, intended to be used within a tmux session
 * so that other processes can easily interface to it, exchanging simple
 * text messages.
 * 
 * The program mostly handles the creation, update and destruction of several
 * shared memory data structures (ImageStreamIO library by O. Guyon), that
 * are refered to as channels.
 * 
 * Once the start command is issued to the DM server shell, a thread monitors
 * the content of the different shared memory data structures, combines them
 * and then send an update command to the DM driver itself.
 * ========================================================================= */

#include <stdio.h>

#ifdef __GNUC__
#  if(__GNUC__ > 3 || __GNUC__ ==3)
#	define _GNUC3_
#  endif
#endif

#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <math.h>
#include <pthread.h>
#include <curses.h>

#include "ImageStruct.h"
#include "ImageStreamIO.h"

#include <BMCApi.h>  // the new API for the MultiDM!

/* =========================================================================
 *                            Global variables
 * ========================================================================= */
#define LINESIZE 256
#define CMDSIZE 200

int wxsz, wysz;          // window size
int ii;                  // dummy index value
IMAGE **shmarray = NULL; // shared memory img pointer (defined in ImageStreamIO.h)
// int dmid        = 1;     // identifier of the DM (remember there will be four!)
int nch         = 4;     // number of DM channels (default = 4)
int dms         = 12;    // linear size of the DM in actuators
int nact        = 140;   // number of "real" actuators of the DM
int nvact       = 144;   // number of "virtual" acutators of the DM
int keepgoing   = 0;     // flag to control the DM update loop
int allocated   = 0;     // flag to control whether the shm structures are allocated
int nch_prev    = 0;     // keep track of the # of channels before a change
char dashline[80] =
  "-----------------------------------------------------------------------------\n";

int ndm = 4; // the number of DMs to be connected
DM *hdms[4];  // the handles for the different deformable mirrors
BMCRC rv;    // result of every interaction with the driver (check status)
uint32_t *map_lut[4];  // the DM actuator mappings

int simmode = 0;  // flag to set to "1" to not attempt to connect to the driver
int timelog = 0;  // flag to set to "1" to log DM response timing

// order to be reshuffled when reassembling the instrument
const char snumbers[4][BMC_SERIAL_NUMBER_LEN+1] = \
  {"17DW019#113", "17DW019#093", "17DW019#122", "17DW019#053"};

/* =========================================================================
 *                       function prototypes
 * ========================================================================= */
// int reset_channel(int chindex);
void print_help();
int log_action(char *msg);
int shm_setup();
void* dm_control_loop(void *_dmid);
double* map2D_2_cmd(double *map2D);

void message_error(char* msg);
void message_OK(char* msg);
void message_info(char *msg);
void echo_query(char* msg);
void MakeOpen(int dmid, DM* hdm);

/* =========================================================================
 *                           DM setup function
 * ========================================================================= */
void MakeOpen(int dmid, DM* hdm) {
  memset(hdm, 0, sizeof(DM));
  printf("Attempting to open device %s\n", snumbers[dmid-1]);
  rv = BMCOpen(hdm, snumbers[dmid-1]);
  
  if (rv) {
    printf("Error %d opening the driver type %u.\n", rv, (unsigned int)hdm->Driver_Type);
    printf("%s\n\n", BMCErrorString(rv));

    printf("Press any key to exit.\n");
    getc(stdin);
    exit(0);
  }
  printf("Opened Device %d with %d actuators.\n", hdm->DevId, hdm->ActCount);
  
  rv = BMCLoadMap(hdm, NULL, map_lut[dmid-1]);  // load the mapping into map_lut
}


/* =========================================================================
 *                Simple messages for the color prompt
 * ========================================================================= */
void message_error(char* msg) {
  attron(COLOR_PAIR(1) | A_BOLD);
  printw("%s                     ", msg);
  attroff(COLOR_PAIR(1) | A_BOLD);
}

void message_OK(char* msg) {
  attron(COLOR_PAIR(2));
  printw("%s                     ", msg);
  attroff(COLOR_PAIR(2));
}

void echo_query(char* msg) {
  attron(COLOR_PAIR(3));
  printw("%s                     \n", msg);
  attroff(COLOR_PAIR(3));
}

void message_info(char* msg) {
  attron(COLOR_PAIR(4));
  printw("%s                     ", msg);
  attroff(COLOR_PAIR(4));
}

/* =========================================================================
 *      Allocates shared memory data structures for the new settings
 * ========================================================================= */
int shm_setup() {
  int ii, kk;
  int shared = 1;
  int NBkw = 10;
  long naxis = 2;
  uint8_t atype = _DATATYPE_DOUBLE;
  uint32_t *imsize;
  char shmname[20];

  // shared memory representation of the DM is a 2D (12x12) map
  imsize = (uint32_t *) malloc(sizeof(uint32_t) * naxis);
  imsize[0] = dms;
  imsize[1] = dms;

  if (shmarray != NULL) { // structure must be freed before reallocation!
    for (kk = 0; kk < ndm; kk++)
      for (ii = 0; ii < nch_prev; ii++) 
	ImageStreamIO_destroyIm(&shmarray[kk][ii]);
    free(shmarray);
    shmarray = NULL;
  }

  shmarray = (IMAGE**) malloc((ndm) * sizeof(IMAGE*));

  // allocates nch arrays (+ 1 for the combined channel)
  for (kk = 0; kk < ndm; kk++) {
    shmarray[kk] = (IMAGE*) malloc((nch+1) * sizeof(IMAGE));
  }

  for (kk = 0; kk < ndm; kk++) {
    // individual channels
    for (ii = 0; ii < nch; ii++) {
      sprintf(shmname, "dm%ddisp%02d", kk+1, ii); // root name of the shm
      ImageStreamIO_createIm_gpu(&shmarray[kk][ii], shmname, naxis, imsize, atype, -1,
				 shared, IMAGE_NB_SEMAPHORE, NBkw, MATH_DATA);
    }
    // the combined array
    sprintf(shmname, "dm%d", kk+1);              // root name of the shm
    ImageStreamIO_createIm_gpu(&shmarray[kk][nch], shmname, naxis, imsize, atype, -1,
			       shared, IMAGE_NB_SEMAPHORE, NBkw, MATH_DATA);
  }
  free(imsize);
  return 0;
}

/* =========================================================================
 *                       Displays the help menu
 * ========================================================================= */
void print_help() {
  char fmt[20] = "%15s %20s %40s\n";
  attron(COLOR_PAIR(3));
  mvprintw(6, 0, dashline);
  printw("                DM control shell help menu\n");
  printw("%s", dashline);
  printw(fmt, "command", "parameters", "description");
  printw("%s", dashline);
  printw(fmt, "help", "",          "prints this help message");
  printw(fmt, "quit", "",          "stops the DM!");
  printw(fmt, "set_nch", "integer", "sets the number of channels to val");
  printw(fmt, "start", "",          "starts the DM (set_nch first!)");
  printw(fmt, "get_nch", "",        "returns the current number of channels");
  printw(fmt, "reset",   "integer", "reset channel #k (-1 for all channels)");
  printw("%s", dashline);
  attroff(COLOR_PAIR(3));
}

/* =========================================================================
 *                 log server interaction in a file
 * ========================================================================= */
int log_action(char *msg) {
  struct timespec now;       // clock readout
  struct tm* ptm;

  FILE* fd = fopen("log_test.log", "a");

  clock_gettime(CLOCK_REALTIME, &now); // what is the time ??
  ptm = localtime(&(now.tv_sec));

  fprintf(fd, "%02d:%02d:%02d  %s\n",
	  ptm->tm_hour, ptm->tm_min, ptm->tm_sec,
	  msg);
  
  fclose(fd);
  return 0;
}

/* =========================================================================
 *        Convert 2D DM map into the command to be sent to the DM
 *
 * 4 corner values of the 2D map must be dropped. Result array is 140 elts
 * ========================================================================= */
double* map2D_2_cmd(double *map2D) {
  int ii, jj;  // dummy indices
  double* cmd = (double*) malloc(nact * sizeof(double));
  // FILE* fd;

  jj = 0;  // index of value to add to the command
  for (ii = 0; ii < nvact; ii++) {  // iterate over "virtual" actuators
    if ((ii == 0) || (ii == 11) || (ii == 132) || (ii == 143))
      continue;
    else {
      cmd[jj] = map2D[ii];
      jj++;
    }
  }
  return cmd;
}

/* =========================================================================
 *                     DM surface control thread
 * ========================================================================= */
void* dm_control_loop(void *_dmid) {
  uint64_t cntrs[nch];
  int ii, kk;  // array indices
  int updated = 0;
  double *cmd; //
  double tmp_map[nvact];  // to store the combination of channels
  struct timespec now; // clock readout

  unsigned int dmid = *((unsigned int *) _dmid);

  char fname[20];
  sprintf(fname, "speed_log_%1d.log", dmid);
  FILE* fd;

  if (timelog)
    fd = fopen(fname, "w");  // Record DM interaction times

  for (ii = 0; ii < nch; ii++)
    cntrs[ii] = shmarray[dmid-1][nch].md->cnt0;  // init shm counters

  while (keepgoing > 0) {
	updated = 0;

	for (ii = 0; ii < nch; ii++) {
	/*  if (ImageStreamIO_semtrywait(&shmarray[dmid-1][ii], ii) == 0) {
	    updated += 1;
	  }
	  usleep(10);*/
	  // printf("\rUpdated! [%d]\n", updated);
	  
	  // look for updates to the shm counters
	  if (shmarray[dmid-1][ii].md->cnt0 > cntrs[ii]) {
		updated += 1;
		cntrs[ii] = shmarray[dmid-1][ii].md->cnt0;
	  }
	  usleep(1);
	}
	if (updated > 0) { // a counter was updated
	  // log_action("MultiDM updated!");

	  // -------- combine the channels -----------
	  for (ii = 0; ii < nvact; ii++) {
		tmp_map[ii] = 0.0; // init temp sum array
		for (kk = 0; kk < nch; kk++) {
		  tmp_map[ii] += shmarray[dmid-1][kk].array.D[ii];
		}
	  }
	  for (ii = 0; ii < nvact; ii++) {
		if (tmp_map[ii] < 0.0)
		  tmp_map[ii] = 0.0;
		if (tmp_map[ii] > 1.0)
		  tmp_map[ii] = 1.0;
	  }

	  // ------- update the shared memory ---------
	  shmarray[dmid-1][nch].md->write = 1;   // signaling about to write
	  for (ii = 0; ii < nvact; ii++) // update the combined channel
		shmarray[dmid-1][nch].array.D[ii] = tmp_map[ii];
	  shmarray[dmid-1][nch].md->cnt1 = 0;
	  shmarray[dmid-1][nch].md->cnt0++;
	  ImageStreamIO_sempost(&shmarray[dmid-1][nch], -1);
	  shmarray[dmid-1][nch].md->write = 0;  // signaling done writing

	  // ------ converting into a command the driver --------
	  // sending to the DM
	  if (simmode != 1) {
	    cmd = map2D_2_cmd(shmarray[dmid-1][nch].array.D);
	    rv = BMCSetArray(hdms[dmid-1], cmd, map_lut[dmid-1]);  // send cmd to DM
	    if (rv) {
	      printw("%s\n\n", BMCErrorString(rv));
	      // printf("%s\n\n", BMCErrorString(rv));
	    }
	    free(cmd);
	  }
	  clock_gettime(CLOCK_REALTIME, &now);   // get time after issuing a command
	  if (timelog)
	    fprintf(fd, "%f\n", 1.0*now.tv_sec + 1e-9*now.tv_nsec);
	}
  }
  if (timelog)
    fclose(fd); // closing the timing log file
  return NULL;
}

/* =========================================================================
 *                                Main program
 * ========================================================================= */
int main() {
  char cmdstring[CMDSIZE];
  char loginfo[LINESIZE];
  int cmdOK = 0;
  int ii, kk;
  int ival = 0;
  char str0[20];
  pthread_t tid_loop; // thread ID for DM control loop
  unsigned int targs[ndm]; // thread integer arguments
  //struct thread_info *tinfo;

  for (ii = 0; ii < ndm; ii++) {
    hdms[ii] = (DM *) malloc(sizeof(DM));
    map_lut[ii] = (uint32_t *) malloc(sizeof(uint32_t)*MAX_DM_SIZE);
  }

 
  if (simmode != 1)
    for (ii = 0; ii < ndm; ii++) {
      MakeOpen(ii+1, hdms[ii]);
      usleep(1000);
    }
  else {
    printf("Simulated DM scenario: the drivers are not connected\n");
    for (ii = 0; ii < ndm; ii++)
      printf("Simulated DM id = %d - serial number = %s.\n", ii+1, snumbers[ii]);
  }
  shm_setup();  // set up startup configuration
  allocated = 1;
 
  // ----- curses specifics -----
  initscr(); // start curses mode
  start_color();
  getmaxyx(stdscr, wysz, wxsz);
  init_pair(1, COLOR_RED,    COLOR_BLACK);
  init_pair(2, COLOR_GREEN,  COLOR_BLACK);
  init_pair(3, COLOR_YELLOW, COLOR_BLACK);
  init_pair(4, COLOR_BLUE,   COLOR_BLACK);

  // --------------------- set-up the prompt --------------------
  attron(COLOR_PAIR(2));
  printw("%s", dashline);
  printw("                   DM CONTROL INTERACTIVE SHELL\n");
  printw("\nDid you launch this program from within a tmux as it is meant?\n");
  printw("\n");
  printw("%s", dashline);
  attroff(COLOR_PAIR(2));


  // --------------------------
  //   start command line
  // --------------------------
  for (;;) { // infinite loop
    cmdOK = 0;
    
    attron(COLOR_PAIR(3));
    move(wysz-4, 0);
    clrtoeol();
    printw("MultiDM > ");
    attroff(COLOR_PAIR(3));
    getstr(cmdstring);
    move(wysz-2, 0);
    
    // --------------------------
    //     process commands
    // --------------------------
    
    // =====================================================
    if (cmdOK == 0)
      if (strncmp(cmdstring, "start", strlen("start")) == 0) {
	if (allocated == 0) {
	  message_error("set the desired number of channels\n");
	}
	else {
	  if (keepgoing == 0) {
	    keepgoing = 1; // raise the flag!
	    sprintf(loginfo, "DM control loop START");
	    log_action(loginfo);
	    message_OK(loginfo);
	    // tinfo = calloc(ndm, sizeof(struct thread_info));
	    for (kk = 0; kk < ndm; kk++) {
	      /* tinfo[kk].thread_num = kk + 1; */
	      /* tinfo[kk].argv_string = argv[optind + tnum]; */
	      targs[kk] = kk+1;
	      //pthread_create(&tid_loop, NULL, dm_control_loop, (void*) (kk+1));
	      pthread_create(&tid_loop, NULL, dm_control_loop, &targs[kk]);
	    }
	  }
	}
	cmdOK = 1;
      }
    
    // =====================================================
    if (cmdOK == 0)
      if (strncmp(cmdstring, "stop", strlen("stop")) == 0) {
	cmdOK = 1;
	if (keepgoing == 1) {
	  sprintf(loginfo, "DM control loop STOP");
	  log_action(loginfo);
	  message_OK(loginfo);
	  keepgoing = 0;		  
	}
	else
	  message_error("DM control loop already stopped");
	cmdOK = 1;
      }
    
    // =====================================================
    //    set the desired number of DM channels
    // =====================================================
    if (cmdOK == 0)
      if (strncmp(cmdstring, "set_nch", strlen("set_nch")) == 0) {
	ival = 0; // reset value
	sscanf(cmdstring, "%s %d", str0, &ival);
	
	sprintf(loginfo, "requesting %d channels", ival);
	echo_query(loginfo);
	
	if (ival == 0) {
	  sprintf(loginfo, "wrong command? %s", cmdstring);
	  message_error(loginfo);
	  print_help();
	  
	} else {
	  clrtoeol();
	  sprintf(loginfo, "%s", cmdstring);
	  log_action(loginfo);
	  nch_prev = nch; // memory of the previous number of channels
	  nch = ival;
	  shm_setup(); // nch_prev);
	  allocated = 1;
	  message_OK("number of channels successfully updated");
	}
	cmdOK = 1;
      }
    
    // =====================================================
    //  returns the current number of available channels
    // =====================================================
    if (cmdOK == 0)
      if (strncmp(cmdstring, "get_nch", strlen("get_nch")) == 0) {
	sprintf(loginfo, "number of channels = %d\n", nch);
	message_info(loginfo);
	cmdOK = 1;
      }
    
    // =====================================================
    //       resets one or all channels (-1 for all)
    // =====================================================
    if (cmdOK == 0)
      if (strncmp(cmdstring, "reset", strlen("reset")) == 0) {
	ival = 0; // reset value
	sscanf(cmdstring, "%s %d", str0, &ival);
	
	// reset_channel(ival);
	cmdOK = 1;
      }
    
    // =====================================================
    //             displays the help menu
    // =====================================================
    if (cmdOK == 0)
      if (strncmp(cmdstring, "help", strlen("help")) == 0) {
	move(wysz-2,0); clrtoeol();
	move(wysz-1,0); clrtoeol();
	print_help();
	cmdOK = 1;
      }
    
    // =====================================================
    if (cmdOK == 0)
      if (strncmp(cmdstring, "quit", strlen("quit")) == 0) {
	if (keepgoing == 0) {
	  message_error("DM shell closed (press key to continue)!");
	  getch();
	  endwin(); // from curses back to regular env!

	  if (simmode != 1) {
	    for (kk = 0; kk < ndm; kk++)
	      rv = BMCClearArray(hdms[kk]);
	    if (rv) {
	      printf("%s\n\n", BMCErrorString(rv));
	      printf("Error %d clearing voltages.\n", rv);
	    }
	    
	    for (kk = 0; kk < ndm; kk++) {
	      rv = BMCClose(hdms[kk]);
	      if (rv) {
		printf("%s\n\n", BMCErrorString(rv));
		printf("Error %d closing the driver.\n", rv);
	      }
	      printf("%s\n\n", BMCErrorString(rv));
	    }
	    for (ii = 0; ii < ndm; ii++) {
	      free(map_lut[ii]);
	    }
	  }
	  if (shmarray != NULL) { // free the data structure
	    for (kk = 0; kk < ndm; kk++) {
	      for (ii = 0; ii < nch + 1; ii++) {
		ImageStreamIO_destroyIm(&shmarray[kk][ii]);
	      }
	    }
	    free(shmarray);
	    shmarray = NULL;
	  }
	  log_action("DM control program quit");
	  exit(0);
	}
	else {
	  message_error("DM loop still running!");
	  cmdOK = 1;
	}
      }
    // =====================================================    
    if (cmdOK == 0) {
      sprintf(loginfo, "Unkown command: %s", cmdstring);
      message_error(loginfo);
      print_help();
    }
	
    // -------------------------
    // end of command processing
    // -------------------------
  }
  // --------------------------
  //  clean ending the program
  // --------------------------
  exit(0);
  }
  
