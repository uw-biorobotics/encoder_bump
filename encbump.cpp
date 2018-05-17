/*
 *  board_test.cpp
 *  Created by Hawkeye 9/29/2010
 *  Updated by Kun Su 4/2018
 *
 * I implement some i/o operations with the BRL USB I/O board.
 *
 **/
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <iostream>
#include <fcntl.h>
#include <time.h>
#include <sched.h>
#include <signal.h>
#include <sys/io.h>
#include <string.h>
#include <math.h>
#include <sys/ioctl.h>

#define PI              ((float)3.1415926535)
#define DAC_WRITE       0x06                // USB packet type
#define NUM_CHAN        8
#define DAC_OFFSET      0x8000              // DAC zero output
#define USB_OUT_LEN     NUM_CHAN*2+3 
#define USB_MAX_IN_LEN  512
#define PORT            0xcc00              // parallel port
#define NSEC_PER_SEC    1000000000          // nanoseconds per sec
#define WD_BIT		0x10                // Watchdog timer bit on portF
#define BRL_RESET_BOARD 10
#define BRL_START_READ 4
int runable=1;

const char * boardstr = "/dev/brl_usb";

/**
 * using clock_nanosleep of librt 
*/
extern int clock_nanosleep(clockid_t __clock_id, int __flags,
			   __const struct timespec *__req,
			   struct timespec *__rem);
void sigTrap(int sig){
  printf("Terminating on signal %d\n", sig);
  runable = 0;
}

/**
 * the struct timespec consists of nanoseconds
 * and seconds. if the nanoseconds are getting
 * bigger than 1000000000 (= 1 second) the
 * variable containing seconds has to be
 * incremented and the nanoseconds decremented
 * by 1000000000.
 */
static inline void tsnorm(struct timespec *ts)
{
  while (ts->tv_nsec >= NSEC_PER_SEC) {
    ts->tv_nsec -= NSEC_PER_SEC;
    ts->tv_sec++;
  }
}

/**
 * increment counter and write to parallelport 
 */
void parport_out()
{
  static unsigned char state=0;
  outb(state++,PORT);
}

/**
 * tsSubtract() - returns time1-time2  or  (0,0) if time2>time1
 */
struct  timespec  tsSubtract ( struct  timespec  time1,
			       struct  timespec  time2)
{ 
  struct  timespec  result ;
  
  /* Subtract the second time from the first. */
  if ((time1.tv_sec < time2.tv_sec) ||
      ((time1.tv_sec == time2.tv_sec) &&
       (time1.tv_nsec <= time2.tv_nsec))) { /* TIME1 <= TIME2? */
    result.tv_sec = result.tv_nsec = 0 ;
  } else {                                  /* TIME1 > TIME2 */
    result.tv_sec = time1.tv_sec - time2.tv_sec ;
    if (time1.tv_nsec < time2.tv_nsec) {
      result.tv_nsec = time1.tv_nsec + 1000000000L - time2.tv_nsec ;
      result.tv_sec-- ;                    /* Borrow a second. */
    } else {
      result.tv_nsec = time1.tv_nsec - time2.tv_nsec ;
    }
  }
  return (result);
}

/**
 * tsSubtract() - returns time1-time2  or  (0,0) if time2>time1
 */
double  tsSubtractd ( struct  timespec  time1,
		      struct  timespec  time2)
{ 
  struct  timespec  ts = tsSubtract(time1, time2);
  double result = ts.tv_sec + double(ts.tv_nsec)/NSEC_PER_SEC;
  return (result);
}

int processEncVal(unsigned char buffer[], int channel)
{
  int result;

  // for USB boards programmed to work with surgical robot
  //
  // encoder value is in bytes 3,4,and 5 of the usb in-packet
  // see atmel_code/main.c: in_packet() for more info
  result = (buffer[3*channel+5]<<16) | (buffer[3*channel+4]<<8) | 
    (buffer[3*channel+3]);
  
  //If there is a problem here, check to see if the USB packet has changed. 
  
  //Handle negative values by padding result with ones
  if (result >> 23) {
    result = result | 0xFF000000;
  }
  return result;
}

#define NS  1
#define US  (1000 * NS)
#define MS  (1000 * US)
#define SEC (1000 * MS)
int main(int argc, char* argv[]) 
{
  int fps[2];
  int boardFile;               // fd of the USB board

  char outBuff[USB_OUT_LEN];
  unsigned char inBuff [USB_MAX_IN_LEN], rl=0x00;
  unsigned int zero_output = 0x8000;
  unsigned int bump_output = 0x8000;
  struct timespec ts,          /*Tracks the timer value */
    bumptimer, timer2,tnow;
  struct sched_param param;   // priority settings
  int interval= 1 * MS;    // task period in nanoseconds
  
  int ret;
  
  signal( SIGINT,&sigTrap);                // catch ^C for graceful close.
  
  /* set process/thread priorities */
  if( argc<3 || atoi(argv[1])==0 || atoi(argv[2])==0 )   
    {
      printf("Usage: encoder_bump <BOARD_ID_1> <BOARD_ID_2>.\n\n");
      printf("To determine board ids, run `ls /dev/brl_usb*  Enter in the integer values here\n");
      exit(-1);
    }
  
  char * BOARD_FILE[2];

  BOARD_FILE[0] =(char*) malloc(20);
  strcpy(BOARD_FILE[0], boardstr);
  strcat(BOARD_FILE[0], argv[1]);
  BOARD_FILE[1] = (char*)malloc(20);
  strcpy(BOARD_FILE[1], boardstr);
  strcat(BOARD_FILE[1], argv[2]);
  
  printf("using boards: %s, %s\n",BOARD_FILE[0],BOARD_FILE[1]);
    
  /* set process/thread priorities */
  printf("using realtime, priority: %d\n",atoi(argv[1]));
  param.sched_priority = atoi(argv[1]);
  /* enable realtime fifo scheduling */
  if(sched_setscheduler(0, SCHED_FIFO, &param)==-1){
    perror("sched_setscheduler failed");
    exit(-1);
  }




  ioperm(PORT,1,1);                     // set parallelport permissions
  memset(inBuff,0x00,USB_MAX_IN_LEN);   // initialize buffer to zeros

  fps[0] = open(BOARD_FILE[0], O_RDWR|O_NONBLOCK); // open board chardev
  if (fps[0] <=0 ){
    perror("Open error");
    errno=0;
    exit(-1);
  } 
  fps[1] = open(BOARD_FILE[1], O_RDWR|O_NONBLOCK); // open board chardev
  if (fps[1] <=0 ){
    perror("Open error");
    errno=0;
    exit(-1);
  }

  printf("Opened boards.\n\n\n Press Initialize button to bump encoders. \n\n\n");
  //brief initialize data retrieval from a USB board
  if(ioctl(fps[0],BRL_RESET_BOARD)!=0){
	perror("ioctl error openin board");
	errno=0;
	exit(-1);
	}
  if(ioctl(fps[1],BRL_RESET_BOARD)!=0){
	perror("ioctl error openin board");
	errno=0;
	exit(-1);
	}
  boardFile = fps[0];
  
  outBuff[0] = DAC_WRITE;        // Type of USB packet.
  outBuff[1] = NUM_CHAN;         // Number of DAC channels.
  outBuff[USB_OUT_LEN-1] = 0x00; // PORTF
  //try to initialize write zero board
  for(int i=0;i<NUM_CHAN;i++){
      outBuff[2*i+2]=(char)DAC_OFFSET;
      outBuff[2*i+3]=(char)DAC_OFFSET>>8;
  }
  if(write(fps[0],outBuff,USB_OUT_LEN)<0){
     perror("cant initialize write zero board");
     errno=0;
     exit(-1);
   }
   if(write(fps[1],outBuff,USB_OUT_LEN)<0){
     perror("cant initialize write zero board");
     errno=0;
     exit(-1);
   }

  clock_gettime(CLOCK_REALTIME,&ts);     // get current time 
  ts.tv_sec++;                           // start after one second
  clock_nanosleep(0, TIMER_ABSTIME, &ts, NULL);

  int j=0;
  runable=1;
  double clockdiff, bump_period = 0.2;
  int arm   = 0;
  int joint = -1;
  int bump  = 0;
  

  int first_read = 1;
  int encCounts[8] = {0};
  int prev_encCounts[8] = {0};
  unsigned int isbumped = 0x00;
  unsigned int armsbumped=0;
  const unsigned int bumptrue = 0xF7;

  while ( runable )
    {
      //initiate read
      ret = ioctl(fps[0],BRL_START_READ,USB_MAX_IN_LEN);
      if(ret<0){
    	  printf("can't initiate read 0");
    	  perror("read error");
    	  errno=0;
    	  exit(-1);
      }
      ret = ioctl(fps[1],BRL_START_READ,USB_MAX_IN_LEN);
      if(ret<0){
    	  printf("can't initiate read 1");
    	  perror("read error");
    	  errno=0;
    	  exit(-1);
      }
      clock_gettime(CLOCK_REALTIME,&tnow);
      int sleeploops = 0;
      while((ts.tv_sec<tnow.tv_sec)||(ts.tv_sec ==tnow.tv_sec && ts.tv_nsec<tnow.tv_nsec))
      {
    	  ts.tv_nsec+=interval;
    	  tsnorm(&ts);
    	  sleeploops++;
      }
      clock_nanosleep(0,TIMER_ABSTIME,&ts,NULL);
    j++; 
	printf("j is %d\n",j);
      // write output values to output buffer
      for (int i=0; i<NUM_CHAN; i++) {
    	  //outBuff[2*i+2] = (char)( zero_output );
    	  //outBuff[2*i+3] = (char)( zero_output >> 8 );
    	  outBuff[2*i+2] = (char)( DAC_OFFSET );
    	  outBuff[2*i+3] = (char)( DAC_OFFSET>>8 );
      }
      
      if (rl == 0) {
		bump = 0;
		
      }
		
      else 
	{
	  if (bump==0) 
	    {
	      clock_gettime(CLOCK_REALTIME,&bumptimer);
	      bump=1;
	      printf("bump.\n");
	    }
	  
	  clock_gettime(CLOCK_REALTIME,&timer2);
	  clockdiff = tsSubtractd(timer2, bumptimer);
	  printf("clockdiff is %f\n",clockdiff);
	  // wait for 800ms
	  if ( joint == -1)
	    {
	      if (clockdiff > 0.8)
		{
		  arm = 0;
		  joint = 0;
		  clock_gettime(CLOCK_REALTIME,&bumptimer);
		  bump_output = DAC_OFFSET;
		}
	    }
	  
	  // loop through arms and joints
	  else
	    {
	      // select next arm
	      if (clockdiff > bump_period || isbumped == bumptrue)
		{
		  clock_gettime(CLOCK_REALTIME,&bumptimer);
		  bump_output = DAC_OFFSET;
		  joint++;
		  if (joint==3) joint ++;

		  // Finished bumping one arm
		  if (joint>=8 ||  isbumped == bumptrue) 
		    { 
		      if (isbumped==bumptrue) armsbumped++;
		      joint=0; 
		      first_read=1;
		      arm++;
		      boardFile = fps[arm];
		      if (arm >= 2)   
			{
			  runable=0;
			  break;
			}
		    }
		}
	      // run sinusoidal bump torque
	      else
		{
		  bump_output = DAC_OFFSET + int(700* sin(2*PI* (clockdiff/bump_period)) );
		}
	      // set single joint output to bump value
	      outBuff[2*joint+2] = (char)( bump_output );
	      outBuff[2*joint+3] = (char)( bump_output >> 8 );
	    }
	}
 
      // toggle watchdog timer
      outBuff[USB_OUT_LEN-1] = 0x01;
      if (ts.tv_nsec % (20*MS) < 10*MS){
    	  outBuff[USB_OUT_LEN-1] |= WD_BIT;
    	  //printf("1:%d\n",outBuff[USB_OUT_LEN-1]);
      }
      else{
	outBuff[USB_OUT_LEN-1] &= ~WD_BIT;
	//outBuff[USB_OUT_LEN-1] |= WD_BIT;
	//printf("2:%d\n",outBuff[USB_OUT_LEN-1]);
	}
      
	
      // write output buffer to board
      ret = write(fps[arm], outBuff, USB_OUT_LEN);
	printf("write 0 ret is %d:\n",ret);
      if ( ret != USB_OUT_LEN) {
	printf("j:%d ret:%d, wlen:%d arm:%d, fp:%d\n",j, ret, USB_OUT_LEN, arm, fps[arm]);
	perror("Write error");
	errno = 0;
	exit(-1);
      }
	
      for (int i=0; i<NUM_CHAN; i++) {
       	outBuff[2*i+2] = (char)( zero_output );
       	outBuff[2*i+3] = (char)( zero_output >> 8 );
      }

      // write ZERO output values to output buffer
      ret = write(fps[1-arm], outBuff, USB_OUT_LEN);
      if ( ret != USB_OUT_LEN) {
	perror("Write error");
	errno = 0;
	exit(-1);
      }
      
      // read current encoder values from board to be bumped
      ret = read(fps[arm], inBuff, USB_MAX_IN_LEN);
	printf("read 0 ret is %d:\n",ret);
      if (ret < 0 ) {
	printf("ret:%d, rlen:%d arm:%d, fp:%d\n", ret, USB_MAX_IN_LEN, arm, fps[arm]);
	perror("read error");
	errno = 0;
	exit(-1);
      }

      // read current encoder values from "OTHER" board
      ret = read(fps[1-arm], inBuff, USB_MAX_IN_LEN);
      if (ret < 0 ) {
	printf("ret:%d, rlen:%d arm:%d fp:%d\n", ret, USB_MAX_IN_LEN,1-arm,fps[1-arm]);
	perror("read error");
	errno = 0;
	exit(-1);
      }
      

      // get encoder values
      for (int i=0; i<8; i++){
	encCounts[i] = processEncVal(inBuff, i);

	// check if encoder value has changed
	if (first_read)
	  {
	    prev_encCounts[i] = encCounts[i];
	    isbumped = 0x00;
	  }
	else if ( i!=3 && prev_encCounts[i] != encCounts[i])
	  {
	    isbumped |= 0x01 << i;
	  }
      }
      first_read = 0;

      // if (bumptrue == isbumped)
      // 	printf("arm %d bumped\n", arm);

      // Get state from PLC
      
      rl = inBuff[2] &  0Xc0;
      rl = rl>>5;
      printf("original rl is %d\n",inBuff[2]);
      if (j%1==0)
	printf("rl:%x, arm:%d, bmp:%x,\t enc: %d\t %d\t %d\t %d\t %d\t %d\t %d\t \n",rl, arm, isbumped, encCounts[0], encCounts[1], encCounts[2], encCounts[4], encCounts[5], encCounts[6], encCounts[7]);

      // if (j%100==0)
      // 	printf("100x\n");

      // Sleep until next timer shot
      ts.tv_nsec += interval;
      tsnorm(&ts);
      clock_nanosleep(0, TIMER_ABSTIME, &ts, NULL);

    }

  printf("\n\nProcedure complete.\n\n\t  %d of 2 arms were successfully bumped.\n\n\n", armsbumped);
  if (armsbumped != 2)
    {
      printf("FAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\tFAIL\n\n\n");
      printf("%d arms failed to bump.  Please investigate.\n\t - 48V power on?\n\n\n",2-armsbumped);
    }
  else
    {
      printf("Let's do the robot =D\n\n\n\n");
    }
 
  
  close(fps[0]);
  close(fps[1]);

  return 0;
}


