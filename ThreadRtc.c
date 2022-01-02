#include "cmsis_os2.h"                                        // CMSIS RTOS header file
#include "bsp.h"
#include "dbgUart.h"

/*----------------------------------------------------------------------------
 *      Thread 1 'Thread_Name': Sample thread
 *---------------------------------------------------------------------------*/
 
void ThreadRtc (void *argument);                                 // thread function
osThreadId_t tid_ThreadRtc;                                      // thread id
 
int Init_ThreadRtc (void) {
 
  tid_ThreadRtc = osThreadNew (ThreadRtc, NULL, NULL);
  if (!tid_ThreadRtc) return(-1);
  
  return(0);
}
 
void ThreadRtc (void *argument) {
 
	uint8_t tim[8];
	
	
	
	
	while (1) {
		; // Insert thread code here...
		osDelay(100);                                       // suspend thread
	}
}



