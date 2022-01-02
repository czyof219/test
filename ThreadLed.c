#include "cmsis_os2.h"                                        // CMSIS RTOS header file
 
/*----------------------------------------------------------------------------
 *      Thread 1 'Thread_Name': Sample thread
 *---------------------------------------------------------------------------*/
 
#include "drv595.h"
#include "chgBat.h"
#include "dbguart.h"
#include "bsp.h"

static void setRG(void);
static void clrRG(void);
extern uint32_t g_flagPwrOff;		//关机标志：0--非关机，其他--关机。

void ThreadLed (void *argument);                                 // thread function
osThreadId_t tid_ThreadLed;                                      // thread id
 
int Init_ThreadLed (void) {
 
	const osThreadAttr_t attr={.priority= osPriorityAboveNormal1};

	tid_ThreadLed = osThreadNew (ThreadLed, NULL, &attr);
	
	if (!tid_ThreadLed) return(-1);

	return(0);
}



void ThreadLed (void *argument) 
{
	
	initLed();
	uartS("ThreadLed run...\r");
	
	osDelay(3000);
	
	while (1) {
		if(0 == g_flagPwrOff){
			LEDR_ON;
			osDelay(2000);
		}
		
		LEDR_OFF;
	
		osDelay(500);
		
		if(0 == g_flagPwrOff){
			LEDR_ON;
			LEDG_ON;
			osDelay(2000);
		}
		
		LEDR_OFF;
		LEDG_OFF;
	
		osDelay(500);
		
		if(IsChging())
			uartS("Charging....\r\n");
	}
}


/*所有灯亮。
*/
static void setRG(void)
{
		set1R();
		set2R();
		set3R();
		set4R();
		set5R();
		set6R();
	
		set1G();
		set2G();
		set3G();
		set4G();
		set5G();
		set6G();
	
		Drv595();
}


/*所有灯灭。
*/
static void clrRG(void)
{
		clr1R();
		clr2R();
		clr3R();
		clr4R();
		clr5R();
		clr6R();
	
		clr1G();
		clr2G();
		clr3G();
		clr4G();
		clr5G();
		clr6G();
	
		Drv595();
}
/********* END OF FILE ********/

