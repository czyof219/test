#include "cmsis_os2.h"                                        // CMSIS RTOS header file
#include <stdint.h>
#include "stm32l4xx.h"
#include "dbgUart.h"


/*----------------------------------------------------------------------------
 *      Thread 1 'Thread_Name': Sample thread
 *---------------------------------------------------------------------------*/
static void initBatChgHal(void);
static int32_t IsChging(void);

void ThreadBatChg (void *argument);                                 // thread function
osThreadId_t tid_ThreadBatChg;                                      // thread id
 
int Init_ThreadBatChg (void) {
	const osThreadAttr_t attr={.priority= osPriorityAboveNormal};
 
	tid_ThreadBatChg = osThreadNew (ThreadBatChg, NULL, &attr);
	if (!tid_ThreadBatChg) return(-1);
  
	return(0);
}
 
void ThreadBatChg (void *argument) {
	
	initBatChgHal();
	
	while (1) {
		if( IsChging() ){
			uartS("Charging...\r\n");
		}
			
		osDelay(3000);
	}
}


/*
PC0---NCHARG
*/
static void initBatChgHal(void)
{
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
	
	/*NCHARG触发引脚，输入，上拉。
	*/
	GPIOC->MODER &=	~(3UL<< 2*0);		//C PC4是输入。
	GPIOC->PUPDR |= (1UL<< 2*0);		//C pull-up	
}


/*判断是否在充电。
返回值：0---不在充电，1---在充电。
*/
static int32_t IsChging(void)
{
	if( GPIOC->IDR & (1UL<< 0) )
		return 0;
	else
		return 1;
}

