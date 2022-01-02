#include "cmsis_os2.h"                                        // CMSIS RTOS header file
#include <stdint.h>
#include "stm32l4xx.h"
#include "dbgUart.h"


#define NTRIG_L			( GPIOC->BSRR |= (1UL<< (16+1)) )	
#define NTRIG_H			( GPIOC->BSRR |= (1UL<< (1)) )

/*----------------------------------------------------------------------------
 *      Thread 1 'Thread_Name': Sample thread
 *---------------------------------------------------------------------------*/
 static void initScanerHal(void);
 
 
void ThreadScaner (void *argument);                                 // thread function
osThreadId_t tid_ThreadScaner;                                      // thread id
 
int Init_ThreadScaner (void) {
	const osThreadAttr_t attr={.priority= osPriorityAboveNormal};
	
	tid_ThreadScaner = osThreadNew (ThreadScaner, NULL, &attr);
	if (!tid_ThreadScaner) return(-1);
  
	return(0);
}
 

/*
*/
void ThreadScaner (void *argument) {
	
	initScanerHal();
	
	while (1) {
		NTRIG_L;
		osDelay(1000);
		NTRIG_H;
		
		
		osDelay(10000);
	}
}


/*配置扫描器用到硬件。
UART5，
PC1---触发引脚，低有效。
*/
static void initScanerHal(void)
{
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
	
	/*NTRIG触发引脚。
	*/
	GPIOC->MODER &= ~(3UL<< 2*1);
	GPIOC->MODER |= (1UL<< 2*1);		//C genernal output.
	GPIOC->BSRR |= (1UL<< 1);			//C output H，不触发。
}
