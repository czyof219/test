#include "cmsis_os2.h"                                        // CMSIS RTOS header file
#include "AdcVol.h" 

/*----------------------------------------------------------------------------
 *      Thread 1 'Thread_Name': Sample thread
 *---------------------------------------------------------------------------*/
 
void ThreadVbat (void *argument);                                 // thread function
osThreadId_t tid_Thread;                                      // thread id
 
 
int Init_ThreadVbat (void) {
	const osThreadAttr_t attr={.priority= osPriorityLow7}; 
  
	tid_Thread = osThreadNew (ThreadVbat, NULL, NULL);
  
	if (!tid_Thread) return(-1);
  
	return(0);
}
 

/*检测VBAT电压的线程
*/
void ThreadVbat (void *argument) {
 
	initAdcHal();
	
	while (1) {
		/*每秒检测一下VBAT电压。
		*/
		osDelay(1000);                                      // suspend thread
		
		startAdc();
		
		/*等待序列转换完成，并清零标记。
		*/
		while(0 == (ADC1->ISR & ADC_ISR_JEOS));		
		ADC1->ISR |= ADC_ISR_JEOS;
		
		
		/*计算VBAT电压值。
		*/
		if(DebugOutEn & OUT_ADC){
			uartHex16(ADC1->JDR1);
			uartHex16(ADC1->JDR2);
			uartHex16(ADC1->JDR3);
			uartHex16(ADC1->JDR4);
		}
		
		uint32_t adc;
		adc= ADC1->JDR1;
		adc+= ADC1->JDR2;
		adc+= ADC1->JDR3;
		adc+= ADC1->JDR4;
		
		adc/=4;
		
	}
}

/**** END OF FILE ****/

