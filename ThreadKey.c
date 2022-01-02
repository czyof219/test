#include "cmsis_os2.h"                                        // CMSIS RTOS header file
#include "stm32l4xx.h"
#include "dbguart.h"
#include "bsp.h"

/*----------------------------------------------------------------------------
 *      Thread 1 'Thread_Name': Sample thread
 *---------------------------------------------------------------------------*/
#define KEY_DN_TIME_PWROFF			(3000)		//C按键按下多久关机。  


//C 移到bsp.h中。
//#define LOCK_PS_ON			( GPIOB->BSRR |= (1UL<< 9))
//#define LOCK_PS_OFF			( GPIOB->BSRR |= (1UL<< (9+16)))


extern uint32_t swNmOut;
extern uint32_t gflag_ok;	//C 是否按了ok键。
uint32_t g_flagPwrOff;		//关机标志：0--非关机，其他--关机。

void ThreadKey (void *argument);                                 // thread function
osThreadId_t tid_ThreadKey;                                      // thread id
 
int Init_ThreadKey (void) {
 
	const osThreadAttr_t attr={.priority= osPriorityAboveNormal1};
		
	tid_ThreadKey = osThreadNew (ThreadKey, NULL, &attr);
	if (!tid_ThreadKey) return(-1);
  
	return(0);
}
 
void ThreadKey (void *argument) {
 
	uint32_t keyDnTime;			//C 按键按下时长。
	uint32_t keyOkDnTime;		//C OK键按下时长。
	
	/*开机按键
	*/
	//osDelay(500);
	
	LOCK_PS_ON;
	g_flagPwrOff= 0;
	
//	if( 0== (GPIOA->IDR & (1UL<< 6)) ){
//			LOCK_PS_ON;
//	}
//	else{
//		while(1);
//	}
	keyOkDnTime= 0;
	
	uartS("Lock PS\r\n");
	
	while(1){
		/*判断是否有5个按键之一的按下。
		只考虑最简单的情况，即只有1个按键按下。
		ESC /K2/PB0是按键输入。
		OK  /K3/PC5是按键输入。
		*/
		if( 0 == KEY_ESC ){
			/*PB0
			*/
			osDelay(10);
			
			if( 0 == KEY_ESC ){
				while( 0 == KEY_ESC ){
					osDelay(10);
				}
				uartS("K2/ESC dn\r\n");
			}
		}
		
		
		if( 0 != KEY_OK ){
			/*PC5
			*/
			osDelay(10);
			
			if( 0 != KEY_OK ){
				while( 0 != KEY_OK ){
					osDelay(10);
					
					keyOkDnTime++;
					
					if(300< keyOkDnTime){
						/*连续按下OK键超过3s，执行关机。
						*/
						LOCK_PWR_OFF;
						g_flagPwrOff= 1;
						
						while(1);
					}
				}
				
				keyOkDnTime= 0;
				
				uartS("K3/OK dn\r\n");
				
				swNmOut= !swNmOut;	//C 取反。
				gflag_ok= 1;
				
			}
		}

		osDelay(10);
	}
	
//	while (1) {
//		if( 0== (GPIOA->IDR &(1UL<< 6)) ){ 
//			keyDnTime+= 100;
//			
//			/*长按下超过_3s ，则关机。
//			*/
//			if(keyDnTime> KEY_DN_TIME_PWROFF){
//				LOCK_PS_OFF;
//				
////				uartS("\t\tPS OFF...\r\n");
//				//while(1);
//			}

//		}
//		else{
//			keyDnTime= 0;
//		}
//		

//		osDelay(100);//C QQ 会更好吗？osThreadYield ();                                         // suspend thread
//	}	

}

/**** END OF FILE ****/

