#include "cmsis_os2.h"                                        // CMSIS RTOS header file
#include "stm32l4xx.h"
#include "dbguart.h"
#include "bsp.h"

/*----------------------------------------------------------------------------
 *      Thread 1 'Thread_Name': Sample thread
 *---------------------------------------------------------------------------*/
#define KEY_DN_TIME_PWROFF			(3000)		//C�������¶�ùػ���  


//C �Ƶ�bsp.h�С�
//#define LOCK_PS_ON			( GPIOB->BSRR |= (1UL<< 9))
//#define LOCK_PS_OFF			( GPIOB->BSRR |= (1UL<< (9+16)))


extern uint32_t swNmOut;
extern uint32_t gflag_ok;	//C �Ƿ���ok����
uint32_t g_flagPwrOff;		//�ػ���־��0--�ǹػ�������--�ػ���

void ThreadKey (void *argument);                                 // thread function
osThreadId_t tid_ThreadKey;                                      // thread id
 
int Init_ThreadKey (void) {
 
	const osThreadAttr_t attr={.priority= osPriorityAboveNormal1};
		
	tid_ThreadKey = osThreadNew (ThreadKey, NULL, &attr);
	if (!tid_ThreadKey) return(-1);
  
	return(0);
}
 
void ThreadKey (void *argument) {
 
	uint32_t keyDnTime;			//C ��������ʱ����
	uint32_t keyOkDnTime;		//C OK������ʱ����
	
	/*��������
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
		/*�ж��Ƿ���5������֮һ�İ��¡�
		ֻ������򵥵��������ֻ��1���������¡�
		ESC /K2/PB0�ǰ������롣
		OK  /K3/PC5�ǰ������롣
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
						/*��������OK������3s��ִ�йػ���
						*/
						LOCK_PWR_OFF;
						g_flagPwrOff= 1;
						
						while(1);
					}
				}
				
				keyOkDnTime= 0;
				
				uartS("K3/OK dn\r\n");
				
				swNmOut= !swNmOut;	//C ȡ����
				gflag_ok= 1;
				
			}
		}

		osDelay(10);
	}
	
//	while (1) {
//		if( 0== (GPIOA->IDR &(1UL<< 6)) ){ 
//			keyDnTime+= 100;
//			
//			/*�����³���_3s ����ػ���
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

//		osDelay(100);//C QQ �������osThreadYield ();                                         // suspend thread
//	}	

}

/**** END OF FILE ****/

