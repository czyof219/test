#include "cmsis_os2.h"                                        // CMSIS RTOS header file
#include "MX25L256.h"
#include "psram.h"
#include "dbguart.h"
#include "bsp.h"

#include <string.h>
#include <stdio.h>

uint8_t PSRAM_BUF[10*1024];

/*----------------------------------------------------------------------------
 *      Thread 1 'Thread_Name': Sample thread
 *---------------------------------------------------------------------------*/
 /*存储线程。
 所有与FLASH芯片相关的操作，都通过该线程实现。
 */
 
 
void ThreadMx25_PSRAM (void *argument);                                 // thread function
osThreadId_t tid_ThreadMx25_PSRAM;                                      // thread id
 
 
/*存储线程的优先级高。
*/ 
int Init_ThreadMx25_PSRAM (void) {
	const osThreadAttr_t attr={.priority= osPriorityHigh, .stack_size= 1024};
 
	tid_ThreadMx25_PSRAM = osThreadNew (ThreadMx25_PSRAM, NULL, &attr);
	if (!tid_ThreadMx25_PSRAM) return(-1);
  
	return(0);
}
 
void ThreadMx25_PSRAM (void *argument) {
	
	uint32_t  adrKK= 0x0;
	uint8_t id[4];
	uint8_t buf[256];
	char outStr[64];		//C 调试信息输出用字符串缓冲。
	
	osDelay(100);
	
	initMx25Hal();
	uartS("init MX25L256\r\n");
	
	initPSRAM();
	uartS("init PSRAM\r\n");
	
	
	if(0 == Mx25GetId(id)){
		sprintf(outStr, "FLASH id: %02x-%02x-%02x\r\n", id[0], id[1], id[2]);
		uartS(outStr);
	}
	else{
		uartS("err: MX25L256's id\r\n");
		while(1){
			osDelay(1000);
		}
	}
	
	
	/*读取PSRAM芯片的id。
	*/
	if( 0 == PsramGetId(id, 4) ){
		/*需丢弃id[0]，根因待查。
		*/
		sprintf(outStr, "PSARM id: %02x-%02x-%02x\r\n", id[1], id[2], id[3]);
		uartS(outStr);
	}
	else{
		uartS("err: PSRAM's id\r\n");
		while(1){
			osDelay(1000);
		}
	}
	
	
	
	while (1) {
	 	TIM5->ARR= 11256-1;			//C 400hz
		TIM5->EGR |= (1UL<< 0);		//C !!
		
		TIM5->CR1 |= TIM_CR1_CEN;
		osDelay(500);
		
		TIM5->CR1 &= ~TIM_CR1_CEN;
		osDelay(3000);
		
		
		TIM5->ARR= 3216-1;				//C 1.4Khz
		TIM5->EGR |= (1UL<< 0);		//C !!
		TIM5->CR1 |= TIM_CR1_CEN;
		osDelay(500);
		
		TIM5->CR1 &= ~TIM_CR1_CEN;
		osDelay(3000);
		
//		
//		/*擦除一个64KB sector.
//		*/
//		if(0 == Mx25BE64K((uint8_t*)0) ){
//			if(OUT_FLASH & DebugOutEn)
//				uartS("erase 64KB\r\n");
//			
//			Mx25Read((uint8_t*)0, buf, 8);
//			
//			uartHex(buf[0]);
//			uartHex(buf[1]);
//			uartHex(buf[2]);
//			uartHex(buf[3]);
//		}	
//		else{
//			if(OUT_FLASH & DebugOutEn)
//				uartS("err: 64KB\r\n");
//		}
//		
//		
//		/*向第0页写入数据。
//		*/
//		memset(buf, 0xCC, 256);
//		
//		if( 0 == Mx25PP(0, buf) ){
//			if(OUT_FLASH & DebugOutEn)
//				uartS("PP成功\r\n");
//			
//			/*读回数据
//			*/
//			memset(buf, 0, 256);
//			
//			Mx25Read((uint8_t*)0, buf, 8);
//			
//			uartHex(buf[0]);
//			uartHex(buf[1]);
//			uartHex(buf[2]);
//			uartHex(buf[3]);
//			uartHex(buf[4]);
//		}
//		else{
//			if(OUT_FLASH & DebugOutEn)
//				uartS("PP失败\r\n");
//		}
		
		
		
		/*PSRAM读写测试。
		*/
		/*清零PSRAM_BUF[]
		*/
		{
			uint32_t i;
			for(i= 0; i< 10*1024; i++){
				PSRAM_BUF[i]= i;
			}
		}
		
		/*向PSRAM写入10KiB。
		*/
		PsramWrite(adrKK, PSRAM_BUF, 4);
		
		
		/*从PSRAM读取10KiB。
		*/
		PsramRead(adrKK, PSRAM_BUF, 4);
		
		/*判断先写后读的操作是否正常。
		*/
		{
			uint32_t i;
			for(i= 6; i< 10*1024; i++){
				if(PSRAM_BUF[i] != (uint8_t)i){
					if(OUT_PSRAM & DebugOutEn){
						sprintf(outStr, "err: PSRAM at %0x\r\n", i);
						uartS(outStr);
					}
					break;
				}
			}
			
			if(i == 10*1024){
				if(OUT_PSRAM & DebugOutEn){
					sprintf(outStr, "PSARM addr= %0x ok\r\n", adrKK);
					uartS(outStr);
				}
			}	
		}
		
		adrKK+= 0x1234;
	}
}

/******** END OF FILE ********/

