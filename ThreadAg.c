#include "cmsis_os2.h"                                        // CMSIS RTOS header file

#include "leds.h"
#include "stm32l4xx.h"
#include "dbgUart.h"
#include "mymessage.h"
#include "bsp.h"


extern osMessageQueueId_t midDisp;

extern uint32_t gVal;			//C Nm值。

void initUart1(uint32_t baud);
static void ToAg(uint8_t addr, uint8_t dat);

extern osMessageQueueId_t midUp;

typedef struct _AG_RECV{
	uint8_t idx;
	uint8_t data[12];
	uint8_t HasHead;	//C 是否收到数据头(0x55,0x53)。
}AG_RECV;

AG_RECV agRecv;

/*满：
*/
typedef struct up{
	uint8_t idxI;
	uint8_t idxO;
	uint8_t data[4][5];	//C 5B为1包，共4包。
}UP;

UP upPC;



/*回调函数：串口接收。
输入 recvData：刚刚接收到的1B。
QQ建议更改为状态机接收。
*/
void cbRecv(uint8_t recvDat)
{
	static uint8_t ledSta;
	uint8_t buf[12];
	
	
	if(0 == agRecv.idx){
		if(0x55 == recvDat){
			agRecv.idx= 1;
			agRecv.HasHead= 0;
		
			return;
		}
		else{
			return;
		}
	}
	else if(1 == agRecv.idx){
		if(0x53 == recvDat){
			agRecv.idx= 2;
			agRecv.HasHead= 1;
		
			return;
		}
		else{
			agRecv.idx= 0;
			agRecv.HasHead= 0;
			
			return;
		}
	}	
	
	if(1 != agRecv.HasHead)
		return;
	
	
	/*保存接收到的数据。
	!!不含0x55,0x53。
	*/
	agRecv.data[agRecv.idx-2]= recvDat;
	
	agRecv.idx++;
	
	/*接收到11B，认为是一包。
	*/
	if(11 == agRecv.idx){
		uint8_t tmp;
		
		tmp= 0x55;
		tmp+= 0x53;
		tmp+= agRecv.data[0];
		tmp+= agRecv.data[1];
		tmp+= agRecv.data[2];
		tmp+= agRecv.data[3];
		tmp+= agRecv.data[4];
		tmp+= agRecv.data[5];
		tmp+= agRecv.data[6];
		tmp+= agRecv.data[7];

		if(tmp == agRecv.data[8]) 
		{
			buf[0]= 0x01;	//C 消息源：角度传感器
			buf[1]= 0;
			buf[2]= agRecv.data[4];
			buf[3]= agRecv.data[5];
			
			/*获取时间戳，但只取32bit中的低24bit。
			*/
			uint32_t xx= TIM5->CNT;
			buf[4]= xx>>16;
			buf[5]= xx>>8;
			buf[6]= xx;
			
			if(osOK != osMessageQueuePut(midUp, buf, 0, NULL)){
				__NOP();
			}
			
			
//			/*调试用
//			*/ 
//			if(ledSta){
//				ledSta= 0;
//				LED1_OFF;
//			}
//			else{
//				ledSta= 1;
//				LED1_ON;
//			}
		}

		
		/*11B为一个完整。为新的一包接收清零。
		*/
		agRecv.idx= 0;
		agRecv.HasHead= 0;
	}	
	
	
	
	
//	/*是否收全角度值。
//	*/
//	if(8 == agRecv.idx){
//		/*不用自己做缓冲，使用消息队列。
//		*/
//		buf[0]= 0x01;	//C 消息源：角度传感器
//		buf[1]= 0;
//		buf[2]= agRecv.data[4];
//		buf[3]= agRecv.data[5];
//		
////		if((buf[2] == 0x0D) &&(0x0 == buf[3])){
////			buf[2]= 0xFF;
////			buf[3]= 0xFF;
////		}
////		
////		if((buf[2] == 0x0E) &&(0x0 == buf[3])){
////			buf[2]= 0xFF;
////			buf[3]= 0xFF;
////		}
//		
//		osMessageQueuePut(midUp, buf, 0, NULL);
//		
//	}
//	else{	
//		if(11 == agRecv.idx){
//			/*11B为一个完整。为新的一包接收清零。
//			*/
//			agRecv.idx= 0;
//			agRecv.HasHead= 0;
//		}
//	}
}


/*----------------------------------------------------------------------------
 *      Thread 1 'Thread_Name': Sample thread
 *---------------------------------------------------------------------------*/
 
void ThreadAg (void *argument);                                 // thread function
osThreadId_t tid_ThreadAg;                                      // thread id
 
int Init_ThreadAg (void) {
	const osThreadAttr_t attr={.priority= osPriorityAboveNormal3};  
	
	tid_ThreadAg = osThreadNew (ThreadAg, NULL, &attr);
	if (!tid_ThreadAg) return(-1);
  
	return(0);
}
 

/*
*/
void ThreadAg (void *argument) {
	
	/*初始化和角度传感器相连的MCU硬件，并配置角度传感器的回传速率。
	*/
	agRecv.idx= 0;
	agRecv.HasHead= 0;
	
	initUart1(115200);		//C 参数115200为示意。
	
	
	//ToAg(4, 0x07);
	
	//ToAg(3, 0x0B);
	uartS("Ag 250Hz\r\n");
	osDelay(1000);
	
	uartS("Ag ok\r\n");
	
	
	while (1) {

		osDelay(500);
	}
}


/*初始化MCU的Uart1。它与角度传感器（WT101或嵌入板卡的模块）通信。
输入参数 baud：固定为115200bps。
发送采用轮询方式。接收采用中断方式。
角度传感器在回传速率为250Hz时，波特率为115200。
角度传感器在回传速率为500Hz时，波特率推荐为230400。
*/
void initUart1(uint32_t baud)
{
	/*配置引脚。
	U1TX---PA9，MCU与角度传感器的连接串口。
	U1RX---PA10，MCU与角度传感器的连接串口。
	*/
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;	//C Enable GPIOB clock.

	GPIOA->MODER &= ~(3UL<< 2*9);		//C 清零。
	GPIOA->MODER |= (2UL<< 2*9);		//C PA9 as AF mode.
	GPIOA->OSPEEDR |= (2UL<< 2*9);		//C PA9 as high speed.
	GPIOA->PUPDR &= ~(3UL<< 2*9);		//C PA9 as no pu，no pd.

	GPIOA->MODER &= ~(3UL<< 2*10);		//C 清零。	
	GPIOA->MODER |= (2UL<< 2*10);		//C PA10 as AF mode.
	GPIOA->OSPEEDR |= (2UL<< 2*10);		//C PA10 as high speed.
	GPIOA->PUPDR |= (1UL<< 2*10);		//C PA10 as pu.
	
	/*复用功能*/
	GPIOA->AFR[1] |= (7UL<< 4*1);		//C PA9
	GPIOA->AFR[1] |= (7UL<< 4*2);		//C PA10.
	
	
	/*配置Uart1的格式：115200，8-n-1。
	*/
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;		//C Enable USART1 clock. 
	
	//C OVER8@CR1=0, 16 samples.
	/* baudrate= fck/(8*(2-OVER8)*UARTDIV). UARTDIV= man.fra.
	例fpclk=84MHz，16samples，115200时，man.fra=45.5625，即0x2D9。
	具体参考RM0090's P978。
	*/
//	USART1->BRR= 260;							//C 30MHz->115200.
	USART1->BRR= 0x271;							//C 72MHz->115200.
		
	/*CR1，CR2默认时，为8-n-1。
	*/
	USART1->CR1 |= (1UL<< 3)|(1UL<< 2);			//C enable TX, RX.
	USART1->CR1 |= (1UL<< 0);					//C enbale USART.
	
	USART1->CR1 |= (1UL<< 5);					//C enable RXNEIE.
	
	/* preemption = 1, sub-priority = 1 */
	NVIC_SetPriority(USART1_IRQn, 1);
	
	/* Enable Interrupt for UART0 channel */
	#if defined(DBG_U2U1_U2U3)
	
	#else
	NVIC_EnableIRQ(USART1_IRQn);	
	#endif
}


/*串口1中断函数。
只处理接收中断。
*/
void USART1_IRQHandler(void)
{
	if(USART1->ISR & USART_ISR_RXNE){	//C RXNE
		/*读DR清零RXNE。
		*/
		cbRecv(USART1->RDR);
	}
	
	
	/*由于串口没有硬件FIFO。有可能发生串口接收数据被overrun。
	这时，需要清零该标志位。因为该标志位也会引起串口中断。
	*/
	if(USART1->ISR & USART_ISR_ORE){
		USART1->ICR |= USART_ICR_ORECF;
	}
		
}

/*串口接收中断处理函数。
接收形如0x55 0x52 xL xH, yL,yH, zL,zH, TL,TH, SUM的角加速度帧或
0x55 0x53 xL xH, yL,yH, zL,zH, TL,TH, SUM的角度帧。
*/
//static uint8_t dataAg[16];

//void USART1_IRQHandler(void)
//{
//	static uint8_t RxIdx;	//C 待接收序号或已接收个数。
//	uint8_t tmp8;
//	uint32_t ag;	//C 角度值，范围是[]。
//	
//	
//	if(USART1->SR & (1UL<< 5)){	//C RXNE
//		tmp8= USART1->DR;
//		
//		if( (0 == RxIdx)){	//C 接收帧头。
//			if(0x55 == tmp8){
//				RxIdx= 1;
//				dataAg[0]= tmp8;
//			}
//		}
//		else if(1 == RxIdx){
//			//if( (0x52 == tmp8) || (0x53 == tmp8) ){	//C 接收到帧头。
//			if( (0x53 == tmp8) ){
//				RxIdx= 2;
//				dataAg[1]= tmp8;
//			}
//			else
//				RxIdx= 0;	//C 重新接收帧头。
//		}
//		else{	//C 接收数据。
//			dataAg[RxIdx]= tmp8;	
//			RxIdx++;
//			
//			if(11 == RxIdx){	//C 已接收个数。一整帧。
//				tmp8= dataAg[0];
//				tmp8+=dataAg[1];
//				tmp8+=dataAg[2];
//				tmp8+=dataAg[3];
//				tmp8+=dataAg[4];
//				tmp8+=dataAg[5];
//				tmp8+=dataAg[6];
//				tmp8+=dataAg[7];
//				tmp8+=dataAg[8];
//				tmp8+=dataAg[9];

//				if(tmp8 != dataAg[10]){
//					uartS("err:SUM\r\n");
//					uartHex(tmp8);
//					uartHex(dataAg[10]);
//				}
//				
//				
//				/*输出角度传感器的原始值6B。
//				*/
////				uartS("\r\nWT:");
//////				uartHex(dataAg[0]);
////				uartHex(dataAg[1]);
////				uartHex(dataAg[2]);
////				uartHex(dataAg[3]);
////				uartHex(dataAg[4]);
////				uartHex(dataAg[5]);
////				uartHex(dataAg[6]);
////				uartHex(dataAg[7]);
////				uartHex(dataAg[8]);
////				uartHex(dataAg[9]);
////				uartHex(dataAg[10]);
////				uartS("\r\n");
//				
//				
//				/*计算角度：Yaw
//				*/
//				ag= 0;
//				ag= dataAg[7];
//				ag<<= 8;
//				ag |= dataAg[6];
//				ag*= 180;
//				ag/= 32768;
////				uartS("ag/Nm:");
////				uartHex16(ag);
////				uartHex32(gVal);
////				uartS("\r\n");
//				
//				
//				RxIdx= 0;
//			}
//		}
//	}
//	
//}


/*MCU向角度传感器发送数据，以配置回传速率、Z轴清零或串口波特率。
addr= 0x76, dat=0,Z轴清零。

addr= 0x03, dat= 6，设置回传速率为10Hz（默认）。
addr= 0x03, dat= 9，设置回传速率为100Hz。
addr= 0x03, dat= 0xA，设置回传速率为125Hz。
addr= 0x03, dat= 0xB，设置回传速率为250Hz。（波特率为115200时，不能设置为250Hz吧？）

addr= 0x04, dat= 0x2，设置串口波特率为9600（默认）。
addr= 0x04, dat= 0x6，设置串口波特率为115200。
addr= 0x04, dat= 0x7，设置串口波特率为230400。
*/
static void ToAg(uint8_t addr, uint8_t dat)
{
	uint8_t send[5];
	
	
	send[0]= 0xFF;
	send[1]= 0xAA;
	send[2]= addr;
	send[3]= dat;
	send[4]= 0x0;
	
	for(uint8_t i= 0; i< 5; i++){
		USART1->TDR= send[i];
		while(0 == ( USART1->ISR & USART_ISR_TXE )) {;}	//C TXE.
	}

}

