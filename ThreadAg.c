#include "cmsis_os2.h"                                        // CMSIS RTOS header file

#include "leds.h"
#include "stm32l4xx.h"
#include "dbgUart.h"
#include "mymessage.h"
#include "bsp.h"


extern osMessageQueueId_t midDisp;

extern uint32_t gVal;			//C Nmֵ��

void initUart1(uint32_t baud);
static void ToAg(uint8_t addr, uint8_t dat);

extern osMessageQueueId_t midUp;

typedef struct _AG_RECV{
	uint8_t idx;
	uint8_t data[12];
	uint8_t HasHead;	//C �Ƿ��յ�����ͷ(0x55,0x53)��
}AG_RECV;

AG_RECV agRecv;

/*����
*/
typedef struct up{
	uint8_t idxI;
	uint8_t idxO;
	uint8_t data[4][5];	//C 5BΪ1������4����
}UP;

UP upPC;



/*�ص����������ڽ��ա�
���� recvData���ոս��յ���1B��
QQ�������Ϊ״̬�����ա�
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
	
	
	/*������յ������ݡ�
	!!����0x55,0x53��
	*/
	agRecv.data[agRecv.idx-2]= recvDat;
	
	agRecv.idx++;
	
	/*���յ�11B����Ϊ��һ����
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
			buf[0]= 0x01;	//C ��ϢԴ���Ƕȴ�����
			buf[1]= 0;
			buf[2]= agRecv.data[4];
			buf[3]= agRecv.data[5];
			
			/*��ȡʱ�������ֻȡ32bit�еĵ�24bit��
			*/
			uint32_t xx= TIM5->CNT;
			buf[4]= xx>>16;
			buf[5]= xx>>8;
			buf[6]= xx;
			
			if(osOK != osMessageQueuePut(midUp, buf, 0, NULL)){
				__NOP();
			}
			
			
//			/*������
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

		
		/*11BΪһ��������Ϊ�µ�һ���������㡣
		*/
		agRecv.idx= 0;
		agRecv.HasHead= 0;
	}	
	
	
	
	
//	/*�Ƿ���ȫ�Ƕ�ֵ��
//	*/
//	if(8 == agRecv.idx){
//		/*�����Լ������壬ʹ����Ϣ���С�
//		*/
//		buf[0]= 0x01;	//C ��ϢԴ���Ƕȴ�����
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
//			/*11BΪһ��������Ϊ�µ�һ���������㡣
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
	
	/*��ʼ���ͽǶȴ�����������MCUӲ���������ýǶȴ������Ļش����ʡ�
	*/
	agRecv.idx= 0;
	agRecv.HasHead= 0;
	
	initUart1(115200);		//C ����115200Ϊʾ�⡣
	
	
	//ToAg(4, 0x07);
	
	//ToAg(3, 0x0B);
	uartS("Ag 250Hz\r\n");
	osDelay(1000);
	
	uartS("Ag ok\r\n");
	
	
	while (1) {

		osDelay(500);
	}
}


/*��ʼ��MCU��Uart1������Ƕȴ�������WT101��Ƕ��忨��ģ�飩ͨ�š�
������� baud���̶�Ϊ115200bps��
���Ͳ�����ѯ��ʽ�����ղ����жϷ�ʽ��
�Ƕȴ������ڻش�����Ϊ250Hzʱ��������Ϊ115200��
�Ƕȴ������ڻش�����Ϊ500Hzʱ���������Ƽ�Ϊ230400��
*/
void initUart1(uint32_t baud)
{
	/*�������š�
	U1TX---PA9��MCU��Ƕȴ����������Ӵ��ڡ�
	U1RX---PA10��MCU��Ƕȴ����������Ӵ��ڡ�
	*/
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;	//C Enable GPIOB clock.

	GPIOA->MODER &= ~(3UL<< 2*9);		//C ���㡣
	GPIOA->MODER |= (2UL<< 2*9);		//C PA9 as AF mode.
	GPIOA->OSPEEDR |= (2UL<< 2*9);		//C PA9 as high speed.
	GPIOA->PUPDR &= ~(3UL<< 2*9);		//C PA9 as no pu��no pd.

	GPIOA->MODER &= ~(3UL<< 2*10);		//C ���㡣	
	GPIOA->MODER |= (2UL<< 2*10);		//C PA10 as AF mode.
	GPIOA->OSPEEDR |= (2UL<< 2*10);		//C PA10 as high speed.
	GPIOA->PUPDR |= (1UL<< 2*10);		//C PA10 as pu.
	
	/*���ù���*/
	GPIOA->AFR[1] |= (7UL<< 4*1);		//C PA9
	GPIOA->AFR[1] |= (7UL<< 4*2);		//C PA10.
	
	
	/*����Uart1�ĸ�ʽ��115200��8-n-1��
	*/
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;		//C Enable USART1 clock. 
	
	//C OVER8@CR1=0, 16 samples.
	/* baudrate= fck/(8*(2-OVER8)*UARTDIV). UARTDIV= man.fra.
	��fpclk=84MHz��16samples��115200ʱ��man.fra=45.5625����0x2D9��
	����ο�RM0090's P978��
	*/
//	USART1->BRR= 260;							//C 30MHz->115200.
	USART1->BRR= 0x271;							//C 72MHz->115200.
		
	/*CR1��CR2Ĭ��ʱ��Ϊ8-n-1��
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


/*����1�жϺ�����
ֻ��������жϡ�
*/
void USART1_IRQHandler(void)
{
	if(USART1->ISR & USART_ISR_RXNE){	//C RXNE
		/*��DR����RXNE��
		*/
		cbRecv(USART1->RDR);
	}
	
	
	/*���ڴ���û��Ӳ��FIFO���п��ܷ������ڽ������ݱ�overrun��
	��ʱ����Ҫ����ñ�־λ����Ϊ�ñ�־λҲ�����𴮿��жϡ�
	*/
	if(USART1->ISR & USART_ISR_ORE){
		USART1->ICR |= USART_ICR_ORECF;
	}
		
}

/*���ڽ����жϴ�������
��������0x55 0x52 xL xH, yL,yH, zL,zH, TL,TH, SUM�ĽǼ��ٶ�֡��
0x55 0x53 xL xH, yL,yH, zL,zH, TL,TH, SUM�ĽǶ�֡��
*/
//static uint8_t dataAg[16];

//void USART1_IRQHandler(void)
//{
//	static uint8_t RxIdx;	//C ��������Ż��ѽ��ո�����
//	uint8_t tmp8;
//	uint32_t ag;	//C �Ƕ�ֵ����Χ��[]��
//	
//	
//	if(USART1->SR & (1UL<< 5)){	//C RXNE
//		tmp8= USART1->DR;
//		
//		if( (0 == RxIdx)){	//C ����֡ͷ��
//			if(0x55 == tmp8){
//				RxIdx= 1;
//				dataAg[0]= tmp8;
//			}
//		}
//		else if(1 == RxIdx){
//			//if( (0x52 == tmp8) || (0x53 == tmp8) ){	//C ���յ�֡ͷ��
//			if( (0x53 == tmp8) ){
//				RxIdx= 2;
//				dataAg[1]= tmp8;
//			}
//			else
//				RxIdx= 0;	//C ���½���֡ͷ��
//		}
//		else{	//C �������ݡ�
//			dataAg[RxIdx]= tmp8;	
//			RxIdx++;
//			
//			if(11 == RxIdx){	//C �ѽ��ո�����һ��֡��
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
//				/*����Ƕȴ�������ԭʼֵ6B��
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
//				/*����Ƕȣ�Yaw
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


/*MCU��Ƕȴ������������ݣ������ûش����ʡ�Z������򴮿ڲ����ʡ�
addr= 0x76, dat=0,Z�����㡣

addr= 0x03, dat= 6�����ûش�����Ϊ10Hz��Ĭ�ϣ���
addr= 0x03, dat= 9�����ûش�����Ϊ100Hz��
addr= 0x03, dat= 0xA�����ûش�����Ϊ125Hz��
addr= 0x03, dat= 0xB�����ûش�����Ϊ250Hz����������Ϊ115200ʱ����������Ϊ250Hz�ɣ���

addr= 0x04, dat= 0x2�����ô��ڲ�����Ϊ9600��Ĭ�ϣ���
addr= 0x04, dat= 0x6�����ô��ڲ�����Ϊ115200��
addr= 0x04, dat= 0x7�����ô��ڲ�����Ϊ230400��
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

