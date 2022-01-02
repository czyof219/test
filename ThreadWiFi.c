#include "cmsis_os2.h"                                        // CMSIS RTOS header file
 
#include "leds.h"
#include "psram.h"
#include "stm32l4xx.h"
#include "dbgUart.h"
#include "mymessage.h"
#include "op.h"
#include "bsp.h"
#include "shuaji.h"			//ˢ��������ص�ͷ�ļ���
#include <stdlib.h>

#define TMO_UPDATING		(1000*10)	//��λ��ms��

//��������λ�����صĴ����롣
#define ERR_NO				0
#define ERR_CRC				1
#define ERR_SIZE			2

const uint8_t mid_0002_hdr[]={
	'0','1','6','3',	//len
	'0','0','0','2',	//MID0002
	'0','0','4',	//rev04
	'0',				//no ack flag
	'0','0',			//station ID
	'1','1',			//spindle ID	
	'0','0',			//seq
	'0',				//
	'0',				//message part num.
};

const uint8_t BOOT_echo[]={
	'0','0','2','4',	//len=24
	'0','0','0','5',	//MID7000
	'0','0','4',		//rev04
	'0',				//no ack flag
	'0','0',			//station ID
	'1','1',			//spindle ID	
	'0','0',			//seq
	'0',				//
	'0',				//message part num.
};

const uint8_t DATA_echo[]={
	'0','0','2','4',	//len=24
	'7','8','0','2',	//MID7000
	'0','0','4',		//rev04
	'0',				//no ack flag
	'0','0',			//station ID
	'1','1',			//spindle ID	
	'0','0',			//seq
	'0',				//
	'0',				//message part num.
};


const uint8_t D2_echo[]={
	'0','0','2','4',	//len=24
	'7','0','0','0',	//MID7000
	'0','0','4',		//rev04
	'0',				//no ack flag
	'0','0',			//station ID
	'1','1',			//spindle ID	
	'0','0',			//seq
	'0',				//
	'0',				//message part num.
};


const uint8_t MID0004_hdr[]={
	'0','0','2','4',	//len=24
	'0','0','0','4',	//MID7000
	'0','0','1',		//rev01
	'0',				//no ack flag
	'0','0',			//station ID
	'1','1',			//spindle ID	
	'0','0',			//seq
	'0',				//
	'0',				//message part num.
};


const uint8_t MID0005_hdr[]={
	'0','0','2','4',	//len=24
	'0','0','0','5',	//MID7000
	'0','0','1',		//rev01
	'0',				//no ack flag
	'0','0',			//station ID
	'1','1',			//spindle ID	
	'0','0',			//seq
	'0',				//
	'0',				//message part num.
};


extern osMessageQueueId_t midDisp;
extern osMessageQueueId_t midOpRecv;
extern uint32_t flag_app_to_boot2;


typedef struct _PROG{
	uint32_t cntKB;			//C ��������еļ�������λΪKiB.
	uint32_t fileaddr;		//C �������ʼ��ַ��
	uint32_t filesize;
	uint32_t filecrc;
	int32_t sn;				//��ˮ�š�
	uint32_t tsStart;		//������ʼʱ��ʱ�����0��ʾδ����������
}PROG;


#define UART_WIFI		USART3

/*���ŵ͵�ƽʱ��WiFi��Ч��
*/
#define WIFI_READY_STA	( (GPIOC->IDR & (1UL<< 3))? 0:1)	
#define WIFI_LINK_STA	( (GPIOC->IDR & (1UL<< 2))? 0:1)

/*����/�ر�WiFiģ�顣
����Ч�����͵�ƽʱ������WiFiģ�顣
*/
#define WIFI_PWR_ON		( GPIOC->BSRR |= (1UL<< (13+16)) )		
#define WIFI_PWR_OFF	( GPIOC->BSRR |= (1UL<< 13) )

MSG_DISP msgDispWiFi;


static uint32_t get_filesize(uint8_t* p);
static uint32_t get_filecrc(uint8_t* p);
static uint32_t get_psram_crc(uint8_t* p, uint32_t size);
static void send_pak(OPPAK* ppak);
static void set_mid_0002_pak(uint8_t* phdr, OPPAK* ppak);
static void set_BOOT_echo_pak(uint8_t* phdr, OPPAK* ppak);
static void set_BOOT_echo_shit_pak(uint8_t* phdr, OPPAK* ppak);
static void set_DATA_echo_pak(uint32_t sn, OPPAK* ppak);
static void set_DATA_ALL_echo_pak(uint32_t code, OPPAK* ppak);
static void go_app(void);
static uint32_t read_app_ver(void);

void initUart3(uint32_t baud);
void initWiFiHal(void);

/*----------------------------------------------------------------------------
 *      Thread 1 'Thread_Name': Sample thread
 *---------------------------------------------------------------------------*/
 
void ThreadWiFi (void *argument);                                 // thread function
osThreadId_t tid_ThreadWiFi;                                      // thread id
 
int Init_ThreadWiFi (void) {
 
	const osThreadAttr_t attr={.priority= osPriorityAboveNormal2, .stack_size= 1024+4096}; 
	
	tid_ThreadWiFi = osThreadNew (ThreadWiFi, NULL, &attr);
	if (!tid_ThreadWiFi) return(-1);
  
	return(0);
}
 

void ThreadWiFi (void *argument) {
 
	static uint8_t wifiSta;
	uint8_t id[4];
	char outStr[64];		//C ������Ϣ������ַ������塣
	osStatus_t status = osOK;
	uint32_t cnt= 0;
	OPPAK Op_Recv;
	PROG prog={.sn=-1, .tsStart= 0};
	MSG_DISP msgDispWiFi;
	uint8_t buf[1024];
	uint8_t bufBk[1024];
	initWiFiHal();
	initUart3(128000);	//C �ѹ̶�Ϊ115200��
	
	initPSRAM();
	uartS("init PSRAM\r\n");
	
	/*��ȡPSRAMоƬ��id��
	*/
	if( 0 == PsramGetId(id, 4) ){
		/*�趪��id[0]��������顣
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
	
//	/*PSRAM�Ķ�д���ԣ�
//	д1K����1K���Ƚϡ�����512KiB��
//	*/
//	for(int i= 0; i< 512; i++){
//		for(int j= 0; j< 1024; j++)
//			buf[j]= rand();
//	
//		PsramWrite(1024*i, buf, 1024);
//		PsramRead(1024*i ,bufBk, 1024);
//		
//		
//		for(int j= 0; j< 1024; j++){
//			if(buf[j] !=bufBk[j]){
//				uartS("err: buf");
//				while(1);
//			}
//		}
//	}
//	
//	uartS("PSRAM 512KiB Test OK\r\n");
	
	
	/*����WiFiģ�顣
	*/
	WIFI_PWR_ON;
	uartS("WiFi ok\r\n");
	
	if(0 == flag_app_to_boot2){
		osDelay(2000);	//C ��ʱ2s�ٷ���WiFiģ�顣
	}
	else{
		uartS("recv boot\r\n");
				
		//����ļ����ƣ�
		uartS(Op_Recv.data);
		
		//��ȡ�ļ���С��CRC��
		int i= 0;
		for(i= 0; i< 100; i++){
			if('\0' == Op_Recv.data[i])
				break;
		}	
		prog.filesize= get_filesize(&Op_Recv.data[i+1]);
		prog.filecrc= get_filecrc(&Op_Recv.data[i+5]);
		prog.fileaddr= 0x08080000;		//QQ�ݶ�ֵ��
		
		uartS("size,crc=");
		uartHex32(prog.filesize);
		uartHex32(prog.filecrc);
		
		set_BOOT_echo_shit_pak(NULL, &Op_Recv);	
		
		send_pak(&Op_Recv);

		uartS("boot first echo\r\n");
		prog.sn= -1;
		
		
		/*���½���*/
		msgDispWiFi.src= MSGDISP_FROM_WIFI;
		msgDispWiFi.data[0]= 'C';
		msgDispWiFi.data[1]= 0;
		msgDispWiFi.data[2]= 0;
		
		osMessageQueuePut(midDisp, &msgDispWiFi, 0, NULL);
	}
	
	
	
	
	while (1) {
	
		status = osMessageQueueGet(midOpRecv, &Op_Recv, NULL, 50);
		
		if(osOK == status){
			prog.tsStart= osKernelGetTickCount();	//!!�ݶ����յ�WIFI���ڵ����ݣ�����Ϊ����������
			
			/*���Header�ṹ�塣
			*/
			uint16_t op_mid= ascii_to_4dig(Op_Recv.hdr.mid);
			uint8_t tmp[3+5];
			
			tmp[0]='M';
			tmp[1]='I';
			tmp[2]='D';
			tmp[3]= Op_Recv.hdr.mid[0];
			tmp[4]= Op_Recv.hdr.mid[1];
			tmp[5]= Op_Recv.hdr.mid[2];
			tmp[6]= Op_Recv.hdr.mid[3];
			tmp[7]= '\0';
			uartS((char*)tmp);
			
			
			if(MID0001 == op_mid){	//MID0001
				uartS("recv MID0001\r\n");
				
				/*�յ�MID0001����Ӧ��MID0002��
				*/
				set_mid_0002_pak((uint8_t*)set_mid_0002_pak, &Op_Recv);
				
				send_pak(&Op_Recv);
				
				uartS("send 0002\r\n");
			}
			else if(MID_BOOT == op_mid){		//������˽��Э�飺����BOOT��
				
				uartS("recv boot\r\n");
				
				//����ļ����ƣ�
				uartS(Op_Recv.data);
				
				//��ȡ�ļ���С��CRC��
				int i= 0;
				for(i= 0; i< 100; i++){
					if('\0' == Op_Recv.data[i])	//!!winidows�У�'*'���ܳ������ļ����С�
						break;
				}	
				prog.filesize= get_filesize(&Op_Recv.data[i+1]);
				prog.filecrc= get_filecrc(&Op_Recv.data[i+5]);
				prog.fileaddr= 0x08080000;		//QQ�ݶ�ֵ��
				
				uartS("size,crc=");
				uartHex32(prog.filesize);
				uartHex32(prog.filecrc);
				
				set_BOOT_echo_pak(NULL, &Op_Recv);	
				
				send_pak(&Op_Recv);
			
				uartS("boot echo\r\n");
				prog.sn= -1;
				
				
				/*���½���*/
				msgDispWiFi.src= MSGDISP_FROM_WIFI;
				msgDispWiFi.data[0]= 'C';
				msgDispWiFi.data[1]= 0;
				msgDispWiFi.data[2]= 0;
				
				osMessageQueuePut(midDisp, &msgDispWiFi, 0, NULL);
			}
			else if(MID_PROG_DATA == op_mid){	//������˽��Э�飺�������ݰ�
				uartS("\r\ndata sn=");
				uint32_t sn= ascii_to_4dig(Op_Recv.data);
				uartHex16(sn);
				
				if(sn == (uint32_t)(prog.sn+ 1)){
					//д��PSRAM�С�
					PsramWrite(sn*1024, Op_Recv.data+4, 1024);	//.data��ƫ��4BΪ���ݿ�ʼ��
					
					set_DATA_echo_pak(sn, &Op_Recv);
					
					send_pak(&Op_Recv);
			
					uartS("DATA echo\r\n");
					
					prog.cntKB= sn;		//�����ѽ��յ������ݴ�С��KB����
					prog.sn+= 1;		//�����ѽ��յ�����ˮ�š�!!���ѽ��յ��ģ��Է�����ˮ�Ų�����ʱ��Ӧ��
					
					/*���½���*/
					msgDispWiFi.src= MSGDISP_FROM_WIFI;
					msgDispWiFi.data[0]= 'D';
					msgDispWiFi.data[1]= prog.cntKB+1;
					msgDispWiFi.data[2]= (prog.filesize+1023)/1024;
					
					osMessageQueuePut(midDisp, &msgDispWiFi, 0, NULL);
				}
				else{
					set_DATA_echo_pak(prog.sn, &Op_Recv);
					
					send_pak(&Op_Recv);
			
					uartS("err: sn, repeat it\r\n");	
				}
			}
			else if(MID_DATA_ALL == op_mid){	//ȫ��������ɰ�
				//int32_t sizeKB= (prog.filesize+1023)/1024;
				if((uint32_t)(prog.sn+1) == (prog.filesize+1023)/1024){
					uartS("\r\ndata all");
					
					if(1){//prog.filecrc == get_psram_crc(0,prog.filesize)){	//CRC��ȷ
						set_DATA_ALL_echo_pak(ERR_NO, &Op_Recv);
						
						send_pak(&Op_Recv);
					
						uartS("all echo\r\n");
						
						
						//��PSARM�ĳ���ˢ����FLASH�У�����ת��APP��
						//���½��棺"flashing..."
						msgDispWiFi.src= MSGDISP_FROM_WIFI;
						msgDispWiFi.data[0]= 'I';
						msgDispWiFi.data[1]= 0;		//��ʾ��flashing...
						msgDispWiFi.data[2]= 0;
						
						osMessageQueuePut(midDisp, &msgDispWiFi, 0, NULL);
						
						uartS("\r\nshuaji:");
						uartHex32(prog.fileaddr);
						uartHex32(prog.filesize);
						
						osDelay(100);	//��ʱ���Ը�����ʾ��
						
						
						__disable_irq();					
						int ret= shuaji(prog.fileaddr,prog.filesize);
						__enable_irq();
						
						if(0 == ret){
							msgDispWiFi.src= MSGDISP_FROM_WIFI;
							msgDispWiFi.data[0]= 'I';
							msgDispWiFi.data[1]= 1;	//��ʾ��flashing...OK
							msgDispWiFi.data[2]= 0;
						
							osMessageQueuePut(midDisp, &msgDispWiFi, 0, NULL);
							
							uartS("shuaji ok\r\n");
							uartS("awaiting manual shut down\r\n");
							osDelay(3000);
							
							//__disable_irq();
							//rst_per();
							//vect_rst();
							while(1){
								osDelay(200);
							}
						}
						else if(-1 == ret){	//CRC,��δʵ��
							msgDispWiFi.src= MSGDISP_FROM_WIFI;
							msgDispWiFi.data[0]= 'I';
							msgDispWiFi.data[1]= 2;	//��ʾ��flashing...CRC������̺��FLASH������У��CRCʧ�ܡ�
							msgDispWiFi.data[2]= 0;
						
							osMessageQueuePut(midDisp, &msgDispWiFi, 0, NULL);
							
							uartS("err:flash...crc\r\n");
						}
						else if(-2 == ret){	//����д���1��
							msgDispWiFi.src= MSGDISP_FROM_WIFI;
							msgDispWiFi.data[0]= 'I';
							msgDispWiFi.data[1]= 3;	//��ʾ��flashing...ERR1������̹�����ʧ�ܡ�
							msgDispWiFi.data[2]= 0;
						
							osMessageQueuePut(midDisp, &msgDispWiFi, 0, NULL);
							
							uartS("err:flash...err1\r\n");
						}
						else if(-3 == ret){	//����д���2��
							msgDispWiFi.src= MSGDISP_FROM_WIFI;
							msgDispWiFi.data[0]= 'I';
							msgDispWiFi.data[1]= 4;	//��ʾ��flashing...ERR2������̹�����ʧ�ܡ�
							msgDispWiFi.data[2]= 0;
						
							osMessageQueuePut(midDisp, &msgDispWiFi, 0, NULL);
							
							uartS("err:flash...err2\r\n");
						}
						
					}
					else{	//CRC����
						set_DATA_ALL_echo_pak(ERR_CRC, &Op_Recv);
						
						send_pak(&Op_Recv);
					
					
						//���½��棺ERR:CRC
						msgDispWiFi.src= MSGDISP_FROM_WIFI;
						msgDispWiFi.data[0]= 'I';
						msgDispWiFi.data[1]= 5;	//��ʾ��PSRAM...CRC
						msgDispWiFi.data[2]= 0;
						
						osMessageQueuePut(midDisp, &msgDispWiFi, 0, NULL);
						
						uartS("err:psram crc\r\n");
					}
				}
				else{
					set_DATA_ALL_echo_pak(ERR_SIZE, &Op_Recv);
					
					send_pak(&Op_Recv);
				
					uartS("err:size\r\n");
					
					//���½��棺ERR:SIZE
					msgDispWiFi.src= MSGDISP_FROM_WIFI;
					msgDispWiFi.data[0]= 'I';
					msgDispWiFi.data[1]= 2;
					msgDispWiFi.data[2]= 0;
					
					osMessageQueuePut(midDisp, &msgDispWiFi, 0, NULL);
				}
			}
			else{
				
			}
		}
		else{	//��ʱ QQ�����䡣
			
		}
		
		
		//WiFi�߳����б�ǡ�
		cnt++;
		if(0 == (cnt%4)){
			uartS("W");	
			
//			/*�Ƿ�����������������//ȥ����ʱ���ơ�
//			*/
//			if(prog.tsStart){
//				if(TMO_UPDATING< (osKernelGetTickCount()- prog.tsStart)){
//					uartS("tmo in updating(20s), now rst\r\n");
//					
//					osDelay(100);
//					
//			
//					//��ת��Ӧ�ó���!!����תǰ��Ҫ�ж�APP�Ƿ���Ч��
//					uint32_t* p= (uint32_t*)0x08080000;
//					
//					if(( *p & 0xFFFC0000) == 0x20000000){
//						uartS("set app flag, boot2->boot\r\n");
//						osDelay(10);
//					
//						//ʹ��BKPxR�ķ��ʡ�
//						RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
//						PWR->CR1 |= PWR_CR1_DBP;
//						
//						/*����BKP0R��boot��ʶ����
//						*/
//						RTC->BKP0R= 0x88888888;
//						
//						__set_PSP(*p);
//						__set_MSP(*p);
//						__set_CONTROL(0);
//						//__disable_irq();
//						
//						rst_per();
//						
//						vect_rst();
//						
//						while(1);
//					}
//					else{
//						uartS("no app, stay in boot2\r\n");	
//						
//						prog.tsStart= osKernelGetTickCount();
//					}
//				}
//			}
		}
	}
}


/*��ʼ��MCU��Uart3������WiFiģ��ͨ�š�
������� baud���̶�Ϊ128000bps��
���Ͳ�����ѯ��ʽ�����ղ����жϷ�ʽ��
*/
void initUart3(uint32_t baud)
{
	/*�������š�
	U3TX---PB10��MCU��WiFiģ������Ӵ��ڡ�
	U3RX---PB11��MCU��WiFiģ������Ӵ��ڡ�
	*/
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;	//C Enable GPIOB clock.

	GPIOB->MODER &= ~(3UL<< 2*10);		//C ���㡣
	GPIOB->MODER |= (2UL<< 2*10);		//C PB10 as AF mode.
	GPIOB->OSPEEDR |= (2UL<< 2*10);		//C PB10 as high speed.
	GPIOB->PUPDR &= ~(3UL<< 2*10);		//C PB10 as no pu��no pd.

	GPIOB->MODER &= ~(3UL<< 2*11);		//C ���㡣	
	GPIOB->MODER |= (2UL<< 2*11);		//C PB11 as AF mode.
	GPIOB->OSPEEDR |= (2UL<< 2*11);		//C PB11 as high speed.
	GPIOB->PUPDR |= (1UL<< 2*11);		//C PB11 as pu.
	
	/*���ù���*/
	GPIOB->AFR[1] |= (7UL<< 4*2);		//C PB10
	GPIOB->AFR[1] |= (7UL<< 4*3);		//C PB11.
	
	
	/*����Uart3�ĸ�ʽ��128000��8-n-1��
	*/
	RCC->APB1ENR1 |= RCC_APB1ENR1_USART3EN;	
	

	/* baudrate= fck/(8*(2-OVER8)*UARTDIV)��OVER8@CR1=0, 16 samples.
	*/
	//USART3->BRR= 0x271;						//C 115200@72MHz.
	USART3->BRR= 0x233;							//C 128000@72MHz.����232��
		
	/*CR1��CR2Ĭ��ʱ��Ϊ8-n-1��
	*/
	USART3->CR1 |= (1UL<< 3)|(1UL<< 2);			//C enable TX, RX.
	USART3->CR1 |= (1UL<< 0);					//C enbale USART.
	
	USART3->CR1 |= (1UL<< 5);					//C enable RXNEIE.
	
	/* preemption = 1, sub-priority = 1 */
	NVIC_SetPriority(USART3_IRQn, 1);
	
	/* Enable Interrupt for UART0 channel */
	NVIC_EnableIRQ(USART3_IRQn);				//C U2U3͸��ʱ���������жϡ�
}


/*
*/
//C Func: DBG_UART interrupt handler.
//C input:  none
//C return: none
void USART3_IRQHandler(void)
{
	static uint8_t tmp;
	static uint32_t idx;
	
	
	if(USART3->ISR & USART_ISR_RXNE){
		tmp= USART3->RDR;
		
		cb_RecvOp2(tmp);
	}
	else if(USART3->ISR &USART_ISR_ORE){
		/*ʹ��RXNE�жϺ��������ORE��Ҳ������жϣ����봦��
		ʵ�ⷢ�֣���230400bpsʱ�������ORE������ܼ򵥣�ֻ��ȡRDR���������𣿿���������2���ⲿ�ж�USART1��EXT�йء�����
		*/
		USART3->ICR |= USART_ICR_ORECF;
		
		uartS("ORE");
	}
}


/*��ʼ��MCU��WiFiģ�飨USR-C216�����������š�
nLink----PC2�����룬��ʾ�Ƿ������ӣ�WiFiģ����ΪAPʱ���������豸�����ϣ�������Ч��
KWiFi----PC13�������ͨ������MOS�ܵ�WiFiģ���Դ������Ч��WiFiģ���е磩��
*/
void initWiFiHal(void)
{
	
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;	//C Enable GPIOB clock.

	
	GPIOC->MODER &=	~(3UL<< 2*2);		//C PC2�����롣
	GPIOC->PUPDR |= (1UL<< 2*2);		//C pull-up	

	GPIOC->MODER &= ~(3UL<< 2*13);
	GPIOC->MODER |= (1UL<< 2*13);		

	WIFI_PWR_ON;//GPIOC->BSRR |= (1UL<< 13);			//C PC13��output H��WiFiģ���޵硣
}


/*���ֽڷ��͡�
*/
static void send_pak(OPPAK* ppak)
{
	uint8_t* p= (uint8_t*)ppak;
	
	for(int i= 0; i< ppak->len; i++){
		UART_WIFI->TDR= p[i];
		while(0 == (UART_WIFI->ISR & USART_ISR_TXE));
	}
	
	UART_WIFI->TDR= 0x0;	//send NULL.
}


/*ʹ��ָ����Header���MID0002��

*/
static void set_mid_0002_pak(uint8_t* phdr, OPPAK* ppak)
{
	uint8_t* p= (uint8_t*)ppak;
	
	for(int i= 0; i< 20; i++){
		p[i]= mid_0002_hdr[i];
	}
	
	for(int i= 20; i< 163; i++){
		p[i]= ' ';	//!!�⹤˵�������ո񣬲�����'0'��
	}
	
	ppak->len= 163;
	
	
	/*����汾��
	*/
	p[83]= '0';
	p[84]= '6';
	
	if(0 == read_app_ver()){	//��ȡӦ�ó���İ汾��
		uint8_t* paddr= (uint8_t*)0x08080600;
		p[85]= *paddr;p[86]= *(paddr+1);
		p[87]= '.';
		p[88]= *(paddr+2);p[89]=*(paddr+3);
		
		uartS("app ver\r\n");
	}
	else{
		//QQ�汾�������д:00.00
		p[85]= '0';p[86]='0';
		p[87]= '.';
		p[88]= '0';p[89]='0';
		
		uartS("00.00 ver\r\n");
	}
}


static void set_BOOT_echo_pak(uint8_t* phdr, OPPAK* ppak)
{
	uint8_t* p= (uint8_t*)ppak;
	
	for(int i= 0; i< 20; i++){
		p[i]= BOOT_echo[i];
	}
	
	p[20]= '7';
	p[21]= '8';
	p[22]= '0';
	p[23]= '0';
	
	ppak->len= 24;
}


static void set_BOOT_echo_shit_pak(uint8_t* phdr, OPPAK* ppak)
{
	uint8_t* p= (uint8_t*)ppak;
	
	for(int i= 0; i< 20; i++){
		p[i]= BOOT_echo[i];
	}
	
	p[20]= '7';
	p[21]= '8';
	p[22]= '0';
	p[23]= '8';
	
	ppak->len= 24;
}

static void set_DATA_echo_pak(uint32_t sn, OPPAK* ppak)
{
	uint8_t* p= (uint8_t*)ppak;
	
	for(int i= 0; i< 20; i++){
		p[i]= DATA_echo[i];
	}
	
	p[20]= 0x30+ (sn/1000);
	sn= sn%1000;
	p[21]= 0x30+ (sn/100);
	sn= sn%100;
	p[22]= 0x30+ (sn/10);
	sn= sn%10;
	p[23]= 0x30+ (sn/1);
	
	ppak->len= 24;
}


/*���ȫ����ɵ�Ӧ���PAK��
����1��ִ�н��
����2�������PAK��ָ�롣
*/
static void set_DATA_ALL_echo_pak(uint32_t code, OPPAK* ppak)
{
	uint8_t* p= (uint8_t*)ppak;
	
	if(0 == code){
		for(int i= 0; i< 20; i++){
			p[i]= MID0005_hdr[i];
		}
		
		p[20]= '7';
		p[21]= '8';
		p[22]= '0';
		p[23]= '3';
	
		ppak->len= 24;
	}
	else{
		for(int i= 0; i< 20; i++){
			p[i]= MID0004_hdr[i];
		}
		p[20]= '7';
		p[21]= '8';
		p[22]= '0';
		p[23]= '3';
		
		p[24]= code/10;
		p[25]= code%10;
		
		ppak->len= 26;
		
	}
}


static uint32_t get_filesize(uint8_t* p)
{
	uint32_t ret;
	ret= p[0];
	ret<<= 8;
	ret|= p[1];
	ret<<= 8;
	ret|= p[2];
	ret<<= 8;
	ret|= p[3];
	
	return ret;
}


static uint32_t get_filecrc(uint8_t* p)
{
	return get_filesize(p);
}


/*����PSRAM��ĳ��ʼ��ַ��CRC32��
�������    p��PSRAM����ʼ��ַ��
������� size�����ȡ�
����ֵ������õ���CRC32��
*/
static uint32_t get_psram_crc(uint8_t* p, uint32_t size)
{
	uint32_t addr;
	uint8_t buf[1024];
	uint32_t crc32= 0xFFFFFFFF;;
	
	
	if(1024< size){	//�ļ���С> 1024���ֶ��1024�İ����㡣
		int k= size/1024;
		int i= 0;
		for(i= 0; i< k; i++){
			PsramRead((uint32_t)p+1024*i, buf, 1024);
			crc32= crc32_no_XOROT(crc32, buf, 1024);
		}
		
		int j= size%1024;
		
		if(0 != j){	//�����1024�����ݡ�
			PsramRead(1024*i, buf, 1024);
			crc32= crc32_no_XOROT(crc32, buf, j);
		}
		
		crc32= crc32 ^ 0xFFFFFFFF;
	}
	else{	//��һ�μ���
		PsramRead((uint32_t)p, buf, 1024);	
		crc32= calculate_fcs32(p, size);
	}
	
	return crc32;
}

typedef  void (*pFunction)(void);

static void go_app(void)
{
	pFunction Jump_To_Application;
	uint32_t StartAddr= 0x08080000;
	uint32_t JumpAddress;
	
	/*�ж�������ַ���Ƿ��г��� */
	if (((*(uint32_t*)StartAddr) & 0xFFFC0000 ) == 0x20000000){ 	//��ջջ��ָ��λ�ã�������IRAM1�С�
		
		rst_per();	//��λ��GPIO֮����������衣
		
		/* Jump to user application 
		*/
		JumpAddress = *(uint32_t*) (StartAddr + 4);		//C ��λ������ַ��
		Jump_To_Application = (pFunction) JumpAddress;
		
		/* Initialize user application's Stack Pointer */
		//C __MSR_MSP(*(vu32*) ApplicationAddress);	
		__set_CONTROL(0); //!!�ǳ���Ҫ����ûɶ�ô���
		__set_MSP(*(uint32_t*) StartAddr);	//C @core_cm3.h
		__set_PSP(*(uint32_t*) StartAddr);	//C @core_cm3.h
		
		__enable_irq();
		Jump_To_Application();				//C QQΪʲô��ֱ��ִ��JumpAddress()��
	}
}


/*��ȡӦ�ó���İ汾��
�̶�λ��Ϊ��08080600��08080601��08080602��08080603��08080604��У��ͣ�
����ֵ��0--�ɹ�������--ʧ�ܡ�
*/
static uint32_t read_app_ver(void)
{
	uint8_t ver[5];
	uint8_t* paddr= (uint8_t*)0x08080600;
	uint8_t chk;
	
	ver[0]= *paddr;
	ver[1]= *(paddr+1);
	ver[2]= *(paddr+2);
	ver[3]= *(paddr+3);
	
	ver[4]= *(paddr+4);
	
	chk= 0;
	for(int i= 0; i< 5; i++){
		chk+= ver[0];
	}
	
	return (uint32_t)chk;
}

/******** END OF FILE ********/

