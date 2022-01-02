/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
 
#ifdef RTE_Compiler_EventRecorder
#include "EventRecorder.h"
#endif

#include <stdlib.h>

#include "bsp.h"
#include "app.h"
#include "dbgUart.h"
#include "uartxx.h"
#include "mymessage.h"
#include "op.h"

#include "GUI.h"
#include "kadacfg.h"


extern int Init_ThreadLed (void);
extern int Init_ThreadKey (void);
extern int Init_ThreadWiFi (void);
extern int Init_ThreadAg (void);
extern int Init_ThreadOled (void);
extern int Init_ThreadPSRAM (void);
extern int Init_ThreadRtc (void);
extern int Init_ThreadBatChg(void);
extern int Init_ThreadScaner(void);
extern int Init_ThreadMx25_PSRAM (void);

//extern void initUart1(uint32_t);		//C ������Ƕ�ģ�������Ĵ��ڡ�
extern void initUart3(uint32_t);	//C ������WIFIģ�������Ĵ��ڡ�!!128000bps.
extern void initWiFiHal(void);


extern DBGUART dbgUart;
extern uint32_t ads1220_ok;

//�汾�š�
uint8_t verboot2[]={"\r\nboot_kada V1.00\r\n"};
uint32_t flagNoJump;	//�յ����'`'��ͣ��boot2������ת��
uint32_t flagRst;		//��λ��־
uint32_t DebugOutEn;

int32_t avg1[9];	//C avg1[0]��ƽ��ֵ�� avg1[1]-avg1[8]�ǰ�ʱ��ǰ���4�������㡣
int32_t avg2[9];	//C avg2[0]��ƽ��ֵ�� avg2[1]-avg2[4]�ǰ�ʱ��ǰ���4�������㡣

	
/*Ť�ص㡣
*/
typedef struct{
	int32_t Nm;		//C Ť��ֵ����ADS1220��ȡ�����з���������
	uint32_t Ts;	//C ʱ���
}NmPOINT;

typedef struct{
	NmPOINT p[512];	//C 512���㡣
	uint32_t idx;	//C ��ǰλ�á�p[idx]Ϊ��д���λ�á�
	NmPOINT max;	//C 512�����е����㡣	
	NmPOINT min;	//C 512�����е���С�㡣
}FIFO_P;


typedef struct{
	NmPOINT point;
	uint32_t cnt;		//C ��ǰֵ
	uint32_t CNT;		//C ����ֵ������Ϊһ���趨����Ϊ������
	int32_t NM;			//C ����ֵ����Ŀ��ֵ��80%��20%����
	uint32_t IsFound;	//C �Ƿ��ҵ���0---δ�ҵ�������---���ҵ��� 
}POINT;


typedef struct _AgDel{
	uint16_t pre;
	uint16_t cur;
	uint16_t del;
}AG_NOW;

/*�������߶Ρ�
*/
typedef struct{
	NmPOINT s;			//C ���
	NmPOINT e;			//C �յ�
	int32_t delta;		//C �յ�-����Nmֵ��Ҳ��ͨ��s��eֱ�Ӽ���õ���
	int32_t interval;	//C �յ�-����ʱ�����Ҳ��ͨ��s��eֱ�Ӽ���õ���
}SLINE;		

typedef struct{
	SLINE line[20];
	uint32_t idx;	//C ����ʱ�ã�����ʹ�þֲ��������Եó����ҡ�
	uint32_t cnt;	//C ʵ���߶ε����������ܳ���20��
}LINE;

LINE kdLine={.idx=0, .cnt=20};

/*�����е�A�㡢B�㡣
*/
POINT pA={.cnt=0, .CNT= 3, .NM= 600000, .IsFound=0};	//C Rising point.
POINT pB={.cnt=0, .CNT= 3, .NM= 120000, .IsFound=0};	//C Falling point.
AG_NOW ag_now= {.pre= 0, .cur= 0, .del= 0};

FIFO_P p512={.max.Nm= 0, .min.Nm= 0};			//C 512���㡣

uint32_t swNmOut;		//C �Ƿ����Nmֵ��

MSG_DISP msgDispNew;

/*�����ú궨��
*/
//#define UART_NM				//C ע�͸��У��򲻴Ӵ������NmЭ�鱨�ġ�
//#define UART_AG				//C ע�͸��У��򲻴Ӵ������AgЭ�鱨�ġ�


osMessageQueueId_t midUp;
osMessageQueueId_t midDisp;
osMessageQueueId_t midOpRecv;	//C �����ѽ���OP���ĵ���Ϣ���С�

/*�ڲ�����������
*/
static int32_t KaDa(FIFO_P* p);
static int32_t KaDa2(FIFO_P* p);
static void go_boot(uint32_t flag);
static int32_t go_app(uint32_t n);		//��ת��app����ѡ1��

/*����SYSCLKΪ30MHz���ӽ�STM32L151RE��32MHz����
û���ҵ�Ƶ�ʱ���ܳ���10����������
*/
void SystemClock_Config(void)
{
	/*ʹ��PLL��ΪSYSCLKǰ��!!����PWR��FLASH���������׽�HardFaul��Ī����������е�ַ��
	Set Voltage scale1 as MCU will run at 80MHz.
	*/
	//C �ϵ�Ĭ��ΪRange1�����Բ���Ҫ���á�
	
	
	/*ʹ��4WS��
	*/
	// RCC->AHB1ENR |= RCC_AHB1ENR_FLASHEN; �ϵ�Ĭ��Ϊ1.
	FLASH->ACR |= FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_4WS;	//C 4 latency. Instruction Prefetch, Data Prefech�ϵ�Ĭ��ʹ�ܡ�
	
	
	/*����PLL(72MHz���Է��㴮��QQ)
	ʹ��HSE��BPY��ֹ�������ȴ��ȶ���
	����PLL��3��������!!PLLVCO��Χ��[64,344]MHz��PLLVCO= HSE*PLLN/PLLM/PLLR��SYSCLK= PLLVCO/PLLDIV.(/2,/3,/4)��
	ѡ��HSEΪPLL src
	ʹ��PLL�����ȴ��ȶ���
	*/
	/*ʹ��HSE���ȴ��ȶ�
	*/
	RCC->CR |= (1UL<< 16);
	while( 0 == (RCC->CR &(1UL<< 17)) ){;}
	
	/*����PLL(16MHz*18/2/2-->72MHz),
	HSE as PLL src.
	PLL�ı�������Ƶ��
	*/
	RCC->PLLCFGR |= (3UL<< 0);
	
	RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLN_Msk);
	RCC->PLLCFGR |= 18UL<< RCC_PLLCFGR_PLLN_Pos;	//C PLLN= 18.����HSEΪ16MHz��
	//RCC->PLLCFGR |= 24UL<< RCC_PLLCFGR_PLLN_Pos;	//C PLLN= 24.����HSEΪ12MHz��	
		
	RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM_Msk);
	RCC->PLLCFGR |= 1UL<< RCC_PLLCFGR_PLLM_Pos;		//C 001: PLLM= 2	

	RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLR);
	RCC->PLLCFGR |= 0UL<< RCC_PLLCFGR_PLLR_Pos;		//C 00: PLLR= 2
		
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLREN;
	
	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLPEN;
	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLQEN;	
		
		
	/*ʹ��PLL���ȴ��ȶ���
	*/
	RCC->CR |= (1UL<< 24);
	while( 0 == (RCC->CR &(1UL<< 25)) ){;}		
	
	
	/*PLL��ΪSYSCLK��������HCLK��PCLK1��PCLK2��
	��Ȼ��������HCLK��PCLK1��PCLK2�ķ�Ƶ���ڽ�PLL��ΪSYSCLK��
	HCLK��no divided����DIV1.
	PCLK1��DIV1��
	PCLK2��DIV1.
	*/
	RCC->CFGR &= RCC_CFGR_HPRE_Msk;		//C HCLK not divided.
	RCC->CFGR &= RCC_CFGR_PPRE1_Msk;	//C PCLK1 not divided
	RCC->CFGR &= RCC_CFGR_PPRE2_Msk;	//C PCLK2 not divided.
	
	/*��PLL��ΪSYSCLK���ȴ���ɡ�
	!!��MSI(00)��ΪPLL�����Բ�����ֱ�Ӹ�ֵ��
	*/
	RCC->CFGR |= (3UL<< 0);				//C PLL as SYSCLK.
	while( (0x3UL<< 2) != (RCC->CFGR & (0x3UL<< 2)) ){;}	//C ��ȡSWS��
	
	
	/*ʹ��MCO/2�����
	��HSE�ֳ�MCO�ĺô���MCO������ƵӰ�졣
	*/
	RCC->CFGR &= ~(0xFUL<< 24);		//C MCOSEL[3:0].
	//RCC->CFGR |= (1UL<< 24);		//C SYSCLK����Ϊʹ����PLL������MAINPLL��
	RCC->CFGR |= (4UL<< 24);		//C ѡ��MCO��ԴΪHSE��
		
	RCC->CFGR &= ~(7UL<< 28);		//C MCOPRE[2:0].
	//RCC->CFGR |= (4UL<< 28);		//C /16����72/16= 4.5MHz��Ҳ����ADS1220�������ʱ�ӡ�
	RCC->CFGR |= (2UL<< 28);		//C /4 ��16/4= 4MHz��	
	
	/*PA8 as MCO.
	*/
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	GPIOA->MODER &= ~(3UL<< 2*8);
	GPIOA->MODER |= (2UL<< 2*8);	//C AF
	GPIOA->OSPEEDR |= (3UL<< 2*8);	//C Very High Speed.
}

/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/
int32_t tmpNm1;
int32_t tmpNm2;
uint32_t flag_app_to_boot2;	//C ��־����app��ת��boot2��


void app_main (void *argument) {
 
	osStatus_t status = osOK;
	uint8_t LedSta= 0;
	uint32_t update_ag;		//C ����Ag��ʱ�̡�
	uint32_t update_nm1;	//C ����Nm1��ʱ�̡�
	uint32_t update_nm2;	//C ����Nm1��ʱ�̡�
	
	uint8_t buf[8];
	uint8_t snd[8];	
	update_ag= 0;
	flagRst= 0xCCCC;
	flag_app_to_boot2= 0;
	
	
	/*Ĭ�ϵ��������Ϣѡ�
	*/
	DebugOutEn |= OUT_FLASH;
	DebugOutEn |= OUT_PSRAM;
	
	
	osDelay(100);
	
	
	/*�жϴ�Ӧ�����յ�������BOOT����������ġ�
	QQ��ʹ��PWRʱ�ӣ��ܷ��ȡRTC->BKP0R��
	*/
	if(0x88886666 == RTC->BKP0R){	//
		//ʹ��BKPxR�ķ��ʡ�
		RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
		PWR->CR1 |= PWR_CR1_DBP;
		
		/*����BKP0R��boot��ʶ����
		*/
		RTC->BKP0R= 0;
		
		flag_app_to_boot2= 1;
		
		uartS("rst from app, ready for update\r\n");
	}
	else{
		uartS("boot->boot2\r\n");
		/*��ʱ2s���Եȴ�'`'�����롣
		*/
		for(int i= 0; i< 4; i++){
			uartS(".");
			osDelay(500);
		}
		
		if(2< flagNoJump){	//����3��'`'��
			uartS("no jump\r\n");
		}
		else{	//��ת��Ӧ�ó���!!����תǰ��Ҫ�ж�APP�Ƿ���Ч��
			uint32_t* p= (uint32_t*)0x08080000;
			
			if(( *p & 0xFFFC0000) == 0x20000000){
				uartS("set app flag, boot2->boot\r\n");
				osDelay(10);
			
				//ʹ��BKPxR�ķ��ʡ�
				RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
				PWR->CR1 |= PWR_CR1_DBP;
				
				/*����BKP0R��boot��ʶ����
				*/
				RTC->BKP0R= 0x88888888;
				
				__disable_irq();
				//__set_PSP(*p);
				__set_CONTROL(0);	//ʵ�������ע�͵���
				
				rst_per();
				
				vect_rst();
				
				while(1);
			}
			else{
				uartS("no app, stay in boot2\r\n");	
			}
		}
	}
	
	
	/* Init the STemWin GUI Library, CRC must be. 
	*/
	RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;	//C ��ʹ��CRC������GUI_Init()����ѭ����
	
	GUI_Init();
	
	
	/*�����̡߳�
	*/
	if(-1 == Init_ThreadKey()){	//C osPriorityAboveNormal1	
		uartS("err:ThreadLed\r\n");
	}
	
	if(-1 == Init_ThreadLed()){
		uartS("err:ThreadLed\r\n");
	}

	if(-1 == Init_ThreadWiFi() ){//C osPriorityAboveNormal2
		uartS("err:ThreadWiFi\r\n");
	}

	if(-1 == Init_ThreadOled() ){//C osPriorityAboveNormal2
		uartS("err:ThreadOled\r\n");
	}
	
	if(-1 == Init_ThreadAg() ){//C osPriorityAboveNormal2
		uartS("err:ThreadAg\r\n");
	}


	
	//Init_ThreadAg();		//C osPriorityAboveNormal3
	//Init_ThreadOled();		//C osPriorityNormal	
	//Init_ThreadVbat();		//C VBAT��ѹ����̡߳�
	
	//Init_ThreadMx25_PSRAM();	//C osPriorityLow7
	//Init_ThreadRtc();
//	Init_ThreadBatChg();
//	Init_ThreadScaner();
	
	
//	/*����ADS1220��2Ƭ��Ť�ش���������
//	*/
//	osDelay(1);
//	if(0 == initADS1220()){
//		ads1220_ok= 1;
//		
//		uartS("initADS1220 ok & START\r\n");
//		
//		CMD_START1();	//C ��1ƬADS1220����ת����
//		CMD_START2();	//C ��2ƬADS1220����ת����
//	}

	/*����ADS1220��2Ƭ��Ť�ش���������
	*/
	osDelay(1);
	if(0 == initADS1220()){
		ads1220_ok= 1;
		
		uartS("initADS1220 ok & START\r\n");
		
		CMD_START1();	//C ��1ƬADS1220����ת����
		CMD_START2();	//C ��2ƬADS1220����ת����
	}
	

	while(1){
		if(0xCCCC != flagRst){	//�����λ
			if(0 == flagRst){
				vect_rst();	//���ϸ�λ��
			}
			else if(flagRst< 20){
				uartS("ready to rst\r\n");
				osDelay(100);
				
				//��ֹ���Դ���
				USART2->CR1 &= ~USART_CR1_UE;
				
				osDelay(flagRst*1000);	//��ʱn���λ��
				
				go_boot(0x88881234);
				
				while(1);
			}
		}
		
		
		status = osMessageQueueGet(midUp, buf, NULL, 50);	//C ����ÿms��������֡����Ϊ�Ƕ���֡/4ms��Nm��֡/3ms����

		if(osOK != status)
			continue;
		
			
		/*�ж��ǽǶ�������
		�ǣ����0xFF 0x01 0x0 RawL RawH���ͳ�ȥ��
		*/
		if(0x01 == buf[0]){
			if(OUT_AG & DebugOutEn)
				uartS("C");
			
			ag_now.pre= ag_now.cur;
			ag_now.cur= buf[3];
			ag_now.cur<<= 8;
			ag_now.cur |= buf[2];
			
			if(ag_now.cur > ag_now.pre){
				ag_now.del= ag_now.cur- ag_now.pre;
			}
			else{
				ag_now.del= ag_now.pre- ag_now.cur;
			}
			
			char str[48];
			if( TOL_AG_DEL< ag_now.del ){
				if( TOL_AG_DEL < (65535- ag_now.del) ){
					/*CW��CCW�Ĳ�ֵ����������ֵ��
					*/
					
					sprintf(str, "err:pre=%d,cur=%d\r\n", ag_now.pre, ag_now.cur);
				}
				else{	
					sprintf(str, "*del=%d\r\n", 65535- ag_now.del);	
				}
			}
			else{
				
				sprintf(str, "del=%d\r\n", ag_now.del);	
			}
			
			uartS(str);
			
			
			uint32_t tmp= osKernelGetTickCount();
			
			if(100 <(tmp- update_ag)){
				/*ÿ��200ms����һ�Ρ�
				*/
				update_ag= tmp;
				
				msgDispNew.src= MSGDISP_FROM_AG;
				msgDispNew.data[0]= buf[3];
				msgDispNew.data[0]<<= 8;
				msgDispNew.data[0] |= buf[2];
				
				osMessageQueuePut(midDisp, &msgDispNew, 0, NULL);
			}
			
			
		#if defined(UART_AG)
			snd[0]= 0xFF;
			snd[1]= 0x01;
			snd[2]= 'j';
			snd[3]= buf[2];
			snd[4]= buf[3];
			
			/*ʱ�����
			*/
			snd[5]= buf[4];
			snd[6]= buf[5];
			snd[7]= buf[6];
			//uartS("A");
		#endif
//				uint16_t i= buf[3];
//				i<<= 8;
//				i|= buf[2];
//				
//				uint8_t str[10];
//				sprintf((char*)str, "%d\r\n", i);
//				
//				uint8_t j= 0;
//				while('\0' != str[j]){
//					USART2->DR= str[j];
//					while( 0 == (USART2->SR & USART_SR_TXE)){;}
//					
//					j++;
//				}
		}
		else if(0x02 == buf[0]){
			if(DebugOutEn & OUT_NM1)
				uartS("X");
			
			/*������չ��
			*/
			if(0x80 & buf[1]){
				tmpNm1= 0xFF;
			}
			else{
				tmpNm1= 0;
			}
			tmpNm1<<= 8;
			tmpNm1|= buf[1];
			tmpNm1<<= 8;
			tmpNm1|= buf[2];
			tmpNm1<<= 8;
			tmpNm1|= buf[3];
			
			
			
			uint32_t kk;	
				
			
			/*������չ��
			��24bit�з�������Ϊ32bit�з�������
			*/
			if(0x80 & buf[1]){
				kk= 0xFF;
			}
			else{
				kk= 0;
			}
			kk<<= 8;
			kk|= buf[1];
			kk<<= 8;
			kk|= buf[2];
			kk<<= 8;
			kk|= buf[3];
			
			
			/*�����Ĵ�ƽ��ֵ��
			*/
			avg1[1]= avg1[2];
			avg1[2]= avg1[3];
			avg1[3]= avg1[4];
			avg1[4]= avg1[5];
			avg1[5]= avg1[6];
			avg1[6]= avg1[7];
			avg1[7]= avg1[8];
			avg1[8]= (int32_t)kk;
			avg1[0]= (avg1[1]+avg1[2]+avg1[3]+avg1[4]+avg1[5]+avg1[6]+avg1[7]+avg1[8])/8;
			
			
			/*�ǵ�1ƬADS1220��Nm���ݣ���0xFF 0x02 MSB nn LSB���ͳ�ȥ��
			*/
			uint32_t tmp= osKernelGetTickCount();
			
			if(100 <(tmp- update_nm1)){
				/*ÿ��200ms����һ�Ρ�
				*/
				update_nm1= tmp;
				
				msgDispNew.src= MSGDISP_FROM_NM1;
				msgDispNew.data[0] = avg1[1];
				
				osMessageQueuePut(midDisp, &msgDispNew, 0, NULL);
			}
		}
		else if(0x03 == buf[0]){
			if(DebugOutEn & OUT_NM1)
				uartS("Y");

			
			/*������չ��
			*/
			if(0x80 & buf[1]){
				tmpNm2= 0xFF;
			}
			else{
				tmpNm2= 0;
			}
			tmpNm2<<= 8;
			tmpNm2|= buf[1];
			tmpNm2<<= 8;
			tmpNm2|= buf[2];
			tmpNm2<<= 8;
			tmpNm2|= buf[3];
			
			
			/*�ǵ�2ƬADS1220��Nm���ݣ���0xFF 0x02 MSB nn LSB���ͳ�ȥ��
			*/
			uint32_t tmp= osKernelGetTickCount();
			
			if(100 <(tmp- update_nm2)){
				/*ÿ��200ms����һ�Ρ�
				*/
				update_nm2= tmp;
				
				msgDispNew.src= MSGDISP_FROM_NM2;
				uint32_t tmp;
				
				/*������չ��
				*/
				if(0x80 & buf[1]){
					tmp= 0xFF;
				}
				else{
					tmp= 0;
				}
				tmp<<= 8;
				tmp|= buf[1];
				tmp<<= 8;
				tmp|= buf[2];
				tmp<<= 8;
				tmp|= buf[3];

				msgDispNew.data[0] = (int32_t)tmp;
				
				osMessageQueuePut(midDisp, &msgDispNew, 0, NULL);
			}
		}
	}//C END while(1)
}


int main (void) {
 
	// System Initialization
	SystemClock_Config();
	SystemCoreClockUpdate();
	
	NVIC_DisableIRQ(TIM2_IRQn);	
	
#ifdef RTE_Compiler_EventRecorder
  // Initialize and start Event Recorder
  EventRecorderInitialize(EventRecordError, 1U);
#endif
  // ...
	/*���õ�����Ϣ�����
	*/
	DebugOutEn |= OUT_ADC;
	
	uint32_t cntOkKeyDn= 0;
	
	/*��ʼ��2��������
	��ʼ������ʹ�����ź�DAC1��
	*/
	initKey();
	//initBuzzHal();
	//initTim5();
	initDAC1();
	//initRTC();
	
	
	/*��ʼ�����Դ��ڡ�
	*/	
	dbgUart.flagTxing= 0;
	dbgUart.hwFIFOcnt= 1;
	dbgUart.initUart= initUart;
	dbgUart.wrTHR= wrTHR;
	dbgUart.intTHR= intTHR;
	dbgUart.UARTn= USART2;
	dbgUart.initUart(0);			//C ����Ϊ0����ʵ���ô���
	
	__enable_irq();	
	
	uartS((char*)verboot2);
	
	
	
	/* Initialize CMSIS-RTOS
	*/
	osKernelInitialize();                 
  
	osThreadNew(app_main, NULL, NULL);    // Create application main thread
	
 	
	midUp = osMessageQueueNew(8, 8, NULL);
	if (!midUp){ 
		while(1);
	}
	
  	midDisp = osMessageQueueNew(8, sizeof(MSG_DISP), NULL);
	if (!midDisp){ 
		while(1);
	} 

	midOpRecv= osMessageQueueNew(4, sizeof(OPPAK), NULL);
	if (!midOpRecv){ 
		while(1);
	}
	
	osKernelStart();                      // Start thread execution
	for (;;) {}
}



/*����x�е�1��������
���������x
�����������
����ֵ��1����������Χ[0,8]��
˵������������������λ����1��������������bit[7:0]��bit[6:0]...��bit[1:0]��
*/
uint32_t Cnt1(uint8_t x)
{
	uint8_t mask= 0xFF;	//C ������ķ�Χ����bit[7:0]-->0xFF. bit[1:0]-->0x03
	uint8_t cnt;
	
	x&= mask;
	
	cnt= 0;
	
	for(uint8_t i= 0; i< 8; i++){
		if( x&(1<< i) )
			cnt++;
	}
	
	return cnt;
}

/*��512�������������߶Ρ�
���������512��FIFO��
�����������
����ֵ��0--��⵽����"
ע�ͣ����򵥷���1����ʵ�֡�
*/
static int32_t KaDa3(FIFO_P* p)
{
	/*��FIFO����Max�㣨QQ��ζ�����󣩡�
	*/
	NmPOINT max={.Nm= 0};
	uint32_t tmp= p->idx;		//C ����һ�¡�
	uint32_t maxIdx= p->idx;	//C ����һ�¡�
	max.Nm= p->p[p->idx].Nm;
	
	for(uint32_t i=1; i< 512; i++){
		p->idx++;
		p->idx&= 0x1FF;	
		
		if(max.Nm< p->p[p->idx].Nm){
			max.Nm= p->p[p->idx].Nm;
			max.Ts= p->p[p->idx].Ts;
			maxIdx= p->idx;
		}			
	}
	

	/*����Max��-->F�㣬�õ�����߶Ρ�
	��maxIdx����ǰ��idx����������
	��һ���򻯣�����2���������ҹ�����ֵ> Z.
	*/
	p->idx= tmp;	//C �ָ�ԭֵ��
	
	
	
	/*�����߶Ρ��Ƿ�������������ֵ>Z����·��
	����У�����Ϊ��⵽���ա��㡣
	*/
	
	return 0;
}


/*��512�������������߶Ρ�
���������512��FIFO��
�����������
����ֵ���߶εĸ�����
ע�ͣ�!!��ʵ������ʱ�䡣
ע��ƽ���仯�Ĺյ㣬�����жϡ�
*/
static int32_t KaDa2(FIFO_P* p)
{
	uint32_t flagDing= 0;	//C ǰ1��-��ǰ��-��1���жϳ����㡣
	uint32_t i,j;
	
	NmPOINT pointPre2,pointPre,pointNxt,pointNxt2;	//C ǰ���㣬ǰһ�㣬����ǰ�㣩����һ�㣬����㡣
	
	kdLine.idx= 0;
	
	
	/*��1����Ϊ�߶ε���㡣
	*/
	kdLine.line[kdLine.idx].s= p->p[p->idx];	//C ǰ2�㣨����һ��������ƣ���
	
	p->idx++;
	p->idx&= 0x01FF;			//C ǰ1�㣨����һ��������ƣ���
	p->idx++;
	p->idx&= 0x01FF;			//C ��ǰ�㡣
	
	/*û����ͷ��ֻ���м�(512-4)����ıȽϡ�
	����Ϊǰ���2�㣬���жϡ�
	*/
	for(i= 0; i< 508; i++){	
		/*ȡ��ǰ�����2�㡣
		*/
		j= p->idx;	//C ��ǰ��
		
		j--;
		j&= 0x01FF;
		pointPre= p->p[j];			//C ǰ1�㣨����һ��������ƣ���
		
		j--;
		j&= 0x01FF;
		pointPre2= p->p[j];			//C ǰ2�㣨����һ��������ƣ���
		
		j= p->idx;	//C ��ǰ��
		
		j++;
		j&= 0x01FF;
		pointNxt= p->p[j];			//C ��1�㣨����һ��������ƣ���
		
		j++;
		j&= 0x01FF;
		pointNxt2= p->p[j];			//C ��2�㣨����һ��������ƣ���
		

		/*���ԵĶ��㣨����֪���йյ㣩����ǰ�������㶼С�򶼴�
		˼·���ȱȽϵ�ǰ���ǰ1�㡢��1��ı仯�������ǰ����һ�����㣬���жϵ�����Ƿ�ͬʱ����deltaֵ��
		���delta�����㣬��Ҫ�ٽ�һ���Ƚϵ�ǰ���ǰ2�㡢��2��ı仯�������ǰ����һ�����㣬���жϵ�����Ƿ�������1������2*deltaֵ��
		��ǰ�㡢ǰ1��ͺ�1��ĵ���ͬʱ����deltaʱ����ǰ���Ƕ��㡣
		��ǰ�㡢ǰ2��ͺ�2��ĵ�������һ������2*deltaʱ����ǰ���Ƕ��㡣
		*/
		flagDing= 0;
		
		if( ((p->p[p->idx].Nm > pointPre.Nm) && (p->p[p->idx].Nm > pointNxt.Nm)) ||
			((p->p[p->idx].Nm < pointPre.Nm) && (p->p[p->idx].Nm < pointNxt.Nm)) ){
			
			if( (abs(p->p[p->idx].Nm - pointPre.Nm) > 1000) &&
				(abs(p->p[p->idx].Nm - pointNxt.Nm) > 1000) ){
				/*��ǰ��Ϊ�յ㣬����Ϊe��
				*/
				kdLine.line[kdLine.idx].e= p->p[p->idx];		//C ��ǰ�߶ε��յ㡣
				flagDing= 1;
				uartS("\r\nnew end\r\n");
					
				kdLine.idx++;
				if(kdLine.idx> kdLine.cnt){
					uartS("err: cnt>20");
					return -3;
				}
				else{
					kdLine.line[kdLine.idx].s= p->p[p->idx];	//C ���߶ε���㡣
				}
			}
		}
			
		
		/*���ڻ����仯�ģ���Ҫ����չ2�㣺��ǰ���ǰ2�㣬��2��Ƚϡ�
		��ǰ���ǰ2�㡢��2��ı仯���෴�ġ��Һ�һ����Ĳ����2��delta�����������Ĳ�඼����2��delta��
		*/
		if(0 == flagDing){
			if( ((p->p[p->idx].Nm > pointPre2.Nm) && (p->p[p->idx].Nm > pointNxt2.Nm)) ||
				((p->p[p->idx].Nm < pointPre2.Nm) && (p->p[p->idx].Nm < pointNxt2.Nm)) ){
			
				if( (abs(p->p[p->idx].Nm - pointPre2.Nm) > 2*1000) ||
					(abs(p->p[p->idx].Nm - pointNxt2.Nm) > 2*1000) ){
					/*��ǰ��Ϊ�յ㣬����Ϊe��
					*/
					kdLine.line[kdLine.idx].e= p->p[p->idx];		//C ��ǰ�߶ε��յ㡣
					uartS("\r\nnew end2\r\n");
						
					kdLine.idx++;
					if(kdLine.idx> kdLine.cnt){
						uartS("err: cnt>20");
						return -3;
					}
					else{
						kdLine.line[kdLine.idx].s= p->p[p->idx];	//C ���߶ε���㡣
					}
				}
			}
		}
		
		
		/*��һ��ѭ����
		*/
		p->idx++;
		p->idx&= 0x01FF;
	}
	
	/*���1����Ϊ�߶ε��յ㡣
	*/
	p->idx++;
	p->idx&= 0x01FF;
	
	kdLine.line[kdLine.idx].e= p->p[p->idx];	//C �߶ε��յ㡣
	
	return kdLine.idx;	//C �߶θ�����
}	


/*��512��������kada��
���������512��FIFO��
�����������
����ֵ��0--�ҵ�������--δ�ҵ���
ע�ͣ�!!��ʵ������ʱ�䡣
���˵�ķ������ж��Ƿ�Ϊ�½���<--������<--�½���<--�����ء�
*/
static int32_t KaDa(FIFO_P* p)
{
	
	int32_t cur;
	uint8_t bitMask= 0;
	uint32_t i= 0;
	
	
	p->idx--;
	p->idx&= 0x01FF;
	
	cur= p->p[p->idx].Nm;
	
	
	/*������512�����е��½��أ�
	*/
	for(i= 0; i< 512; i++){
		
		p->idx--;
		p->idx&= 0x01FF;	
		
		bitMask<<= 1;
		
		if(cur< p->p[p->idx].Nm){
			/*��ǰ���ǰһ����С���½���
			*/
			bitMask|= 0x01;
		}
		
		cur= p->p[p->idx].Nm;
		
		
		/*�������__���У�������5�����㡣
		*/
		if(5 <= Cnt1(bitMask)){
			/*�ҵ��½��ء�
			*/
			uartHex16(i);
			uartS("1st falling\r\n");
			
			break;
		}
	}
	
	
	/*�������������ء�
	*/
	bitMask= 0;
	
	for(; i< 512; i++){
		p->idx--;
		p->idx&= 0x01FF;	
		
		bitMask<<= 1;
		
		if(cur> p->p[p->idx].Nm){
			/*��ǰ���ǰһ�����������
			*/
			bitMask|= 0x01;
		}
		
		cur= p->p[p->idx].Nm;
		
		
		/*�������__���У�������5�����㡣
		*/
		if(5 <= Cnt1(bitMask)){
			/*�ҵ������ء�
			*/
			uartHex16(i);
			uartS("2nd raising\r\n");
			break;
		}	
	}
	
		
	/*���������½��ء�
	*/
	bitMask= 0;
	
	for(; i< 512; i++){
		p->idx--;
		p->idx&= 0x01FF;	
		
		bitMask<<= 1;
		
		if(cur< p->p[p->idx].Nm){
			/*��ǰ���ǰһ�����С��������
			*/
			bitMask|= 0x01;
		}
		
		cur= p->p[p->idx].Nm;
		
		
		/*�������__���У�������5�����㡣
		*/
		if(5 <= Cnt1(bitMask)){
			/*�ҵ������ء�
			*/
			uartHex16(i);
			uartS("3rd falling\r\n");
			break;
		}	
	}
	
	
	/*�������������ء�
	*/
	bitMask= 0;
	
	for(; i< 512; i++){
		p->idx--;
		p->idx&= 0x01FF;	
		
		bitMask<<= 1;
		
		if(cur> p->p[p->idx].Nm){
			/*��ǰ���ǰһ����ȴ�������
			*/
			bitMask|= 0x01;
		}
		
		cur= p->p[p->idx].Nm;
		
		
		/*�������__���У�������5�����㡣
		*/
		if(5 <= Cnt1(bitMask)){
			/*�ҵ������ء�
			*/
			uartHex16(i);
			uartS("4th raising\r\n");
			break;
		}	
	}

	
	if(512 != i)
		return 0;	//C ���ҵ�
	else
		return -1;	//C δ�ҵ�
}


/*��ת��boot����תǰ�����ñ�־��boot��YModem���жϸñ�־��ִ��YModem()����������ת��Ӧ�ã�boot2����
*/
static void go_boot(uint32_t flag)
{
	//ʹ��BKPxR�ķ��ʡ�
	RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
	PWR->CR1 |= PWR_CR1_DBP;
	
	/*BKP0R����Ϊboot��ʶ��BKP1R����Ϊ�ļ���С��
	*/
	RTC->BKP0R= flag;
	RTC->BKP1R= 0;
	RTC->BKP2R= 0;
	
	vect_rst();		//C ֻ��λ�ˣ�����λ�������裨��GPIO����
}



/*�����������������塣
*/
typedef  void (*pFunction)(void);
pFunction Jump_To_Application;


/*��ת��APP n��
�ȼ��Ӧ�ó����ջ����ַ�Ƿ���[2000_0000, 2003_FFFF]��
QQ����APP֮ǰ��ӦlockFlash����δʵ�֣���
!!����ȷ��Ӧ�ó���ʹ�õĶ�ջ
*/
static int32_t go_app(uint32_t n)
{
	uint32_t app;
	
	app= *(__IO uint32_t*) (n);	//Ӧ�ó����ջ��λ�á�(����ĵ�1��uint32_t����
	
	if((app & 0xFFFC0000) == 0x20000000){
		
		//__enable_irq();
		//__disable_irq();
		NVIC_DisableIRQ(USART2_IRQn);
		//NVIC_SystemReset();
		SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
		SysTick->CTRL &= ~SysTick_CTRL_CLKSOURCE_Msk;
		
		//ʹ��HSI��ΪSYSCLK��
		RCC->CFGR |= (1UL<< 0);				//C HSI as SYSCLK.
		while( (0x1UL<< 2) != (RCC->CFGR & (0x1UL<< 2)) ){;}	//C ��ȡSWS��

		rst_per();
			
		__set_CONTROL(0); //!!�ǳ���Ҫ��
		
		//C jump to app address and start.
		app = *(__IO uint32_t*) (n + 4);
		/* Jump to user application */
		Jump_To_Application = (pFunction)app;
		/* Initialize user application's Stack Pointer */
		__set_MSP(*(__IO uint32_t*) n);
	
		Jump_To_Application();	
	
		return 0;//C QQ
	}
	else{
		return -1;
	}
}


/*HardFault��������
�ݶ��壬����HardFault�͹ص硣
*/
void HardFault_Handler(void)
{
	LOCK_PS_OFF;
}

/**** END OF FILE ****/

