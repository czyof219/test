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

//extern void initUart1(uint32_t);		//C 配置与角度模块相连的串口。
extern void initUart3(uint32_t);	//C 配置与WIFI模块相连的串口。!!128000bps.
extern void initWiFiHal(void);


extern DBGUART dbgUart;
extern uint32_t ads1220_ok;

//版本号。
uint8_t verboot2[]={"\r\nboot_kada V1.00\r\n"};
uint32_t flagNoJump;	//收到多个'`'，停在boot2，不跳转。
uint32_t flagRst;		//复位标志
uint32_t DebugOutEn;

int32_t avg1[9];	//C avg1[0]是平均值。 avg1[1]-avg1[8]是按时间前后的4个连续点。
int32_t avg2[9];	//C avg2[0]是平均值。 avg2[1]-avg2[4]是按时间前后的4个连续点。

	
/*扭矩点。
*/
typedef struct{
	int32_t Nm;		//C 扭矩值（从ADS1220读取的是有符号数）。
	uint32_t Ts;	//C 时间戳
}NmPOINT;

typedef struct{
	NmPOINT p[512];	//C 512个点。
	uint32_t idx;	//C 当前位置。p[idx]为待写入的位置。
	NmPOINT max;	//C 512个点中的最大点。	
	NmPOINT min;	//C 512个点中的最小点。
}FIFO_P;


typedef struct{
	NmPOINT point;
	uint32_t cnt;		//C 当前值
	uint32_t CNT;		//C 设置值，可认为一旦设定，即为常量。
	int32_t NM;			//C 符合值（如目标值的80%或20%）。
	uint32_t IsFound;	//C 是否找到。0---未找到，其他---已找到。 
}POINT;


typedef struct _AgDel{
	uint16_t pre;
	uint16_t cur;
	uint16_t del;
}AG_NOW;

/*单调的线段。
*/
typedef struct{
	NmPOINT s;			//C 起点
	NmPOINT e;			//C 终点
	int32_t delta;		//C 终点-起点的Nm值。也可通过s，e直接计算得到。
	int32_t interval;	//C 终点-起点的时间戳。也可通过s，e直接计算得到。
}SLINE;		

typedef struct{
	SLINE line[20];
	uint32_t idx;	//C 遍历时用，以免使用局部变量，显得程序乱。
	uint32_t cnt;	//C 实际线段的数量。不能超过20。
}LINE;

LINE kdLine={.idx=0, .cnt=20};

/*曲线中的A点、B点。
*/
POINT pA={.cnt=0, .CNT= 3, .NM= 600000, .IsFound=0};	//C Rising point.
POINT pB={.cnt=0, .CNT= 3, .NM= 120000, .IsFound=0};	//C Falling point.
AG_NOW ag_now= {.pre= 0, .cur= 0, .del= 0};

FIFO_P p512={.max.Nm= 0, .min.Nm= 0};			//C 512个点。

uint32_t swNmOut;		//C 是否输出Nm值。

MSG_DISP msgDispNew;

/*开关用宏定义
*/
//#define UART_NM				//C 注释该行，则不从串口输出Nm协议报文。
//#define UART_AG				//C 注释该行，则不从串口输出Ag协议报文。


osMessageQueueId_t midUp;
osMessageQueueId_t midDisp;
osMessageQueueId_t midOpRecv;	//C 保存已接收OP报文的消息队列。

/*内部函数声明。
*/
static int32_t KaDa(FIFO_P* p);
static int32_t KaDa2(FIFO_P* p);
static void go_boot(uint32_t flag);
static int32_t go_app(uint32_t n);		//跳转到app（多选1）

/*设置SYSCLK为30MHz（接近STM32L151RE的32MHz）。
没有找到频率变大不能超过10倍的描述。
*/
void SystemClock_Config(void)
{
	/*使用PLL作为SYSCLK前，!!配置PWR和FLASH。否则，容易进HardFaul或莫名其妙的运行地址。
	Set Voltage scale1 as MCU will run at 80MHz.
	*/
	//C 上电默认为Range1。所以不需要配置。
	
	
	/*使能4WS。
	*/
	// RCC->AHB1ENR |= RCC_AHB1ENR_FLASHEN; 上电默认为1.
	FLASH->ACR |= FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_4WS;	//C 4 latency. Instruction Prefetch, Data Prefech上电默认使能。
	
	
	/*配置PLL(72MHz，以方便串口QQ)
	使能HSE（BPY禁止），并等待稳定。
	配置PLL的3个参数。!!PLLVCO范围是[64,344]MHz。PLLVCO= HSE*PLLN/PLLM/PLLR。SYSCLK= PLLVCO/PLLDIV.(/2,/3,/4)。
	选择HSE为PLL src
	使能PLL，并等待稳定。
	*/
	/*使能HSE，等待稳定
	*/
	RCC->CR |= (1UL<< 16);
	while( 0 == (RCC->CR &(1UL<< 17)) ){;}
	
	/*设置PLL(16MHz*18/2/2-->72MHz),
	HSE as PLL src.
	PLL的倍数，分频。
	*/
	RCC->PLLCFGR |= (3UL<< 0);
	
	RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLN_Msk);
	RCC->PLLCFGR |= 18UL<< RCC_PLLCFGR_PLLN_Pos;	//C PLLN= 18.用于HSE为16MHz。
	//RCC->PLLCFGR |= 24UL<< RCC_PLLCFGR_PLLN_Pos;	//C PLLN= 24.用于HSE为12MHz。	
		
	RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM_Msk);
	RCC->PLLCFGR |= 1UL<< RCC_PLLCFGR_PLLM_Pos;		//C 001: PLLM= 2	

	RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLR);
	RCC->PLLCFGR |= 0UL<< RCC_PLLCFGR_PLLR_Pos;		//C 00: PLLR= 2
		
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLREN;
	
	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLPEN;
	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLQEN;	
		
		
	/*使能PLL，等待稳定。
	*/
	RCC->CR |= (1UL<< 24);
	while( 0 == (RCC->CR &(1UL<< 25)) ){;}		
	
	
	/*PLL作为SYSCLK，并设置HCLK，PCLK1，PCLK2。
	当然是先设置HCLK、PCLK1和PCLK2的分频，在将PLL作为SYSCLK。
	HCLK：no divided，即DIV1.
	PCLK1：DIV1。
	PCLK2：DIV1.
	*/
	RCC->CFGR &= RCC_CFGR_HPRE_Msk;		//C HCLK not divided.
	RCC->CFGR &= RCC_CFGR_PPRE1_Msk;	//C PCLK1 not divided
	RCC->CFGR &= RCC_CFGR_PPRE2_Msk;	//C PCLK2 not divided.
	
	/*将PLL作为SYSCLK，等待完成。
	!!由MSI(00)变为PLL，可以不清零直接赋值。
	*/
	RCC->CFGR |= (3UL<< 0);				//C PLL as SYSCLK.
	while( (0x3UL<< 2) != (RCC->CFGR & (0x3UL<< 2)) ){;}	//C 读取SWS。
	
	
	/*使能MCO/2输出。
	从HSE分出MCO的好处，MCO不受主频影响。
	*/
	RCC->CFGR &= ~(0xFUL<< 24);		//C MCOSEL[3:0].
	//RCC->CFGR |= (1UL<< 24);		//C SYSCLK（因为使用了PLL，就是MAINPLL）
	RCC->CFGR |= (4UL<< 24);		//C 选择MCO的源为HSE。
		
	RCC->CFGR &= ~(7UL<< 28);		//C MCOPRE[2:0].
	//RCC->CFGR |= (4UL<< 28);		//C /16。即72/16= 4.5MHz，也就是ADS1220最大输入时钟。
	RCC->CFGR |= (2UL<< 28);		//C /4 即16/4= 4MHz。	
	
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
uint32_t flag_app_to_boot2;	//C 标志，由app跳转到boot2。


void app_main (void *argument) {
 
	osStatus_t status = osOK;
	uint8_t LedSta= 0;
	uint32_t update_ag;		//C 更新Ag的时刻。
	uint32_t update_nm1;	//C 更新Nm1的时刻。
	uint32_t update_nm2;	//C 更新Nm1的时刻。
	
	uint8_t buf[8];
	uint8_t snd[8];	
	update_ag= 0;
	flagRst= 0xCCCC;
	flag_app_to_boot2= 0;
	
	
	/*默认调试输出信息选项。
	*/
	DebugOutEn |= OUT_FLASH;
	DebugOutEn |= OUT_PSRAM;
	
	
	osDelay(100);
	
	
	/*判断从应用中收到“进入BOOT命令”后重启的。
	QQ不使能PWR时钟，能否读取RTC->BKP0R。
	*/
	if(0x88886666 == RTC->BKP0R){	//
		//使能BKPxR的访问。
		RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
		PWR->CR1 |= PWR_CR1_DBP;
		
		/*清零BKP0R（boot标识）。
		*/
		RTC->BKP0R= 0;
		
		flag_app_to_boot2= 1;
		
		uartS("rst from app, ready for update\r\n");
	}
	else{
		uartS("boot->boot2\r\n");
		/*延时2s，以等待'`'的输入。
		*/
		for(int i= 0; i< 4; i++){
			uartS(".");
			osDelay(500);
		}
		
		if(2< flagNoJump){	//至少3个'`'。
			uartS("no jump\r\n");
		}
		else{	//跳转到应用程序。!!但跳转前需要判断APP是否有效。
			uint32_t* p= (uint32_t*)0x08080000;
			
			if(( *p & 0xFFFC0000) == 0x20000000){
				uartS("set app flag, boot2->boot\r\n");
				osDelay(10);
			
				//使能BKPxR的访问。
				RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
				PWR->CR1 |= PWR_CR1_DBP;
				
				/*设置BKP0R（boot标识）。
				*/
				RTC->BKP0R= 0x88888888;
				
				__disable_irq();
				//__set_PSP(*p);
				__set_CONTROL(0);	//实测此语句可注释掉。
				
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
	RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;	//C 不使能CRC，进入GUI_Init()会死循环。
	
	GUI_Init();
	
	
	/*创建线程。
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
	//Init_ThreadVbat();		//C VBAT电压检测线程。
	
	//Init_ThreadMx25_PSRAM();	//C osPriorityLow7
	//Init_ThreadRtc();
//	Init_ThreadBatChg();
//	Init_ThreadScaner();
	
	
//	/*配置ADS1220，2片（扭矩传感器）。
//	*/
//	osDelay(1);
//	if(0 == initADS1220()){
//		ads1220_ok= 1;
//		
//		uartS("initADS1220 ok & START\r\n");
//		
//		CMD_START1();	//C 第1片ADS1220开启转换。
//		CMD_START2();	//C 第2片ADS1220开启转换。
//	}

	/*配置ADS1220，2片（扭矩传感器）。
	*/
	osDelay(1);
	if(0 == initADS1220()){
		ads1220_ok= 1;
		
		uartS("initADS1220 ok & START\r\n");
		
		CMD_START1();	//C 第1片ADS1220开启转换。
		CMD_START2();	//C 第2片ADS1220开启转换。
	}
	

	while(1){
		if(0xCCCC != flagRst){	//命令：复位
			if(0 == flagRst){
				vect_rst();	//马上复位。
			}
			else if(flagRst< 20){
				uartS("ready to rst\r\n");
				osDelay(100);
				
				//禁止调试串口
				USART2->CR1 &= ~USART_CR1_UE;
				
				osDelay(flagRst*1000);	//延时n秒后复位。
				
				go_boot(0x88881234);
				
				while(1);
			}
		}
		
		
		status = osMessageQueueGet(midUp, buf, NULL, 50);	//C 几乎每ms都有数据帧（因为角度是帧/4ms，Nm是帧/3ms）。

		if(osOK != status)
			continue;
		
			
		/*判断是角度数据吗？
		是，则加0xFF 0x01 0x0 RawL RawH后发送出去。
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
					/*CW、CCW的差值都大于允许值。
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
				/*每个200ms更新一次。
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
			
			/*时间戳。
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
			
			/*符号扩展。
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
				
			
			/*符号扩展。
			将24bit有符号数变为32bit有符号数。
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
			
			
			/*计算四次平均值。
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
			
			
			/*是第1片ADS1220的Nm数据，加0xFF 0x02 MSB nn LSB后发送出去。
			*/
			uint32_t tmp= osKernelGetTickCount();
			
			if(100 <(tmp- update_nm1)){
				/*每个200ms更新一次。
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

			
			/*符号扩展。
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
			
			
			/*是第2片ADS1220的Nm数据，加0xFF 0x02 MSB nn LSB后发送出去。
			*/
			uint32_t tmp= osKernelGetTickCount();
			
			if(100 <(tmp- update_nm2)){
				/*每个200ms更新一次。
				*/
				update_nm2= tmp;
				
				msgDispNew.src= MSGDISP_FROM_NM2;
				uint32_t tmp;
				
				/*符号扩展。
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
	/*配置调试信息输出：
	*/
	DebugOutEn |= OUT_ADC;
	
	uint32_t cntOkKeyDn= 0;
	
	/*初始化2个按键。
	初始化喇叭使能引脚和DAC1。
	*/
	initKey();
	//initBuzzHal();
	//initTim5();
	initDAC1();
	//initRTC();
	
	
	/*初始化调试串口。
	*/	
	dbgUart.flagTxing= 0;
	dbgUart.hwFIFOcnt= 1;
	dbgUart.initUart= initUart;
	dbgUart.wrTHR= wrTHR;
	dbgUart.intTHR= intTHR;
	dbgUart.UARTn= USART2;
	dbgUart.initUart(0);			//C 参数为0，无实际用处。
	
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



/*计算x中的1个个数。
输入参数：x
输出参数：无
返回值：1个个数，范围[0,8]。
说明：依据情况，计算低位段中1个个数。可以是bit[7:0]，bit[6:0]...或bit[1:0]。
*/
uint32_t Cnt1(uint8_t x)
{
	uint8_t mask= 0xFF;	//C 待计算的范围。如bit[7:0]-->0xFF. bit[1:0]-->0x03
	uint8_t cnt;
	
	x&= mask;
	
	cnt= 0;
	
	for(uint8_t i= 0; i< 8; i++){
		if( x&(1<< i) )
			cnt++;
	}
	
	return cnt;
}

/*在512点中搜索各个线段。
输入参数：512点FIFO。
输出参数：无
返回值：0--检测到“哒"
注释：“简单方案1”的实现。
*/
static int32_t KaDa3(FIFO_P* p)
{
	/*在FIFO中找Max点（QQ如何定义最大）。
	*/
	NmPOINT max={.Nm= 0};
	uint32_t tmp= p->idx;		//C 保存一下。
	uint32_t maxIdx= p->idx;	//C 保存一下。
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
	

	/*遍历Max点-->F点，得到多个线段。
	从maxIdx到当前的idx（不含）。
	进一步简化：连续2次上升，且共上升值> Z.
	*/
	p->idx= tmp;	//C 恢复原值。
	
	
	
	/*遍历线段。是否有上升且上升值>Z的线路。
	如果有，则认为检测到“哒”点。
	*/
	
	return 0;
}


/*在512点中搜索各个线段。
输入参数：512点FIFO。
输出参数：无
返回值：线段的个数。
注释：!!须实测运行时间。
注意平滑变化的拐点，不易判断。
*/
static int32_t KaDa2(FIFO_P* p)
{
	uint32_t flagDing= 0;	//C 前1点-当前点-后1点判断出顶点。
	uint32_t i,j;
	
	NmPOINT pointPre2,pointPre,pointNxt,pointNxt2;	//C 前二点，前一点，（当前点），后一点，后二点。
	
	kdLine.idx= 0;
	
	
	/*第1个点为线段的起点。
	*/
	kdLine.line[kdLine.idx].s= p->p[p->idx];	//C 前2点（这是一个点的名称）。
	
	p->idx++;
	p->idx&= 0x01FF;			//C 前1点（这是一个点的名称）。
	p->idx++;
	p->idx&= 0x01FF;			//C 当前点。
	
	/*没有两头，只有中间(512-4)个点的比较。
	暂认为前后各2点，不判断。
	*/
	for(i= 0; i< 508; i++){	
		/*取出前、后各2点。
		*/
		j= p->idx;	//C 当前点
		
		j--;
		j&= 0x01FF;
		pointPre= p->p[j];			//C 前1点（这是一个点的名称）。
		
		j--;
		j&= 0x01FF;
		pointPre2= p->p[j];			//C 前2点（这是一个点的名称）。
		
		j= p->idx;	//C 当前点
		
		j++;
		j&= 0x01FF;
		pointNxt= p->p[j];			//C 后1点（这是一个点的名称）。
		
		j++;
		j&= 0x01FF;
		pointNxt2= p->p[j];			//C 后2点（这是一个点的名称）。
		

		/*明显的顶点（看书知不叫拐点）：比前、后两点都小或都大。
		思路：先比较当前点和前1点、后1点的变化。如果当前点是一个顶点，则判断点间差距是否同时满足delta值。
		如果delta不满足，需要再进一步比较当前点和前2点、后2点的变化。如果当前点是一个顶点，则判断点间差距是否至少有1个满足2*delta值。
		当前点、前1点和后1点的点间距同时满足delta时，当前点是顶点。
		当前点、前2点和后2点的点间距至少一个满足2*delta时，当前点是顶点。
		*/
		flagDing= 0;
		
		if( ((p->p[p->idx].Nm > pointPre.Nm) && (p->p[p->idx].Nm > pointNxt.Nm)) ||
			((p->p[p->idx].Nm < pointPre.Nm) && (p->p[p->idx].Nm < pointNxt.Nm)) ){
			
			if( (abs(p->p[p->idx].Nm - pointPre.Nm) > 1000) &&
				(abs(p->p[p->idx].Nm - pointNxt.Nm) > 1000) ){
				/*当前点为拐点，保存为e。
				*/
				kdLine.line[kdLine.idx].e= p->p[p->idx];		//C 当前线段的终点。
				flagDing= 1;
				uartS("\r\nnew end\r\n");
					
				kdLine.idx++;
				if(kdLine.idx> kdLine.cnt){
					uartS("err: cnt>20");
					return -3;
				}
				else{
					kdLine.line[kdLine.idx].s= p->p[p->idx];	//C 新线段的起点。
				}
			}
		}
			
		
		/*对于缓慢变化的，需要再扩展2点：当前点和前2点，后2点比较。
		当前点对前2点、后2点的变化是相反的。且和一个点的差距是2倍delta，比须和两点的差距都满足2倍delta。
		*/
		if(0 == flagDing){
			if( ((p->p[p->idx].Nm > pointPre2.Nm) && (p->p[p->idx].Nm > pointNxt2.Nm)) ||
				((p->p[p->idx].Nm < pointPre2.Nm) && (p->p[p->idx].Nm < pointNxt2.Nm)) ){
			
				if( (abs(p->p[p->idx].Nm - pointPre2.Nm) > 2*1000) ||
					(abs(p->p[p->idx].Nm - pointNxt2.Nm) > 2*1000) ){
					/*当前点为拐点，保存为e。
					*/
					kdLine.line[kdLine.idx].e= p->p[p->idx];		//C 当前线段的终点。
					uartS("\r\nnew end2\r\n");
						
					kdLine.idx++;
					if(kdLine.idx> kdLine.cnt){
						uartS("err: cnt>20");
						return -3;
					}
					else{
						kdLine.line[kdLine.idx].s= p->p[p->idx];	//C 新线段的起点。
					}
				}
			}
		}
		
		
		/*下一个循环。
		*/
		p->idx++;
		p->idx&= 0x01FF;
	}
	
	/*最后1个点为线段的终点。
	*/
	p->idx++;
	p->idx&= 0x01FF;
	
	kdLine.line[kdLine.idx].e= p->p[p->idx];	//C 线段的终点。
	
	return kdLine.idx;	//C 线段个数。
}	


/*在512点中搜索kada。
输入参数：512点FIFO。
输出参数：无
返回值：0--找到，其他--未找到。
注释：!!须实测运行时间。
倒退点的方法，判断是否为下降沿<--上升沿<--下降沿<--上升沿。
*/
static int32_t KaDa(FIFO_P* p)
{
	
	int32_t cur;
	uint8_t bitMask= 0;
	uint32_t i= 0;
	
	
	p->idx--;
	p->idx&= 0x01FF;
	
	cur= p->p[p->idx].Nm;
	
	
	/*最多遍历512个点中的下降沿：
	*/
	for(i= 0; i< 512; i++){
		
		p->idx--;
		p->idx&= 0x01FF;	
		
		bitMask<<= 1;
		
		if(cur< p->p[p->idx].Nm){
			/*当前点比前一个点小，下降。
			*/
			bitMask|= 0x01;
		}
		
		cur= p->p[p->idx].Nm;
		
		
		/*连续监测__次中，有至少5次满足。
		*/
		if(5 <= Cnt1(bitMask)){
			/*找到下降沿。
			*/
			uartHex16(i);
			uartS("1st falling\r\n");
			
			break;
		}
	}
	
	
	/*继续查找上升沿。
	*/
	bitMask= 0;
	
	for(; i< 512; i++){
		p->idx--;
		p->idx&= 0x01FF;	
		
		bitMask<<= 1;
		
		if(cur> p->p[p->idx].Nm){
			/*当前点比前一个点大，上升。
			*/
			bitMask|= 0x01;
		}
		
		cur= p->p[p->idx].Nm;
		
		
		/*连续监测__次中，有至少5次满足。
		*/
		if(5 <= Cnt1(bitMask)){
			/*找到上升沿。
			*/
			uartHex16(i);
			uartS("2nd raising\r\n");
			break;
		}	
	}
	
		
	/*继续查找下降沿。
	*/
	bitMask= 0;
	
	for(; i< 512; i++){
		p->idx--;
		p->idx&= 0x01FF;	
		
		bitMask<<= 1;
		
		if(cur< p->p[p->idx].Nm){
			/*当前点比前一个点比小，上升。
			*/
			bitMask|= 0x01;
		}
		
		cur= p->p[p->idx].Nm;
		
		
		/*连续监测__次中，有至少5次满足。
		*/
		if(5 <= Cnt1(bitMask)){
			/*找到上升沿。
			*/
			uartHex16(i);
			uartS("3rd falling\r\n");
			break;
		}	
	}
	
	
	/*继续查找上升沿。
	*/
	bitMask= 0;
	
	for(; i< 512; i++){
		p->idx--;
		p->idx&= 0x01FF;	
		
		bitMask<<= 1;
		
		if(cur> p->p[p->idx].Nm){
			/*当前点比前一个点比大，上升。
			*/
			bitMask|= 0x01;
		}
		
		cur= p->p[p->idx].Nm;
		
		
		/*连续监测__次中，有至少5次满足。
		*/
		if(5 <= Cnt1(bitMask)){
			/*找到上升沿。
			*/
			uartHex16(i);
			uartS("4th raising\r\n");
			break;
		}	
	}

	
	if(512 != i)
		return 0;	//C 已找到
	else
		return -1;	//C 未找到
}


/*跳转到boot。跳转前须设置标志。boot（YModem）判断该标志后，执行YModem()，而不是跳转到应用（boot2）。
*/
static void go_boot(uint32_t flag)
{
	//使能BKPxR的访问。
	RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
	PWR->CR1 |= PWR_CR1_DBP;
	
	/*BKP0R设置为boot标识，BKP1R设置为文件大小。
	*/
	RTC->BKP0R= flag;
	RTC->BKP1R= 0;
	RTC->BKP2R= 0;
	
	vect_rst();		//C 只复位核，不复位内置外设（如GPIO）。
}



/*函数类型声明、定义。
*/
typedef  void (*pFunction)(void);
pFunction Jump_To_Application;


/*跳转到APP n。
先检测应用程序的栈顶地址是否在[2000_0000, 2003_FFFF]。
QQ启动APP之前，应lockFlash（暂未实现）。
!!必须确保应用程序使用的堆栈
*/
static int32_t go_app(uint32_t n)
{
	uint32_t app;
	
	app= *(__IO uint32_t*) (n);	//应用程序的栈顶位置。(程序的第1个uint32_t）。
	
	if((app & 0xFFFC0000) == 0x20000000){
		
		//__enable_irq();
		//__disable_irq();
		NVIC_DisableIRQ(USART2_IRQn);
		//NVIC_SystemReset();
		SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
		SysTick->CTRL &= ~SysTick_CTRL_CLKSOURCE_Msk;
		
		//使用HSI作为SYSCLK。
		RCC->CFGR |= (1UL<< 0);				//C HSI as SYSCLK.
		while( (0x1UL<< 2) != (RCC->CFGR & (0x1UL<< 2)) ){;}	//C 读取SWS。

		rst_per();
			
		__set_CONTROL(0); //!!非常重要。
		
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


/*HardFault处理函数。
暂定义，进入HardFault就关电。
*/
void HardFault_Handler(void)
{
	LOCK_PS_OFF;
}

/**** END OF FILE ****/

