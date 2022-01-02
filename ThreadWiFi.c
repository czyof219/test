#include "cmsis_os2.h"                                        // CMSIS RTOS header file
 
#include "leds.h"
#include "psram.h"
#include "stm32l4xx.h"
#include "dbgUart.h"
#include "mymessage.h"
#include "op.h"
#include "bsp.h"
#include "shuaji.h"			//刷机操作相关的头文件。
#include <stdlib.h>

#define TMO_UPDATING		(1000*10)	//单位是ms。

//扳手向上位机返回的错误码。
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
	uint32_t cntKB;			//C 传输过程中的计数，单位为KiB.
	uint32_t fileaddr;		//C 程序的起始地址。
	uint32_t filesize;
	uint32_t filecrc;
	int32_t sn;				//流水号。
	uint32_t tsStart;		//升级开始时的时间戳。0表示未开启升级。
}PROG;


#define UART_WIFI		USART3

/*引脚低电平时，WiFi有效。
*/
#define WIFI_READY_STA	( (GPIOC->IDR & (1UL<< 3))? 0:1)	
#define WIFI_LINK_STA	( (GPIOC->IDR & (1UL<< 2))? 0:1)

/*开启/关闭WiFi模块。
低有效，即低电平时，开启WiFi模块。
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
	char outStr[64];		//C 调试信息输出用字符串缓冲。
	osStatus_t status = osOK;
	uint32_t cnt= 0;
	OPPAK Op_Recv;
	PROG prog={.sn=-1, .tsStart= 0};
	MSG_DISP msgDispWiFi;
	uint8_t buf[1024];
	uint8_t bufBk[1024];
	initWiFiHal();
	initUart3(128000);	//C 已固定为115200。
	
	initPSRAM();
	uartS("init PSRAM\r\n");
	
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
	
//	/*PSRAM的读写测试：
//	写1K，读1K，比较。工作512KiB。
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
	
	
	/*开启WiFi模块。
	*/
	WIFI_PWR_ON;
	uartS("WiFi ok\r\n");
	
	if(0 == flag_app_to_boot2){
		osDelay(2000);	//C 延时2s再访问WiFi模块。
	}
	else{
		uartS("recv boot\r\n");
				
		//输出文件名称：
		uartS(Op_Recv.data);
		
		//提取文件大小，CRC。
		int i= 0;
		for(i= 0; i< 100; i++){
			if('\0' == Op_Recv.data[i])
				break;
		}	
		prog.filesize= get_filesize(&Op_Recv.data[i+1]);
		prog.filecrc= get_filecrc(&Op_Recv.data[i+5]);
		prog.fileaddr= 0x08080000;		//QQ暂定值。
		
		uartS("size,crc=");
		uartHex32(prog.filesize);
		uartHex32(prog.filecrc);
		
		set_BOOT_echo_shit_pak(NULL, &Op_Recv);	
		
		send_pak(&Op_Recv);

		uartS("boot first echo\r\n");
		prog.sn= -1;
		
		
		/*更新界面*/
		msgDispWiFi.src= MSGDISP_FROM_WIFI;
		msgDispWiFi.data[0]= 'C';
		msgDispWiFi.data[1]= 0;
		msgDispWiFi.data[2]= 0;
		
		osMessageQueuePut(midDisp, &msgDispWiFi, 0, NULL);
	}
	
	
	
	
	while (1) {
	
		status = osMessageQueueGet(midOpRecv, &Op_Recv, NULL, 50);
		
		if(osOK == status){
			prog.tsStart= osKernelGetTickCount();	//!!暂定，收到WIFI串口的数据，就认为开启升级。
			
			/*填充Header结构体。
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
				
				/*收到MID0001，则应答MID0002。
				*/
				set_mid_0002_pak((uint8_t*)set_mid_0002_pak, &Op_Recv);
				
				send_pak(&Op_Recv);
				
				uartS("send 0002\r\n");
			}
			else if(MID_BOOT == op_mid){		//升级用私有协议：进入BOOT。
				
				uartS("recv boot\r\n");
				
				//输出文件名称：
				uartS(Op_Recv.data);
				
				//提取文件大小，CRC。
				int i= 0;
				for(i= 0; i< 100; i++){
					if('\0' == Op_Recv.data[i])	//!!winidows中，'*'不能出现在文件名中。
						break;
				}	
				prog.filesize= get_filesize(&Op_Recv.data[i+1]);
				prog.filecrc= get_filecrc(&Op_Recv.data[i+5]);
				prog.fileaddr= 0x08080000;		//QQ暂定值。
				
				uartS("size,crc=");
				uartHex32(prog.filesize);
				uartHex32(prog.filecrc);
				
				set_BOOT_echo_pak(NULL, &Op_Recv);	
				
				send_pak(&Op_Recv);
			
				uartS("boot echo\r\n");
				prog.sn= -1;
				
				
				/*更新界面*/
				msgDispWiFi.src= MSGDISP_FROM_WIFI;
				msgDispWiFi.data[0]= 'C';
				msgDispWiFi.data[1]= 0;
				msgDispWiFi.data[2]= 0;
				
				osMessageQueuePut(midDisp, &msgDispWiFi, 0, NULL);
			}
			else if(MID_PROG_DATA == op_mid){	//升级用私有协议：升级数据包
				uartS("\r\ndata sn=");
				uint32_t sn= ascii_to_4dig(Op_Recv.data);
				uartHex16(sn);
				
				if(sn == (uint32_t)(prog.sn+ 1)){
					//写入PSRAM中。
					PsramWrite(sn*1024, Op_Recv.data+4, 1024);	//.data后偏移4B为数据开始。
					
					set_DATA_echo_pak(sn, &Op_Recv);
					
					send_pak(&Op_Recv);
			
					uartS("DATA echo\r\n");
					
					prog.cntKB= sn;		//更新已接收到的数据大小（KB）。
					prog.sn+= 1;		//更新已接收到的流水号。!!是已接收到的，以方便流水号不连续时的应答。
					
					/*更新界面*/
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
			else if(MID_DATA_ALL == op_mid){	//全部升级完成包
				//int32_t sizeKB= (prog.filesize+1023)/1024;
				if((uint32_t)(prog.sn+1) == (prog.filesize+1023)/1024){
					uartS("\r\ndata all");
					
					if(1){//prog.filecrc == get_psram_crc(0,prog.filesize)){	//CRC正确
						set_DATA_ALL_echo_pak(ERR_NO, &Op_Recv);
						
						send_pak(&Op_Recv);
					
						uartS("all echo\r\n");
						
						
						//将PSARM的程序，刷机到FLASH中，并跳转到APP。
						//更新界面："flashing..."
						msgDispWiFi.src= MSGDISP_FROM_WIFI;
						msgDispWiFi.data[0]= 'I';
						msgDispWiFi.data[1]= 0;		//显示：flashing...
						msgDispWiFi.data[2]= 0;
						
						osMessageQueuePut(midDisp, &msgDispWiFi, 0, NULL);
						
						uartS("\r\nshuaji:");
						uartHex32(prog.fileaddr);
						uartHex32(prog.filesize);
						
						osDelay(100);	//延时，以更新显示。
						
						
						__disable_irq();					
						int ret= shuaji(prog.fileaddr,prog.filesize);
						__enable_irq();
						
						if(0 == ret){
							msgDispWiFi.src= MSGDISP_FROM_WIFI;
							msgDispWiFi.data[0]= 'I';
							msgDispWiFi.data[1]= 1;	//显示：flashing...OK
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
						else if(-1 == ret){	//CRC,暂未实现
							msgDispWiFi.src= MSGDISP_FROM_WIFI;
							msgDispWiFi.data[0]= 'I';
							msgDispWiFi.data[1]= 2;	//显示：flashing...CRC。即编程后对FLASH中数据校验CRC失败。
							msgDispWiFi.data[2]= 0;
						
							osMessageQueuePut(midDisp, &msgDispWiFi, 0, NULL);
							
							uartS("err:flash...crc\r\n");
						}
						else if(-2 == ret){	//编程中错误1。
							msgDispWiFi.src= MSGDISP_FROM_WIFI;
							msgDispWiFi.data[0]= 'I';
							msgDispWiFi.data[1]= 3;	//显示：flashing...ERR1。即编程过程中失败。
							msgDispWiFi.data[2]= 0;
						
							osMessageQueuePut(midDisp, &msgDispWiFi, 0, NULL);
							
							uartS("err:flash...err1\r\n");
						}
						else if(-3 == ret){	//编程中错误2。
							msgDispWiFi.src= MSGDISP_FROM_WIFI;
							msgDispWiFi.data[0]= 'I';
							msgDispWiFi.data[1]= 4;	//显示：flashing...ERR2。即编程过程中失败。
							msgDispWiFi.data[2]= 0;
						
							osMessageQueuePut(midDisp, &msgDispWiFi, 0, NULL);
							
							uartS("err:flash...err2\r\n");
						}
						
					}
					else{	//CRC错误
						set_DATA_ALL_echo_pak(ERR_CRC, &Op_Recv);
						
						send_pak(&Op_Recv);
					
					
						//更新界面：ERR:CRC
						msgDispWiFi.src= MSGDISP_FROM_WIFI;
						msgDispWiFi.data[0]= 'I';
						msgDispWiFi.data[1]= 5;	//显示：PSRAM...CRC
						msgDispWiFi.data[2]= 0;
						
						osMessageQueuePut(midDisp, &msgDispWiFi, 0, NULL);
						
						uartS("err:psram crc\r\n");
					}
				}
				else{
					set_DATA_ALL_echo_pak(ERR_SIZE, &Op_Recv);
					
					send_pak(&Op_Recv);
				
					uartS("err:size\r\n");
					
					//更新界面：ERR:SIZE
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
		else{	//超时 QQ待补充。
			
		}
		
		
		//WiFi线程运行标记。
		cnt++;
		if(0 == (cnt%4)){
			uartS("W");	
			
//			/*是否升级过程中升级。//去掉超时机制。
//			*/
//			if(prog.tsStart){
//				if(TMO_UPDATING< (osKernelGetTickCount()- prog.tsStart)){
//					uartS("tmo in updating(20s), now rst\r\n");
//					
//					osDelay(100);
//					
//			
//					//跳转到应用程序。!!但跳转前需要判断APP是否有效。
//					uint32_t* p= (uint32_t*)0x08080000;
//					
//					if(( *p & 0xFFFC0000) == 0x20000000){
//						uartS("set app flag, boot2->boot\r\n");
//						osDelay(10);
//					
//						//使能BKPxR的访问。
//						RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
//						PWR->CR1 |= PWR_CR1_DBP;
//						
//						/*清零BKP0R（boot标识）。
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


/*初始化MCU的Uart3。它与WiFi模块通信。
输入参数 baud：固定为128000bps。
发送采用轮询方式。接收采用中断方式。
*/
void initUart3(uint32_t baud)
{
	/*配置引脚。
	U3TX---PB10，MCU与WiFi模块的连接串口。
	U3RX---PB11，MCU与WiFi模块的连接串口。
	*/
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;	//C Enable GPIOB clock.

	GPIOB->MODER &= ~(3UL<< 2*10);		//C 清零。
	GPIOB->MODER |= (2UL<< 2*10);		//C PB10 as AF mode.
	GPIOB->OSPEEDR |= (2UL<< 2*10);		//C PB10 as high speed.
	GPIOB->PUPDR &= ~(3UL<< 2*10);		//C PB10 as no pu，no pd.

	GPIOB->MODER &= ~(3UL<< 2*11);		//C 清零。	
	GPIOB->MODER |= (2UL<< 2*11);		//C PB11 as AF mode.
	GPIOB->OSPEEDR |= (2UL<< 2*11);		//C PB11 as high speed.
	GPIOB->PUPDR |= (1UL<< 2*11);		//C PB11 as pu.
	
	/*复用功能*/
	GPIOB->AFR[1] |= (7UL<< 4*2);		//C PB10
	GPIOB->AFR[1] |= (7UL<< 4*3);		//C PB11.
	
	
	/*配置Uart3的格式：128000，8-n-1。
	*/
	RCC->APB1ENR1 |= RCC_APB1ENR1_USART3EN;	
	

	/* baudrate= fck/(8*(2-OVER8)*UARTDIV)。OVER8@CR1=0, 16 samples.
	*/
	//USART3->BRR= 0x271;						//C 115200@72MHz.
	USART3->BRR= 0x233;							//C 128000@72MHz.（或232）
		
	/*CR1，CR2默认时，为8-n-1。
	*/
	USART3->CR1 |= (1UL<< 3)|(1UL<< 2);			//C enable TX, RX.
	USART3->CR1 |= (1UL<< 0);					//C enbale USART.
	
	USART3->CR1 |= (1UL<< 5);					//C enable RXNEIE.
	
	/* preemption = 1, sub-priority = 1 */
	NVIC_SetPriority(USART3_IRQn, 1);
	
	/* Enable Interrupt for UART0 channel */
	NVIC_EnableIRQ(USART3_IRQn);				//C U2U3透传时，不开启中断。
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
		/*使能RXNE中断后，如果出现ORE，也会进入中断，必须处理。
		实测发现，在230400bps时，会出现ORE（程序很简单，只读取RDR都来不及吗？可能与其他2个外部中断USART1和EXT有关。）。
		*/
		USART3->ICR |= USART_ICR_ORECF;
		
		uartS("ORE");
	}
}


/*初始化MCU与WiFi模块（USR-C216）相连的引脚。
nLink----PC2，输入，表示是否建立连接（WiFi模块作为AP时，被其他设备连接上），低有效。
KWiFi----PC13，输出。通过控制MOS管的WiFi模块电源，低有效（WiFi模块有电）。
*/
void initWiFiHal(void)
{
	
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;	//C Enable GPIOB clock.

	
	GPIOC->MODER &=	~(3UL<< 2*2);		//C PC2是输入。
	GPIOC->PUPDR |= (1UL<< 2*2);		//C pull-up	

	GPIOC->MODER &= ~(3UL<< 2*13);
	GPIOC->MODER |= (1UL<< 2*13);		

	WIFI_PWR_ON;//GPIOC->BSRR |= (1UL<< 13);			//C PC13，output H，WiFi模块无电。
}


/*逐字节发送。
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


/*使用指定的Header填充MID0002。

*/
static void set_mid_0002_pak(uint8_t* phdr, OPPAK* ppak)
{
	uint8_t* p= (uint8_t*)ppak;
	
	for(int i= 0; i< 20; i++){
		p[i]= mid_0002_hdr[i];
	}
	
	for(int i= 20; i< 163; i++){
		p[i]= ' ';	//!!吴工说必须填充空格，不能是'0'。
	}
	
	ppak->len= 163;
	
	
	/*填入版本号
	*/
	p[83]= '0';
	p[84]= '6';
	
	if(0 == read_app_ver()){	//读取应用程序的版本。
		uint8_t* paddr= (uint8_t*)0x08080600;
		p[85]= *paddr;p[86]= *(paddr+1);
		p[87]= '.';
		p[88]= *(paddr+2);p[89]=*(paddr+3);
		
		uartS("app ver\r\n");
	}
	else{
		//QQ版本好如何填写:00.00
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


/*填充全发完成的应答的PAK。
参数1：执行结果
参数2：待填充PAK的指针。
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


/*计算PSRAM中某起始地址的CRC32。
输入参数    p：PSRAM的起始地址。
输入参数 size：长度。
返回值：计算得到的CRC32。
*/
static uint32_t get_psram_crc(uint8_t* p, uint32_t size)
{
	uint32_t addr;
	uint8_t buf[1024];
	uint32_t crc32= 0xFFFFFFFF;;
	
	
	if(1024< size){	//文件大小> 1024，分多个1024的包计算。
		int k= size/1024;
		int i= 0;
		for(i= 0; i< k; i++){
			PsramRead((uint32_t)p+1024*i, buf, 1024);
			crc32= crc32_no_XOROT(crc32, buf, 1024);
		}
		
		int j= size%1024;
		
		if(0 != j){	//最后不足1024的数据。
			PsramRead(1024*i, buf, 1024);
			crc32= crc32_no_XOROT(crc32, buf, j);
		}
		
		crc32= crc32 ^ 0xFFFFFFFF;
	}
	else{	//仅一次计算
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
	
	/*判断启动地址处是否有程序 */
	if (((*(uint32_t*)StartAddr) & 0xFFFC0000 ) == 0x20000000){ 	//堆栈栈顶指针位置，必须在IRAM1中。
		
		rst_per();	//复位除GPIO之外的内置外设。
		
		/* Jump to user application 
		*/
		JumpAddress = *(uint32_t*) (StartAddr + 4);		//C 复位向量地址。
		Jump_To_Application = (pFunction) JumpAddress;
		
		/* Initialize user application's Stack Pointer */
		//C __MSR_MSP(*(vu32*) ApplicationAddress);	
		__set_CONTROL(0); //!!非常重要，但没啥用处。
		__set_MSP(*(uint32_t*) StartAddr);	//C @core_cm3.h
		__set_PSP(*(uint32_t*) StartAddr);	//C @core_cm3.h
		
		__enable_irq();
		Jump_To_Application();				//C QQ为什么不直接执行JumpAddress()。
	}
}


/*读取应用程序的版本。
固定位置为：08080600，08080601，08080602，08080603，08080604（校验和）
返回值：0--成功，其他--失败。
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

