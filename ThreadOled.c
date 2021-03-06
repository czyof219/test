
#include <stdio.h>
#include <stdlib.h>

#include "cmsis_os2.h"                                        // CMSIS RTOS header file
#include "stm32l4xx.h"
#include "oled.h" 
#include "dbguart.h"
#include "mymessage.h"
#include <string.h>


#include "GUI.h"
extern GUI_CONST_STORAGE GUI_FONT GUI_Fontshengjizhong;
extern GUI_CONST_STORAGE GUI_FONT GUI_Fontzhengzaiguanji;
extern GUI_CONST_STORAGE GUI_FONT GUI_Fontniujubanshou;



extern GUI_CONST_STORAGE GUI_BITMAP bmlogo_wifi;			//C wifi logo.
extern GUI_CONST_STORAGE GUI_BITMAP bmlogo_wifi4;
extern GUI_CONST_STORAGE GUI_BITMAP bmwifi_12_12;
extern GUI_CONST_STORAGE GUI_BITMAP bmwifi_12_12_no;

extern uint8_t VBuf[];

extern osMessageQueueId_t midDisp;

uint32_t gflag_ok;

MSG_DISP msgDisp;


typedef struct{
	int32_t Nm[8];	//C
	uint32_t idx;	//C 待写入的位置。
}XX;

XX points={.idx= 0};
XX len={.idx= 0};		//C 长度


/*----------------------------------------------------------------------------
 *      Thread 1 'Thread_Name': Sample thread
!!OLED的y范围是[32, 95]。
 *---------------------------------------------------------------------------*/
 
void ThreadOled (void *argument);                                 // thread function
osThreadId_t tid_ThreadOled;                                      // thread id
 
int Init_ThreadOled (void) {
	const osThreadAttr_t attr={.priority= osPriorityAboveNormal}; 
	
	tid_ThreadOled = osThreadNew (ThreadOled, NULL, &attr);
	
	if (!tid_ThreadOled){ 
		uartS("Oled ko\r\n");
		
		return -1;
	}
	
	return(0);
}



void ThreadOled (void *argument) {
	
	uint8_t i= 0;
	int32_t ret;
	osStatus_t status = osOK;
	uint32_t isNew= 0;
	uint32_t step;
	
	
	initOled();
	uartS("\rinit oled ok\r");
	
	Set_Display_On_Off(1);
	
	
	/*绘制标题栏
	*/
	//GUI_SetPenSize(4);
	//GUI_DrawRect(80,2,80+15,2+7);
	GUI_SetColor(GUI_BLUE);
	GUI_SetFont(&GUI_Font20_ASCII);
	GUI_DispStringAt("BOOT2",0,32);
	//GUI_DrawLine(0,32+64, 16, 32+63);
	
	isNew= 1;
	//GUI_DrawRect(0,0,127,127);
	
	
	step= 1;
	while (1) {
		
		
		status = osMessageQueueGet(midDisp, &msgDisp, NULL, 50);
		
		if(osOK == status){
			
			/*解析显示数据
			*/
			if(MSGDISP_FROM_AG == msgDisp.src){

				/*msgDisp.data[0]为角度值。
				*/
				msgDisp.data[0]*= 180;	
				msgDisp.data[0]*= 100;	//C 扩大100倍，以显示小数点后2位。
				msgDisp.data[0]/= 32768;
				
				{
					uint32_t zs;	//C 整数部分
					uint32_t xs;	//C 小数部分
					uint8_t agStr[8];
					
					zs= msgDisp.data[0]/100;
					xs= msgDisp.data[0]%100;
					
					sprintf((char*)agStr, "%03d.%02d",zs,xs);
					agStr[7]='\0';
					
					GUI_SetColor(GUI_CYAN);
					GUI_SetFont(&GUI_Font20_ASCII);
					GUI_DispStringAt((char*)agStr,0,32+20);
				}
				
				isNew= 1;
			}
			else if(MSGDISP_FROM_NM1 == msgDisp.src){

				/*msgDisp.data[0]为扭矩值。
				*/
				msgDisp.data[0]/= 100;
				float f= 1.461988*msgDisp.data[0];
				
				{
					uint8_t agStr[20];
					
					sprintf((char*)agStr, "%08d  ",msgDisp.data[0]);
					//sprintf((char*)agStr, "%.3f",f);
					agStr[19]='\0';
					GUI_SetColor(GUI_BLUE);
					GUI_SetFont(&GUI_Font20_ASCII);
					GUI_DispStringAt((char*)agStr,0,32+20);
				}
				
				
				isNew= 1;
			}
			else if(MSGDISP_FROM_NM2 == msgDisp.src){

				/*msgDisp.data[0]为扭矩值。
				*/
				msgDisp.data[0]/= 100;	
				
				{
					uint8_t agStr[20];
					
					sprintf((char*)agStr, "%08d  ",msgDisp.data[0]);
					agStr[19]='\0';
					GUI_SetColor(GUI_BLUE);
					GUI_SetFont(&GUI_Font20_ASCII);
					GUI_DispStringAt((char*)agStr,0,32+20+20);
				}
				
				
				isNew= 1;
			}

//			if(MSGDISP_FROM_WIFI == msgDisp.src){
//				uartS("WIFI disp\r\n");
//				
//				/*msgDisp.data[0]为WiFi连接状态: 0--未连接，其他--连接。
//				*/
//				if('C' == msgDisp.data[0]){			//显示升级提示。
//					uint8_t agStr[20];
//					
//					sprintf((char*)agStr, "PRG UPDATING...");
//					agStr[19]='\0';
//					GUI_SetColor(GUI_BLUE);
//					//GUI_SetFont(&GUI_Font16_ASCII);
//					GUI_SetFont(&GUI_Fontshengjizhong);
//					GUI_UC_SetEncodeUTF8();
//					strcpy((char*)agStr, "鍗囩骇锛庯紟");
//					GUI_DispStringAt((char*)agStr,0,16*2);
//					
//					isNew= 1;
//				}else if('D' == msgDisp.data[0]){	//显示升级进度。
//					uint8_t agStr[20];
//					
//					sprintf((char*)agStr, "%04d/%04d", msgDisp.data[1],msgDisp.data[2]);
//					agStr[19]='\0';
//					GUI_SetColor(GUI_BLUE);
//					GUI_SetFont(&GUI_Font24_ASCII);
//					GUI_DispStringAt((char*)agStr,0,16*4);
//					
//					isNew= 1;
//				}else if('I' == msgDisp.data[0]){	//显示提示信息。
//					uint8_t agStr[30];
//					if(0 == msgDisp.data[1]) { 
//						//sprintf((char*)agStr, "Flshing...   ");
//						//sprintf((char*)agStr, "鍒锋満涓?...   ");	//涓嶈兘杩欎箞浣跨敤
//						//strcpy((char*)agStr, "鍒锋満涓紟锛?");
//						strcpy((char*)agStr, "鍒锋満涓? 锛庯紟");		//QQ"鍒锋満涓?"灏变笉琛?
//						GUI_SetFont(&GUI_Fontshengjizhong);
//						GUI_UC_SetEncodeUTF8();
//					}
//					else if(1 == msgDisp.data[1]){
//						//sprintf((char*)agStr, "Flshing...OK ");
//						strcpy((char*)agStr, "鎴愬姛锛庤閲嶅惎");
//						GUI_SetFont(&GUI_Fontshengjizhong);
//						GUI_UC_SetEncodeUTF8();
//					}
//					else if(2 == msgDisp.data[1]){
//						//sprintf((char*)agStr, "Flshing:CRC  ");
//						strcpy((char*)agStr, "鍒锋満锛庯紟CRC");
//						GUI_SetFont(&GUI_Fontshengjizhong);
//						GUI_UC_SetEncodeUTF8();
//					}
//					else if(3 == msgDisp.data[1]){
//						//sprintf((char*)agStr, "Flshing:ERR1 ");
//						strcpy((char*)agStr, "鍒锋満锛庯紟ERR1");
//						GUI_SetFont(&GUI_Fontshengjizhong);
//						GUI_UC_SetEncodeUTF8();
//					}
//					else if(4 == msgDisp.data[1]){
//						//sprintf((char*)agStr, "Flshing:ERR2 ");
//						strcpy((char*)agStr, "鍒锋満锛庯紟ERR2");
//						GUI_SetFont(&GUI_Fontshengjizhong);
//						GUI_UC_SetEncodeUTF8();
//					}
//					else if(5 == msgDisp.data[1]){
//						strcpy((char*)agStr, "PSRAM:CRC    ");
//						GUI_SetFont(&GUI_Font24_ASCII);
//					}
//					
//					agStr[19]='\0';
//					GUI_SetColor(GUI_BLUE);
//					
//					GUI_DispStringAt((char*)agStr,0,16*2);
//					
//					isNew= 1;
//				}
//			}
		}
		
		
		/*是否更新整屏显示。
		*/
		if(0 != isNew)
		{
			isNew= 0;

//			for(int i= 0; i< 32*1024; i++){
//				VBuf[i]= 0xCC;
//			}
//			//显示汉字。
//			GUI_SetFont(&GUI_Fontzhengzaiguanji);
//			GUI_SetColor(GUI_BLUE);
//			GUI_UC_SetEncodeUTF8();
//			GUI_DispStringAt("鍏虫満",0,16*3);
			
			ret= toLCD(VBuf, 32*1024);
		}
	
	}
}
