
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
	uint32_t idx;	//C ��д���λ�á�
}XX;

XX points={.idx= 0};
XX len={.idx= 0};		//C ����


/*----------------------------------------------------------------------------
 *      Thread 1 'Thread_Name': Sample thread
!!OLED��y��Χ��[32, 95]��
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
	
	
	/*���Ʊ�����
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
			
			/*������ʾ����
			*/
			if(MSGDISP_FROM_AG == msgDisp.src){

				/*msgDisp.data[0]Ϊ�Ƕ�ֵ��
				*/
				msgDisp.data[0]*= 180;	
				msgDisp.data[0]*= 100;	//C ����100��������ʾС�����2λ��
				msgDisp.data[0]/= 32768;
				
				{
					uint32_t zs;	//C ��������
					uint32_t xs;	//C С������
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

				/*msgDisp.data[0]ΪŤ��ֵ��
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

				/*msgDisp.data[0]ΪŤ��ֵ��
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
//				/*msgDisp.data[0]ΪWiFi����״̬: 0--δ���ӣ�����--���ӡ�
//				*/
//				if('C' == msgDisp.data[0]){			//��ʾ������ʾ��
//					uint8_t agStr[20];
//					
//					sprintf((char*)agStr, "PRG UPDATING...");
//					agStr[19]='\0';
//					GUI_SetColor(GUI_BLUE);
//					//GUI_SetFont(&GUI_Font16_ASCII);
//					GUI_SetFont(&GUI_Fontshengjizhong);
//					GUI_UC_SetEncodeUTF8();
//					strcpy((char*)agStr, "升级．．");
//					GUI_DispStringAt((char*)agStr,0,16*2);
//					
//					isNew= 1;
//				}else if('D' == msgDisp.data[0]){	//��ʾ�������ȡ�
//					uint8_t agStr[20];
//					
//					sprintf((char*)agStr, "%04d/%04d", msgDisp.data[1],msgDisp.data[2]);
//					agStr[19]='\0';
//					GUI_SetColor(GUI_BLUE);
//					GUI_SetFont(&GUI_Font24_ASCII);
//					GUI_DispStringAt((char*)agStr,0,16*4);
//					
//					isNew= 1;
//				}else if('I' == msgDisp.data[0]){	//��ʾ��ʾ��Ϣ��
//					uint8_t agStr[30];
//					if(0 == msgDisp.data[1]) { 
//						//sprintf((char*)agStr, "Flshing...   ");
//						//sprintf((char*)agStr, "刷机中...   ");	//不能这么使用
//						//strcpy((char*)agStr, "刷机中．．");
//						strcpy((char*)agStr, "刷机中 ．．");		//QQ"刷机中"就不行
//						GUI_SetFont(&GUI_Fontshengjizhong);
//						GUI_UC_SetEncodeUTF8();
//					}
//					else if(1 == msgDisp.data[1]){
//						//sprintf((char*)agStr, "Flshing...OK ");
//						strcpy((char*)agStr, "成功．请重启");
//						GUI_SetFont(&GUI_Fontshengjizhong);
//						GUI_UC_SetEncodeUTF8();
//					}
//					else if(2 == msgDisp.data[1]){
//						//sprintf((char*)agStr, "Flshing:CRC  ");
//						strcpy((char*)agStr, "刷机．．CRC");
//						GUI_SetFont(&GUI_Fontshengjizhong);
//						GUI_UC_SetEncodeUTF8();
//					}
//					else if(3 == msgDisp.data[1]){
//						//sprintf((char*)agStr, "Flshing:ERR1 ");
//						strcpy((char*)agStr, "刷机．．ERR1");
//						GUI_SetFont(&GUI_Fontshengjizhong);
//						GUI_UC_SetEncodeUTF8();
//					}
//					else if(4 == msgDisp.data[1]){
//						//sprintf((char*)agStr, "Flshing:ERR2 ");
//						strcpy((char*)agStr, "刷机．．ERR2");
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
		
		
		/*�Ƿ����������ʾ��
		*/
		if(0 != isNew)
		{
			isNew= 0;

//			for(int i= 0; i< 32*1024; i++){
//				VBuf[i]= 0xCC;
//			}
//			//��ʾ���֡�
//			GUI_SetFont(&GUI_Fontzhengzaiguanji);
//			GUI_SetColor(GUI_BLUE);
//			GUI_UC_SetEncodeUTF8();
//			GUI_DispStringAt("关机",0,16*3);
			
			ret= toLCD(VBuf, 32*1024);
		}
	
	}
}
