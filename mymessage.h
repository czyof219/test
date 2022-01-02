
/**
*/

#ifndef _MYMESSAGE_H_DJKLFJDFJIODUOFIU1312312
#define _MYMESSAGE_H_DJKLFJDFJIODUOFIU1312312

#include <stdint.h>

typedef struct{
	uint32_t src;		//C 
	int32_t data[4];	//C 最多4个点。
}MSG_DISP;


#define MSGDISP_FROM_AG			(0)
#define MSGDISP_FROM_NM1  		(1)
#define MSGDISP_FROM_NM2  		(2)
#define MSGDISP_FROM_WIFI  		(3)

#endif

/******** END OF FILE ********/

