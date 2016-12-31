#ifndef __SEARCH_H
#define __SEARCH_H

#include "ISO14230.h"
#include "ISO15765.h"
#include "ISO9141.h"
#include "main.h"
#include "CAN.h"

typedef enum 
{
	NONE = 0,
	PWM_1850,
	vPWM_1850,
	K_9141_5,
	K_14230_5,
	K_14230_fast,
	CAN_STD_500,
	CAN_EXT_500,
	CAN_STD_250,
	CAN_EXT_250,
}CARprotocol;

CARprotocol ProtocolSearch(void);
CARprotocol CanProSearch(void);
void FlowCon(void);
void K_Reminder(void);

#endif
