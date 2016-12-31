#ifndef _TIMELENGTH_H_
#define _TIMELENGTH_H_

#include "stm32f10x_bkp.h"
#include "RTC.h"

typedef struct
{
	uint32_t idle_one;              //本次怠速时长
	uint32_t idle_all;              //总怠速时长
	uint32_t driving_one;           //本次行驶时长
	uint32_t driving_all;           //总行驶时长
} TIMELEN;
void TIMELENGTH(void);


#endif



