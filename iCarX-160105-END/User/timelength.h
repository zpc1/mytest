#ifndef _TIMELENGTH_H_
#define _TIMELENGTH_H_

#include "stm32f10x_bkp.h"
#include "RTC.h"

typedef struct
{
	uint32_t idle_one;              //���ε���ʱ��
	uint32_t idle_all;              //�ܵ���ʱ��
	uint32_t driving_one;           //������ʻʱ��
	uint32_t driving_all;           //����ʻʱ��
} TIMELEN;
void TIMELENGTH(void);


#endif



