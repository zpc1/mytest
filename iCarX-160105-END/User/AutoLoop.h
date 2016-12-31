#ifndef _AUTOLOOP_H_
#define _AUTOLOOP_H_

#include "common.h"
#include "BSP.h"
#include "main.h"

typedef struct
{
	uint32_t v0100;
	uint32_t v0120;
	uint32_t v0140;
	uint8_t SID_now;
	uint8_t PID_now;
	uint8_t loop_time;			//�ɼ�֧�����ѭ������
	uint8_t first;				//�ɼ�֧�����˳�����
}PIDsupport;

typedef struct {
	uint8_t  write_flag;		//�����Ƿ���Ҫ�洢�����벢�ɼ�����֡
	uint8_t  dtc_dog;			//�ɼ�������ʱ�Ĺ�������ֹECUOFF����ѭ��
	uint16_t DTC_temp;			//���𶳽�֡�Ĺ�����
	uint16_t DTC_now[20];		//�����뻺��
}MSG_DTC;

typedef enum 
{
	ECU_OFF = 0,
	ECU_ON,
	IDLE,
	DRIVING
} ECUstat;

void OBD_Main(void);						//��ѭ��
void Cycle_Main(uint8_t SID,uint8_t PID);	//��ѭ�� 
void DynInfCop(void);						//�����
void PID_check(void);						//֧������
ECUstat state_judge(void);					//�ж�״̬
uint8_t PID_chose(PIDsupport *supp);		//ѡ����һ��PID
uint8_t PID_chose2(PIDsupport *supp);
uint8_t Main_send(uint8_t SID,uint8_t PID);	//��׼���������
void PIDFileRead(char* PIDnum);

#endif
