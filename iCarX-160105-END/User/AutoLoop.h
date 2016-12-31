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
	uint8_t loop_time;			//采集支持码的循环变量
	uint8_t first;				//采集支持码的顺序变量
}PIDsupport;

typedef struct {
	uint8_t  write_flag;		//表征是否需要存储故障码并采集冻结帧
	uint8_t  dtc_dog;			//采集故障码时的狗狗，防止ECUOFF后死循环
	uint16_t DTC_temp;			//引起冻结帧的故障码
	uint16_t DTC_now[20];		//故障码缓冲
}MSG_DTC;

typedef enum 
{
	ECU_OFF = 0,
	ECU_ON,
	IDLE,
	DRIVING
} ECUstat;

void OBD_Main(void);						//总循环
void Cycle_Main(uint8_t SID,uint8_t PID);	//大循环 
void DynInfCop(void);						//命令处理
void PID_check(void);						//支持搜索
ECUstat state_judge(void);					//判断状态
uint8_t PID_chose(PIDsupport *supp);		//选择下一个PID
uint8_t PID_chose2(PIDsupport *supp);
uint8_t Main_send(uint8_t SID,uint8_t PID);	//标准发送与接收
void PIDFileRead(char* PIDnum);

#endif
