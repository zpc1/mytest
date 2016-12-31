#ifndef __SIM900A_H
#define __SIM900A_H

#include "USART.h"
#include "main.h"

typedef enum 
{
	CONNECTING = 0,
	CHECK,
	DATA,
	REBOOT,
	CXGLL,
} GPRSstat;

typedef struct {
	GPRSstat step;
	uint8_t sendok;
	uint8_t recvok;
	uint8_t pos;
	uint8_t dog;
	uint16_t point;
	uint8_t at_after_data;
	uint8_t msg[300];
}MSG_COMM_GPRS;

uint8_t SIM900A_Init(void);
uint8_t SIM900A_CMD(uint8_t *p);  
uint8_t SIM900A_Rx(uint8_t flag);  

uint8_t GPRS_connect(void);
uint8_t GPRS_check_connect(void);
uint8_t GPRS_check_status(void);
uint8_t GPRS_Send_CMD(uint16_t);
uint8_t GPRS_Signal(void);
uint8_t GPRS_reset(void);
uint8_t GPRS_CGREG(uint8_t *num);
uint16_t GPRS_YE(void);
void GPRS_time(void);
void GPRS_DTC_message(void);

#endif
