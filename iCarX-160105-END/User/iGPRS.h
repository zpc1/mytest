#ifndef __IGPRS_H
#define __IGPRS_H

#include "ISO14230.h"
#include "ISO15765.h"
#include "ISO9141.h"
#include "Search.h"
#include "main.h"

#define GPRS_DATA_START 		15	  //GPRS��̬����ͷ��С
#define GPRS_DATA_BUFFER_SIZE 	500	  //GPRS��̬���ݷ��ͻ�����
#define GPRS_DATA_RESPOND_TIME  15000 //GPRS��̬���ݻ�ִ���ʱ�䣨ms��
#define GPRS_DATA_POWEROFF_TIME	60000 //GPRSϨ��֡���ͼ����ms��

void GPRS_Main(void);
void GPRSRx(void);
void GPRSSend(void);
void GPRS_Data(void);
void GPRS_Flow_Check(void);

#endif
