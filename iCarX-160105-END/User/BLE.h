#ifndef __BLE_H
#define __BLE_H

#include "USART.h"

typedef struct BT_MSG 
{  
    uint8_t rxBt[21];        //���������ܵ������ݻ��棬����ȥ���ո��Ԥ����
    uint8_t rxLen;           //ÿ�λ�����������յ������ݳ���
    uint8_t rxLenTot;        //���������յ������ݰ��ܳ���
    uint8_t txCAN[8];        //CAN�������ݻ���
    
    uint8_t rxCAN[8];        //CAN�������ݻ���
    uint8_t txBt[21];        //�����������ݻ���
    uint8_t txLen;           //���������͵����ݳ���
}BT_MSG;

typedef struct MSG_COMM_BT 
{  
    uint8_t mode;                                                   //����ģʽ,�����Ƿ���ʾID,���з���
    uint8_t state;                                                  //��ǰ״̬,1-���ڷ���,3-���ڽ���,2-�������,4-�������,0-���е�
    uint8_t ECUNumber;                                              //���ؽ����ECU�ĸ���
    uint8_t BtTxState[8];                                           //ָʾ8��ECU���ص������Ƿ���ɣ��з��أ�,0-δ���,1-���
    uint8_t FrameNum[8];                                            //����ÿ��ECU���ص�֡����
    uint8_t txLen[8];                                               //���͵����ݳ���
    uint8_t rxLen;                                                  //���յ��������ܳ���
    uint8_t btHeadLen[8];                                           //������Ϣͷ����
    
    uint8_t tx[8][100];                                             //�������ݻ���    
    uint8_t rx[48];                                                 //�������ݻ���
    uint8_t btHead[8][12];                                          //������Ϣͷ
}MSG_COMM_BT;

typedef struct MSG_COMM 
{  
    uint8_t mode;                                                   //����ģʽ,�����Ƿ���ʾID,���з���
    uint8_t state;                                                  //��ǰ״̬,1-���ڷ���,3-���ڽ���,2-�������,4-�������,0-���е�
    uint8_t txLen;                                                  //���͵����ݳ���
    uint8_t rxLen;                                                  //���յ��������ܳ���    
    uint8_t tx[48];                                              //�������ݻ���    
    uint8_t rx[48];                                                 //�������ݻ���
}MSG_COMM;

//2011��3��16�����
typedef struct MSG_COMM_J1850
{
    uint8_t useBufferNum;//����ָʾ��ǰ���յ���֡����������ռ���˼���buffer��
    uint8_t position;//ָʾbuffer��λ�ã�0-4��ֵ������Ǽ���Ҫʹ�õ�buffer�����
    uint8_t bufferLen[30];//���ڴ��5��1850�������ĳ���
    uint8_t bufferState[30];//���ڴ��5����������״̬��0-δռ�ã�1-�����У�2-�������
    uint8_t buffer[30][12];//���ڴ�Ŵ���λ�����յ���1850��֡
}MSG_COMM_J1850;
//2011��3��16�����

enum
{
    COMM_IDLE,
    COMM_RX,
    COMM_RX_OK,      
    COMM_SEND,
    COMM_SEND_OK,
};

void BLE_Main(void);
void BLEAutoBack(void);
uint8_t BLEATFile(char *filename);
void BtSend(void);
uint8_t AT_master_BLE(uint8_t cmd);
void OilCount(void);
void Max_speed(void);

#endif
