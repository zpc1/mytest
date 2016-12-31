#ifndef __BLE_H
#define __BLE_H

#include "USART.h"

typedef struct BT_MSG 
{  
    uint8_t rxBt[21];        //从蓝牙接受到的数据缓存，经过去除空格的预处理。
    uint8_t rxLen;           //每次缓存从蓝牙接收到的数据长度
    uint8_t rxLenTot;        //从蓝牙接收到的数据包总长度
    uint8_t txCAN[8];        //CAN发送数据缓存
    
    uint8_t rxCAN[8];        //CAN接受数据缓存
    uint8_t txBt[21];        //蓝牙发送数据缓存
    uint8_t txLen;           //从蓝牙发送的数据长度
}BT_MSG;

typedef struct MSG_COMM_BT 
{  
    uint8_t mode;                                                   //工作模式,比如是否显示ID,换行符等
    uint8_t state;                                                  //当前状态,1-正在发送,3-正在接收,2-发送完毕,4-接收完毕,0-空闲等
    uint8_t ECUNumber;                                              //返回结果的ECU的个数
    uint8_t BtTxState[8];                                           //指示8个ECU返回的数据是否完成（有返回）,0-未完成,1-完成
    uint8_t FrameNum[8];                                            //保存每个ECU返回的帧数据
    uint8_t txLen[8];                                               //发送的数据长度
    uint8_t rxLen;                                                  //接收到的数据总长度
    uint8_t btHeadLen[8];                                           //蓝牙信息头长度
    
    uint8_t tx[8][100];                                             //发送数据缓存    
    uint8_t rx[48];                                                 //接受数据缓存
    uint8_t btHead[8][12];                                          //蓝牙信息头
}MSG_COMM_BT;

typedef struct MSG_COMM 
{  
    uint8_t mode;                                                   //工作模式,比如是否显示ID,换行符等
    uint8_t state;                                                  //当前状态,1-正在发送,3-正在接收,2-发送完毕,4-接收完毕,0-空闲等
    uint8_t txLen;                                                  //发送的数据长度
    uint8_t rxLen;                                                  //接收到的数据总长度    
    uint8_t tx[48];                                              //发送数据缓存    
    uint8_t rx[48];                                                 //接受数据缓存
}MSG_COMM;

//2011年3月16日添加
typedef struct MSG_COMM_J1850
{
    uint8_t useBufferNum;//用于指示当前接收到几帧，即完整地占用了几个buffer。
    uint8_t position;//指示buffer的位置，0-4，值代表的是即将要使用的buffer的序号
    uint8_t bufferLen[30];//用于存放5个1850缓存区的长度
    uint8_t bufferState[30];//用于存放5个缓存区的状态，0-未占用，1-接收中，2-接收完成
    uint8_t buffer[30][12];//用于存放从下位机接收到的1850的帧
}MSG_COMM_J1850;
//2011年3月16日添加

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
