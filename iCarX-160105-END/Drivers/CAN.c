/**
  ******************************************************************************
  * @file    CAN.c 
  * @author  PDAger iCar team
  * @version V0.5.0
  * @date    12-August-2011
  * @brief   CAN init.
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "CAN.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void can_init(const u32 Baud,int mode)
{
	CAN_InitTypeDef        CAN_InitStructure;
  	CAN_FilterInitTypeDef  CAN_FilterInitStructure;

  	/* CAN register init */
  	CAN_DeInit(CAN1);
  	CAN_StructInit(&CAN_InitStructure);

  	/* CAN cell init */
  	CAN_InitStructure.CAN_TTCM=DISABLE;
  	CAN_InitStructure.CAN_ABOM=DISABLE;
  	CAN_InitStructure.CAN_AWUM=DISABLE;
  	CAN_InitStructure.CAN_NART=ENABLE;	 //出错重发
  	CAN_InitStructure.CAN_RFLM=DISABLE;
  	CAN_InitStructure.CAN_TXFP=DISABLE;
  	CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;
  	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;  	
  	switch(Baud)
    {                   
        case 500000:CAN_InitStructure.CAN_BS1=CAN_BS1_6tq;
  					CAN_InitStructure.CAN_BS2=CAN_BS2_5tq;
					CAN_InitStructure.CAN_Prescaler=6;//500K波特率     36M/(1+6+5)/6                                                          
                  break;
        case 250000:CAN_InitStructure.CAN_BS1=CAN_BS1_8tq;
  					CAN_InitStructure.CAN_BS2=CAN_BS2_7tq;
					CAN_InitStructure.CAN_Prescaler=9;//250K波特率     36M/(1+8+7)/9                                    
                  break;
			default:CAN_InitStructure.CAN_BS1=CAN_BS1_9tq;
  					CAN_InitStructure.CAN_BS2=CAN_BS2_8tq;
					CAN_InitStructure.CAN_Prescaler=4;
				  break;
    } 		

  	/* CAN filter init */
  	CAN_FilterInitStructure.CAN_FilterNumber		= 0;
  	CAN_FilterInitStructure.CAN_FilterMode			= CAN_FilterMode_IdMask;
  	CAN_FilterInitStructure.CAN_FilterScale			= CAN_FilterScale_32bit;
	if(mode == CAN_STD_ID)							//标准帧滤波0x7Ex
	{
  		CAN_FilterInitStructure.CAN_FilterIdHigh 		= 0xFC00;	//0b 1111 1101 0000			
		CAN_FilterInitStructure.CAN_FilterIdLow 		= 0x0000;
		CAN_FilterInitStructure.CAN_FilterMaskIdHigh 	= 0xFE00; 	//0b 1111 1111 0010
		CAN_FilterInitStructure.CAN_FilterMaskIdLow 	= 0x0006;
	}
	else if(mode == CAN_EXT_ID)         			//扩展帧滤波0x18DAF1xx
	{
  		CAN_FilterInitStructure.CAN_FilterIdHigh 		= 0xC6D7;   //0b 000110001101101011110001xxxxxxxx				
		CAN_FilterInitStructure.CAN_FilterIdLow 		= 0x8804; 	//0b    11000110110101111000100000000100 
		CAN_FilterInitStructure.CAN_FilterMaskIdHigh 	= 0xFFFF; 
		CAN_FilterInitStructure.CAN_FilterMaskIdLow 	= 0xF806;
	}
  	CAN_FilterInitStructure.CAN_FilterFIFOAssignment= CAN_FIFO0;
  	CAN_FilterInitStructure.CAN_FilterActivation	= ENABLE;			 
  	CAN_FilterInit(&CAN_FilterInitStructure);

	CAN_ITConfig(CAN1, CAN_IT_FMP0, DISABLE);
	CAN_Init(CAN1, &CAN_InitStructure);

	DelayMs(500);						  
}	

uint8_t TaskCanRx(CanRxMsg *RxMessage)
{   	      
    if((CAN_MessagePending(CAN1, CAN_FIFO0) < 1)) 
	    return 0;//注意没有接收到数据到这里就结束了
	
		RxMessage->StdId=0x00;
		RxMessage->ExtId=0x00;
  	RxMessage->IDE=0;
  	RxMessage->DLC=0;
  	RxMessage->Data[0]=0x00;
  	RxMessage->Data[1]=0x00;
		RxMessage->Data[2]=0x00;
  	RxMessage->Data[3]=0x00;
		RxMessage->Data[4]=0x00;
  	RxMessage->Data[5]=0x00;
		RxMessage->Data[6]=0x00;
  	RxMessage->Data[7]=0x00;	    

    //有数据则接收,非中断 获取接收到的数据的ID
    CAN_Receive(CAN1,CAN_FIFO0,RxMessage);

    return 1;       
}

