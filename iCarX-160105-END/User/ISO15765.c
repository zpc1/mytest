/**
  ******************************************************************************
  * @file    ISO15765.c 
  * @author  PDAger iCar team
  * @version V0.5.0
  * @date    12-August-2011
  * @brief   CAN collect.
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "ISO15765.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern CanTxMsg TxMessage;
extern CanRxMsg RxMessage;
extern uint32_t sysClock;
extern MSG_COMM_BT btMsg;
extern MSG_COMM proMsg;	
extern uint8_t timeInval;
extern uint32_t TimeRem;//记录当前的时间，即当前的sysClock的值
extern uint8_t  FraNum[8];//记录各ECU返回的数据的总帧数
extern uint8_t multi_flag;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
uint8_t ISO15765Main(void)
{  
#ifndef ICAR_DOCK	
	send15765();
#else
	SciToCan();
#endif

	if(proMsg.state==COMM_SEND_OK)//蓝牙命令发送正常后才可以向ECU发数据         
	{
		if((get15765() == 0)&&(multi_flag == 0))
			return 0;
		else						
			while(0!=get15765());		
		return 1;				
	} 
	return 0;
}

void send15765(void)
{    
	while(SciToCan() != 0);//发送失败则再发
}

uint8_t get15765(void)
{   
#ifdef ICAR_DOCK

	uint8_t i,buffer[32] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};	
	uint16_t temp = 0;
	 
    TimeRem=sysClock;

	while(SCI_GetLen(4,0) < 3)//至少要收到3字节
	{
		if(sysClock-TimeRem > 5000)//1s未收到回复
            return 0;
	}
	SCI_Receive(4,3,&buffer[0]);
	
	if((buffer[0] == 0x5A)&&(buffer[1] == 0xA5))//头正确
	{
		if(buffer[2] < 3)
			goto out;
	 	while(SCI_GetLen(4,0) < (buffer[2]+1))//多收一位(header长度)
		{
			if(sysClock-TimeRem > 1000)
	            goto out;
		}
		SCI_Receive(4,buffer[2]+1,&buffer[3]);					//先收DATA数据

		/*printf("data = ");
		for(i=0;i<20;i++)
			printf(" %1X",buffer[i]);
		printf("\n");
		 */
		if((buffer[3]==0x4A)&&(buffer[4]==0)&&(buffer[5]==0)&&(buffer[6]==0x4E))//先判断是否有数（ECU是否断电）5A A5 04 4A 00 00 4E
			goto out;

		while(SCI_GetLen(4,0) < buffer[buffer[2]+3])
		{
			if(sysClock-TimeRem > 1000)
	            goto out;
		}
		SCI_Receive(4,buffer[3+buffer[2]]+1,&buffer[4+buffer[2]]);//再收header数据
		
		for(i=2;i<buffer[2]+buffer[buffer[2]+3]+4;i++)			  //和校验
			temp += buffer[i];
		if((uint8_t)temp != buffer[buffer[2]+buffer[buffer[2]+3]+4])//和校验错误
			goto out;

		btMsg.txLen[btMsg.ECUNumber] = buffer[2];				//存储数据长度
		for(i=0;i<buffer[2];i++)
            btMsg.tx[btMsg.ECUNumber][i] = buffer[i+3];

		btMsg.btHeadLen[btMsg.ECUNumber] = buffer[3+buffer[2]];	//存储头长度
		for(i=0;i<buffer[2];i++)
			btMsg.btHead[btMsg.ECUNumber][i] = buffer[i+buffer[2]+4];

		btMsg.BtTxState[btMsg.ECUNumber] = 1;//设置状态为完成
        btMsg.FrameNum[btMsg.ECUNumber] = 1;//设置帧数目
        btMsg.ECUNumber++;
        proMsg.state = COMM_RX_OK;

		if(SCI_GetLen(4,0) > 0)
			SCI_Receive(4,SCI_GetLen(4,0),&buffer[0]);

		return 1;	
	}
out:
	if(SCI_GetLen(4,0) > 0)
		SCI_Receive(4,SCI_GetLen(4,0),&buffer[0]);	
	return 0;

#else	 
	//uint8_t i;

    TimeRem=sysClock;
    while(!CAN_GetFlagStatus(CAN1,CAN_FLAG_FMP0))//接收第一帧数据
    {
       	if(sysClock-TimeRem > TIME_LIMIT)
		{
			return 0;//停止函数执行
		}
    }				
	if(TaskCanRx(&RxMessage))//接收数据成功
   	{	
		/*printf("CAN = ");
		for(i=0;i<8;i++)
			printf("%1X ",RxMessage.Data[i]);
		printf("\n"); */
										
		CanToSci();//从处理通过CAN接收到的数据
		if(RxMessage.Data[0]==0x10)
	    	FlowCon();//多帧数据流控制	
		return 1;
   	}
	return 2;

#endif
}

//将CAN接收的数据转化为蓝牙数据包
void CanToSci(void)//这里的sci参数应该为二进制形式的
{
    uint8_t i,j;//循环变量
		static uint32_t IDBuffer[8];
    static uint8_t DataLenBuffer;//临时存储通过CAN返回的数据的总长度                         
            
    //开始处理数据         
    if((0<RxMessage.Data[0])&&(RxMessage.Data[0]<=9))
    {
        //蓝牙返回数据数据头处理（保存了CANID和数据长度）
        if(proMsg.mode==0)//标准帧模式
        {
						IDBuffer[btMsg.ECUNumber]=RxMessage.StdId;//保存ID
						DataLenBuffer=RxMessage.Data[0];//保存数据长度	   btMsg.btHead[ECU][0][1][2][3][4][5][6]
            btMsg.btHead[btMsg.ECUNumber][0]='7'; //							'7''E''x'' ''0''x'' '   
            for(i=0;i<2;i++)//第一到二个元素				   btMsg.tx[ECU][0]	[1]	[2][3][4][5]
            {									  //						 41	 xx	 xx	xx xx xx
												  //		   btMsg.txLen[ECU] = lenth
                btMsg.btHead[btMsg.ECUNumber][i+1]=IntToChar(IDBuffer[btMsg.ECUNumber]%256)[i];
            }
            btMsg.btHead[btMsg.ECUNumber][3]=' ';
            for(i=0;i<2;i++)
            {
                btMsg.btHead[btMsg.ECUNumber][i+4]=IntToChar(DataLenBuffer)[i];
            }
            btMsg.btHead[btMsg.ECUNumber][6]=' ';
            btMsg.btHeadLen[btMsg.ECUNumber]=7;//记录头数据长度
        }//END (proMsg.mode==0)
        else if(proMsg.mode==1)//扩展帧
        {
					IDBuffer[btMsg.ECUNumber]=RxMessage.ExtId;//保存ID
        	DataLenBuffer=RxMessage.Data[0];//保存数据长度
            for(i=0;i<12;i++)
            {
                switch(i)
                {
                    case 0:  btMsg.btHead[btMsg.ECUNumber][i]='1';
                             break;
                    case 1:  btMsg.btHead[btMsg.ECUNumber][i]='8';
                             break;
                    case 2:  btMsg.btHead[btMsg.ECUNumber][i]='d';
                             break;
                    case 3:  btMsg.btHead[btMsg.ECUNumber][i]='a';
                             break;
                    case 4:  btMsg.btHead[btMsg.ECUNumber][i]='f';
                             break;        
                    case 5:  btMsg.btHead[btMsg.ECUNumber][i]='1';
                             break;
                    case 6:  btMsg.btHead[btMsg.ECUNumber][i]=IntToChar(IDBuffer[btMsg.ECUNumber]%256)[i-6];
                             break;                
                    case 7:  btMsg.btHead[btMsg.ECUNumber][i]=IntToChar(IDBuffer[btMsg.ECUNumber]%256)[i-6];
                             break;        
                    case 8:  btMsg.btHead[btMsg.ECUNumber][i]=' ';
                             break;
                    case 9:  btMsg.btHead[btMsg.ECUNumber][i]=IntToChar(DataLenBuffer)[i-9];
                             break;
                    case 10: btMsg.btHead[btMsg.ECUNumber][i]=IntToChar(DataLenBuffer)[i-9];
                             break;
                    case 11: btMsg.btHead[btMsg.ECUNumber][i]=' ';
                             break;
                    default: break;
                }
            }//END (i=0;i<12;i++)
            btMsg.btHeadLen[btMsg.ECUNumber]=12;//记录头数据长度
        }//END (proMsg.mode==1)
        //保存数据

        for(i=0;i<DataLenBuffer;i++)
        {
            btMsg.tx[btMsg.ECUNumber][i]=RxMessage.Data[i+1];
        }
        btMsg.txLen[btMsg.ECUNumber]=RxMessage.Data[0];//存储长度
        btMsg.BtTxState[btMsg.ECUNumber]=1;//设置状态为完成
        btMsg.FrameNum[btMsg.ECUNumber]=1;//设置帧数目
        btMsg.ECUNumber+=1;
        proMsg.state=COMM_RX_OK;
    }//END ((0<canTx.CAN_Data[0])&&(canTx.CAN_Data[0]<=7))
    //多帧数据的第一帧
    else if(RxMessage.Data[0]==0x10)
    {   
        DataLenBuffer=RxMessage.Data[1];//保存数据长度;
				multi_flag = 1;	     
        //确定帧数目
        if((DataLenBuffer-6)%7==0)
            btMsg.FrameNum[btMsg.ECUNumber]=(DataLenBuffer-6)/7+1;
        else                                        
            btMsg.FrameNum[btMsg.ECUNumber]=(DataLenBuffer-6)/7+2;
       
        //蓝牙返回数据数据头处理（只保存了CANID，为了处理方便）
        if(proMsg.mode==0)//标准帧模式
        {	
						IDBuffer[btMsg.ECUNumber]=RxMessage.StdId;//保存ID
            btMsg.btHead[btMsg.ECUNumber][0]='7';
            for(i=0;i<2;i++)//第一到二个元素
            {
                btMsg.btHead[btMsg.ECUNumber][i+1]=IntToChar(IDBuffer[btMsg.ECUNumber]%256)[i];
            }
            btMsg.btHead[btMsg.ECUNumber][3]=' ';
            btMsg.btHeadLen[btMsg.ECUNumber]=4;//存储头数据长度
        }//END (proMsg.mode==0)
        else if(proMsg.mode==1)//扩展帧
        {
						IDBuffer[btMsg.ECUNumber]=RxMessage.ExtId;//保存ID
            for(i=0;i<12;i++)
            {
                switch(i)
                {
                    case 0:  btMsg.btHead[btMsg.ECUNumber][i]='1';
                             break;
                    case 1:  btMsg.btHead[btMsg.ECUNumber][i]='8';
                             break;
                    case 2:  btMsg.btHead[btMsg.ECUNumber][i]='d';
                             break;
                    case 3:  btMsg.btHead[btMsg.ECUNumber][i]='a';
                             break;
                    case 4:  btMsg.btHead[btMsg.ECUNumber][i]='f';
                             break;        
                    case 5:  btMsg.btHead[btMsg.ECUNumber][i]='1';
                             break;
                    case 6:  btMsg.btHead[btMsg.ECUNumber][i]=IntToChar(IDBuffer[btMsg.ECUNumber]%256)[i-6];
                             break;                
                    case 7:  btMsg.btHead[btMsg.ECUNumber][i]=IntToChar(IDBuffer[btMsg.ECUNumber]%256)[i-6];
                             break;        
                    case 8:  btMsg.btHead[btMsg.ECUNumber][i]=' ';
                             break;
                    default: break;
                }
            }//END (i=0;i<12;i++)
            btMsg.btHeadLen[btMsg.ECUNumber]=9;//存储头数据长度
        }//END (proMsg.mode==1)
        //保存数据（将数据通通保存，包括PCI和长度）
			
        for(i=0;i<6;i++)										//10 14 49 02 xx xx xx xx
        {
            btMsg.tx[btMsg.ECUNumber][i]=RxMessage.Data[i+2];	//0~5:49 02 xx xx xx xx
        }
        btMsg.txLen[btMsg.ECUNumber]=6;//存储长度
        FraNum[btMsg.ECUNumber]=1;//记录已经获取到的帧数-1帧
        btMsg.ECUNumber+=1;//返回结果的ECU个数加1 			  ////////////////////////////////////////////////////
           
    }//END (canTx.CAN_Data[0]==0x10)
    //多帧数据的连续帧
    else if((0x21<=RxMessage.Data[0])&&(RxMessage.Data[0]<=0x2f))
    {
				multi_flag++;
				if(multi_flag>=10)
					multi_flag = 0;

        for(i=0;i<8;i++)//查找ID
        {
						if(proMsg.mode==0)//标准帧模式
            {
								if(IDBuffer[i]==RxMessage.StdId)
                	break;
            }
						else if(proMsg.mode==1)
						{
								if(IDBuffer[i]==RxMessage.ExtId)
                	break;
            }
        }

        for(j=0;j<7;j++)//保存数据
        {
            btMsg.tx[i][btMsg.txLen[i]+j]=RxMessage.Data[j+1];
        }
        FraNum[i]+=1;//帧数加1
        btMsg.txLen[i]+=7;
        //判断多帧数据是否接收完毕
        if(btMsg.FrameNum[i]==FraNum[i])
        {
            proMsg.state=COMM_RX_OK;
            btMsg.BtTxState[i]=1;//设置状态为接收完成
						btMsg.txLen[i] = DataLenBuffer;
        }
    }//END ((0x21<=canTx.CAN_Data[0])&&(canTx.CAN_Data[0]<=0x2f))
}

//将蓝牙接收的数据帧转化为CAN的数据帧,并发送出去
uint8_t SciToCan(void)//这里的sci参数应该为二进制形式的 
{
#ifndef ICAR_DOCK

    uint8_t i,m, TransmitMailbox, send_fail = 0;

    if(btMsg.rxLen>0)//ID配置，在搜索协议完成后mode的值就应该配置好
    {
        proMsg.state=COMM_SEND;	//正在发送

        if(proMsg.mode == 0)		//proMsg.mode用来表征是标准ID（0）形式还是扩展ID形式（1） 
        {
            TxMessage.StdId=0x7DF;		//采用标准帧 7df
			TxMessage.ExtId=0;
			TxMessage.IDE=CAN_ID_STD;
        }
        else
        {    
			TxMessage.StdId=0;                                                             
            TxMessage.ExtId=0x18DB33F1;	//采用扩展帧 18db33f1
			TxMessage.IDE=CAN_ID_EXT;
        }
        for(i=0;i<=btMsg.rxLen;i++)
        {
            if(i==0)
            {
                proMsg.tx[0]=btMsg.rxLen;
            }
            else
            {
                proMsg.tx[i]=btMsg.rx[i-1];
            }
        }
        for(m=0;m<8;m++)
        	TxMessage.Data[m]=proMsg.tx[m];//把将要发送的数据地址can.tx赋给canTx.CAN_Data
        
        TxMessage.DLC=8;//数据长度设为8
  		TxMessage.RTR=CAN_RTR_DATA;//数据帧

        TimeRem=sysClock;  
        TransmitMailbox = CAN_Transmit(CAN1, &TxMessage);		
        while(CAN_TransmitStatus(CAN1, TransmitMailbox) != CANTXOK)//等待直到发送完成
        {
            if(sysClock-TimeRem>=(TIME_LIMIT*2))
            {	
				send_fail++;				
				if(send_fail>5)	  //失败次数过多，取消发送
				{
					#ifdef PRINT
						printf("send fail\n");
					#endif
					send_fail = 0;
                }
				return send_fail;
            }
        }
        proMsg.state=COMM_SEND_OK;//发送完毕
        
        for(i=0;i<=btMsg.rxLen;i++)
        {
            proMsg.tx[i]=0;//清零
        }
        btMsg.rxLen=0; 
		send_fail = 0;
    }//END (btMsg.rxLen>0)
	return send_fail;
#else

	uint8_t i,len,buffer1[100],buffer[32] = {0x5A,0xA5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

    if(btMsg.rxLen>0)
    {
        proMsg.state=COMM_SEND;	//正在发送
        
        for(i=0;i<=btMsg.rxLen;i++)	
        {
            if(i==0)
                buffer[2]=btMsg.rxLen+1;		 //长度
            else
                buffer[2+i]=btMsg.rx[i-1];		 //参数	
        }
		for(i=2;i<2+buffer[2];i++)
			buffer[2+buffer[2]] += buffer[i]; 	 //sum
        
		len = SCI_GetLen(4,0);
		//printf("scican len = %d\n",len);
		if(len > 0)
			SCI_Receive(4,len,&buffer1[0]);
				
		SCI_Transmit(4,32,&buffer[0]);			 //发送

        proMsg.state=COMM_SEND_OK;//发送完毕

        btMsg.rxLen=0; 
    }//END (btMsg.rxLen>0)
	return 0;
#endif
}


