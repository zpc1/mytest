/**
  ******************************************************************************
  * @file    ISO14230.c 
  * @author  PDAger iCar team
  * @version V0.5.0
  * @date    12-August-2011
  * @brief   K-line collect.
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "ISO14230.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t error14230 = 0;
uint8_t dog14230 = 0;
extern uint8_t timeInval;
extern uint32_t TimeRem_K;
extern uint32_t sysClock;
extern MSG_COMM_BT btMsg;
extern MSG_COMM proMsg;
extern ECUstat car_state;
extern CARprotocol sysMode;
extern struct SCIBuffer SCI4;
extern uint8_t AddressBuff[8];

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
uint8_t ISO14230Main(void)
{   
	uint8_t i;

	for(i=0;i<8;i++)
        AddressBuff[i]=0;

    SciToKwp();   
    proMsg.state=COMM_RX; //接收中
    if(get14230() == 0)
		return 0;
	else
#ifdef ICAR_DOCK
		return 1;
#else
		while(0!=get14230());
#endif				
	return 1;		           
}

uint8_t get14230(void)
{   
#ifdef ICAR_DOCK

	uint8_t len,i,buffer[32] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};	
	uint16_t temp = 0;
	uint8_t ecu_all,ecu_num,times = 0;

	error14230 = 0;
out2:	 
	for(i=0;i<32;i++)
		buffer[i] = 0;

    TimeRem_K=sysClock;
	while(SCI_GetLen(4,0) < 4)//至少要收到4字节
	{
		if(sysClock-TimeRem_K > 2000)//2000ms未收到回复
			goto out;
	}
	dog14230 = 0;
	while(SCI_GetLen(4,0) >= 3)
	{
		SCI_Receive(4,2,&buffer[0]);
		if((buffer[0] == 0x5A)&&(buffer[1] == 0xA5))
		{
			if(SCI_GetLen(4,0) >= 2)
				SCI_Receive(4,2,&buffer[2]);
			break;
		}
	}
	//			ECU数量
	//		5A A5 xx 6 41 01 xx xx xx xx 7 7 E 8 x 0 4 x sum
	//		| 头 |  | 	   DATA        |    header   |
	if((buffer[0] == 0x5A)&&(buffer[1] == 0xA5))//头正确
	{
		if(buffer[3] < 3)
			goto out;

		ecu_all = ((buffer[2] & 0xF0) >> 4) & 0x0F;
		ecu_num = (buffer[2] & 0x0F) + 1;

		printf("ecu_all = %d, ecu_num = %d,",ecu_all,ecu_num);

	 	while(SCI_GetLen(4,0) < (buffer[3]+1))//多收一位
		{
			if(sysClock-TimeRem_K > 1000)
	            goto out;
		}
		SCI_Receive(4,buffer[3]+1,&buffer[4]);					//先收DATA数据

		if((buffer[3]==0x4A)&&(buffer[4]==0)&&(buffer[5]==0)&&(buffer[6]==0x4E))//先判断是否有数（ECU是否断电）5A A5 04 4A 00 00 4E
			goto out;

		while(SCI_GetLen(4,0) < buffer[buffer[3]+4])
		{
			if(sysClock-TimeRem_K > 1000)
	            goto out;
		}
		SCI_Receive(4,buffer[4+buffer[3]]+1,&buffer[5+buffer[3]]);//再收header数据

		temp = 0;
		for(i=2;i<buffer[3]+buffer[buffer[3]+4]+5;i++)			  //和校验
			temp += buffer[i];

		if((uint8_t)(temp & 0x00FF) != buffer[buffer[3]+buffer[buffer[3]+4]+5])//和校验错误
			goto out;

		btMsg.txLen[btMsg.ECUNumber] = buffer[3];				//存储数据长度
		for(i=0;i<buffer[3];i++)
            btMsg.tx[btMsg.ECUNumber][i] = buffer[i+4];

		btMsg.btHeadLen[btMsg.ECUNumber] = buffer[4+buffer[3]];	//存储头长度
		for(i=0;i<buffer[4+buffer[3]];i++)
			btMsg.btHead[btMsg.ECUNumber][i] = buffer[i+buffer[3]+5];

		btMsg.BtTxState[btMsg.ECUNumber] = 1;//设置状态为完成
        btMsg.FrameNum[btMsg.ECUNumber] = 1;//设置帧数目
        btMsg.ECUNumber++;
        proMsg.state = COMM_RX_OK;

		printf("Data = ");
		for(i=0;i<20;i++)
			printf("%1X ",buffer[i]);
		printf("\n");	 	  

		if(ecu_num != ecu_all) //多ECU数据
			goto out2;

		if(SCI_GetLen(4,0) > 0)
			SCI_Receive(4,SCI_GetLen(4,0),&buffer[0]);

		btMsg.rxLen = 0;

		return 1;	
	}
out:			//丢数，重发一次
	times++;
	printf("times = %d\n",times);
	if(times >= 5)
		goto out1;
	SciToKwp();
	goto out2;
out1:
	if(SCI_GetLen(4,0) > 0)
		SCI_Receive(4,SCI_GetLen(4,0),&buffer[0]);
	printf("out14230\n");
	btMsg.rxLen = 0;	
	return 0;

#else

	TimeRem_K=sysClock;
    while(SCI_GetLen(2,0)==0)//如果没接收到数据，等待。
    {
        if(sysClock-TimeRem_K>TIME_LIMIT_K)//标准里的限制是50ms
            return 0;
    }				
	KwpRx();//接收一帧数据，并保存到proMsg的RX中
    if(proMsg.rxLen>3)
    {
        KwpToSci();//将接收的数据按规定的格式转化，并保存到btMsg中
		return 1;
    } 
	return 2;

#endif
}	
	   
void KwpToSci(void)
{
    uint8_t i;//循环变量
    uint8_t j;//循环变量
    uint8_t ECUSch=0;//标记是否本ECU已经发回来过数据,0为没有，1位有                        
            
    //搜寻是否本ECU已经发回来过数据
    for(i=0;i<8;i++)
    {
        if(proMsg.rx[2] == AddressBuff[i])
        {
            ECUSch=1;
            break;
        }//END (proMsg.rx[2]==AddressBuff[i])
    }
    //处理数据
    if(ECUSch == 0)//这是该ECU首次发回数据
    {
        //把ECU的地址把保存下来，以便后面比较使用
        AddressBuff[btMsg.ECUNumber]=proMsg.rx[2];
        
        //处理数据头;
        for(i=0;i<2;i++)
        {
            btMsg.btHead[btMsg.ECUNumber][i]=IntToChar(proMsg.rx[0])[i];
        }
        btMsg.btHead[btMsg.ECUNumber][2]=' ';
        for(i=0;i<2;i++)
        {
            btMsg.btHead[btMsg.ECUNumber][i+3]=IntToChar(proMsg.rx[1])[i];
        }
        btMsg.btHead[btMsg.ECUNumber][5]=' ';
        for(i=0;i<2;i++)
        {
            btMsg.btHead[btMsg.ECUNumber][i+6]=IntToChar(proMsg.rx[2])[i];
        }
        btMsg.btHead[btMsg.ECUNumber][8]=' ';
        btMsg.btHeadLen[btMsg.ECUNumber]=9;//设置头的长度       
        //保存数据
        for(i=0;i<proMsg.rx[0]-0x80+1;i++)
        {
            btMsg.tx[btMsg.ECUNumber][i]=proMsg.rx[i+3];
        }
        btMsg.txLen[btMsg.ECUNumber]=proMsg.rx[0]-0x80;//保存长度
        
	//	printf("recv = %1X %1X %1X %1X %1X %1X %1X\n",btMsg.tx[0][0],btMsg.tx[0][1],btMsg.tx[0][2],btMsg.tx[0][3],btMsg.tx[0][4],btMsg.tx[0][5],btMsg.tx[0][6]);

        //设置相关状态
        btMsg.BtTxState[btMsg.ECUNumber]=1;
        btMsg.FrameNum[btMsg.ECUNumber]=1;
        btMsg.ECUNumber+=1;
        btMsg.state=COMM_SEND_OK;       
    }
    else//ECU多帧数据的连续帧
    {
        //保存数据
        for(j=3;j<proMsg.rx[0]-0x80;j++)
        {
            btMsg.tx[i][j-3+btMsg.txLen[i]]=proMsg.rx[j+3];
        }
        btMsg.txLen[i]+=proMsg.rx[0]-0x80-3;//保存长度        
        //设置状态
        btMsg.FrameNum[i]+=1;
    }
}

//将蓝牙接收的数据帧转化为K线的数据帧,并发送出去
void SciToKwp(void)
{
#ifndef ICAR_DOCK

    uint8_t i=0;       //用于循环语句
    uint16_t sum=0; 
    
    //发送中
    proMsg.state=COMM_SEND; 
    
    //定义数据头
    proMsg.tx[0]=0xC0+btMsg.rxLen;     //数据帧形式和数据段长度 
    proMsg.tx[1]=0x33;                 //目标地址
    proMsg.tx[2]=0xF1;                 //源地址
    //proMsg.txLen=3;                  //K线发送数据帧长度
    //定义数据
    //proMsg.tx[3]=btMsg.rx[0];                 //数据的第一个字节需要变动
    for(i=0;i<btMsg.rxLen;i++)
    {
        proMsg.tx[i+3]=btMsg.rx[i];    
    }
    proMsg.txLen=3+btMsg.rxLen;
    
    //计算并定义校验和
    for(i=0;i<proMsg.txLen;i++)
    {
        sum+=proMsg.tx[i];
    }
    proMsg.tx[proMsg.txLen]=(UINT8)(sum);
    proMsg.txLen++;//加上校验和的长度

	//printf("send = %1X %1X %1X %1X %1X %1X %1X\n",proMsg.tx[0],proMsg.tx[1],proMsg.tx[2],proMsg.tx[3],proMsg.tx[4],proMsg.tx[5],proMsg.tx[6]);

    SCI_Transmit(2,proMsg.txLen,&proMsg.tx[0]);//发送K线数据帧
    btMsg.rxLen=0;//蓝牙接收数据长度归零
    
    //将自己发出去的数据先接收到，然后扔掉
    TimeRem_K=sysClock;
    while(SCI_GetLen(2,0)<proMsg.txLen)
    {
        if(sysClock-TimeRem_K>=TIME_LIMIT_K)
        {  
            return;//停止函数执行
        } 
    }//END (SCI_GetLen(2,0)<proMsg.txLen)
    //接收自己发出的命令，但因为没有设置proMsg.rxLen，所以相当于接收到又扔掉了
    SCI_Receive(2,proMsg.txLen,&proMsg.rx[0]);
    
    ///////////测试////////////
    //SCI_Transmit(1,proMsg.txLen,&proMsg.rx[0]);
    //SendEnd();
    ///////////测试/////////// 
    
    proMsg.state=COMM_SEND_OK;//发送完毕，设置状态 

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
		if(len > 0)
			SCI_Receive(4,len,&buffer1[0]);	

		SCI_Transmit(4,32,&buffer[0]);			 //发送

        proMsg.state=COMM_SEND_OK;//发送完毕

        //btMsg.rxLen = 0; 
    }//END (btMsg.rxLen>0)

#endif   
}

//K线接收函数，每次只接收一帧数据
void KwpRx(void)
{    
    uint8_t i,len;           //表示本次接收的数据个数
    
    //proMsg.state=COMM_RX; //接收中    
    
    //接收一帧数据的第一个字节，包含数据格式和数据长度信息
    SCI_Receive(2,1,&proMsg.rx[0]);
    if((0x81<=proMsg.rx[0])&&(proMsg.rx[0]<=0x87))//有用数据
    {
        //proMsg.rxLen=1;
        len=proMsg.rx[0]-0x80+3;
        
        //接收其他数数据，包含了两个字节的地址信息和一个字节的校验和
        while(SCI_GetLen(2,0)<len)
        {
            if(sysClock-TimeRem_K>=TIME_LIMIT_K)
            {  
                return;//停止函数执行
            }
        }//END (SCI_GetLen(2,0)<len)
        SCI_Receive(2,len,&proMsg.rx[1]);//接收数据
		
	/*	printf("proMsg = ");
		for(i=0;i<=len;i++)
			printf("%1X ",proMsg.rx[i]);
		printf("\n");

		printf("btMsg = ");
		for(i=0;i<=len;i++)
			printf("%1X ",btMsg.rx[i]);
		printf("\n");  */

        if((proMsg.rx[1]==0xF1)&&(proMsg.rx[3]==btMsg.rx[0]+0x40))
        {
            proMsg.rxLen=len+1;//设置长度
            proMsg.state=COMM_RX_OK;//K线接收完成一帧
        }
        else
        {
            proMsg.rxLen=0;
        }
    }
}

