/**
  ******************************************************************************
  * @file    ISO9141.c 
  * @author  PDAger iCar team
  * @version V0.5.0
  * @date    21-September-2011
  * @brief   K-line collect.
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "ISO9141.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern uint8_t timeInval;
extern uint32_t TimeRem_K;
extern uint32_t sysClock;
extern MSG_COMM_BT btMsg;
extern MSG_COMM proMsg;
extern uint8_t error14230;
extern uint8_t dog14230;
extern ECUstat car_state;
extern CARprotocol sysMode;
extern struct SCIBuffer SCI4;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
uint8_t ISO9141Main(void)
{   
    SciToKwp9141();   
    proMsg.state=COMM_RX; //接收中
    if(get9141() == 0)
		return 0;
	else
		while(0!=get9141());
				
	return 1;		           
}

uint8_t get9141(void)
{   
#ifndef ICAR_DOCK
	uint8_t i;
	   
	for(i=0;i<48;i++)
    {
        proMsg.rx[i]=0;
    }
    proMsg.rxLen=0;
	TimeRem_K=sysClock;
    while(SCI_GetLen(2,0)==0)//如果没接收到数据，等待。
    {
        if(sysClock-TimeRem_K>TIME_LIMIT_K)//标准里的限制是50ms
            return 0;
    }				
	KwpRx9141();//接收一帧数据，并保存到proMsg的RX中
    if(proMsg.rxLen>3)
    {
        KwpToSci9141();//将接收的数据按规定的格式转化，并保存到btMsg中
		return 1;
    } 
	return 2;
#else
	uint8_t len,i,buffer[32] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};	
	uint16_t temp = 0;
	uint8_t times = 0;

	error14230 = 0;
out2:	 
    TimeRem_K=sysClock;
	while(SCI_GetLen(4,0) < 3)//至少要收到3字节
	{
		if(sysClock-TimeRem_K > 2000)//2s未收到回复
		{
			if(error14230 < 2)	 //重试2次
			{
				btMsg.rxLen = 2;
				error14230++;
				SciToKwp();
				TimeRem_K=sysClock;
			}
			else
			{
				dog14230++;
				if(dog14230 > 5)
				{
					NRF24L01_init();
					sysMode = NONE;
					car_state = ECU_OFF;

					len=SCI_GetLen(4,0);
					if(len > 0)
						SCI_Receive(4,len,buffer);
					SCI4.RxPos = 0;
					SCI4.RxLen = 0;
				}
				goto out1;
			}
		}
	}
	dog14230 = 0;
	while(SCI_GetLen(4,0) >= 2)
	{
		SCI_Receive(4,2,&buffer[0]);
		if((buffer[0] == 0x5A)&&(buffer[1] == 0xA5))
		{
			if(SCI_GetLen(4,0) >= 1)
				SCI_Receive(4,1,&buffer[2]);
			break;
		}
	}
	if((buffer[0] == 0x5A)&&(buffer[1] == 0xA5))//头正确
	{
		if(buffer[2] < 3)
			goto out;
	 	while(SCI_GetLen(4,0) < (buffer[2]+1))//多收一位
		{
			if(sysClock-TimeRem_K > 1000)
	            goto out;
		}
		SCI_Receive(4,buffer[2]+1,&buffer[3]);					//先收DATA数据
		
		if((buffer[3]==0x4A)&&(buffer[4]==0)&&(buffer[5]==0)&&(buffer[6]==0x4E))//先判断是否有数（ECU是否断电）5A A5 04 4A 00 00 4E
			goto out;

		while(SCI_GetLen(4,0) < buffer[buffer[2]+3])
		{
			if(sysClock-TimeRem_K > 1000)
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
		for(i=0;i<buffer[3+buffer[2]];i++)
			btMsg.btHead[btMsg.ECUNumber][i] = buffer[i+buffer[2]+4];

		btMsg.BtTxState[btMsg.ECUNumber] = 1;//设置状态为完成
        btMsg.FrameNum[btMsg.ECUNumber] = 1;//设置帧数目
        btMsg.ECUNumber++;
        proMsg.state = COMM_RX_OK;

	/*	printf("Data = ");
		for(i=0;i<20;i++)
			printf("%1X ",buffer[i]);
		printf("\n");	 */

		if(SCI_GetLen(4,0) > 0)
			SCI_Receive(4,SCI_GetLen(4,0),&buffer[0]);

		return 1;	
	}
out:			//丢数，重发一次
	times++;
	if(times >= 5)
		goto out1;
	SciToKwp();
	goto out2;
out1:
	if(SCI_GetLen(4,0) > 0)
		SCI_Receive(4,SCI_GetLen(4,0),&buffer[0]);
	printf("out9141\n");	
	return 0;

#endif
}		   

void KwpToSci9141(void)
{
    uint8_t i,j;//循环变量
    uint8_t ECUSch=0;//标记是否本ECU已经发回来过数据,0为没有，1位有
	static uint8_t AddressBuff[8];                        
            
    //搜寻是否本ECU已经发回来过数据
    for(i=0;i<8;i++)
    {
        if(proMsg.rx[2]==AddressBuff[i])
        {
            ECUSch=1;
            break;
        }//END (proMsg.rx[2]==AddressBuff[i])
    }
    //处理数据
    if(ECUSch==0)//这是该ECU首次发回数据或单帧数据
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
        for(i=0;i<proMsg.rxLen;i++)		//48 6B xx SID PID D0 D1 D2 D3 D4 D5 sum
        {
            btMsg.tx[btMsg.ECUNumber][i]=proMsg.rx[i+3];
        }
        btMsg.txLen[btMsg.ECUNumber]=proMsg.rxLen-4;//保存数据长度，减掉48 6B xx和sum
        //设置相关状态
        btMsg.BtTxState[btMsg.ECUNumber]=1;
        btMsg.FrameNum[btMsg.ECUNumber]=1;
        btMsg.ECUNumber+=1;
        btMsg.state=COMM_SEND_OK;       
    }
    else//ECU多帧数据的连续帧////////////////////////////////////////////////////////////////////////// 
    {
		for(j=3;j<proMsg.rxLen;j++)
        {
            btMsg.tx[i][j-3+btMsg.txLen[i]]=proMsg.rx[j+3];
        }
        btMsg.txLen[i]+=proMsg.rx[0]-0x80-3;//保存长度        
        //设置状态
        btMsg.FrameNum[i]+=1;
    }
}

//将蓝牙接收的数据帧转化为K线的数据帧,并发送出去
void SciToKwp9141(void)
{
#ifndef ICAR_DOCK
    uint8_t i=0;       //用于循环语句
    uint16_t sum=0; 
    
    //发送中
    proMsg.state=COMM_SEND; 
    
    //定义数据头
    proMsg.tx[0]=0x68;     //数据帧形式和数据段长度 
    proMsg.tx[1]=0x6A;                 //目标地址
    proMsg.tx[2]=0xF1;                 //源地址
    
    //定义数据
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
    proMsg.tx[proMsg.txLen]=(uint8_t)sum;
    proMsg.txLen++;//加上校验和的长度
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
    proMsg.state=COMM_SEND_OK;//发送完毕，设置状态 
#else
	uint8_t i,len,buffer1[32],buffer[32] = {0x5A,0xA5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

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

        btMsg.rxLen=0; 
    }//END (btMsg.rxLen>0)
#endif   
}

//K线接收函数，每次只接收一帧数据
void KwpRx9141(void)
{    
    uint8_t len;           //表示本次接收的数据个数
    
    //proMsg.state=COMM_RX; //接收中    
    
    //接收一帧数据的第一个字节，包含数据格式和数据长度信息
    SCI_Receive(2,1,&proMsg.rx[0]);
    if(proMsg.rx[0] == 0x48) //有用数据
    {
		DelayMs(10);
        len=SCI_GetLen(2,0);
        SCI_Receive(2,len,&proMsg.rx[1]);//接收数据
        if((proMsg.rx[1]==0x6B)&&(proMsg.rx[3]==btMsg.rx[0]+0x40))
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

