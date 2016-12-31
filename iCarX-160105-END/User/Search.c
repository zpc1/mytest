/**
  ******************************************************************************
  * @file    Search.c 
  * @author  PDAger iCar team
  * @version V0.5.0
  * @date    27-September-2011
  * @brief   Protocol search:CAN(15765),K(14230、9141).
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "Search.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern CanTxMsg TxMessage;
extern CanRxMsg RxMessage;
extern CARprotocol sysMode;
extern uint32_t KComRem;
extern MSG_COMM_BT btMsg;
extern MSG_COMM proMsg;
extern uint8_t timeInval;
extern uint32_t sysClock;
extern ECUstat car_state;
extern uint16_t time_m;
extern uint8_t pid_save;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
CARprotocol ProtocolSearch(void)
{
#ifndef ICAR_DOCK
	GPIO_InitTypeDef GPIO_InitStructure;
    uint8_t i=1;
    uint8_t keyByte2;
    uint8_t sciRxLen=0;
    uint8_t sciRxData[8];
    uint8_t sciTxLen;
    uint8_t sciTxData[8]={ 0xC1,0x33,0xf1,0x81,0x66,0,0,0 };
    uint32_t timeRem;	
   
    sysMode = CanProSearch();   

/*********************************************** K线初始化:5波特率 ************************************************************/ 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;	        //USART2 TX	—— PA2
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;    //输出
  	GPIO_Init(GPIOA, &GPIO_InitStructure);		    	//A端口 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;	        //USART2 RX	—— PA3
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;		//输入
  	GPIO_Init(GPIOA, &GPIO_InitStructure);		        //A端口

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;	        //L-line	—— PA4
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    //输出
  	GPIO_Init(GPIOA, &GPIO_InitStructure);		    	//A端口 

    SetState(2,0);                               //停止SCI2
    SCI_Receive(2,SCI_GetLen(2,0),&sciRxData[0]);//第一次初始化失败，后续初始化有用
     
   	//send address 0x33(0b00110011)	
	GPIO_ResetBits(GPIOA, GPIO_Pin_4);  //K_L (ISO_L)，L线
	GPIO_SetBits(GPIOA, GPIO_Pin_2);  	//TXD2(ISO_K)，K线
    timeRem=sysClock;
    while(sysClock-timeRem < 310);		//至少300ms的高电平，这里设置为310ms
    
    GPIO_SetBits(GPIOA, GPIO_Pin_4);  	//K_L (ISO_L)，L线
	GPIO_ResetBits(GPIOA, GPIO_Pin_2);  //TXD2(ISO_K)，K线
    timeRem=sysClock;
    while(sysClock-timeRem < 200);		//5 baud初始化，起始位占用200ms

   	GPIO_ResetBits(GPIOA, GPIO_Pin_4);  //K_L (ISO_L)，L线
	GPIO_SetBits(GPIOA, GPIO_Pin_2);  	//TXD2(ISO_K)，K线
    timeRem=sysClock;
    while(sysClock-timeRem < 400);		//5 baud初始化，00b占用400ms

    GPIO_SetBits(GPIOA, GPIO_Pin_4);  	//K_L (ISO_L)，L线
	GPIO_ResetBits(GPIOA, GPIO_Pin_2);  //TXD2(ISO_K)，K线
    timeRem=sysClock;
    while(sysClock-timeRem<400);		//5 baud初始化，11b占用400ms

    GPIO_ResetBits(GPIOA, GPIO_Pin_4);  //K_L (ISO_L)，L线
	GPIO_SetBits(GPIOA, GPIO_Pin_2);  	//TXD2(ISO_K)，K线
    timeRem=sysClock;
    while(sysClock-timeRem<400);		//5 baud初始化，00b占用400ms

    GPIO_SetBits(GPIOA, GPIO_Pin_4);  	//K_L (ISO_L)，L线
	GPIO_ResetBits(GPIOA, GPIO_Pin_2);  //TXD2(ISO_K)，K线
    timeRem=sysClock;
    while(sysClock-timeRem<400);		//5 baud初始化，11b占用400ms

    GPIO_ResetBits(GPIOA, GPIO_Pin_4);  //K_L (ISO_L)，L线
	GPIO_SetBits(GPIOA, GPIO_Pin_2);  	//TXD2(ISO_K)，K线	  
    
/***************************************** K线初始化完成 初始化USART2********************************************************/	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;	        //USART2 TX	—— PA2
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;    	//复用推挽输出
  	GPIO_Init(GPIOA, &GPIO_InitStructure);		    	//A端口 
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;	        //USART2 RX	—— PA3
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//复用开漏输入
  	GPIO_Init(GPIOA, &GPIO_InitStructure);		        //A端口
	usart2_init();										//串口2（K线）

	USART_ClearFlag(USART2,USART_FLAG_RXNE);			//清除接收标志位，防止接收到自己发送的数据
    SetState(2,1);//设置SCI2为开启状态
    
    KComRem=sysClock;
    timeRem=sysClock;
	while(i==1) 
    {
        if(sysClock-timeRem>1002)//超过200(stop)+W1+0x55 502
            break;
        if(SCI_GetLen(2,0)==1)       //收到数据
        {
            timeRem=sysClock;
            sciRxLen=SCI_Receive(2,1,&sciRxData[0]);         //回收数据到data[0]，应收到0x55        
        }
        if(sciRxLen==1)
        {
            if(sciRxData[0]==0x55) //之前是0x55
            {
                while(1)//等待KB1和KB2
                {
                    if(sysClock-timeRem>45)//超时判断
                    {
                        i=0;
                        break;
                    }
                    if(SCI_GetLen(2,0)==2)
                    {
                        timeRem=sysClock;
                        sciRxLen=SCI_Receive(2,2,&sciRxData[0]);						
                    }					 
					printf("[0] = %1X, [1] = %1X\n",sciRxData[0],sciRxData[1]);
                    if(sciRxLen==2)								//   ---_-__--___----
                    {   										//0b 1110100110001111
						                                                      //KB2必为0x8F
                        if(((sciRxData[0]==0xe9)&&(sciRxData[1]==0x8f))||\
                           ((sciRxData[0]==0x6b)&&(sciRxData[1]==0x8f))||\
                           ((sciRxData[0]==0x6d)&&(sciRxData[1]==0x8f))||\
                           ((sciRxData[0]==0xef)&&(sciRxData[1]==0x8f)))
                        {
							timeRem = sysClock;
	                        while(sysClock-timeRem<=25);//等待50ms                         	                            
								keyByte2=~sciRxData[1];     //0x70
	                        SCI_Transmit(2,1,&keyByte2);//发送~KB2，之后会收到~address
	                        timeRem = sysClock;
	                            
							while(sysClock-timeRem<=60);//等待50ms
	                            sciRxLen=SCI_Receive(2,8,&sciRxData[0]);

                            if((sciRxLen==2)&&(sciRxData[1]==0xcc))    //~0x33(address之前发的0x33)
                            {
                                sysMode = K_14230_5;//协议为14230，5波特率
                                return sysMode;
                            }
                            else
                            {
                                i=0;
                                break;
                            }                          
                        }
                        else if(((sciRxData[0]==0x08)&&(sciRxData[1]==0x08))||\
                                ((sciRxData[0]==0x94)&&(sciRxData[1]==0x94)))
                        {
                            while(sysClock-timeRem<=25);//等待50ms
                            keyByte2=~sciRxData[1];
                            SCI_Transmit(2,1,&keyByte2);//发送~KB2=0xF7 or 0x6B
                            timeRem=sysClock;
                            while(sysClock-timeRem<=60);//等待50ms
                            sciRxLen=SCI_Receive(2,2,&sciRxData[0]);    //0xF7 or 0x6B

                            if((sciRxLen==2)&&(sciRxData[1]==0xcc))     //~0x33
                            {
                                sysMode = K_9141_5;//协议为9141，5波特率
                                return sysMode;
                            }
                            else
                            {
                                i=0;
                                break;
                            }
                        }
                        else
                        {
                            i=0;
                            break;
                        }
                    }
                }
            }
        }
    }
/*********************************************** K线初始化:快速 ************************************************************/ 
    if(sysMode==0)                //5 baud 初始化失败？
    {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, DISABLE);
		USART_DeInit(USART2);  			//首先停止SCI
  		USART_Cmd(USART2, DISABLE);
		SetState(2,0);

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;	        //USART2 TX	—— PA2
  		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    //输出
  		GPIO_Init(GPIOA, &GPIO_InitStructure);		    	//A端口 
  		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;	        //USART2 RX	—— PA3
  		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;		//输入
  		GPIO_Init(GPIOA, &GPIO_InitStructure);		        //A端口
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;	        //L-line	—— PA4
  		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    //输出
  		GPIO_Init(GPIOA, &GPIO_InitStructure);		    	//B端口
        
        GPIO_ResetBits(GPIOA, GPIO_Pin_4);  //K_L (ISO_L)，L线
		GPIO_SetBits(GPIOA, GPIO_Pin_2);  	//TXD2(ISO_K)，K线
        timeRem=sysClock;
        while(sysClock-timeRem<310);		//至少300ms的高电平，这里设置为310ms

        GPIO_SetBits(GPIOA, GPIO_Pin_4);  	//K_L (ISO_L)，L线
		GPIO_ResetBits(GPIOA, GPIO_Pin_2);  //TXD2(ISO_K)，K线
        timeRem=sysClock;
        while(sysClock-timeRem<25);			//25ms的低电平(24-26ms低电平)

        GPIO_ResetBits(GPIOA, GPIO_Pin_4);  //K_L (ISO_L)，L线
		GPIO_SetBits(GPIOA, GPIO_Pin_2);  	//TXD2(ISO_K)，K线
        timeRem=sysClock;
        while(sysClock-timeRem<25);			//25ms的高电平(24-26ms高电平)

        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;	        //USART2 TX	—— PA2
  		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;    	//复用推挽输出
  		GPIO_Init(GPIOA, &GPIO_InitStructure);		    	//A端口 
  		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;	        //USART2 RX	—— PA3
  		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//复用开漏输入
  		GPIO_Init(GPIOA, &GPIO_InitStructure);		        //A端口
		usart2_init();										//串口2（K线）

        sciTxLen=SCI_Transmit(2,5,&sciTxData[0]);
        KComRem=sysClock;
        if(sciTxLen>=5)
        {
            timeRem=sysClock;
            while(sysClock-timeRem<100);	//等待100ms
            sciRxLen=SCI_GetLen(2,0);
            SCI_Receive(2,6,&sciRxData[0]);	//把自己发出的数据接收并扔掉
            sciRxLen=SCI_GetLen(2,0);
            if(sciRxLen>3)//不包含自己发出去的字节
            {
                SCI_Receive(2,sciRxLen,&sciRxData[0]);
                SCI_Transmit(2,sciRxLen,&sciRxData[0]);                
                sysMode = K_14230_fast;
                return sysMode;
            }
            else
            {
                sysMode = NONE;
				RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, DISABLE);
				USART_DeInit(USART2);
  				USART_Cmd(USART2, DISABLE);
                SetState(2,0);//关闭用于K线的SCI;
            }
        }
        else
        {
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, DISABLE);
			USART_DeInit(USART2);
  			USART_Cmd(USART2, DISABLE);
            SetState(2,0);//关闭用于K线的SCI
            sysMode = NONE;
        }
    }
    
     			   
    return sysMode;
	
#else
	uint8_t i,j,len,buffer1[32],buffer[32] = {0x5A,0xA5,3,0x0A,0,0x0D,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	uint16_t temp = 0;
	uint32_t clock = 0;

	len = SCI_GetLen(4,0);
	if(len > 0)
		SCI_Receive(4,len,&buffer1[0]);
	
	SCI_Transmit(4,32,&buffer[0]);//发送AT命令：请求协议号

	clock = sysClock;

	while(SCI_GetLen(4,0) == 0)
	{
		if(sysClock-clock > 15000)//15s未收到回复
            goto out;
	}
	len = SCI_GetLen(4,0);
	for(j=0;j<len;j++)
	{
		SCI_Receive(4,1,&buffer[0]);

		if(buffer[0] == 0x5A)
		{
		 	while(SCI_GetLen(4,0) == 0)
			{
				if(sysClock-clock > 1000)
		            goto out;
			}
			SCI_Receive(4,1,&buffer[1]);
			if(buffer[1] == 0xA5)//头正确
			{
				while(SCI_GetLen(4,0) == 0)
				{
					if(sysClock-clock > 1000)
			            goto out;
				}
				SCI_Receive(4,1,&buffer[2]);//接收长度
				if(buffer[2] < 3)			//长度至少是3
					goto out;  
				while(SCI_GetLen(4,0) < buffer[2])		//5A A5 04 4A 00 xx sum
				{
					if(sysClock-clock > 1000)
			            goto out;
				}
				SCI_Receive(4,buffer[2],&buffer[3]);
				for(i=2;i < buffer[2]+2;i++)	
					temp+=buffer[i];			
				if((buffer[3] != 0x4A)||(buffer[4] != 0)||(buffer[5] == 0)||(buffer[6] != (uint8_t)temp))//校验不对
					goto out;			

				if((sysMode != PWM_1850)&&(sysMode != vPWM_1850))
					sysMode = (CARprotocol)buffer[5];
				else
					sysMode = NONE;

				printf("search = ");
				for(i=0;i<8;i++)
					printf("%1X ",buffer[i]);
				
				if(SCI_GetLen(4,0) > 0)
					SCI_Receive(4,SCI_GetLen(4,0),&buffer[0]);	

				return sysMode;
			}			
		}
	}
out:
	if(SCI_GetLen(4,0) > 0)
		SCI_Receive(4,SCI_GetLen(4,0),&buffer[0]);	
	return NONE;
#endif  
}

CARprotocol CanProSearch(void)
{	
    uint32_t TimeRem;
    uint8_t i,j,k,m,ResponseState=0;
    uint8_t TransmitMailbox,ProSear[8]={ 2,1,0,0,0,0,0,0 };//用于存放CAN协议搜索时数据场的内容，实际意义为0100
  
    //开始搜索CAN协议
    for(i=1;i<=2;i++)
    {	
        can_init(i*250000,CAN_STD_ID);
	          				
        //////////////////标准帧模式搜索//////////////////////////       
		for(m=0;m<8;m++)
			TxMessage.Data[m]=ProSear[m];
		TxMessage.StdId=0x7DF;			   //0b 1111 1011 111	
		TxMessage.ExtId=0;
  		TxMessage.RTR=CAN_RTR_DATA;
  		TxMessage.IDE=CAN_ID_STD;
  		TxMessage.DLC=8;

        //判断是否处于发送状态       	
		j=1;				  
		while(j!=5)
		{	
			TimeRem=sysClock;
			TransmitMailbox = CAN_Transmit(CAN1, &TxMessage);				
        	while(CAN_TransmitStatus(CAN1, TransmitMailbox) != CANTXOK)	  //CAN_TransmitStatus(CAN1, 0)：发送成功返回1
        	{
				if(sysClock-TimeRem>TIME_LIMIT)
            	{
                	j++;  //若发送失败，重试1次
                	break;//离开while循环
            	}
        	}
			j++;
		} 
		j=0;
        //发送成功，开始接收
        TimeRem=sysClock;
        k=1;
		RxMessage.Data[2] = 5;
        while(k)
        {	
            while(!TaskCanRx(&RxMessage))
            {				
                if(sysClock-TimeRem>TIME_LIMIT)//在固定的时间内接收完所有的数据
                {
                    k=0;//超时，不再等待
                    break;//离开while循环
                }
            }
            if(RxMessage.Data[2]==0)
			{
                ResponseState=1;//有响应
			}
		}
        if((i==1)&&(ResponseState==1))
        {
            proMsg.mode=0;//设置can消息为标准帧模式
            return CAN_STD_250;//搜索完成,CAN标准帧,250kHz,离开for循环
        }
        else if((i==2)&&(ResponseState==1))
        {
            proMsg.mode=0;//设置can消息为标准帧模式
            return CAN_STD_500;//搜索完成,CAN标准帧,500kHz,离开for循环
        }
        else//某波特率下，以标准帧发送无回复，接着以扩展帧的形式发送
        {   
			can_init(i*250000,CAN_EXT_ID);
				                          
			for(m=0;m<8;m++)
				TxMessage.Data[m]=ProSear[m];
			TxMessage.StdId=0;
			TxMessage.ExtId=0x18DB33F1;
  			TxMessage.RTR=CAN_RTR_DATA;
  			TxMessage.IDE=CAN_ID_EXT;
  			TxMessage.DLC=8;
			
            TransmitMailbox = CAN_Transmit(CAN1, &TxMessage);

			j=1;				  
			while(j!=5)
			{	
				TimeRem=sysClock;
				TransmitMailbox = CAN_Transmit(CAN1, &TxMessage);				
	        	while(CAN_TransmitStatus(CAN1, TransmitMailbox) != CANTXOK)	  //CAN_TransmitStatus(CAN1, 0)：发送成功返回1
	        	{
					if(sysClock-TimeRem>TIME_LIMIT)
	            	{
	                	j++;  //若发送失败，重试1次
	                	break;//离开while循环
	            	}
	        	}
				j++;
			} 
			j=0;
            
            //发送成功，开始接收
            TimeRem=sysClock;
            k=1;
			RxMessage.Data[2] = 5;
            while(k)
            {
                while(!TaskCanRx(&RxMessage))
                {
                    if(sysClock-TimeRem>TIME_LIMIT)//在固定的时间内接收完所有的数据
                    {
                        k=0;//超时，不再等待
                        break;//离开while循环
                    }
                }
                if(RxMessage.Data[2]==0)
				{
                    ResponseState=1;//有响应
				}
            }
            if((i==1)&&(ResponseState==1))
            {
                proMsg.mode=1;//设置can消息为扩展帧模式
                return CAN_EXT_250;//搜索完成,CAN扩展帧,250kHz,离开for循环
            }
            else if((i==2)&&(ResponseState==1))
            {
                proMsg.mode=1;//设置can消息为扩展帧模式
                return CAN_EXT_500;//搜索完成,CAN扩展帧,500kHz,离开for循环
            }
            else
            {
                if(i==1)
                    continue;//结束本次for循环，开始下次
                else
                    return NONE;//非15765协议，结束搜索该协议,离开for循环
            }
        }
    }
	return NONE; //i>3，非15765协议      
}

//防止K线通讯中断
void K_Reminder(void)
{
#ifndef ICAR_DOCK
    uint8_t len;//记录实际收到的数据
    uint32_t timeRem;
    
    if(((K_9141_5 <= sysMode) && (sysMode <= K_14230_fast)) && (sysClock - KComRem > 4500))
    {
        proMsg.tx[0]=0xC2;//数据帧形式和数据段长度 
        proMsg.tx[1]=0x33;//目标地址
        proMsg.tx[2]=0xF1;//源地址
        proMsg.tx[3]=0x01;
        proMsg.tx[4]=0x00;
        proMsg.tx[5]=0xE7;
        KComRem=sysClock;
        SCI_Transmit(2,6,&proMsg.tx[0]);//发送K线数据帧
        //将自己发出去的数据先接收到，然后扔掉
        timeRem=sysClock;
        while(SCI_GetLen(2,0)<6)
        {
            if(sysClock-timeRem>=20)
            {  
                break;//停止函数执行
            } 
        }
        //接收自己发出的命令，但因为没有设置proMsg.rxLen，所以相当于接收到又扔掉了
        SCI_Receive(2,6,&proMsg.rx[0]);
        
        //接收返回的数据
        while(SCI_GetLen(2,0)<10)
        {
            if(sysClock-timeRem>=60)
            {  
                //i=0;
                break;//停止函数执行
            }
        }
        len=SCI_Receive(2,10,&proMsg.rx[0]);
        if(len<3)
        {
			car_state = ECU_OFF;
            sysMode = NONE;
        }
        
        timeRem=sysClock;//一个命令发送完毕后，另一个命令的发送至少要50ms
        while(sysClock-timeRem<100)
            __nop();        
        SCI_Receive(2,10,&proMsg.rx[0]);      
    }
#endif
}

//CAN多帧数据流控制函数
void FlowCon(void)
{
#ifndef ICAR_DOCK
  uint8_t j,m,TransmitMailbox,FlowData[8]={0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  uint32_t TimeRem;
    
	for(m=0;m<8;m++)
    	TxMessage.Data[m]=FlowData[m];//发送的数据
  TxMessage.DLC=8;//发送数据长度设置
	TxMessage.StdId = 0x7E0;

	j = 1;				  	
	TimeRem = sysClock;
	TransmitMailbox = CAN_Transmit(CAN1, &TxMessage);				
    while(CAN_TransmitStatus(CAN1, TransmitMailbox) != CANTXOK)	  //CAN_TransmitStatus(CAN1, 0)：发送成功返回1
    {
				if(sysClock - TimeRem > TIME_LIMIT)
        {
            j++;  //若发送失败，重试1次
						if(j > 3)
            	break;//离开while循环
						else
						{
							TimeRem = sysClock;
							TransmitMailbox = CAN_Transmit(CAN1, &TxMessage);
						}
        }
    }   	
#endif
}

