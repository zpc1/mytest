/**
  ******************************************************************************
  * @file    Search.c 
  * @author  PDAger iCar team
  * @version V0.5.0
  * @date    27-September-2011
  * @brief   Protocol search:CAN(15765),K(14230��9141).
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

/*********************************************** K�߳�ʼ��:5������ ************************************************************/ 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;	        //USART2 TX	���� PA2
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;    //���
  	GPIO_Init(GPIOA, &GPIO_InitStructure);		    	//A�˿� 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;	        //USART2 RX	���� PA3
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;		//����
  	GPIO_Init(GPIOA, &GPIO_InitStructure);		        //A�˿�

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;	        //L-line	���� PA4
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    //���
  	GPIO_Init(GPIOA, &GPIO_InitStructure);		    	//A�˿� 

    SetState(2,0);                               //ֹͣSCI2
    SCI_Receive(2,SCI_GetLen(2,0),&sciRxData[0]);//��һ�γ�ʼ��ʧ�ܣ�������ʼ������
     
   	//send address 0x33(0b00110011)	
	GPIO_ResetBits(GPIOA, GPIO_Pin_4);  //K_L (ISO_L)��L��
	GPIO_SetBits(GPIOA, GPIO_Pin_2);  	//TXD2(ISO_K)��K��
    timeRem=sysClock;
    while(sysClock-timeRem < 310);		//����300ms�ĸߵ�ƽ����������Ϊ310ms
    
    GPIO_SetBits(GPIOA, GPIO_Pin_4);  	//K_L (ISO_L)��L��
	GPIO_ResetBits(GPIOA, GPIO_Pin_2);  //TXD2(ISO_K)��K��
    timeRem=sysClock;
    while(sysClock-timeRem < 200);		//5 baud��ʼ������ʼλռ��200ms

   	GPIO_ResetBits(GPIOA, GPIO_Pin_4);  //K_L (ISO_L)��L��
	GPIO_SetBits(GPIOA, GPIO_Pin_2);  	//TXD2(ISO_K)��K��
    timeRem=sysClock;
    while(sysClock-timeRem < 400);		//5 baud��ʼ����00bռ��400ms

    GPIO_SetBits(GPIOA, GPIO_Pin_4);  	//K_L (ISO_L)��L��
	GPIO_ResetBits(GPIOA, GPIO_Pin_2);  //TXD2(ISO_K)��K��
    timeRem=sysClock;
    while(sysClock-timeRem<400);		//5 baud��ʼ����11bռ��400ms

    GPIO_ResetBits(GPIOA, GPIO_Pin_4);  //K_L (ISO_L)��L��
	GPIO_SetBits(GPIOA, GPIO_Pin_2);  	//TXD2(ISO_K)��K��
    timeRem=sysClock;
    while(sysClock-timeRem<400);		//5 baud��ʼ����00bռ��400ms

    GPIO_SetBits(GPIOA, GPIO_Pin_4);  	//K_L (ISO_L)��L��
	GPIO_ResetBits(GPIOA, GPIO_Pin_2);  //TXD2(ISO_K)��K��
    timeRem=sysClock;
    while(sysClock-timeRem<400);		//5 baud��ʼ����11bռ��400ms

    GPIO_ResetBits(GPIOA, GPIO_Pin_4);  //K_L (ISO_L)��L��
	GPIO_SetBits(GPIOA, GPIO_Pin_2);  	//TXD2(ISO_K)��K��	  
    
/***************************************** K�߳�ʼ����� ��ʼ��USART2********************************************************/	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;	        //USART2 TX	���� PA2
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;    	//�����������
  	GPIO_Init(GPIOA, &GPIO_InitStructure);		    	//A�˿� 
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;	        //USART2 RX	���� PA3
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//���ÿ�©����
  	GPIO_Init(GPIOA, &GPIO_InitStructure);		        //A�˿�
	usart2_init();										//����2��K�ߣ�

	USART_ClearFlag(USART2,USART_FLAG_RXNE);			//������ձ�־λ����ֹ���յ��Լ����͵�����
    SetState(2,1);//����SCI2Ϊ����״̬
    
    KComRem=sysClock;
    timeRem=sysClock;
	while(i==1) 
    {
        if(sysClock-timeRem>1002)//����200(stop)+W1+0x55 502
            break;
        if(SCI_GetLen(2,0)==1)       //�յ�����
        {
            timeRem=sysClock;
            sciRxLen=SCI_Receive(2,1,&sciRxData[0]);         //�������ݵ�data[0]��Ӧ�յ�0x55        
        }
        if(sciRxLen==1)
        {
            if(sciRxData[0]==0x55) //֮ǰ��0x55
            {
                while(1)//�ȴ�KB1��KB2
                {
                    if(sysClock-timeRem>45)//��ʱ�ж�
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
						                                                      //KB2��Ϊ0x8F
                        if(((sciRxData[0]==0xe9)&&(sciRxData[1]==0x8f))||\
                           ((sciRxData[0]==0x6b)&&(sciRxData[1]==0x8f))||\
                           ((sciRxData[0]==0x6d)&&(sciRxData[1]==0x8f))||\
                           ((sciRxData[0]==0xef)&&(sciRxData[1]==0x8f)))
                        {
							timeRem = sysClock;
	                        while(sysClock-timeRem<=25);//�ȴ�50ms                         	                            
								keyByte2=~sciRxData[1];     //0x70
	                        SCI_Transmit(2,1,&keyByte2);//����~KB2��֮����յ�~address
	                        timeRem = sysClock;
	                            
							while(sysClock-timeRem<=60);//�ȴ�50ms
	                            sciRxLen=SCI_Receive(2,8,&sciRxData[0]);

                            if((sciRxLen==2)&&(sciRxData[1]==0xcc))    //~0x33(address֮ǰ����0x33)
                            {
                                sysMode = K_14230_5;//Э��Ϊ14230��5������
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
                            while(sysClock-timeRem<=25);//�ȴ�50ms
                            keyByte2=~sciRxData[1];
                            SCI_Transmit(2,1,&keyByte2);//����~KB2=0xF7 or 0x6B
                            timeRem=sysClock;
                            while(sysClock-timeRem<=60);//�ȴ�50ms
                            sciRxLen=SCI_Receive(2,2,&sciRxData[0]);    //0xF7 or 0x6B

                            if((sciRxLen==2)&&(sciRxData[1]==0xcc))     //~0x33
                            {
                                sysMode = K_9141_5;//Э��Ϊ9141��5������
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
/*********************************************** K�߳�ʼ��:���� ************************************************************/ 
    if(sysMode==0)                //5 baud ��ʼ��ʧ�ܣ�
    {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, DISABLE);
		USART_DeInit(USART2);  			//����ֹͣSCI
  		USART_Cmd(USART2, DISABLE);
		SetState(2,0);

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;	        //USART2 TX	���� PA2
  		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    //���
  		GPIO_Init(GPIOA, &GPIO_InitStructure);		    	//A�˿� 
  		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;	        //USART2 RX	���� PA3
  		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;		//����
  		GPIO_Init(GPIOA, &GPIO_InitStructure);		        //A�˿�
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;	        //L-line	���� PA4
  		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    //���
  		GPIO_Init(GPIOA, &GPIO_InitStructure);		    	//B�˿�
        
        GPIO_ResetBits(GPIOA, GPIO_Pin_4);  //K_L (ISO_L)��L��
		GPIO_SetBits(GPIOA, GPIO_Pin_2);  	//TXD2(ISO_K)��K��
        timeRem=sysClock;
        while(sysClock-timeRem<310);		//����300ms�ĸߵ�ƽ����������Ϊ310ms

        GPIO_SetBits(GPIOA, GPIO_Pin_4);  	//K_L (ISO_L)��L��
		GPIO_ResetBits(GPIOA, GPIO_Pin_2);  //TXD2(ISO_K)��K��
        timeRem=sysClock;
        while(sysClock-timeRem<25);			//25ms�ĵ͵�ƽ(24-26ms�͵�ƽ)

        GPIO_ResetBits(GPIOA, GPIO_Pin_4);  //K_L (ISO_L)��L��
		GPIO_SetBits(GPIOA, GPIO_Pin_2);  	//TXD2(ISO_K)��K��
        timeRem=sysClock;
        while(sysClock-timeRem<25);			//25ms�ĸߵ�ƽ(24-26ms�ߵ�ƽ)

        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;	        //USART2 TX	���� PA2
  		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;    	//�����������
  		GPIO_Init(GPIOA, &GPIO_InitStructure);		    	//A�˿� 
  		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;	        //USART2 RX	���� PA3
  		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//���ÿ�©����
  		GPIO_Init(GPIOA, &GPIO_InitStructure);		        //A�˿�
		usart2_init();										//����2��K�ߣ�

        sciTxLen=SCI_Transmit(2,5,&sciTxData[0]);
        KComRem=sysClock;
        if(sciTxLen>=5)
        {
            timeRem=sysClock;
            while(sysClock-timeRem<100);	//�ȴ�100ms
            sciRxLen=SCI_GetLen(2,0);
            SCI_Receive(2,6,&sciRxData[0]);	//���Լ����������ݽ��ղ��ӵ�
            sciRxLen=SCI_GetLen(2,0);
            if(sciRxLen>3)//�������Լ�����ȥ���ֽ�
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
                SetState(2,0);//�ر�����K�ߵ�SCI;
            }
        }
        else
        {
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, DISABLE);
			USART_DeInit(USART2);
  			USART_Cmd(USART2, DISABLE);
            SetState(2,0);//�ر�����K�ߵ�SCI
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
	
	SCI_Transmit(4,32,&buffer[0]);//����AT�������Э���

	clock = sysClock;

	while(SCI_GetLen(4,0) == 0)
	{
		if(sysClock-clock > 15000)//15sδ�յ��ظ�
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
			if(buffer[1] == 0xA5)//ͷ��ȷ
			{
				while(SCI_GetLen(4,0) == 0)
				{
					if(sysClock-clock > 1000)
			            goto out;
				}
				SCI_Receive(4,1,&buffer[2]);//���ճ���
				if(buffer[2] < 3)			//����������3
					goto out;  
				while(SCI_GetLen(4,0) < buffer[2])		//5A A5 04 4A 00 xx sum
				{
					if(sysClock-clock > 1000)
			            goto out;
				}
				SCI_Receive(4,buffer[2],&buffer[3]);
				for(i=2;i < buffer[2]+2;i++)	
					temp+=buffer[i];			
				if((buffer[3] != 0x4A)||(buffer[4] != 0)||(buffer[5] == 0)||(buffer[6] != (uint8_t)temp))//У�鲻��
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
    uint8_t TransmitMailbox,ProSear[8]={ 2,1,0,0,0,0,0,0 };//���ڴ��CANЭ������ʱ���ݳ������ݣ�ʵ������Ϊ0100
  
    //��ʼ����CANЭ��
    for(i=1;i<=2;i++)
    {	
        can_init(i*250000,CAN_STD_ID);
	          				
        //////////////////��׼֡ģʽ����//////////////////////////       
		for(m=0;m<8;m++)
			TxMessage.Data[m]=ProSear[m];
		TxMessage.StdId=0x7DF;			   //0b 1111 1011 111	
		TxMessage.ExtId=0;
  		TxMessage.RTR=CAN_RTR_DATA;
  		TxMessage.IDE=CAN_ID_STD;
  		TxMessage.DLC=8;

        //�ж��Ƿ��ڷ���״̬       	
		j=1;				  
		while(j!=5)
		{	
			TimeRem=sysClock;
			TransmitMailbox = CAN_Transmit(CAN1, &TxMessage);				
        	while(CAN_TransmitStatus(CAN1, TransmitMailbox) != CANTXOK)	  //CAN_TransmitStatus(CAN1, 0)�����ͳɹ�����1
        	{
				if(sysClock-TimeRem>TIME_LIMIT)
            	{
                	j++;  //������ʧ�ܣ�����1��
                	break;//�뿪whileѭ��
            	}
        	}
			j++;
		} 
		j=0;
        //���ͳɹ�����ʼ����
        TimeRem=sysClock;
        k=1;
		RxMessage.Data[2] = 5;
        while(k)
        {	
            while(!TaskCanRx(&RxMessage))
            {				
                if(sysClock-TimeRem>TIME_LIMIT)//�ڹ̶���ʱ���ڽ��������е�����
                {
                    k=0;//��ʱ�����ٵȴ�
                    break;//�뿪whileѭ��
                }
            }
            if(RxMessage.Data[2]==0)
			{
                ResponseState=1;//����Ӧ
			}
		}
        if((i==1)&&(ResponseState==1))
        {
            proMsg.mode=0;//����can��ϢΪ��׼֡ģʽ
            return CAN_STD_250;//�������,CAN��׼֡,250kHz,�뿪forѭ��
        }
        else if((i==2)&&(ResponseState==1))
        {
            proMsg.mode=0;//����can��ϢΪ��׼֡ģʽ
            return CAN_STD_500;//�������,CAN��׼֡,500kHz,�뿪forѭ��
        }
        else//ĳ�������£��Ա�׼֡�����޻ظ�����������չ֡����ʽ����
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
	        	while(CAN_TransmitStatus(CAN1, TransmitMailbox) != CANTXOK)	  //CAN_TransmitStatus(CAN1, 0)�����ͳɹ�����1
	        	{
					if(sysClock-TimeRem>TIME_LIMIT)
	            	{
	                	j++;  //������ʧ�ܣ�����1��
	                	break;//�뿪whileѭ��
	            	}
	        	}
				j++;
			} 
			j=0;
            
            //���ͳɹ�����ʼ����
            TimeRem=sysClock;
            k=1;
			RxMessage.Data[2] = 5;
            while(k)
            {
                while(!TaskCanRx(&RxMessage))
                {
                    if(sysClock-TimeRem>TIME_LIMIT)//�ڹ̶���ʱ���ڽ��������е�����
                    {
                        k=0;//��ʱ�����ٵȴ�
                        break;//�뿪whileѭ��
                    }
                }
                if(RxMessage.Data[2]==0)
				{
                    ResponseState=1;//����Ӧ
				}
            }
            if((i==1)&&(ResponseState==1))
            {
                proMsg.mode=1;//����can��ϢΪ��չ֡ģʽ
                return CAN_EXT_250;//�������,CAN��չ֡,250kHz,�뿪forѭ��
            }
            else if((i==2)&&(ResponseState==1))
            {
                proMsg.mode=1;//����can��ϢΪ��չ֡ģʽ
                return CAN_EXT_500;//�������,CAN��չ֡,500kHz,�뿪forѭ��
            }
            else
            {
                if(i==1)
                    continue;//��������forѭ������ʼ�´�
                else
                    return NONE;//��15765Э�飬����������Э��,�뿪forѭ��
            }
        }
    }
	return NONE; //i>3����15765Э��      
}

//��ֹK��ͨѶ�ж�
void K_Reminder(void)
{
#ifndef ICAR_DOCK
    uint8_t len;//��¼ʵ���յ�������
    uint32_t timeRem;
    
    if(((K_9141_5 <= sysMode) && (sysMode <= K_14230_fast)) && (sysClock - KComRem > 4500))
    {
        proMsg.tx[0]=0xC2;//����֡��ʽ�����ݶγ��� 
        proMsg.tx[1]=0x33;//Ŀ���ַ
        proMsg.tx[2]=0xF1;//Դ��ַ
        proMsg.tx[3]=0x01;
        proMsg.tx[4]=0x00;
        proMsg.tx[5]=0xE7;
        KComRem=sysClock;
        SCI_Transmit(2,6,&proMsg.tx[0]);//����K������֡
        //���Լ�����ȥ�������Ƚ��յ���Ȼ���ӵ�
        timeRem=sysClock;
        while(SCI_GetLen(2,0)<6)
        {
            if(sysClock-timeRem>=20)
            {  
                break;//ֹͣ����ִ��
            } 
        }
        //�����Լ��������������Ϊû������proMsg.rxLen�������൱�ڽ��յ����ӵ���
        SCI_Receive(2,6,&proMsg.rx[0]);
        
        //���շ��ص�����
        while(SCI_GetLen(2,0)<10)
        {
            if(sysClock-timeRem>=60)
            {  
                //i=0;
                break;//ֹͣ����ִ��
            }
        }
        len=SCI_Receive(2,10,&proMsg.rx[0]);
        if(len<3)
        {
			car_state = ECU_OFF;
            sysMode = NONE;
        }
        
        timeRem=sysClock;//һ���������Ϻ���һ������ķ�������Ҫ50ms
        while(sysClock-timeRem<100)
            __nop();        
        SCI_Receive(2,10,&proMsg.rx[0]);      
    }
#endif
}

//CAN��֡���������ƺ���
void FlowCon(void)
{
#ifndef ICAR_DOCK
  uint8_t j,m,TransmitMailbox,FlowData[8]={0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  uint32_t TimeRem;
    
	for(m=0;m<8;m++)
    	TxMessage.Data[m]=FlowData[m];//���͵�����
  TxMessage.DLC=8;//�������ݳ�������
	TxMessage.StdId = 0x7E0;

	j = 1;				  	
	TimeRem = sysClock;
	TransmitMailbox = CAN_Transmit(CAN1, &TxMessage);				
    while(CAN_TransmitStatus(CAN1, TransmitMailbox) != CANTXOK)	  //CAN_TransmitStatus(CAN1, 0)�����ͳɹ�����1
    {
				if(sysClock - TimeRem > TIME_LIMIT)
        {
            j++;  //������ʧ�ܣ�����1��
						if(j > 3)
            	break;//�뿪whileѭ��
						else
						{
							TimeRem = sysClock;
							TransmitMailbox = CAN_Transmit(CAN1, &TxMessage);
						}
        }
    }   	
#endif
}

