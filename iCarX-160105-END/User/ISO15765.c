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
extern uint32_t TimeRem;//��¼��ǰ��ʱ�䣬����ǰ��sysClock��ֵ
extern uint8_t  FraNum[8];//��¼��ECU���ص����ݵ���֡��
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

	if(proMsg.state==COMM_SEND_OK)//���������������ſ�����ECU������         
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
	while(SciToCan() != 0);//����ʧ�����ٷ�
}

uint8_t get15765(void)
{   
#ifdef ICAR_DOCK

	uint8_t i,buffer[32] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};	
	uint16_t temp = 0;
	 
    TimeRem=sysClock;

	while(SCI_GetLen(4,0) < 3)//����Ҫ�յ�3�ֽ�
	{
		if(sysClock-TimeRem > 5000)//1sδ�յ��ظ�
            return 0;
	}
	SCI_Receive(4,3,&buffer[0]);
	
	if((buffer[0] == 0x5A)&&(buffer[1] == 0xA5))//ͷ��ȷ
	{
		if(buffer[2] < 3)
			goto out;
	 	while(SCI_GetLen(4,0) < (buffer[2]+1))//����һλ(header����)
		{
			if(sysClock-TimeRem > 1000)
	            goto out;
		}
		SCI_Receive(4,buffer[2]+1,&buffer[3]);					//����DATA����

		/*printf("data = ");
		for(i=0;i<20;i++)
			printf(" %1X",buffer[i]);
		printf("\n");
		 */
		if((buffer[3]==0x4A)&&(buffer[4]==0)&&(buffer[5]==0)&&(buffer[6]==0x4E))//���ж��Ƿ�������ECU�Ƿ�ϵ磩5A A5 04 4A 00 00 4E
			goto out;

		while(SCI_GetLen(4,0) < buffer[buffer[2]+3])
		{
			if(sysClock-TimeRem > 1000)
	            goto out;
		}
		SCI_Receive(4,buffer[3+buffer[2]]+1,&buffer[4+buffer[2]]);//����header����
		
		for(i=2;i<buffer[2]+buffer[buffer[2]+3]+4;i++)			  //��У��
			temp += buffer[i];
		if((uint8_t)temp != buffer[buffer[2]+buffer[buffer[2]+3]+4])//��У�����
			goto out;

		btMsg.txLen[btMsg.ECUNumber] = buffer[2];				//�洢���ݳ���
		for(i=0;i<buffer[2];i++)
            btMsg.tx[btMsg.ECUNumber][i] = buffer[i+3];

		btMsg.btHeadLen[btMsg.ECUNumber] = buffer[3+buffer[2]];	//�洢ͷ����
		for(i=0;i<buffer[2];i++)
			btMsg.btHead[btMsg.ECUNumber][i] = buffer[i+buffer[2]+4];

		btMsg.BtTxState[btMsg.ECUNumber] = 1;//����״̬Ϊ���
        btMsg.FrameNum[btMsg.ECUNumber] = 1;//����֡��Ŀ
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
    while(!CAN_GetFlagStatus(CAN1,CAN_FLAG_FMP0))//���յ�һ֡����
    {
       	if(sysClock-TimeRem > TIME_LIMIT)
		{
			return 0;//ֹͣ����ִ��
		}
    }				
	if(TaskCanRx(&RxMessage))//�������ݳɹ�
   	{	
		/*printf("CAN = ");
		for(i=0;i<8;i++)
			printf("%1X ",RxMessage.Data[i]);
		printf("\n"); */
										
		CanToSci();//�Ӵ���ͨ��CAN���յ�������
		if(RxMessage.Data[0]==0x10)
	    	FlowCon();//��֡����������	
		return 1;
   	}
	return 2;

#endif
}

//��CAN���յ�����ת��Ϊ�������ݰ�
void CanToSci(void)//�����sci����Ӧ��Ϊ��������ʽ��
{
    uint8_t i,j;//ѭ������
		static uint32_t IDBuffer[8];
    static uint8_t DataLenBuffer;//��ʱ�洢ͨ��CAN���ص����ݵ��ܳ���                         
            
    //��ʼ��������         
    if((0<RxMessage.Data[0])&&(RxMessage.Data[0]<=9))
    {
        //����������������ͷ����������CANID�����ݳ��ȣ�
        if(proMsg.mode==0)//��׼֡ģʽ
        {
						IDBuffer[btMsg.ECUNumber]=RxMessage.StdId;//����ID
						DataLenBuffer=RxMessage.Data[0];//�������ݳ���	   btMsg.btHead[ECU][0][1][2][3][4][5][6]
            btMsg.btHead[btMsg.ECUNumber][0]='7'; //							'7''E''x'' ''0''x'' '   
            for(i=0;i<2;i++)//��һ������Ԫ��				   btMsg.tx[ECU][0]	[1]	[2][3][4][5]
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
            btMsg.btHeadLen[btMsg.ECUNumber]=7;//��¼ͷ���ݳ���
        }//END (proMsg.mode==0)
        else if(proMsg.mode==1)//��չ֡
        {
					IDBuffer[btMsg.ECUNumber]=RxMessage.ExtId;//����ID
        	DataLenBuffer=RxMessage.Data[0];//�������ݳ���
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
            btMsg.btHeadLen[btMsg.ECUNumber]=12;//��¼ͷ���ݳ���
        }//END (proMsg.mode==1)
        //��������

        for(i=0;i<DataLenBuffer;i++)
        {
            btMsg.tx[btMsg.ECUNumber][i]=RxMessage.Data[i+1];
        }
        btMsg.txLen[btMsg.ECUNumber]=RxMessage.Data[0];//�洢����
        btMsg.BtTxState[btMsg.ECUNumber]=1;//����״̬Ϊ���
        btMsg.FrameNum[btMsg.ECUNumber]=1;//����֡��Ŀ
        btMsg.ECUNumber+=1;
        proMsg.state=COMM_RX_OK;
    }//END ((0<canTx.CAN_Data[0])&&(canTx.CAN_Data[0]<=7))
    //��֡���ݵĵ�һ֡
    else if(RxMessage.Data[0]==0x10)
    {   
        DataLenBuffer=RxMessage.Data[1];//�������ݳ���;
				multi_flag = 1;	     
        //ȷ��֡��Ŀ
        if((DataLenBuffer-6)%7==0)
            btMsg.FrameNum[btMsg.ECUNumber]=(DataLenBuffer-6)/7+1;
        else                                        
            btMsg.FrameNum[btMsg.ECUNumber]=(DataLenBuffer-6)/7+2;
       
        //����������������ͷ����ֻ������CANID��Ϊ�˴����㣩
        if(proMsg.mode==0)//��׼֡ģʽ
        {	
						IDBuffer[btMsg.ECUNumber]=RxMessage.StdId;//����ID
            btMsg.btHead[btMsg.ECUNumber][0]='7';
            for(i=0;i<2;i++)//��һ������Ԫ��
            {
                btMsg.btHead[btMsg.ECUNumber][i+1]=IntToChar(IDBuffer[btMsg.ECUNumber]%256)[i];
            }
            btMsg.btHead[btMsg.ECUNumber][3]=' ';
            btMsg.btHeadLen[btMsg.ECUNumber]=4;//�洢ͷ���ݳ���
        }//END (proMsg.mode==0)
        else if(proMsg.mode==1)//��չ֡
        {
						IDBuffer[btMsg.ECUNumber]=RxMessage.ExtId;//����ID
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
            btMsg.btHeadLen[btMsg.ECUNumber]=9;//�洢ͷ���ݳ���
        }//END (proMsg.mode==1)
        //�������ݣ�������ͨͨ���棬����PCI�ͳ��ȣ�
			
        for(i=0;i<6;i++)										//10 14 49 02 xx xx xx xx
        {
            btMsg.tx[btMsg.ECUNumber][i]=RxMessage.Data[i+2];	//0~5:49 02 xx xx xx xx
        }
        btMsg.txLen[btMsg.ECUNumber]=6;//�洢����
        FraNum[btMsg.ECUNumber]=1;//��¼�Ѿ���ȡ����֡��-1֡
        btMsg.ECUNumber+=1;//���ؽ����ECU������1 			  ////////////////////////////////////////////////////
           
    }//END (canTx.CAN_Data[0]==0x10)
    //��֡���ݵ�����֡
    else if((0x21<=RxMessage.Data[0])&&(RxMessage.Data[0]<=0x2f))
    {
				multi_flag++;
				if(multi_flag>=10)
					multi_flag = 0;

        for(i=0;i<8;i++)//����ID
        {
						if(proMsg.mode==0)//��׼֡ģʽ
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

        for(j=0;j<7;j++)//��������
        {
            btMsg.tx[i][btMsg.txLen[i]+j]=RxMessage.Data[j+1];
        }
        FraNum[i]+=1;//֡����1
        btMsg.txLen[i]+=7;
        //�ж϶�֡�����Ƿ�������
        if(btMsg.FrameNum[i]==FraNum[i])
        {
            proMsg.state=COMM_RX_OK;
            btMsg.BtTxState[i]=1;//����״̬Ϊ�������
						btMsg.txLen[i] = DataLenBuffer;
        }
    }//END ((0x21<=canTx.CAN_Data[0])&&(canTx.CAN_Data[0]<=0x2f))
}

//���������յ�����֡ת��ΪCAN������֡,�����ͳ�ȥ
uint8_t SciToCan(void)//�����sci����Ӧ��Ϊ��������ʽ�� 
{
#ifndef ICAR_DOCK

    uint8_t i,m, TransmitMailbox, send_fail = 0;

    if(btMsg.rxLen>0)//ID���ã�������Э����ɺ�mode��ֵ��Ӧ�����ú�
    {
        proMsg.state=COMM_SEND;	//���ڷ���

        if(proMsg.mode == 0)		//proMsg.mode���������Ǳ�׼ID��0����ʽ������չID��ʽ��1�� 
        {
            TxMessage.StdId=0x7DF;		//���ñ�׼֡ 7df
			TxMessage.ExtId=0;
			TxMessage.IDE=CAN_ID_STD;
        }
        else
        {    
			TxMessage.StdId=0;                                                             
            TxMessage.ExtId=0x18DB33F1;	//������չ֡ 18db33f1
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
        	TxMessage.Data[m]=proMsg.tx[m];//�ѽ�Ҫ���͵����ݵ�ַcan.tx����canTx.CAN_Data
        
        TxMessage.DLC=8;//���ݳ�����Ϊ8
  		TxMessage.RTR=CAN_RTR_DATA;//����֡

        TimeRem=sysClock;  
        TransmitMailbox = CAN_Transmit(CAN1, &TxMessage);		
        while(CAN_TransmitStatus(CAN1, TransmitMailbox) != CANTXOK)//�ȴ�ֱ���������
        {
            if(sysClock-TimeRem>=(TIME_LIMIT*2))
            {	
				send_fail++;				
				if(send_fail>5)	  //ʧ�ܴ������࣬ȡ������
				{
					#ifdef PRINT
						printf("send fail\n");
					#endif
					send_fail = 0;
                }
				return send_fail;
            }
        }
        proMsg.state=COMM_SEND_OK;//�������
        
        for(i=0;i<=btMsg.rxLen;i++)
        {
            proMsg.tx[i]=0;//����
        }
        btMsg.rxLen=0; 
		send_fail = 0;
    }//END (btMsg.rxLen>0)
	return send_fail;
#else

	uint8_t i,len,buffer1[100],buffer[32] = {0x5A,0xA5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

    if(btMsg.rxLen>0)
    {
        proMsg.state=COMM_SEND;	//���ڷ���
        
        for(i=0;i<=btMsg.rxLen;i++)	
        {
            if(i==0)
                buffer[2]=btMsg.rxLen+1;		 //����
            else
                buffer[2+i]=btMsg.rx[i-1];		 //����	
        }
		for(i=2;i<2+buffer[2];i++)
			buffer[2+buffer[2]] += buffer[i]; 	 //sum
        
		len = SCI_GetLen(4,0);
		//printf("scican len = %d\n",len);
		if(len > 0)
			SCI_Receive(4,len,&buffer1[0]);
				
		SCI_Transmit(4,32,&buffer[0]);			 //����

        proMsg.state=COMM_SEND_OK;//�������

        btMsg.rxLen=0; 
    }//END (btMsg.rxLen>0)
	return 0;
#endif
}


