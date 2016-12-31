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
    proMsg.state=COMM_RX; //������
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
    while(SCI_GetLen(2,0)==0)//���û���յ����ݣ��ȴ���
    {
        if(sysClock-TimeRem_K>TIME_LIMIT_K)//��׼���������50ms
            return 0;
    }				
	KwpRx9141();//����һ֡���ݣ������浽proMsg��RX��
    if(proMsg.rxLen>3)
    {
        KwpToSci9141();//�����յ����ݰ��涨�ĸ�ʽת���������浽btMsg��
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
	while(SCI_GetLen(4,0) < 3)//����Ҫ�յ�3�ֽ�
	{
		if(sysClock-TimeRem_K > 2000)//2sδ�յ��ظ�
		{
			if(error14230 < 2)	 //����2��
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
	if((buffer[0] == 0x5A)&&(buffer[1] == 0xA5))//ͷ��ȷ
	{
		if(buffer[2] < 3)
			goto out;
	 	while(SCI_GetLen(4,0) < (buffer[2]+1))//����һλ
		{
			if(sysClock-TimeRem_K > 1000)
	            goto out;
		}
		SCI_Receive(4,buffer[2]+1,&buffer[3]);					//����DATA����
		
		if((buffer[3]==0x4A)&&(buffer[4]==0)&&(buffer[5]==0)&&(buffer[6]==0x4E))//���ж��Ƿ�������ECU�Ƿ�ϵ磩5A A5 04 4A 00 00 4E
			goto out;

		while(SCI_GetLen(4,0) < buffer[buffer[2]+3])
		{
			if(sysClock-TimeRem_K > 1000)
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
		for(i=0;i<buffer[3+buffer[2]];i++)
			btMsg.btHead[btMsg.ECUNumber][i] = buffer[i+buffer[2]+4];

		btMsg.BtTxState[btMsg.ECUNumber] = 1;//����״̬Ϊ���
        btMsg.FrameNum[btMsg.ECUNumber] = 1;//����֡��Ŀ
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
out:			//�������ط�һ��
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
    uint8_t i,j;//ѭ������
    uint8_t ECUSch=0;//����Ƿ�ECU�Ѿ�������������,0Ϊû�У�1λ��
	static uint8_t AddressBuff[8];                        
            
    //��Ѱ�Ƿ�ECU�Ѿ�������������
    for(i=0;i<8;i++)
    {
        if(proMsg.rx[2]==AddressBuff[i])
        {
            ECUSch=1;
            break;
        }//END (proMsg.rx[2]==AddressBuff[i])
    }
    //��������
    if(ECUSch==0)//���Ǹ�ECU�״η������ݻ�֡����
    {
        //��ECU�ĵ�ַ�ѱ����������Ա����Ƚ�ʹ��
        AddressBuff[btMsg.ECUNumber]=proMsg.rx[2];
        
        //��������ͷ;
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
        btMsg.btHeadLen[btMsg.ECUNumber]=9;//����ͷ�ĳ���       
        //��������
        for(i=0;i<proMsg.rxLen;i++)		//48 6B xx SID PID D0 D1 D2 D3 D4 D5 sum
        {
            btMsg.tx[btMsg.ECUNumber][i]=proMsg.rx[i+3];
        }
        btMsg.txLen[btMsg.ECUNumber]=proMsg.rxLen-4;//�������ݳ��ȣ�����48 6B xx��sum
        //�������״̬
        btMsg.BtTxState[btMsg.ECUNumber]=1;
        btMsg.FrameNum[btMsg.ECUNumber]=1;
        btMsg.ECUNumber+=1;
        btMsg.state=COMM_SEND_OK;       
    }
    else//ECU��֡���ݵ�����֡////////////////////////////////////////////////////////////////////////// 
    {
		for(j=3;j<proMsg.rxLen;j++)
        {
            btMsg.tx[i][j-3+btMsg.txLen[i]]=proMsg.rx[j+3];
        }
        btMsg.txLen[i]+=proMsg.rx[0]-0x80-3;//���泤��        
        //����״̬
        btMsg.FrameNum[i]+=1;
    }
}

//���������յ�����֡ת��ΪK�ߵ�����֡,�����ͳ�ȥ
void SciToKwp9141(void)
{
#ifndef ICAR_DOCK
    uint8_t i=0;       //����ѭ�����
    uint16_t sum=0; 
    
    //������
    proMsg.state=COMM_SEND; 
    
    //��������ͷ
    proMsg.tx[0]=0x68;     //����֡��ʽ�����ݶγ��� 
    proMsg.tx[1]=0x6A;                 //Ŀ���ַ
    proMsg.tx[2]=0xF1;                 //Դ��ַ
    
    //��������
    for(i=0;i<btMsg.rxLen;i++)
    {
        proMsg.tx[i+3]=btMsg.rx[i];    
    }
    proMsg.txLen=3+btMsg.rxLen;
    
    //���㲢����У���
    for(i=0;i<proMsg.txLen;i++)
    {
        sum+=proMsg.tx[i];
    }
    proMsg.tx[proMsg.txLen]=(uint8_t)sum;
    proMsg.txLen++;//����У��͵ĳ���
    SCI_Transmit(2,proMsg.txLen,&proMsg.tx[0]);//����K������֡
    btMsg.rxLen=0;//�����������ݳ��ȹ���

    //���Լ�����ȥ�������Ƚ��յ���Ȼ���ӵ�
    TimeRem_K=sysClock;
    while(SCI_GetLen(2,0)<proMsg.txLen)
    {
        if(sysClock-TimeRem_K>=TIME_LIMIT_K)
        {  
            return;//ֹͣ����ִ��
        } 
    }//END (SCI_GetLen(2,0)<proMsg.txLen)
    //�����Լ��������������Ϊû������proMsg.rxLen�������൱�ڽ��յ����ӵ���
    SCI_Receive(2,proMsg.txLen,&proMsg.rx[0]);   
    proMsg.state=COMM_SEND_OK;//������ϣ�����״̬ 
#else
	uint8_t i,len,buffer1[32],buffer[32] = {0x5A,0xA5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

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
		if(len > 0)
			SCI_Receive(4,len,&buffer1[0]);

		SCI_Transmit(4,32,&buffer[0]);			 //����

        proMsg.state=COMM_SEND_OK;//�������

        btMsg.rxLen=0; 
    }//END (btMsg.rxLen>0)
#endif   
}

//K�߽��պ�����ÿ��ֻ����һ֡����
void KwpRx9141(void)
{    
    uint8_t len;           //��ʾ���ν��յ����ݸ���
    
    //proMsg.state=COMM_RX; //������    
    
    //����һ֡���ݵĵ�һ���ֽڣ��������ݸ�ʽ�����ݳ�����Ϣ
    SCI_Receive(2,1,&proMsg.rx[0]);
    if(proMsg.rx[0] == 0x48) //��������
    {
		DelayMs(10);
        len=SCI_GetLen(2,0);
        SCI_Receive(2,len,&proMsg.rx[1]);//��������
        if((proMsg.rx[1]==0x6B)&&(proMsg.rx[3]==btMsg.rx[0]+0x40))
        {
            proMsg.rxLen=len+1;//���ó���
            proMsg.state=COMM_RX_OK;//K�߽������һ֡
        }
        else
        {
            proMsg.rxLen=0;
        }
    }
}

