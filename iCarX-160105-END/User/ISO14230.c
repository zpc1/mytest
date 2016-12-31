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
    proMsg.state=COMM_RX; //������
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
	while(SCI_GetLen(4,0) < 4)//����Ҫ�յ�4�ֽ�
	{
		if(sysClock-TimeRem_K > 2000)//2000msδ�յ��ظ�
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
	//			ECU����
	//		5A A5 xx 6 41 01 xx xx xx xx 7 7 E 8 x 0 4 x sum
	//		| ͷ |  | 	   DATA        |    header   |
	if((buffer[0] == 0x5A)&&(buffer[1] == 0xA5))//ͷ��ȷ
	{
		if(buffer[3] < 3)
			goto out;

		ecu_all = ((buffer[2] & 0xF0) >> 4) & 0x0F;
		ecu_num = (buffer[2] & 0x0F) + 1;

		printf("ecu_all = %d, ecu_num = %d,",ecu_all,ecu_num);

	 	while(SCI_GetLen(4,0) < (buffer[3]+1))//����һλ
		{
			if(sysClock-TimeRem_K > 1000)
	            goto out;
		}
		SCI_Receive(4,buffer[3]+1,&buffer[4]);					//����DATA����

		if((buffer[3]==0x4A)&&(buffer[4]==0)&&(buffer[5]==0)&&(buffer[6]==0x4E))//���ж��Ƿ�������ECU�Ƿ�ϵ磩5A A5 04 4A 00 00 4E
			goto out;

		while(SCI_GetLen(4,0) < buffer[buffer[3]+4])
		{
			if(sysClock-TimeRem_K > 1000)
	            goto out;
		}
		SCI_Receive(4,buffer[4+buffer[3]]+1,&buffer[5+buffer[3]]);//����header����

		temp = 0;
		for(i=2;i<buffer[3]+buffer[buffer[3]+4]+5;i++)			  //��У��
			temp += buffer[i];

		if((uint8_t)(temp & 0x00FF) != buffer[buffer[3]+buffer[buffer[3]+4]+5])//��У�����
			goto out;

		btMsg.txLen[btMsg.ECUNumber] = buffer[3];				//�洢���ݳ���
		for(i=0;i<buffer[3];i++)
            btMsg.tx[btMsg.ECUNumber][i] = buffer[i+4];

		btMsg.btHeadLen[btMsg.ECUNumber] = buffer[4+buffer[3]];	//�洢ͷ����
		for(i=0;i<buffer[4+buffer[3]];i++)
			btMsg.btHead[btMsg.ECUNumber][i] = buffer[i+buffer[3]+5];

		btMsg.BtTxState[btMsg.ECUNumber] = 1;//����״̬Ϊ���
        btMsg.FrameNum[btMsg.ECUNumber] = 1;//����֡��Ŀ
        btMsg.ECUNumber++;
        proMsg.state = COMM_RX_OK;

		printf("Data = ");
		for(i=0;i<20;i++)
			printf("%1X ",buffer[i]);
		printf("\n");	 	  

		if(ecu_num != ecu_all) //��ECU����
			goto out2;

		if(SCI_GetLen(4,0) > 0)
			SCI_Receive(4,SCI_GetLen(4,0),&buffer[0]);

		btMsg.rxLen = 0;

		return 1;	
	}
out:			//�������ط�һ��
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
    while(SCI_GetLen(2,0)==0)//���û���յ����ݣ��ȴ���
    {
        if(sysClock-TimeRem_K>TIME_LIMIT_K)//��׼���������50ms
            return 0;
    }				
	KwpRx();//����һ֡���ݣ������浽proMsg��RX��
    if(proMsg.rxLen>3)
    {
        KwpToSci();//�����յ����ݰ��涨�ĸ�ʽת���������浽btMsg��
		return 1;
    } 
	return 2;

#endif
}	
	   
void KwpToSci(void)
{
    uint8_t i;//ѭ������
    uint8_t j;//ѭ������
    uint8_t ECUSch=0;//����Ƿ�ECU�Ѿ�������������,0Ϊû�У�1λ��                        
            
    //��Ѱ�Ƿ�ECU�Ѿ�������������
    for(i=0;i<8;i++)
    {
        if(proMsg.rx[2] == AddressBuff[i])
        {
            ECUSch=1;
            break;
        }//END (proMsg.rx[2]==AddressBuff[i])
    }
    //��������
    if(ECUSch == 0)//���Ǹ�ECU�״η�������
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
        for(i=0;i<proMsg.rx[0]-0x80+1;i++)
        {
            btMsg.tx[btMsg.ECUNumber][i]=proMsg.rx[i+3];
        }
        btMsg.txLen[btMsg.ECUNumber]=proMsg.rx[0]-0x80;//���泤��
        
	//	printf("recv = %1X %1X %1X %1X %1X %1X %1X\n",btMsg.tx[0][0],btMsg.tx[0][1],btMsg.tx[0][2],btMsg.tx[0][3],btMsg.tx[0][4],btMsg.tx[0][5],btMsg.tx[0][6]);

        //�������״̬
        btMsg.BtTxState[btMsg.ECUNumber]=1;
        btMsg.FrameNum[btMsg.ECUNumber]=1;
        btMsg.ECUNumber+=1;
        btMsg.state=COMM_SEND_OK;       
    }
    else//ECU��֡���ݵ�����֡
    {
        //��������
        for(j=3;j<proMsg.rx[0]-0x80;j++)
        {
            btMsg.tx[i][j-3+btMsg.txLen[i]]=proMsg.rx[j+3];
        }
        btMsg.txLen[i]+=proMsg.rx[0]-0x80-3;//���泤��        
        //����״̬
        btMsg.FrameNum[i]+=1;
    }
}

//���������յ�����֡ת��ΪK�ߵ�����֡,�����ͳ�ȥ
void SciToKwp(void)
{
#ifndef ICAR_DOCK

    uint8_t i=0;       //����ѭ�����
    uint16_t sum=0; 
    
    //������
    proMsg.state=COMM_SEND; 
    
    //��������ͷ
    proMsg.tx[0]=0xC0+btMsg.rxLen;     //����֡��ʽ�����ݶγ��� 
    proMsg.tx[1]=0x33;                 //Ŀ���ַ
    proMsg.tx[2]=0xF1;                 //Դ��ַ
    //proMsg.txLen=3;                  //K�߷�������֡����
    //��������
    //proMsg.tx[3]=btMsg.rx[0];                 //���ݵĵ�һ���ֽ���Ҫ�䶯
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
    proMsg.tx[proMsg.txLen]=(UINT8)(sum);
    proMsg.txLen++;//����У��͵ĳ���

	//printf("send = %1X %1X %1X %1X %1X %1X %1X\n",proMsg.tx[0],proMsg.tx[1],proMsg.tx[2],proMsg.tx[3],proMsg.tx[4],proMsg.tx[5],proMsg.tx[6]);

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
    
    ///////////����////////////
    //SCI_Transmit(1,proMsg.txLen,&proMsg.rx[0]);
    //SendEnd();
    ///////////����/////////// 
    
    proMsg.state=COMM_SEND_OK;//������ϣ�����״̬ 

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
		if(len > 0)
			SCI_Receive(4,len,&buffer1[0]);	

		SCI_Transmit(4,32,&buffer[0]);			 //����

        proMsg.state=COMM_SEND_OK;//�������

        //btMsg.rxLen = 0; 
    }//END (btMsg.rxLen>0)

#endif   
}

//K�߽��պ�����ÿ��ֻ����һ֡����
void KwpRx(void)
{    
    uint8_t i,len;           //��ʾ���ν��յ����ݸ���
    
    //proMsg.state=COMM_RX; //������    
    
    //����һ֡���ݵĵ�һ���ֽڣ��������ݸ�ʽ�����ݳ�����Ϣ
    SCI_Receive(2,1,&proMsg.rx[0]);
    if((0x81<=proMsg.rx[0])&&(proMsg.rx[0]<=0x87))//��������
    {
        //proMsg.rxLen=1;
        len=proMsg.rx[0]-0x80+3;
        
        //�������������ݣ������������ֽڵĵ�ַ��Ϣ��һ���ֽڵ�У���
        while(SCI_GetLen(2,0)<len)
        {
            if(sysClock-TimeRem_K>=TIME_LIMIT_K)
            {  
                return;//ֹͣ����ִ��
            }
        }//END (SCI_GetLen(2,0)<len)
        SCI_Receive(2,len,&proMsg.rx[1]);//��������
		
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
            proMsg.rxLen=len+1;//���ó���
            proMsg.state=COMM_RX_OK;//K�߽������һ֡
        }
        else
        {
            proMsg.rxLen=0;
        }
    }
}

