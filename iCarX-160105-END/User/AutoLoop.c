/**
  ******************************************************************************
  * @file    AutoLoop.c 
  * @author  PDAger iCar team
  * @version V0.5.0
  * @date    12-August-2011
  * @brief   Main loop.
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "AutoLoop.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern uint8_t PID_cfg[], WqData[], WqFlag;
extern uint32_t TimeWq, TimeWqLimit, TimesWqLimit;
extern uint32_t sysClock,KComRem;
extern MSG_COMM proMsg;	
extern MSG_COMM_BT btMsg;
extern ECUstat car_state;
extern uint16_t first_go,time_m;
extern CARprotocol sysMode;
extern PIDsupport support,supptemp,frame_support;
extern uint8_t loop_time;
extern uint8_t first,DTC_num;
extern uint16_t DTC_temp,DTC_now[];
extern uint8_t led_flag;
extern uint8_t write_flag;
extern uint8_t dtc_dog;
extern uint32_t speed;
extern MSG_COMM_GPRS 	GPRS;
extern bool OBDLoopFlag;

/*******************************************С��***************************************/
extern uint32_t Ignition_times;

/*******************************************С��***************************************/

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : OBD_Main
* Input          : None
* Output         : None
* Return         : None
* Description    : �����ɼ�ѭ�������ݳ���״̬���ж������ж��µĳ���״̬
					Ϩ����Э��
					�ϵ磺��Э�鵫������ת�٣�PID��010C��Ϊ��
					���٣�������ת�ٲ�Ϊ�㵫���٣�PID��010D��Ϊ��
					��ʻ�����ٲ�Ϊ��
*******************************************************************************/
extern uint32_t send,recv;
void OBD_Main(void)
{
	K_Reminder();		//K������
	
	if(OBDLoopFlag == 0)
		return;
	
	switch(car_state)			//���ݳ���״̬����
	{
		case ECU_OFF:				//����״̬��Ϩ��׼����Ѱ��������Э�� 
					if(time_m>=2)				   	//�ȴ�10s��һ��Э��
					{						
						ProtocolSearch();		   	//����Э��
						printf("AutoLoop: search sysMode = %d\n",sysMode);
						if(sysMode == NONE)			//û���ѵ�Э��
						{
							printf("AutoLoop: search failed\n");
							//printf("send recv = %d, %d\n",send,recv);
							car_state = ECU_OFF;   //��Э�飬��ΪϨ��״̬
							time_m = 0;			   //�����ȴ�10s
						}
						else					   //�ѵ�Э����
							car_state = ECU_ON;	   //Ϩ�𡪡�>�ϵ�
					}	
				break;

		case ECU_ON:				//����״̬���ϵ磬׼���ռ�����ECU֧�ֵ�PID��֧���룩 
					if((support.v0100 == 0)&&(support.v0120 == 0)&&(support.v0140 == 0)&&(loop_time == 0))	
						first = 0;			//��һ�ο�������������֧����		
					if(loop_time <= 2)		//�Ѽ���ʶ��loop_time��0->1->2->3
						PID_check();		//Ѱ��֧������
					else
						car_state = state_judge();//֧����������ϣ���ʼ�жϳ���״̬����Ҫ�Ƿ�����ת�ټ����٣�	
					if(car_state >= IDLE)
					{
						WqFlag = 0;
						Ignition_times++;
					}		
				break;
		case IDLE:								//IDLE��DRIVING���������ͬ�����ǲɼ�����������
		case DRIVING:
					 if((support.SID_now == 0)&&(support.PID_now == 0))//��ֹPIDֵ�쳣��0101~05FF��
					 {
					 	support.SID_now = 1; //����SID
						support.PID_now = 1; //����PID
					 }					 
					 Cycle_Main(support.SID_now,support.PID_now);//��̬�����복��״̬ѭ��					 
				break;
		default: car_state = ECU_OFF;
				break;
	}	
}
/*******************************************************************************
* Function Name  : Cycle_Main
* Input          : SID��PID
* Output         : None
* Return         : None
* Description    : ��̬�����복��״̬ѭ�������òɼ������溯�����жϳ���״̬�����١���ʻ�����������ѣ�����ƽ��
*******************************************************************************/
void Cycle_Main(uint8_t SID,uint8_t PID)
{
	static uint8_t i,j,k;
	
	if(Main_send(SID,PID) == 0)//���÷�������պ������ɼ���̬���ݣ���Ϩ���򷵻���
	{
		car_state = ECU_OFF;//Ϩ����
		sysMode = NONE;		//Э���ʶ  
	}

	if((btMsg.tx[0][1] == 0x0C)&&(btMsg.tx[0][2] == 0)&&(btMsg.tx[0][3] == 0))//������ת�٣�010C��Ϊ0
	{
		car_state = ECU_ON;//��𡪡�>�ϵ�
	}	
	if((btMsg.tx[0][1] == 0x0D)&&(btMsg.tx[0][2] == 0)&&(car_state == DRIVING))//���٣�010D��Ϊ0����ʻing��
		car_state = IDLE;			//����
	if((btMsg.tx[0][1] == 0x0D)&&(btMsg.tx[0][2] > 0)&&(car_state == IDLE))//���٣�010D������0������ing��
		car_state = DRIVING;		//��ʻ											 
	if(car_state >= IDLE)//��������ʱ					
		DynInfCop();	 //PID����������һ��PID��OBD����ϣ������洢����
	
	if(WqFlag == 1)//β�����
	{
		if((btMsg.tx[0][1] > 0x13)&&(btMsg.tx[0][1] < 0x20))
		{
			i++;WqData[i] = btMsg.tx[0][1];
			i++;WqData[i] = btMsg.tx[0][2];
			if(btMsg.txLen[0] > 3)
			{i++;WqData[i] = btMsg.tx[0][3];}
			if(btMsg.txLen[0] > 4)
			{i++;WqData[i] = btMsg.tx[0][4];}
			if(btMsg.txLen[0] > 5)
			{i++;WqData[i] = btMsg.tx[0][5];}
			j++;
		}
		if(j >= TimesWqLimit)
		{
			WqData[0] = i;
			WqFlag = 0;
			PIDFileRead("PID.cfg");
		}	
	}
	else
	{
		BLEAutoBack();
		if(((k == 1)&&((sysClock - TimeWq) > TimeWqLimit))||((k == 0)&&((sysClock - TimeWq) > 10000)))
		{
			TimeWq = sysClock;
			WqFlag = 1;
			i=0;j=0;k=1;
		//	PIDFileRead("PID01.cfg");
		//	printf("weiqi jiance\n");
		}
	}
}
/*******************************************************************************
* Function Name  : DynInfCop
* Input          : None
* Output         : None
* Return         : None
* Description    : ��̬����PID�������������붳��֡PID����
*******************************************************************************/
void DynInfCop(void)	  
{	
	uint8_t i,j,ECU_flag,point,point_temp,freeze_PID_temp;

	if(support.SID_now == 0)		//��ʹ֪��û���ܣ�����д����
		support.SID_now = 1;

	if(support.SID_now == 1)		//SID = 01������̬����ѭ��
	{
		if(btMsg.tx[0][1] == 1)		//����0101������ҹ��ϵ�MIL
		{
			for(i=0;i<btMsg.ECUNumber;i++)		//�ѷ������ݣ���ʼ����
			{
				if(btMsg.tx[i][2] >= 0x80) 		//MILΪ1�����й���
				{
					support.SID_now = 3;		//׼����SID = 03��������ѭ����
					support.PID_now = 0;		//��ʵPID�ǲ�����
					dtc_dog = 1;				//׼��ι������ֹͻȻ�䳵��Ϩ�������
				}
			}
			if(support.SID_now == 3)			//�й���
			{
				led_flag = 2;
				LED1_ON							//���ϵ���~
			}
			else								//�޹���
			{
				DTC_num = 0;
				LED1_OFF						//���ϵ���~
			}  
		}
		if(support.SID_now == 1)				//�޹��ϣ��������̬����ѭ��
		{
			dtc_dog = 0;						//����Ҫ��������
			support.PID_now = PID_chose(&support);//����֧�����뵱ǰPID������һ��PID
		}
	}//��̬����PID���ҽ���
	else if(support.SID_now == 7)//07ż�����ϲ�ѯ
	{
		point = 1;				//����������ʶ
		DTC_now[0] = 0;
		for(i=0;i<btMsg.ECUNumber;i++)//�Ѿ�����07������ԭʼ������ȡ������
		{
			point_temp = point;//���ö�ECU����ָ��
			if(btMsg.tx[i][1] > 0)//43 06 0143 0196 0234 02CD 0357 0A24
			{					  
				if((sysMode >= CAN_STD_500) && (sysMode <= CAN_EXT_250))//CAN����ȡ
				{
					printf("AutoLoop: 07 dtc btMsg.len = %d,btMsg.tx = ",btMsg.txLen[i]);
					for(j=0;j<20;j++)
						printf("%1X ",btMsg.tx[i][j]);
					printf("\n");
					for(j = point_temp;j<100;j++)
					{  
						DTC_now[j] = ((uint16_t)btMsg.tx[i][1+(j-point_temp)*2] << 8) + btMsg.tx[i][2+(j-point_temp)*2];//ԭʼ������
						point++;								   //���ϸ���+1
						if((j - point_temp + 1) >= ((btMsg.txLen[i]-1)/2))//���������������ݣ�btMsg.txLen[i]�����ݳ��ȣ�
							break; //����ѭ��
					}
				}
				else if((sysMode >= K_9141_5) && (sysMode <= K_14230_fast))//K�ߣ�����ͬ��
				{
					for(j = point_temp;j<100;j++)
					{  
						DTC_now[j] = ((uint16_t)btMsg.tx[i][1+(j-point_temp)*2] << 8) + btMsg.tx[i][2+(j-point_temp)*2];//ԭʼ������
						point++;
						if((j - point_temp + 1) >= ((btMsg.txLen[i]-1)/2))
							break;
					}
				} 				
			}
			DTC_now[0] = point-1;	//��������
		}							
		if(DTC_now[0] > 0)//��ż������
		{					
			printf("AutoLoop: 07 DTC_now = ");
			for(j=0;j<20;j++)
				printf("%1X ",DTC_now[j]);
			printf("\n");		
			DTC_num = 2;
			AT_master_BLE(1);			//�������͹�����						
		}
		else
			DTC_num = 0;

		support.PID_now = 1;//ż������������ϣ�����������̬����
		support.SID_now = 1;
	}
	else 			//03���ù�����Ͷ���֡ѭ��
	{
		if(support.SID_now == 3)//�Ѿ�����03����������룬׼����020200�ɼ�����֡
		{
			point = 1;
			DTC_now[0] = 0;
			for(i=0;i<btMsg.ECUNumber;i++)//ͬż������
			{
				point_temp = point;
				if(btMsg.tx[i][1] > 0)//43 0143 0196 0234 02CD 0357 0A24
				{				 
					if((sysMode >= CAN_STD_500) && (sysMode <= CAN_EXT_250))
					{
				/*		printf("AutoLoop: 03 dtc btMsg.len = %d,btMsg.tx = ",btMsg.txLen[i]);
						for(j=0;j<20;j++)
							printf("%1X ",btMsg.tx[i][j]);
						printf("\n");*/
						for(j = point_temp;j<100;j++)
						{  
							DTC_now[j] = ((uint16_t)btMsg.tx[i][1+(j-point_temp)*2] << 8) + btMsg.tx[i][2+(j-point_temp)*2];//ԭʼ������
							point++;
							if((j - point_temp + 1) >= ((btMsg.txLen[i]-1)/2))
								break;
						}
					}
					else if((sysMode >= K_9141_5) && (sysMode <= K_14230_fast))
					{
						printf("AutoLoop: 03 dtc btMsg.len = %d,btMsg.tx = ",btMsg.txLen[i]);
						for(j=0;j<20;j++)
							printf("%1X ",btMsg.tx[i][j]);
						printf("\n");
						for(j = point_temp;j<100;j++)
						{  
							DTC_now[j] = ((uint16_t)btMsg.tx[i][1+(j-point_temp)*2] << 8) + btMsg.tx[i][2+(j-point_temp)*2];//ԭʼ������
							point++;
							if((j - point_temp + 1) >= ((btMsg.txLen[i]-1)/2))
								break;
						}
					}			
				} 
				DTC_now[0] = point - 1;
			}		
			if(DTC_now[0] > 0)//�й���
			{	
	/*			printf("AutoLoop: 03 DTC_now = ");
				for(j=0;j<20;j++)
					printf("%1X ",DTC_now[j]);
				printf("\n");*/
				//GPRS_DTC_message();
				AT_master_BLE(0);				//�������͹�����
				DTC_num = 1;
				LED1_ON	//���ϵ���~										  
			}
			if(write_flag == 1)					//�ù����Ǵ�Ŷ���ף���Ҫ�����忴����֡
			{	
				printf("AutoLoop: ready to collect freeze data\n");		
				support.SID_now = 2;
				support.PID_now = 2;			//020200���鿴���޶���֡��PID
			}
			else								//���Ǵ������Ѿ��������֡��������ˣ�����
			{
				dtc_dog = 0;					//��˵�˲��Ǵ��ˣ������ؼҰ�
				support.SID_now = 1;			//�ؼҼ����ɶ�̬����
				support.PID_now = 1;
				support.PID_now = PID_chose(&support);//�ղɵ�0101����0101�����Ǹ�PID��
			}
		}
		else if(support.SID_now == 2)//�Ѳ�������벢�ж���֡����ʼ�ɼ�����֡
		{
			if(support.PID_now == 2)		   //����020200,�������޶���֡
			{
				for(i=0;i<btMsg.ECUNumber;i++) 
				{
					DTC_temp = ((uint16_t)btMsg.tx[i][3] << 8) + btMsg.tx[i][4];//���𶳽�֡�Ĺ�����
					printf("AutoLoop: freeze dtc = %1X\n",DTC_temp);
					if(DTC_temp != 0)//�ж���֡
					{
						support.SID_now = 2;   //�ж���֡������020000�ɼ�����֧֡����
						support.PID_now = 0;
						ECU_flag = i;		  //�洢ECU��
						break;
					}
					else
					{
						dtc_dog = 0;
						support.SID_now = 1;   //�޶���֡�������ɼ���̬����
						support.PID_now = 0;
					}
				}	
			}
			else if((support.PID_now == 0)||(support.PID_now == 0x20)||(support.PID_now == 0x40))//����020000,��ʼѰ�Ҷ����PID
			{
				if(support.PID_now == 0)//����֧֡��020300~021F00
				{
					frame_support.v0100 = ((btMsg.tx[ECU_flag][3] << 24)|(btMsg.tx[ECU_flag][4] << 16)|(btMsg.tx[ECU_flag][5] << 8)|(btMsg.tx[ECU_flag][6]));
					printf("AutoLoop: freeze support 00 = %1X\n",frame_support.v0100);
					if((frame_support.v0100 & 0x01) == 1)//֧��022000
						support.PID_now = 0x20;
					else								   //��֧��022000������Ҫ�����ɼ�֧������
						support.PID_now = 1;
				}
				else if(support.PID_now == 0x20)//����֧֡��022000~023F00
				{
					frame_support.v0120 = ((btMsg.tx[ECU_flag][3] << 24)|(btMsg.tx[ECU_flag][4] << 16)|(btMsg.tx[ECU_flag][5] << 8)|(btMsg.tx[ECU_flag][6]));									
					printf("AutoLoop: freeze support 20 = %1X\n",frame_support.v0120);
					if((frame_support.v0120 & 0x01) == 1)//֧��024000
						support.PID_now = 0x40;
					else
						support.PID_now = 1;
				}
				else if(support.PID_now == 0x40)//����֧֡��024100~025F00
				{
					frame_support.v0140 = ((btMsg.tx[ECU_flag][3] << 24)|(btMsg.tx[ECU_flag][4] << 16)|(btMsg.tx[ECU_flag][5] << 8)|(btMsg.tx[ECU_flag][6]));									
					printf("AutoLoop: freeze support 40 = %1X\n",frame_support.v0140);
					support.PID_now = 1;
				}
				if(support.PID_now == 1)//02֧��������ϣ���ʼ��ʽ�ɼ�����֡
				{
					frame_support.PID_now = 2; //��ʼ��PID����020200��ʼ�ɼ�
					frame_support.SID_now = 2;
					support.PID_now = PID_chose(&frame_support);//��һ����Ҫ���͵Ķ���PID
					if(support.PID_now > 0x5F)//��һ��PID������Χ��һ�㲻�����
					{
						support.PID_now = PID_chose(&support);//�޶���֡
						support.SID_now = 1;   
						return;
					}
				}
			}
			else  //����֡�ռ�	 020100 ~ 025F00
			{	
				freeze_PID_temp = support.PID_now;				
				frame_support.PID_now = support.PID_now;
				support.PID_now = PID_chose(&frame_support);//��һ����Ҫ���͵Ķ���PID	
				printf("AutoLoop: freeze collect 02 %1X 00\n",support.PID_now);					   															
				if(support.PID_now < freeze_PID_temp)//���������еĶ���֡
				{
					support.SID_now = 1;   	//����֡�ɼ���ϣ������ɼ���̬����
					support.PID_now = PID_chose(&support);
					dtc_dog = 0;		//ι�����					
				}
			}									
		}
	}
}
/*******************************************************************************
* Function Name  : PID_check
* Input          : None
* Output         : ֧����
* Return         : None
* Description    : ������״̬Ϊ�ϵ�ʱ��֧����ɼ�����̬����or����֡��
					first��0->20->40->2->1
*******************************************************************************/
void PID_check(void)		   
{
	static uint8_t num = 0;
	uint8_t i;	
	
/*	printf("first = %d, loop_time = %d, data = ",first,loop_time);
	for(i=0;i<8;i++)
		printf("%1X ",btMsg.tx[0][i]);
	printf("\n");	 
*/
	if(first != 1)			 	//׼���ɼ�ECU֧�ֵ�PID��Ϣ��first��֧����ɼ�������ʶ��Ϊ1���Ѿ��ɼ����������ֵ��
	{
		if(first != 2)		  		//�Ȼ��֧����Ϣ��Ϊ2���Ѿ��ɼ����ˣ�		
		{
			loop_time = 1;			//ѭ���ɼ���ʶ��֧����ѭ����������������Ӱ�죩

			if(0 == Main_send(0x01,first))//����0100,0120,0140������0��Ϩ���ˣ�������			
			{
				support.v0100 = 0;
				support.v0120 = 0;
				support.v0140 = 0;
				loop_time = 0;
				first = 0;
			}		
 
			for(i=0;i<btMsg.ECUNumber;i++)
			{
				if(btMsg.tx[i][1] == 0x00)//�յ�0100�Ļظ�����0101~0120��֧����
				{					
					supptemp.v0100 |= ((btMsg.tx[i][2] << 24)|(btMsg.tx[i][3] << 16)|(btMsg.tx[i][4] << 8)|(btMsg.tx[i][5]));
					if((btMsg.tx[i][5] & 0x01) == 1)//�б�Ҫ�����ɼ�0120�Ժ�ģ�
						first = 0x20;
					else
						first = 2;
				}
				else if(btMsg.tx[i][1] == 0x20)//�յ�0120�Ļظ�����0121~0140��֧����
				{
					supptemp.v0120 |= ((btMsg.tx[i][2] << 24)|(btMsg.tx[i][3] << 16)|(btMsg.tx[i][4] << 8)|(btMsg.tx[i][5]));
					if((btMsg.tx[i][5] & 0x01) == 1)//�б�Ҫ�����ɼ�0140�Ժ�ģ�
						first = 0x40;
					else
						first = 2;
				}
				else if(btMsg.tx[i][1] == 0x40)//�յ�0140�Ļظ�����0141~015F��֧����
				{
					supptemp.v0140 |= ((btMsg.tx[i][2] << 24)|(btMsg.tx[i][3] << 16)|(btMsg.tx[i][4] << 8)|(btMsg.tx[i][5]));			
					first = 2;//ȫ���ɼ���ϣ�׼�������Ƿ�ȫ����ֵ			
				}
			}
		}
		else						   	//ѭ���ɼ�һ�Σ���������ֵ
		{	
			num = PID_chose2(&supptemp);	
			if((num == 1)||(loop_time == 1))//ѭ��1�����
				loop_time++;
			if(loop_time > 3)	   //�Ѿ�ѭ��2�Σ�������ϣ��˳�
			{
				first = 1;		   //֧����ȫ��ok��׼������
				loop_time = 0;
				return;	
       }
			if(0==Main_send(0x01,num))//��֧�������л�ȡ����PID���ɼ����ݣ�����0��Ϩ����
			{
			/*	support.v0100 = 0;
				support.v0120 = 0;
				support.v0140 = 0;
				loop_time = 0;
				first = 0;*/
			}
			for(i=0;i<btMsg.ECUNumber;i++)
			{
				if(btMsg.tx[i][0] != 0x7F)		//7F����ֵ����֧������delete
				{
					if(led_flag == 0)
						led_flag = 1;
					if(btMsg.tx[i][1] < 0x20)	//֧�����ݱ��浽support��
						support.v0100 |= (1 << (uint32_t)(32 - btMsg.tx[i][1]));
					else if(btMsg.tx[i][1] < 0x40)
						support.v0120 |= (1 << (uint32_t)(32 - (btMsg.tx[i][1] - 0x20)));
					else
						support.v0140 |= (1 << (uint32_t)(32 - (btMsg.tx[i][1] - 0x40)));			
				}
			}			
		}		
	}
}
/*******************************************************************************
* Function Name  : state_judge
* Input          : None
* Output         : ����״̬
* Return         : ECUstat
* Description    : ���ϵ�ʱ���жϳ���״̬
*******************************************************************************/
ECUstat state_judge(void)
{
	uint32_t speed = 0;
	
	if(sysMode == NONE)
	 	return ECU_OFF;
	else
	{
		if(0==Main_send(0x01,0x0C))
			return ECU_OFF;	 //����010C������ת��
		speed = ((uint32_t)btMsg.tx[0][2] << 8)+btMsg.tx[0][3];	//������ת��
		if(speed == 0)
			return ECU_ON;		 //������Ϊ0
		Main_send(0x01,0x0D);	 //����010D����
		speed = btMsg.tx[0][2];	 //����
		if(speed == 0)
		{
			return IDLE;		 //��������Ϊ0������Ϊ0
		}
	}
	return DRIVING;
}
/*******************************************************************************
* Function Name  : PID_chose
* Input          : PID
* Output         : PID
* Return         : PID
* Description    : ���ݵ�ǰPID��֧����ѡ����һ��PID����̬����or����֡��
*******************************************************************************/
uint8_t PID_chose(PIDsupport *supp)
{
	static uint8_t first_read,PID_times,temp,weiqi_times;
	static uint8_t PID_map[10][9] ={{0,0,0,0,0,0,0,0,0},
																	{0,0,0,0,1,0,0,0,0},
																	{0,0,0,1,0,0,0,1,0},
																	{0,0,1,0,0,1,0,0,1},
																	{0,1,0,1,0,1,0,1,0},
																	{1,0,1,0,1,0,1,0,1},
																	{1,1,0,1,1,0,1,1,0},
																	{1,1,1,0,1,1,1,0,1},
																	{1,1,1,1,0,1,1,1,1},
																	{1,1,1,1,1,1,1,1,1}};//3:1/3,5:1/2,9:1
	uint8_t count = 0,num = 0;

	if(WqFlag == 1)
	{	
		weiqi_times++;
		if(weiqi_times <= (TimesWqLimit/2))
		{
			supp->PID_now = 0x14;
			return 0x14;
		}
		else
		{
			supp->PID_now = 0x15;
			return 0x15;
		}
	}
	else
		weiqi_times = 0;

	if(first_read != 1)
	{
		first_read = 1;
		PID_times = 0;
		PIDFileRead("PID.cfg");
	}	

	count = supp->PID_now;	//��ǰPID			

	if((supp->v0100 == 0)&&(supp->v0120 == 0)&&(supp->v0140 == 0)&&(supp->SID_now == 1))//ѹ����û�ɹ�֧������ʧ�󣨶ϵ磩��
	{
		car_state = ECU_OFF;
		return 0;
	}

retry:	
	temp = count;
	num = 0;
	while(num == 0)//ѭ�����ң�ò�����������ã�
	{
		count++;				//01~5A
		if((count == 0x20)||(count == 0x40))//����0120��0140
			count++;
		if(count > 0x5A)					//���ᳬ��015A
		{
			count = 0;
			//supp->SID_now = 7;	//��07�ɼ�ż������ 
		}
		if(count < 0x20)
			num = ((supp->v0100 & ((uint32_t)(1<<(32-count)))) >> (32-count));	//0101~011F
		else if(count < 0x40)
			num = ((supp->v0120 & ((uint32_t)(1<<(32-(count-0x20))))) >> (32-(count-0x20))); //0121~013F
		else
			num = ((supp->v0140 & ((uint32_t)(1<<(32-(count-0x40))))) >> (32-(count-0x40))); //0141~015A
	}

	supp->PID_now = count;			

	if(supp->SID_now == 1)
	{
		if(count < temp) //ѭ��һ��
		{
			PID_times++;
			if(PID_times > 8)
				PID_times = 0;
		}
	
		if(PID_map[PID_cfg[count]][PID_times] == 0)	  //δ���ɼ�Ƶ��
			goto retry;
	}

	return count;
}

uint8_t PID_chose2(PIDsupport *supp)
{
	static uint8_t first_read,PID_times,temp;
	uint8_t count = 0,num = 0;

	count = supp->PID_now;	//��ǰPID			

	temp = count;
	num = 0;
	while(num == 0)//ѭ�����ң�ò�����������ã�
	{
		count++;				//01~5A
		if((count == 0x20)||(count == 0x40))//����0120��0140
			count++;
		if(count > 0x5A)					//���ᳬ��015A
		{
			count = 0;
			//supp->SID_now = 7;	//��07�ɼ�ż������ 
		}
		if(count < 0x20)
			num = ((supp->v0100 & ((uint32_t)(1<<(32-count)))) >> (32-count));	//0101~011F
		else if(count < 0x40)
			num = ((supp->v0120 & ((uint32_t)(1<<(32-(count-0x20))))) >> (32-(count-0x20))); //0121~013F
		else
		{
			num = ((supp->v0140 & ((uint32_t)(1<<(32-(count-0x40))))) >> (32-(count-0x40))); //0141~015A
			//printf("count = %1X, num = %1X\n",count,num);
		}
	}

	supp->PID_now = count;			

	return count;
}
/*******************************************************************************
* Function Name  : Main_send
* Input          : SID��PID
* Output         : None
* Return         : status
* Description    : ����PID���������
*******************************************************************************/
uint8_t Main_send(uint8_t SID,uint8_t PID)//���ͽ��պ���
{
	uint8_t i;

    //״̬��λ
    for(i=0;i<8;i++)
    {
        btMsg.btHeadLen[i]=0;
        btMsg.BtTxState[i]=0;
        btMsg.FrameNum[i]=0;
        btMsg.txLen[i]=0;
    }
    btMsg.ECUNumber=0;
		
	btMsg.rx[0] = SID;			 //btMsg.rx����Ҫ�ɼ���SID��PID�ţ���ISO15765Main��ISO14230Main�е��÷���
	btMsg.rx[1]	= PID;
	btMsg.rxLen = 2;			 //���ݳ���2

	if((SID == 3)||(SID == 7))
	{
    	btMsg.rxLen = 1;	//03 or 07�����ݳ���1
		btMsg.rx[1]	= 0;
	}
	else if(SID == 2)
    	btMsg.rxLen = 3;	//020200 or 020000 or 02xx00   ���ݳ���3
	else if(SID == 1)
    	btMsg.rxLen = 2;	//01 xx	���ݳ���2
 
    if(sysMode == NONE)		//���ݳ���Э�鷢����ɼ�
		car_state = ECU_OFF;
    else if((CAN_STD_500 <= sysMode) && (sysMode <= CAN_EXT_250)) //CAN
	{		
		return ISO15765Main();      //���������      
	}
	else if((sysMode == K_14230_5) || (sysMode == K_14230_fast))//K-14230
    {
        KComRem=sysClock;
        return ISO14230Main();
    }
	else if(sysMode == K_9141_5)	//K-9141
    {
        KComRem=sysClock;
        return ISO9141Main();
    } 
	return 0;
}

/*******************************************************************************
* Function Name  : PID_file_backup
* Input          : SID��PID
* Output         : None
* Return         : status
* Description    : PID��Ƶ�ĵ�����
*******************************************************************************/
void PIDFileRead(char* PIDnum)
{
	FIL file;
	UINT rc;
	uint16_t res;
	char PID[] = "PID.cfg";
	char PID01[] = "PID01.cfg";
	uint8_t PID_data[] =   "0333955353999999993355555555000303995555555599530353555555555555035909399999555033355555599";	
	uint8_t PID01_data[] = "0000000000000000000099000000000000000000000000000000000000000000000000000000000000000000000";	 
	uint8_t PID_temp[100],i,j;
	
	for(i=0;i<100;i++)
		PID_temp[i] = 0;
	f_chdir("/");
	res = f_chdir("BACKUP");
	if(res == FR_OK)				//���������ļ���
	{
		res = f_open(&file, PIDnum, FA_OPEN_EXISTING | FA_READ);//�򿪺�̨�����ļ�
		while((res != FR_OK)||(file.fsize < 10))
		{
			f_open(&file, PID, FA_CREATE_NEW | FA_WRITE);//�򿪺�̨�����ļ�
			f_write(&file, PID_data, 91, &rc);
			f_close(&file);
			f_open(&file, PID01, FA_CREATE_NEW | FA_WRITE);//�򿪺�̨�����ļ�
			f_write(&file, PID01_data, 91, &rc);
			f_close(&file);
			printf("AutoLoop: PID file restore ok\n");	
			res = f_open(&file, PIDnum, FA_OPEN_EXISTING | FA_READ);
		}
		f_read(&file, &PID_temp, file.fsize, &rc);//���ļ�				
		j = 0;
		for(i=0;i<file.fsize;i++)
		{
			if(PID_temp[i] >= '0')
			{						
				PID_cfg[j] = PID_temp[i] - '0';
				j++;
			}
		}
		f_close(&file);
	}
	f_chdir("/");	
}
