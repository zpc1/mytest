/**
  ******************************************************************************
  * @file    SIM900A.c 
  * @author  PDAger iCar team
  * @version V0.5.0
  * @date    20-October-2011
  * @brief   GPRS init.
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "SIM900A.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
extern MSG_COMM proMsg;	
extern MSG_COMM_BT btMsg;
extern uint32_t TimeRem_SIM;
extern uint32_t sysClock;
extern ECUstat car_state;
extern uint8_t IPR_flag;
extern uint16_t Soft_version[2];
extern MSG_COMM_GPRS GPRS;
extern uint8_t ID_CFG[5];
extern uint8_t led_flag;
extern CARprotocol sysMode;
extern uint16_t DTC_now[20];
/* Private variables ---------------------------------------------------------*/
uint8_t AT[]		="AT";				//�����ź�
uint8_t ATE0[]		="ATE0";			//�رջ���
uint8_t ATV1[]		="ATV1";			//���������OK/V1
uint8_t AT_IPR[]    ="AT+IPR=9600";
uint8_t AT_CSQ[] 	="AT+CSQ";			//��ѯ�ź�

uint8_t AT_CIPSTART[50]	="AT+CIPSTART=";//���ӷ�ʽ\���ӵ�ַ\Ŀ��˿�
uint8_t AT_CIPSEND[15]	="AT+CIPSEND=";	//GPRS������������
uint8_t AT_CIPSTATUS[]	="AT+CIPSTATUS";//TCP����״̬
uint8_t AT_CLPORT[]		="AT+CLPORT=\"UDP\",2001"; 

uint8_t AT_CPOWD[] = "AT+CPOWD=1";		//ģ��ػ�����Ҫ��λPOWKEY���ܿ�����
uint8_t AT_CGATT[] = "AT+CGATT=1";		//GPRS����
uint8_t AT_CGREG[] = "AT+CGREG?";		//����ע��״̬
uint8_t AT_CGREG_SET[] = "AT+CGREG=2";	//��ѯע��״̬ʱ���ػ�վ��Ϣ

uint8_t AT_CMGD[] = "AT+CMGD=0,4";	//ɾ�����ж���
uint8_t AT_CNMI[] = "AT+CNMI?";			//��ѯ����
uint8_t AT_CMGF1[] = "AT+CMGF=1";	   	//�����ı�ģʽ
uint8_t AT_CSCS[] = "AT+CSCS=\"GSM\"";
uint8_t AT_CMGS[] = "AT+CMGS=\"+8610086\"";
uint8_t AT_CMGS_me[] = "AT+CMGS=\"+8615201457677\"";
uint8_t AT_CMGR[] = "AT+CMGR=1";
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : SIM900A_Init
* Input          : None
* Output         : None
* Return         : None
* Description    : SIM900A��MC323A��ʼ������
*******************************************************************************/
uint8_t SIM900A_Init(void)
{
	uint8_t endchar[] = {0x0D,0x0A};  
	FIL file;
	UINT rc;
	uint8_t error_flag,IP_read[50];
	uint16_t i,res;	 
	char backup_IP[] = "IP.cfg";	//��̨IP�ļ�
	char backup_ID[] = "ID.cfg";	//�豸ID�ļ�
	char backup_SV[] = "SV.cfg";	//����汾�ļ�
	char backup_IPR[] = "IPR.tmp";	//�����ʳ�ʼ����ʱ�ļ�

	f_chdir("/");
	res = f_chdir("BACKUP");
	if(res == FR_OK)				//���������ļ���
	{
		res = f_open(&file, backup_IP, FA_OPEN_EXISTING | FA_READ);//�򿪺�̨�����ļ�
		if((res == FR_OK)&&(file.fsize > 20))
		{
			for(i=12;i<50;i++)
				AT_CIPSTART[i] = 0;				//����AT��������
			f_read(&file, &IP_read, file.fsize, &rc);//���ļ�
			for(i=0;i<file.fsize;i++)
				AT_CIPSTART[i+12] = IP_read[i];	//����AT�����дIP
			f_close(&file);
		}

		res = f_open(&file, backup_ID, FA_OPEN_EXISTING | FA_READ);//���豸�������ļ�
		if((res == FR_OK)&&(file.fsize == 4))
		{
			f_read(&file, &ID_CFG, file.fsize, &rc);
			f_close(&file);
		}

		res = f_open(&file, backup_SV, FA_OPEN_EXISTING | FA_READ);//������汾�ļ�
		if((res == FR_OK)&&(file.fsize == 2))
		{
			f_read(&file, &Soft_version, file.fsize, &rc);
			f_close(&file);
		}
		else
		{
			f_write(&file, &Soft_version, 2, &rc);
			f_close(&file);
		}
	}
	f_chdir("/");

	printf("SIM900A: server ip ");
	puts((char *)AT_CIPSTART);//������
	printf("\n");
	
	for(i=13;i<50;i++)
	{
		if(AT_CIPSTART[i] == 0)
		{
			AT_CIPSTART[i] = 0x0D;	 //������AT�������ϻس���
			AT_CIPSTART[i+1] = 0x0A;			
			break;
		}	
	}						

	printf("SIM900A: init..\n");
	error_flag = 0;

	error_flag += SIM900A_CMD(AT);				//ͬ��ָ��AT			
	error_flag += SIM900A_CMD(ATE0);			//�����������		
	error_flag += SIM900A_CMD(ATV1);			//�����ظ���ʽ

	if(IPR_flag == 1)				//��Ҫ���ò����ʣ���usart3_init�ﶨ�壩
	{
		printf("SIM900A: baudrate init..\n");
		
		while(!USART_GetFlagStatus(USART3, USART_FLAG_TXE));
		USART_SendData(USART3, AT[0]);
		while(!USART_GetFlagStatus(USART3, USART_FLAG_TXE));
		USART_SendData(USART3, AT[1]);
		while(!USART_GetFlagStatus(USART3, USART_FLAG_TXE));
		USART_SendData(USART3, endchar[0]);
		while(!USART_GetFlagStatus(USART3, USART_FLAG_TXE));
		USART_SendData(USART3, endchar[1]);

		DelayMs(1000);
		SIM900A_Rx(0);

		for(i=0;i<11;i++)
		{	
			while(!USART_GetFlagStatus(USART3, USART_FLAG_TXE));
			USART_SendData(USART3, AT_IPR[i]);
		}
		while(!USART_GetFlagStatus(USART3, USART_FLAG_TXE));
		USART_SendData(USART3, endchar[0]);
		while(!USART_GetFlagStatus(USART3, USART_FLAG_TXE));
		USART_SendData(USART3, endchar[1]);

		DelayMs(1000);

		printf("SIM900A: buadrate send ok\n");
		//SCI_Transmit(3,11,&AT_IPR[0]);//��������=9600��������
		//SCI_Transmit(3,2,&endchar[0]);
		TimeRem_SIM = sysClock;
		while(SIM900A_Rx(0) == 0)	  //���ջظ�
		{
			if((sysClock-TimeRem_SIM) > 4000)//����2s�޻ظ��������ط�
				return error_flag;
		}
		f_chdir("/BACKUP");
		f_unlink(backup_IPR);//�״�����9600������������ϣ�ɾ�����ļ����Ժ󲻻���ִ�д˶γ���
		usart3_init();		 //��ʼ��9600
	}
	error_flag += SIM900A_CMD(AT_CGREG_SET);//��ʾ��վ��Ϣ

	//�������ͻ�������ʼ��
	GPRS.recvok = 0;  //��̨��ִ
	GPRS.sendok = 1;  //���ͳɹ�
	GPRS.dog = 0;	  //
	GPRS.pos = 0;	  //������ָ��
	for(i=0;i<300;i++)
		GPRS.msg[i] = 0;//���ݻ�����
	
	if(error_flag == 0)
		DelayMs(1000);	

	return error_flag;
}

/*******************************************************************************
* Function Name  : SIM900A_CMD
* Input          : None
* Output         : None
* Return         : None
* Description    : SIM900A��MC323A����AT�������
*******************************************************************************/	  
uint8_t SIM900A_CMD(uint8_t *p)
{
	static uint8_t flag = 0;
	uint8_t endchar[] = {0x0D,0x0A}; 
  	
	while(*p)				//���ַ�����
	{
	  	SCI_Transmit(3,1,p);//�͵�usart���ͻ�����
		p++;
	}
	SCI_Transmit(3,2,&endchar[0]);
	TimeRem_SIM = sysClock;	
	while(SIM900A_Rx(0) == 0)//��������
	{
		if((sysClock-TimeRem_SIM) > 3000)//����3s�޻ظ��������ط�
		{
			flag++;
			if(flag > 3)
			{
				printf("SIM900A cmd break\n");
				GPRS.step = REBOOT;
			}
			return 1;
		}
	}
	flag = 0;
	return 0;
}

/*******************************************************************************
* Function Name  : SIM900A_Rx
* Input          : None
* Output         : None
* Return         : None
* Description    : SIM900A���ճ���
*******************************************************************************/
uint8_t SIM900A_Rx(uint8_t flag)
{	 
	uint16_t i,len = 0;

	if(flag == 0)
	{
		len = SCI_GetLen(3,0);
	    if(len > 0)					//ֻҪSCI�յ����ͽ����ó�
			{	
				DelayMs(10);
				len = SCI_GetLen(3,0);
				for(i=0;i<300;i++)
					GPRS.msg[i] = 0;		
				SCI3_Receive(0,len,&GPRS.msg[0]);
				GPRS.dog = 0;
			/*	printf("SIM900A: GPRS.msg = ");
				puts(&GPRS.msg[0]);
				printf("\n");*/
			}
	}  
	else
		SCI3_Receive(1,len,&GPRS.msg[0]);	//�������ع�

	return len;
}

/*******************************************************************************
* Function Name  : GPRS_connect
* Input          : None
* Output         : None
* Return         : None
* Description    : 	SIM900A�������ӳ���
					MC323A��ȡIP�뽨������
*******************************************************************************/
uint8_t GPRS_connect(void)
{  
	static uint8_t flag = 0;
	static uint8_t error_flag = 0;
	static uint32_t TIME = 0;
	uint16_t i;										   

	if(flag == 0)
	{		
		printf("SIM900A: GPRS connecting..\n");
		if(GPRS_check_status() > 2)//����ǰ��ȷ��״̬��ֻ��IP inital,UDP closed,IP status����״̬��������
		{
			GPRS.step = REBOOT;
			return 0;	
		}
		GPRS.step = CONNECTING;
		printf("SIM900A: GPRS status ok, send AT_cipstart\n");	
		SIM900A_Rx(0);
		DelayMs(500);		
		SIM900A_CMD(AT_CIPSTART);			//����Ŀ��IP�˿�
		flag = 1;
		TIME = sysClock;
		DelayMs(1000);
	}
	else
	{
		DelayMs(10);
		SIM900A_Rx(0);	
		for(i=0;i<290;i++)
		{
			if(!mem_compare(&GPRS.msg[i], "CONNECT OK"))
			{
				GPRS.step = DATA;
				SIM900A_CMD(AT_CLPORT);//���ñ��ؽ��ն˿ںš�UDP����2001
				flag = 0;
				if(led_flag == 0)
					led_flag = 1;
				return 0;
			}
			if(!mem_compare(&GPRS.msg[i], "ALREADY CONNECT"))
			{
				GPRS.step = DATA;
				SIM900A_CMD(AT_CLPORT);//���ñ��ؽ��ն˿ںš�UDP����2001
				flag = 0;
				if(led_flag == 0)
					led_flag = 1;
				return 0;
			}			
		}		
		for(i=0;i<290;i++) 
		{ 
			if((!mem_compare(&GPRS.msg[i], "+CGREG:"))&&((sysClock - TIME) > 10000))
			{
				TIME = sysClock;
				flag = 0;
				return 1;
			} 
			if(((!mem_compare(&GPRS.msg[i], "ERROR"))||(!mem_compare(&GPRS.msg[i], "CONNECT FAIL")))&&((sysClock - TIME) > 10000))//ERROR
			{
				error_flag++;
				if(error_flag > 5)
					GPRS.step = REBOOT;
				TIME = sysClock;
				flag = 0;		//��������
				return 1;
			} 
		}
		if((sysClock - TIME) > 60000)//����1����δ����
		{
			TIME = sysClock;
			flag = 0;
			return 1;
		}		
		SIM900A_Rx(1);
	}

	return 1;
}

/*******************************************************************************
* Function Name  : GPRS_check_connect
* Input          : None
* Output         : None
* Return         : None
* Description    : 	SIM900A��MC323A�жϵȴ����ӳɹ�����
*******************************************************************************/
uint8_t GPRS_check_connect(void)
{ 
	uint16_t i;
	static uint8_t connect_dog = 0;

	if(SIM900A_Rx(0) == 0)
	{
		connect_dog++;
		if(connect_dog >= 50)
		{
			connect_dog = 0;
			GPRS.step = REBOOT;
			return 1;
		}
		GPRS.step = CHECK;
		return 1;
	}
	for(i=0;i<290;i++)
	{
		if(!mem_compare(&GPRS.msg[i], "CONNECT OK"))
		{
			printf("SIM900A: connect OK\n");
			GPRS.step = DATA;
			GPRS.msg[299] = 0x5A;
			connect_dog = 0;
			if(led_flag == 0)
				led_flag = 1;
			return 0;
		}
	}
	SIM900A_Rx(1);
	DelayMs(10);
	connect_dog++;
	if(connect_dog >= 50)
	{
		connect_dog = 0;
		GPRS.step = REBOOT;
	}
	return 1;
}

/*******************************************************************************
* Function Name  : GPRS_check_status
* Input          : None
* Output         : None
* Return         : None
* Description    : 	SIM900A��MC323A��ѯ����״̬
*******************************************************************************/
uint8_t GPRS_check_status(void) //��ѯ����״̬
{
	uint8_t times = 0;
	uint16_t i,j = 0;
	static uint8_t con = 0;

	//printf("SIM900A: check status:\n");
	SIM900A_Rx(0);
	SIM900A_CMD(AT_CIPSTATUS);
	while(1)
	{
		times++;
		for(i=0;i<290;i++)
		{
			if(!mem_compare(&GPRS.msg[i], "CONNECT OK"))
			{	
				printf("SIM900A: status CONNECT OK\n");
				return 0;
			}
			if(!mem_compare(&GPRS.msg[i], "SEND OK"))
			{	
				printf("SIM900A: status SEND OK\n");
				return 0;
			}
			if(!mem_compare(&GPRS.msg[i], "UDP CLOSED"))		 
			{
				printf("SIM900A: status UDP CLOSED\n");
				return 1;
			}
			if(!mem_compare(&GPRS.msg[i], "IP INITIAL"))
			{
				printf("SIM900A: status IP INITIAL\n");
				return 1;
			}
			if(!mem_compare(&GPRS.msg[i], "IP STATUS"))					
			{
				printf("SIM900A: status IP STATUS\n");
				return 1;
			}
			if((!mem_compare(&GPRS.msg[i], "CONNECTING"))||(!mem_compare(&GPRS.msg[i], "IP CONFIG")))
			{		
				printf("SIM900A: status CONNECTING\n");
				con++;
				if(con>10)
					return 1;
				return 2;
			}		
		}
		for(i=0;i<290;i++)
		{
			if(!mem_compare(&GPRS.msg[i], "DEACT"))//+PDP:DEACT
			{
				j++;
				if(j > 5)
				{
					printf("SIM900A: status +PDP:DEACT!\n");
					return 5;
				}
				else
				{
					printf("SIM900A: status error, check again..\n");
					DelayMs(100);
					SIM900A_Rx(0);
					SIM900A_CMD(AT_CIPSTATUS);
					break;
				}
			}
	
			if(!mem_compare(&GPRS.msg[i], "ERROR"))
			{	
				j++;
				if(j > 3)
				{
					printf("SIM900A: status ERROR!\n");	
					return 4;
				}
				else
				{
					printf("SIM900A: status error, check again..\n");
					DelayMs(100);
					SIM900A_Rx(0);
					SIM900A_CMD(AT_CIPSTATUS);
					break;
				}
			}
		}
		SIM900A_Rx(1);
		DelayMs(10);
		SIM900A_Rx(0);
		if(times > 2)
			break;
	}
	return 3;
}

/*******************************************************************************
* Function Name  : GPRS_Send_CMD
* Input          : None
* Output         : None
* Return         : None
* Description    : SIM900A��MC323A��������
*******************************************************************************/
uint8_t GPRS_Send_CMD(uint16_t lenth)
{	
	uint16_t i,j;

	if(lenth <= 1)
		return 1;

	for(i=11;i<15;i++)
		AT_CIPSEND[i] = 0;
	if(lenth < 10)
	{
		AT_CIPSEND[11] = lenth + '0';
	}
	else if(lenth < 100)
	{
		AT_CIPSEND[11] = lenth/10 + '0';
		AT_CIPSEND[12] = lenth%10 + '0';
	}
	else
	{
		AT_CIPSEND[11] = lenth/100 + '0';
		AT_CIPSEND[12] = (lenth%100)/10 + '0';
		AT_CIPSEND[13] = (lenth%100)%10 + '0';
	}
	printf("SIM900A: send cmd ");
	puts((char *)AT_CIPSEND);
	printf("\n");
	SIM900A_CMD(AT_CIPSEND);
	j=0;
	DelayMs(10);
	while(1)
	{
		for(i=0;i<290;i++)
			if((GPRS.msg[i] == 0x3E)&&(GPRS.msg[i+1] == 0x20))//����ģ��׼�����
				return 0;
		for(i=0;i<280;i++)
			if((!mem_compare(&GPRS.msg[i], "SEND"))&&(!mem_compare(&GPRS.msg[i+8], "ERROR")))
			{
				printf("SIM900A: send error!\n");
				return 1;
			}
		SIM900A_Rx(1);
		DelayMs(10);
		SIM900A_Rx(0);
		j++;
		if(j > 10)
			break;
	}
	return 1;
}

/*******************************************************************************
* Function Name  : GPRS_reset
* Input          : None
* Output         : None
* Return         : None
* Description    : SIM900A��MC323AӲ����λ
*******************************************************************************/
uint8_t GPRS_reset(void)	//Ӳ����λ�����ȼ������IPR.tmp�ļ����������ʼ��Ϊ115200�����ʽ���SIM900A�Ĳ����ʸ��ģ�Ȼ��ı�PWRKEY���ŵ�ƽ�����ػ�
{
	static uint32_t TimeRem_GPRS = 0;
	static uint8_t step = 0;
	uint16_t i;
	uint16_t j;

	switch(step)
	{
		case 0:	step = 1;
				TimeRem_GPRS = sysClock;
				PWRKEY_UP					//PWRKEY����
				SIM900A_Rx(0);
				printf("SIM900A: reset step = %d\n",step);
			break;
		case 1:	if((sysClock - TimeRem_GPRS) < 10000)//�ȴ�һ�����������POWKEY 1������
				{					
					SIM900A_Rx(0);
					for(i=0;i<290;i++)
					{
						if((!mem_compare(&GPRS.msg[i], "Call Ready")))//||(!mem_compare(&GPRS.msg[i], "CGREG"))) //Call Ready
						{
							GPRS.step = CONNECTING;
							for(j=i+5;j<290;j++)
								if(!mem_compare(&GPRS.msg[i], "POWER"))
									GPRS.step = REBOOT;
							if(GPRS.step == CONNECTING)
							{
								TimeRem_GPRS = sysClock;
								if(SIM900A_Init() == 0)
									step = 0;								
								printf("SIM900A: reset step = %d\n",step);
								return step;
							}
						}
					}
					SIM900A_Rx(1);
				}
				else
				{
					if(IPR_flag == 1)
						SIM900A_Init();
					TimeRem_GPRS = sysClock;
					PWRKEY_DOWN				//PWRKEY����
					step = 2;
					printf("SIM900A: reset step = %d\n",step);
				}
			break;
		case 2:	if((sysClock - TimeRem_GPRS) > 2000)//����POWKEY 2��������߹ػ�
				{
					step = 3;
					printf("SIM900A: reset step = %d\n",step);
					TimeRem_GPRS = sysClock;
					SIM900A_Rx(0);
					PWRKEY_UP				//PWRKEY����
				}
			break;
		case 3: if((sysClock - TimeRem_GPRS) > 30000)//�ػ�20�������POWKEY 1������
				{
					if(IPR_flag == 1)
					{
						SIM900A_Rx(0);
						SIM900A_Init();
					}					
					TimeRem_GPRS = sysClock;
					PWRKEY_DOWN				//PWRKEY����
					step = 4;
					printf("SIM900A: reset step = %d\n",step);
				}
				else
				{
					SIM900A_Rx(0);
					for(i=0;i<290;i++)
					{
						if((!mem_compare(&GPRS.msg[i], "Call Ready"))||(!mem_compare(&GPRS.msg[i], "CGREG: 1"))) //Call Ready
						{
							GPRS.step = CONNECTING;
							for(j=i+5;j<290;j++)
								if(!mem_compare(&GPRS.msg[i], "POWER"))
									GPRS.step = REBOOT;
							if(GPRS.step == CONNECTING)
							{
								TimeRem_GPRS = sysClock;
								if(SIM900A_Init() == 0)
									step = 0;								
								printf("SIM900A: reset step = %d\n",step);
								return step;
							}
						}
					}
					SIM900A_Rx(1);
				}
			break;
		case 4: if((sysClock - TimeRem_GPRS) > 2000)//����POWKEY 3��������߿���
				{
					TimeRem_GPRS = sysClock;
					PWRKEY_UP				//PWRKEY����
					step = 5;
					printf("SIM900A: reset step = %d\n",step);
				}
			break;
		case 5: if((sysClock - TimeRem_GPRS) > 30000)//����20���׼����ʼ��
				{
					if(IPR_flag == 1)
						SIM900A_Init();					
					step = 0;
					printf("SIM900A: reset step = %d\n",step);
				}
				else
				{
					SIM900A_Rx(0);
					for(i=0;i<290;i++)
					{
						if((!mem_compare(&GPRS.msg[i], "Call Ready")))//||(!mem_compare(&GPRS.msg[i], "CGREG"))) //Call Ready
						{
							GPRS.step = CONNECTING;
							for(j=i+5;j<290;j++)
								if(!mem_compare(&GPRS.msg[i], "POWER"))
									GPRS.step = REBOOT;
							if(GPRS.step == CONNECTING)
							{
								TimeRem_GPRS = sysClock;
								if(SIM900A_Init() == 0)
									step = 0;								
								printf("SIM900A: reset step = %d\n",step);
								return step;
							}
						}
					}
					SIM900A_Rx(1);
				}
			break;
		default:step = 0;
			break;
	}	
	return step;
}

/*******************************************************************************
* Function Name  : GPRS_Signal
* Input          : None
* Output         : None
* Return         : None
* Description    : SIM900A��MC323A�ɼ��ź�
*******************************************************************************/
uint8_t GPRS_Signal(void) //�ɼ��ź�
{
	uint8_t signal1 = 0;
	uint8_t signal2 = 0;
	uint16_t i;

	SIM900A_CMD(AT_CSQ);
	while(1)
	{
		for(i=0;i<290;i++)
		{
			if(!mem_compare(&GPRS.msg[i], "CSQ:"))
			{
				signal1 = GPRS.msg[i+5]-'0';
				signal2	= GPRS.msg[i+6]-'0';
				signal1 = signal1*10+signal2;
			} 
		}
		if(signal1 > 0)
			break;
		else
		{
			SIM900A_Rx(1);
			DelayMs(10);
			SIM900A_Rx(0);
		}	
	}
	return signal1;
}

/*******************************************************************************
* Function Name  : GPRS_CGREG
* Input          : None
* Output         : None
* Return         : None
* Description    : SIM900A��վ��Ϣ
*******************************************************************************/
uint8_t GPRS_CGREG(uint8_t *num)
{
	uint16_t i, flag = 0;
	uint8_t data[8];

	printf("AT_CGREG\n");
	SIM900A_CMD(AT_CGREG);	//+CGREG: n,stat,lac,ci
	DelayMs(20);
	SIM900A_Rx(1);
	SIM900A_Rx(0);
	for(i=0;i<275;i++)		//+CGREG: 2,1,"xxxx","xxxx"
		if((!mem_compare(&GPRS.msg[i], "+CGREG: 2,"))
		 &&(!mem_compare(&GPRS.msg[i+11], ",\""))
		 &&(!mem_compare(&GPRS.msg[i+17], "\",\""))
		 &&(GPRS.msg[i+24] == '\"'))
		{
			printf("SIM900A: CGREG OK\n");
			flag = i+13;//��Ϣ��ʼλ
			break;		
		}

	if(flag == 0)
	{
		for(i=0;i<4;i++)
			*(num+i) = 0xFF;
		return 1;
	}

	for(i=0;i<4;i++)
	{
		data[i] = CharToInt(GPRS.msg[flag+i]);		  //��վ��Ϣ(2�ֽ�)
		data[i+4] = CharToInt(GPRS.msg[flag+i+7]);	  //С����Ϣ(2�ֽ�)
	}
	for(i=0;i<4;i++)
		*(num+i) = (uint8_t)((data[i*2] << 4) & 0xF0)|(data[i*2+1] & 0x0F);
  
	printf("SIM900A: CGREG = ");
	for(i=0;i<4;i++)
		printf("%1X ",*(num+i));	
	printf("\n");

	return 0;
}

/*******************************************************************************
* Function Name  : GPRS_YE
* Input          : None
* Output         : None
* Return         : None
* Description    : 10086������
*******************************************************************************/
uint16_t GPRS_YE(void)
{
	static uint16_t step = 0;
	static uint8_t send_buffer[3] = "ye";
	uint32_t time;
	uint16_t i,j,len,k,flag,buffer[20],value = 0xFFFF;

	if(GPRS.step != DATA)	//������
		return 0xFFFF;

	while(1)
	{
		IWDG_ReloadCounter();
		step++;
		if(step > 2000)
		{
			step = 0;
			GPRS.step = DATA;
			return 0xFFFF;
		}
		switch(step)
		{
			case 1: printf("SIM900A: send CMGD\n");
					SIM900A_CMD(AT_CMGD);
				break;
			case 5:printf("SIM900A: send CMGF\n");
					SIM900A_CMD(AT_CMGF1);
				break;
			case 10:printf("SIM900A: send CSCS\n");
					SIM900A_CMD(AT_CSCS);
				break;
			case 15:printf("SIM900A: send CMGS\n");
					SIM900A_CMD(AT_CMGS);
				break;
			case 50:SIM900A_Rx(0);
					step = 0;
					for(i=0;i<250;i++)	
					 	if((GPRS.msg[i] == 0x3E)&&(GPRS.msg[i+1] == 0x20))
						{	
							step = 50;
							break;
						}					
				break;
			case 80:SIM900A_Rx(0);
					for(i=0;i<250;i++)
					{	
					 	if(!mem_compare(&GPRS.msg[i], "ERROR"))
						{	
							GPRS.step = DATA;	
							return 0xFFFF;
						}
					}
					send_buffer[2] = 0x1A;
					SCI_Transmit(3,3,send_buffer);
					printf("SIM900A: 10086 ye message OK\n");
					GPRS.sendok = 1;
				break;
			case 120:flag = 1;
					time = sysClock;
					while(flag)
					{		
						if((sysClock - time) > 30000)
							break;
						printf("SIM900A: send CNMI\n");
						SIM900A_CMD(AT_CNMI);
						DelayMs(3000);
						SIM900A_Rx(1);
					 	SIM900A_Rx(0);
						for(i=0;i<250;i++)	
						 	if(!mem_compare(&GPRS.msg[i], "+CMTI:"))
							{	
								flag = 0;
								break;
							}
						IWDG_ReloadCounter();	
					}
					printf("SIM900A: send CMGR\n");
					SIM900A_CMD(AT_CMGR);
				break;
			case 170:SIM900A_Rx(1);
					SIM900A_Rx(0);
					for(i=0;i<290;i++)
					{								
						if(!mem_compare(&GPRS.msg[i], "62374F59989D4E3A"))	 //�˻����Ϊ
						{	
							for(j=i+16;j<290;j++)
							{
								if(!mem_compare(&GPRS.msg[j], "5143FF0C"))	 //Ԫ��
								{
									for(k=0;k<j-i-16;k++)
										buffer[k] = GPRS.msg[k+i+16]; 
									len = 0;
									for(k=0;k<=((j-i-16)<<2);k++)
									{
										if(buffer[k*4+3] == 'E')
											break;
										buffer[k] = (buffer[k*4+3] - '0');
										len++;	
									}		
									switch(len)
									{
										case 0:	value = 0;
											break;
										case 1:	value = buffer[0];
											break;
										case 2:	value = buffer[0]*10 + buffer[1];
											break;
										case 3:	value = buffer[0]*100 + buffer[1]*10 + buffer[2];
												if(value >= 0xFF)
													value = 250;
											break;
										default:value = 250;
											break;
									}							
									printf("SIM900A: Sim-card money left = %d RMB\n",value);
									GPRS_time();
									step = 0;
									return value;
								}
							}
							break;
						}
					}
					step = 0;
					GPRS.step = DATA;
					printf("SIM900A: message OK\n");
					printf("SIM900A: send CMGD\n");
					SIM900A_CMD(AT_CMGD);
					GPRS.sendok = 1;
				return 0xFFFF;
			default:DelayMs(10);break;
		}
	}
}

/*******************************************************************************
* Function Name  : GPRS_time
* Input          : None
* Output         : None
* Return         : None
* Description    : 10086��ʱ
*******************************************************************************/
void GPRS_time(void)
{
#if 0
	uint8_t buffer[17];
	RTC_timer time;
	uint16_t i,j;

	for(i=0;i<290;i++)
	{
		if(!mem_compare(&GPRS.msg[i], "\"10086\",\"\",\""))
		{
			printf("SIM900A: gprs time correct\n");

			for(j=0;j<17;j++)
				buffer[j] = GPRS.msg[i+12+j];  //12/11/09,21:08:06

			time.year = (buffer[0] - '0')*10 + (buffer[1] - '0');			
			time.month = (buffer[3] - '0')*10 + (buffer[4] - '0');
			time.date = (buffer[6] - '0')*10 + (buffer[7] - '0');
			time.hour = (buffer[9] - '0')*10 + (buffer[10] - '0');
			time.minute = (buffer[12] - '0')*10 + (buffer[13] - '0');
			time.second = (buffer[15] - '0')*10 + (buffer[16] - '0');

			if(time.year < 12)
				return;
			if((time.month > 12)||(time.month < 1))
				return;
			if((time.date > 31)||(time.date < 1))
				return;
			if((time.hour > 24)||(time.hour < 1))
				return;
			if((time.minute > 60)||(time.minute < 1))
				return;
			if((time.second > 60)||(time.second < 1))
				return;

			set_time(&time);
			display_time();
			break;
		}	
	}
#endif
}

/*******************************************************************************
* Function Name  : GPRS_DTC_message
* Input          : None
* Output         : None
* Return         : None
* Description    : ��������ŷ���
*******************************************************************************/
void GPRS_DTC_message(void)
{
#if 0
	static uint8_t tt = 0;
	static uint8_t step = 0;
	static uint32_t tttt = 0;
	uint8_t i,j,send_buffer[100];

	if(GPRS.step != DATA)	//������
		return;

	if((sysClock - tttt) > 86400000)
		tt = 0;

	if(tt > 0)
		return;

	tt++;
	tttt = sysClock;

	if((sysMode >= CAN_STD_500) && (sysMode <= CAN_EXT_250))
		send_buffer[0] = 'C';
	else if((sysMode >= K_9141_5) && (sysMode <= K_14230_fast))
		send_buffer[0] = 'K';
	else
		send_buffer[0] = '?';	

	send_buffer[1] = ':';
	send_buffer[2] = (btMsg.txLen[0]/16);
	send_buffer[3] = (btMsg.txLen[0]%16);
	for(i=2;i<4;i++)
	{
		if(send_buffer[i] > 9)
			send_buffer[i] += 'A' - 10;
		else
			send_buffer[i] += '0';
	}
	send_buffer[4] = ' ';
	for(i=0;i<15;i++)
	{
		send_buffer[i*3+5] = (btMsg.tx[0][i]/16);
		send_buffer[i*3+6] = (btMsg.tx[0][i]%16);
		for(j=(i*3+5);j<(i*3+7);j++)
		{
			if(send_buffer[j] > 9)
				send_buffer[j] += 'A' - 10;
			else
				send_buffer[j] += '0';
		}
		send_buffer[i*3+7] = ' ';
	}
	send_buffer[50] = '-';
	send_buffer[51] = '-';
	for(i=0;i<10;i++)
	{										   			  
		send_buffer[i*5+51] = (DTC_now[i]/4096);		
		send_buffer[i*5+52] = ((DTC_now[i]%4096)/256);	 
		send_buffer[i*5+53] = ((DTC_now[i]%256)/16);	
		send_buffer[i*5+54] = (DTC_now[i]%16);			
		for(j=(i*5+51);j<(i*5+55);j++)
		{
			if(send_buffer[j] > 9)
				send_buffer[j] += 'A' - 10;
			else
				send_buffer[j] += '0';
		}
		send_buffer[i*5+55] = ' ';
	}
	send_buffer[95] = 0x1A;

	while(1)
	{
		step++;
		switch(step)
		{
			case 1: printf("SIM900A: send CMGD\n");
					SIM900A_CMD(AT_CMGD);
				break;
			case 5:printf("SIM900A: send CMGF\n");
					SIM900A_CMD(AT_CMGF1);
				break;
			case 10:printf("SIM900A: send CSCS\n");
					SIM900A_CMD(AT_CSCS);
				break;
			case 15:printf("SIM900A: send CMGS\n");
					SIM900A_CMD(AT_CMGS_me);
				break;
			case 80:SIM900A_Rx(0);
					for(i=0;i<250;i++)	
					 	if(!mem_compare(&GPRS.msg[i], "ERROR"))	
							return;
					SCI_Transmit(3,96,send_buffer);
					GPRS.sendok = 1;
				break;
			case 90:step = 0;
					printf("SIM900A: message OK\n");
					printf("SIM900A: send CMGD\n");
					GPRS.sendok = 1;
				return;	
			default:DelayMs(10);break;
		}
		IWDG_ReloadCounter();
	}
#endif
}
