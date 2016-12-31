#include "BLE.h"
#include "AutoLoop.h"
#include "timelength.h"
#include "mpu6050.h"


extern ECUstat car_state;
extern CARprotocol sysMode;
extern MSG_COMM_BT btMsg;
extern uint32_t sysClock;
extern MSG_COMM proMsg;
extern uint8_t WqData[];
extern bool OBDLoopFlag;
extern uint16_t DTC_now[20];
extern MSG_COMM_GPRS GPRS;
extern uint8_t led_flag;
extern uint8_t ID_CFG[5];
extern PIDsupport supptemp;
extern vu16 ADC_ConvertedValue[]; 
/*******************************************С��*****************************************/
extern uint8_t Soft_Version;  //����汾   
extern uint8_t Hard_Version;  //Ӳ���汾   
extern uint8_t Mil_Data[3];  //�������
extern uint8_t Ex_Data[2];   //������  
extern uint8_t Max_Vspeed_this;           //������߳���
extern uint8_t Max_Vspeed_all;           //��ʷ��߳���

extern uint16_t Oil_ave;     //����ƽ���ͺ� 
extern uint16_t Oil_this;    //���κ�����
extern uint16_t Oil_this_gprs;  //����GPRS�ϴ��ı��κ�����
extern uint16_t Oil_aveall;   //��ƽ���ͺ�
extern uint16_t Oil_all;     //�ܺ�����
extern uint16_t Max_Espeed_this;            //�������ת��
extern uint16_t Max_Espeed_all;            //��ʷ���ת��

extern uint32_t Ter_Version; //�ն˱��  
extern uint32_t Oilloop_time;
extern uint32_t Oil_ave1;
//extern uint32_t Milestone[3];   //�������
extern uint32_t Mile_one;
extern uint32_t Mile_all;
//extern uint32_t Oil_all;     //�ܺ�����
extern uint32_t timelen_idle_one_at; //AT 15�����ȡ�ı��ε���ʱ��
extern uint32_t timelen_driving_one_at;  //AT 15�����ȡ�ı�����ʻʱ��

uint8_t Espeed_hour = 0;   //�������ת��ʱ��Сʱ
uint8_t Espeed_minute = 0;
uint8_t Espeed_second = 0;
uint8_t Vspeed_hour = 0;   //������߳��ٷ���ʱ�� Сʱ
uint8_t Vspeed_minute = 0;
uint8_t Vspeed_second = 0;

/*******************************************С��*****************************************/

/*******************************************************************************
* Function Name  : BLE_Main
* Input          : None
* Output         : None
* Return         : None
* Description    : ������������ɼ���������������
*******************************************************************************/
void BLE_Main(void)
{
	uint8_t frame,sid,i,j,k,len = 0;
	static uint8_t pos = 0, BLE_buffer[250];
	uint8_t send_buffer[200] = {"$OBD ECUOFF\n"};
	uint8_t file_buffer[50];
	uint16_t vol;
//	short aacx = 0,aacy = 0,aacz = 0;		//���ٶȴ�����ԭʼ����
//	short gyrox = 0,gyroy = 0,gyroz = 0;	//������ԭʼ����

//	OilCount();
	TIMELENGTH();
	Max_speed();
	mpu6050_send_idledata();
	mpu6050_judge();
//	MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//�õ����ٶȴ���������
//	MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//�õ�����������
//	mpu6050_send_data(aacx,aacy,aacz,gyrox,gyroy,gyroz);
	
	
	len=SCI_GetLen(1,0); //�鿴�����������ݻ���
  if(len>0)            //ֻҪ�յ����ͽ����ó�
  {   	
		if(pos >= 250)
			pos = 0;
		SCI_Receive(1,len,&BLE_buffer[pos]);
		pos+=len;
		
		for(i=0;i<245;i++)
		{
			if(!mem_compare(&BLE_buffer[i], "$OBD 0")) 	//"$OBD 03\n $OBD 0100\n $OBD 020200\n
			{
				frame = 0;
				if(BLE_buffer[i+7] == '\n')
					frame = 2;
				else if(BLE_buffer[i+9] == '\n')
					frame = 3;
				else if(BLE_buffer[i+11] == '\n')
					frame = 4;
				else
					continue;
				
				sid = BLE_buffer[i+6] - '0';
				if((sid > 0)&&(sid <= 9))
				{								
					if(OBDLoopFlag == 1)
					{		
						mem_copy(send_buffer,"$OBD LOOPing\n");
						SCI_Transmit(1,13,send_buffer);
						btMsg.state = COMM_RX_OK; 
						for(j=0;j<250;j++)
							BLE_buffer[j] = 0;
						pos = 0;
						return;
					}	
					if(sysMode == NONE)   //����δ����
					{		
						mem_copy(send_buffer,"$OBD ECUOFF\n");
						SCI_Transmit(1,12,send_buffer);
						btMsg.state = COMM_RX_OK; 
						for(j=0;j<250;j++)
							BLE_buffer[j] = 0;
						pos = 0;
						return;
					}			
					else if((car_state < 2))
					{
						mem_copy(send_buffer,"$OBD ECUON\n");
						SCI_Transmit(1,11,send_buffer);
						btMsg.state = COMM_RX_OK; 
						for(j=0;j<250;j++)
							BLE_buffer[j] = 0;
						pos = 0;
						return;
					}
					
					switch(frame)
					{
							case 2: btMsg.rx[0] = sid;//ServiceID
											btMsg.rxLen = 1;	 //03 or 07
									break;
							case 3:	btMsg.rx[0] = sid;//ServiceID
											btMsg.rx[1]	= (BLE_buffer[i+7] - '0')*16 + CharToInt(BLE_buffer[i+8]);//PID
											//pid_save = btMsg.rx[1];
											btMsg.rxLen = 2;	 //01xx
									break;
							case 4: btMsg.rx[0] = sid;//ServiceID
											btMsg.rx[1]	= (BLE_buffer[i+7] - '0')*16 + CharToInt(BLE_buffer[i+8]);//PID
											btMsg.rx[2]	= (BLE_buffer[i+9] - '0')*16 + CharToInt(BLE_buffer[i+10]);//frame
											btMsg.rxLen = 3;	 //02xx00
									break;
							default:return;
					}
					
					for(i=0;i<8;i++)
					{
							btMsg.btHeadLen[i]=0;
							btMsg.BtTxState[i]=0;
							btMsg.FrameNum[i]=0;
							btMsg.txLen[i]=0;
					}
					btMsg.ECUNumber=0;
					
					if((btMsg.rx[0] == 1)&&(btMsg.rx[1]	== 0x42)&&((supptemp.v0140 & 0x40000000) == 0))//��ѹ
					{
						vol = (uint32_t)ADC_ConvertedValue[0]*46/10+700;
						btMsg.ECUNumber = 1;				
						btMsg.BtTxState[0] = 1;
						btMsg.FrameNum[0] = 1;		
						btMsg.txLen[0] = 4;
						btMsg.tx[0][0] = 0x41;
						btMsg.tx[0][1] = 0x42;
						btMsg.tx[0][2] = (uint8_t)(vol>>8);
						btMsg.tx[0][3] = (uint8_t)vol&0xFF;
						proMsg.state = COMM_RX_OK;
					}
					else
					{
						if((CAN_STD_500 <= sysMode) && (sysMode <= CAN_EXT_250)) //CAN        
							ISO15765Main();//ʹ��CAN������ת��ΪCAN���ݰ�,������;ͬʱ��CAN���յ�����ת��      
						else if((sysMode == K_14230_5) || (sysMode == K_14230_fast))
							ISO14230Main();	//ͬ��
						else if(sysMode == K_9141_5)
							ISO9141Main();
					}
					btMsg.state=COMM_IDLE;
        						 
					if(proMsg.state==COMM_RX_OK) //�������ݷ��Ͳ�������ϣ�׼����������
					{		
            BtSend();			//�����ظ����յ�������                   
            proMsg.state=COMM_IDLE;
					}
					else 					//�������ݲɼ�ʧ�ܣ���NO_DATA
					{
						mem_copy(send_buffer,"$OBD NODATA\n");
						SCI_Transmit(1,12,send_buffer);
						btMsg.state = COMM_RX_OK; 
						proMsg.state=COMM_IDLE;
						for(j=0;j<250;j++)
							BLE_buffer[j] = 0;
						pos = 0;
						return;					
					}
					btMsg.state = COMM_RX_OK; 
					for(j=0;j<250;j++)
						BLE_buffer[j] = 0;
					pos = 0;
					break;
				}
			}
			/*****************************************************
			AT���
			$AT 01\n��OBDЭ��
			$AT 02\n������״̬
			$AT 03\n��֧����
			$AT 04\n��OBD���ֹͣ��ѭ����
			$AT 05\n��˲���ͺ�
			$AT 06\n��β���ŷţ�PID01.cfg��
			$AT 07\n��������������
			$AT 08\n�������루������
			$AT 09\n��ż�������루������
			$AT 10\n���ļ����䣨���ã�������
			$AT 11 Len1 Len2 ���ֵ\n�����У��   
			$AT 12 Len1 Len2 ������\n�����������룬�����ͺļ���   
			$AT 13\n���ն���Ϣ��ѯ
			$AT 15\n�����ͱ��ε���ʱ����������ʻʱ����������ʻ��̣�������
			*****************************************************/
			if((!mem_compare(&BLE_buffer[i], "$AT "))&&(BLE_buffer[i+6] == '\n')) 	
			{
				sid = (BLE_buffer[i+4] - '0')*10+(BLE_buffer[i+5] - '0');
				switch(sid)
				{
					case 1: switch(sysMode)
									{
										case NONE:	mem_copy(send_buffer,"$AT 01 00NONE\n");
																send_buffer[7] = 0;
																send_buffer[8] = 4;
																SCI_Transmit(1,14,send_buffer);
													break;
										case K_9141_5:mem_copy(send_buffer,"$AT 01 00K9141\n");
																send_buffer[7] = 0;
																send_buffer[8] = 5;
																SCI_Transmit(1,15,send_buffer);
													break;
										case K_14230_5:	mem_copy(send_buffer,"$AT 01 00K14230\n");
																send_buffer[7] = 0;
																send_buffer[8] = 6;
																SCI_Transmit(1,16,send_buffer);
													break;
										case K_14230_fast:	mem_copy(send_buffer,"$AT 01 00K14230\n");
																send_buffer[7] = 0;
																send_buffer[8] = 6;
																SCI_Transmit(1,16,send_buffer);
													break;
										case CAN_STD_500:	mem_copy(send_buffer,"$AT 01 00CANSTD500\n");
																send_buffer[7] = 0;
																send_buffer[8] = 9;
																SCI_Transmit(1,19,send_buffer);
													break;
										case CAN_EXT_500:	mem_copy(send_buffer,"$AT 01 00CANEXT500\n");
																send_buffer[7] = 0;
																send_buffer[8] = 9;
																SCI_Transmit(1,19,send_buffer);
													break;
										case CAN_STD_250:	mem_copy(send_buffer,"$AT 01 00CANSTD250\n");
																send_buffer[7] = 0;
																send_buffer[8] = 9;
																SCI_Transmit(1,19,send_buffer);
													break;
										case CAN_EXT_250:	mem_copy(send_buffer,"$AT 01 00CANEXT250\n");
																send_buffer[7] = 0;
																send_buffer[8] = 9;
																SCI_Transmit(1,19,send_buffer);
													break;
										default:break;
									}
							break;
					case 2:	switch(car_state)
									{
										case ECU_OFF:	mem_copy(send_buffer,"$AT 02 00ECUOFF\n");
																	send_buffer[7] = 0;
																	send_buffer[8] = 6;
																	SCI_Transmit(1,16,send_buffer);
											break;
										case ECU_ON:	mem_copy(send_buffer,"$AT 02 00ECUON\n");
																	send_buffer[7] = 0;
																	send_buffer[8] = 5;
																	SCI_Transmit(1,15,send_buffer);
											break;
										case IDLE:		mem_copy(send_buffer,"$AT 02 00IDLE\n");
																	send_buffer[7] = 0;
																	send_buffer[8] = 4;
																	SCI_Transmit(1,14,send_buffer);
											break;
										case DRIVING:	mem_copy(send_buffer,"$AT 02 00DRIVING\n");
																	send_buffer[7] = 0;
																	send_buffer[8] = 7;
																	SCI_Transmit(1,17,send_buffer);
											break;
									}
							break;
					case 3:	mem_copy(send_buffer,"$AT 03 00");  	//֧����			//$AT 03 xx xx xx xx xx xx xx xx xx xx xx xx		
									send_buffer[7] = 0;
									send_buffer[8] = 0x0C;
									send_buffer[9] = (char)((supptemp.v0100 & 0xFF000000) >> 24);
									send_buffer[10] = (char)((supptemp.v0100 & 0xFF0000) >> 16);
									send_buffer[11] = (char)((supptemp.v0100 & 0xFF00) >> 8);
									send_buffer[12] = (char)(supptemp.v0100 & 0xFF);
									send_buffer[13] = (char)((supptemp.v0120 & 0xFF000000) >> 24);
									send_buffer[14] = (char)((supptemp.v0120 & 0xFF0000) >> 16);
									send_buffer[15] = (char)((supptemp.v0120 & 0xFF00) >> 8);
									send_buffer[16] = (char)(supptemp.v0120 & 0xFF);
									send_buffer[17] = (char)((supptemp.v0140 & 0xFF000000) >> 24);
									send_buffer[18] = (char)((supptemp.v0140 & 0xFF0000) >> 16);
									send_buffer[19] = (char)((supptemp.v0140 & 0xFF00) >> 8);
									send_buffer[20] = (char)(supptemp.v0140 & 0xFF);
									send_buffer[21] = '\n';
									SCI_Transmit(1,22,send_buffer);
							break;
					case 4:	//$AT 04\n��ֹͣOBD��ѭ��
									if(OBDLoopFlag == 0)
									{
										OBDLoopFlag = 1;
										mem_copy(send_buffer,"$AT 04 00LoopStart\n");
										send_buffer[7] = 0;
										send_buffer[8] = 9;
										SCI_Transmit(1,19,send_buffer);
									}
									else if(OBDLoopFlag == 1)
									{
										OBDLoopFlag = 0;
										mem_copy(send_buffer,"$AT 04 00LoopStop\n");
										send_buffer[7] = 0;
										send_buffer[8] = 8;
										SCI_Transmit(1,18,send_buffer);
										for(j=0;j<8;j++)
										{
											btMsg.btHeadLen[j]=0;
											btMsg.BtTxState[j]=0;
											btMsg.FrameNum[j]=0;
											btMsg.txLen[j]=0;
										}
										btMsg.ECUNumber=0;
										btMsg.rxLen=0;
										for(j=0;j<48;j++)
											btMsg.rx[j] = 0; 
									}
							break;
					case 5: //˲���ͺ�			
									OilCount();
							break;
					case 6: //$AT 06\n����ȡβ���ŷŵ�OBD����
									mem_copy(send_buffer,"$AT 06 00");
									if(WqData[0] == 0)
									{
										mem_copy(&send_buffer[9],"NODATA\n");
										send_buffer[7] = 0;
										send_buffer[8] = 6;
										SCI_Transmit(1,16,send_buffer);
									}
									else
									{
										for(j=0;j<WqData[0];j++)
											send_buffer[j+9] = WqData[j+1];
										send_buffer[WqData[0]+9] = '\n';
										send_buffer[7] = 0;
										send_buffer[8] = WqData[0];
										SCI_Transmit(1,WqData[0]+10,send_buffer);
									}
									btMsg.state = COMM_RX_OK; 
							break;
						case 7:
							break;
						case 8:AT_master_BLE(0);
							break;
						case 9:AT_master_BLE(1);
							break;
						case 13: mem_copy(send_buffer,"$AT 13 00");     //�ն���Ϣ��ѯ
						         send_buffer[7] = 0;
						         send_buffer[8] = 6;
						         send_buffer[9] = (char)((Ter_Version & 0xFF000000) >> 24);
						         send_buffer[10] = (char)((Ter_Version & 0xFF0000) >> 16);
						         send_buffer[11] = (char)((Ter_Version & 0xFF00) >> 8);
						         send_buffer[12] = (char)(Ter_Version & 0xFF);
						         send_buffer[13] = Soft_Version;
						         send_buffer[14] = Hard_Version;
						         send_buffer[15] = '\n';
						         SCI_Transmit(1,16,send_buffer);
						  break;
						case 15: 
							       mem_copy(send_buffer,"$AT 15 ");
						         send_buffer[7] = 0;
						         send_buffer[8] = 12;
						         send_buffer[9] = (char)((timelen_idle_one_at & 0xFF000000) >> 24);
										 send_buffer[10] = (char)((timelen_idle_one_at & 0xFF0000) >> 16);
										 send_buffer[11] = (char)((timelen_idle_one_at & 0xFF00) >> 8);
										 send_buffer[12] = (char)(timelen_idle_one_at & 0xFF);
                     send_buffer[13] = (char)((timelen_driving_one_at & 0xFF000000) >> 24);
                     send_buffer[14] = (char)((timelen_driving_one_at & 0xFF0000) >> 16);
                     send_buffer[15] = (char)((timelen_driving_one_at & 0xFF00) >> 8);
                     send_buffer[16] = (char)(timelen_driving_one_at & 0xFF);
										 send_buffer[17] = (char)((Mile_one & 0xFF000000) >> 24);
										 send_buffer[18] = (char)((Mile_one & 0xFF0000) >> 16);
										 send_buffer[19] = (char)((Mile_one & 0xFF00) >> 8);
										 send_buffer[20] = (char)(Mile_one & 0xFF);
										 send_buffer[21] = '\n';
										 SCI_Transmit(1,22,send_buffer);
						  break;
						         
						default:break;
				}
				
				for(j=0;j<250;j++)
					BLE_buffer[j] = 0;
				pos = 0;
				break;
			}
			
			if(!mem_compare(&BLE_buffer[i], "$AT 10 ")) 	
			{
				for(j=i+7;j<i+50;j++)
					if(BLE_buffer[j] == '\n')
						break;
				if(j == (i+50))
					return;
				for(k=0;k<50;k++)
					file_buffer[k] = 0;
				for(k=i+7;k<j;k++)
					file_buffer[k-i-7] = BLE_buffer[k];
				BLEATFile(file_buffer);
			}
			
			if((!mem_compare(&BLE_buffer[i], "$AT 11 "))&&(BLE_buffer[i+12] == '\n'))   //  ���У��
			{
					Mil_Data[0] = BLE_buffer[i+9];
					Mil_Data[1] = BLE_buffer[i+10];
					Mil_Data[2] = BLE_buffer[i+11];
					if((Mil_Data[0] == BLE_buffer[i+9])&&(Mil_Data[1] == BLE_buffer[i+10])&&(Mil_Data[2] == BLE_buffer[i+11]))
					{
						mem_copy(send_buffer,"$AT 11 ");
						mem_copy(&send_buffer[9],"OK\n");
						send_buffer[7] = 0;
						send_buffer[8] = 2;
						SCI_Transmit(1,12,send_buffer);													
					}
					else
					{
						mem_copy(send_buffer,"$AT 11 ");
						mem_copy(&send_buffer[9],"ERROR\n");
						send_buffer[7] = 0;
						send_buffer[8] = 5;
						SCI_Transmit(1,15,send_buffer);
					}
				for(j=0;j<250;j++)
					BLE_buffer[j] = 0;
				pos = 0;
				break;
			}
			
			if((!mem_compare(&BLE_buffer[i], "$AT 12 "))&&(BLE_buffer[i+11] == '\n'))   //������У��
			{
          Ex_Data[0] = BLE_buffer[i+9];
					Ex_Data[1] = BLE_buffer[i+10];
					if((Ex_Data[0] == BLE_buffer[i+9])&&(Ex_Data[1] == BLE_buffer[i+10]))
					{
						mem_copy(send_buffer,"$AT 12 ");
						mem_copy(&send_buffer[9],"OK\n");
						send_buffer[7] = 0;
						send_buffer[8] = 2;
						SCI_Transmit(1,12,send_buffer);													
					}
					else
					{
					  mem_copy(send_buffer,"$AT 12 ");
						mem_copy(&send_buffer[9],"ERROR\n");
						send_buffer[7] = 0;
						send_buffer[8] = 5;
						SCI_Transmit(1,15,send_buffer);
					}										 
				for(j=0;j<250;j++)
					BLE_buffer[j] = 0;
				pos = 0;
				break;
			}
		}		
	}
}
/*******************************************************************************
* Function Name  : BLEATFile
* Input          : filename
* Output         : None
* Return         : OK
* Description    : �ļ��������������
*******************************************************************************/
uint8_t BLEATFile(char *filename)
{
	UINT rc;	
	static FIL file;
	uint16_t i,j,len,pos = 0,res;
	uint8_t send_buffer[100];
	uint8_t BLE_buffer[805];
	uint16_t sum, now, now_temp, all, num;
	uint32_t temp,TimRem_File;
	
	for(i=0;i<50;i++)
		if(filename[i] == 0)
		{
			len = i;
			break;
		}
	f_chdir("/");
		
	if(mem_compare(filename, "Update.bin"))
	{
		res = f_chdir("BACKUP");
		if(res != FR_OK)
		{
			f_mkdir("BACKUP");
			f_chdir("BACKUP");
		}
	}
	f_unlink(filename);
	res = f_open(&file, filename, FA_CREATE_NEW | FA_WRITE);
	if(res == FR_OK)
	{
		mem_copy(send_buffer,"$AT 10 00OK\n");
		send_buffer[7] = 0;
		send_buffer[8] = len+2;
		SCI_Transmit(1,9,send_buffer);	
		SCI_Transmit(1,len,filename);
		SCI_Transmit(1,3,&send_buffer[9]);
		f_sync(&file);
	}
	else
	{
		mem_copy(send_buffer,"$AT 10 00Error\n");
		send_buffer[7] = 0;
		send_buffer[8] = len+5;
		SCI_Transmit(1,9,send_buffer);	
		SCI_Transmit(1,len,filename);
		SCI_Transmit(1,6,&send_buffer[9]);
		return 1;
	}
	now_temp = 0;
	TimRem_File = sysClock;
	while(1)
	{
//		if((sysClock - TimRem_File) > 5000)
//		{
//			f_close(&file);
//			f_unlink(filename);
//			mem_copy(send_buffer,"$AT 10 00Timeout\n");
//			send_buffer[7] = 0;
//			send_buffer[8] = len+7;
//			SCI_Transmit(1,9,send_buffer);	
//			SCI_Transmit(1,len,filename);
//			SCI_Transmit(1,8,&send_buffer[9]);
//			break;
//		}
		len=SCI_GetLen(1,0); //�鿴�����������ݻ���
		if(len>0)            //ֻҪ�յ����ͽ����ó�
		{   	
			if(pos >= 800)
				pos = 0;
			SCI_Receive(1,len,&BLE_buffer[pos]);
			pos+=len;
					
			for(i=0;i<800;i++)
			{
				if(!mem_compare(&BLE_buffer[i], "$AT 10 "))
				{
					TimRem_File = sysClock;
					num = ((uint16_t)BLE_buffer[i+7]<<8) + BLE_buffer[i+8];//�ֽ���
					now = ((uint16_t)BLE_buffer[i+9]<<8) + BLE_buffer[i+10];//��ǰ���
					all = ((uint16_t)BLE_buffer[i+11]<<8) + BLE_buffer[i+12];//�����
					sum = ((uint16_t)BLE_buffer[i+13]<<8) + BLE_buffer[i+14];//У��
					
					if(BLE_buffer[i+num+9] == '\n')//��������
					{					
						temp = 0;
						for(j=0;j<num-6;j++)
							temp += BLE_buffer[j+i+15];
						if(sum != (uint16_t)(temp&0xFFFF))//У�����
						{
							BLE_buffer[i+11] = (uint8_t)((temp&0xFF00)>>8);
							BLE_buffer[i+12] = (uint8_t)(temp&0xFF);
							mem_copy(send_buffer,"$AT 10 00Error\n");
							send_buffer[7] = 0;
							send_buffer[8] = 9;
							SCI_Transmit(1,9,send_buffer);
							SCI_Transmit(1,4,&BLE_buffer[i+9]);
							SCI_Transmit(1,6,&send_buffer[9]);							
						}
						else
						{
							if(now != now_temp)
							{
								now_temp = now;
								res = f_write(&file, &BLE_buffer[i+15], num-6, &rc);//д��
								f_sync(&file);
								mem_copy(send_buffer,"$AT 10 00OK\n");
								BLE_buffer[i+11] = (uint8_t)((temp&0xFF00)>>8);
								BLE_buffer[i+12] = (uint8_t)(temp&0xFF);
								send_buffer[7] = 0;
								send_buffer[8] = 6;
								SCI_Transmit(1,9,send_buffer);
								SCI_Transmit(1,4,&BLE_buffer[i+9]);
								SCI_Transmit(1,3,&send_buffer[9]);
								if(now == all)//�������
								{
									f_close(&file);
									if(!mem_compare(filename, "Update.bin"))
									{
										f_unlink("UpOK.tmp");
										f_unlink("Update.tmp");
										while(1);//Reboot
									}
									else
										return 0;//Return
								}
							}
						}
						for(j=0;j<805;j++)
							BLE_buffer[j] = 0;
						pos = 0;
						break;
					}
				}
			}
		}
		IWDG_ReloadCounter();
	}
}

/*******************************************************************************
* Function Name  : AT_master
* Input          : None
* Output         : None
* Return         : None
* Description    : AT�������������ͣ�BLE��:
					����0		$AT 08 00 0D 06 01 43 01 96 02 34 02 CD 03 57 0A 24\n	
					����1		$AT 09 00 0D 06 01 43 01 96 02 34 02 CD 03 57 0A 24\n	
*******************************************************************************/
uint8_t AT_master_BLE(uint8_t cmd) 
{
	uint8_t i,buffer[100] = {"$AT 08 "};

	switch(cmd)
	{
		case 0:	buffer[5] = '8';
										buffer[9] = (uint8_t)DTC_now[0];//��������
										if(buffer[9] == 0)				//�޹���
										{
											mem_copy(&buffer[9],"NODTC\n");
											buffer[7] = 0;
											buffer[8] = 5;
											SCI_Transmit(1,15,buffer);
											return 0;
										}
										else
										{
											for(i=1;i<=DTC_now[0];i++)	//������
											{								  
												buffer[9+i*2-1] = (uint8_t)((DTC_now[i] & 0xFF00) >> 8);
												buffer[9+i*2] = (uint8_t)(DTC_now[i] & 0xFF);
											}
											i--;
											buffer[7] = 0;
											buffer[8] = i*2+1;
											buffer[10+i*2] = '\n';
										}	
										SCI_Transmit(1,11+i*2,buffer);
									break;
		case 1: buffer[5] = '9';
										buffer[9] = (uint8_t)DTC_now[0];//��������
										if(buffer[9] == 0)				//�޹���
										{
											mem_copy(&buffer[9],"NODTC\n");
											buffer[7] = 0;
											buffer[8] = 5;
											SCI_Transmit(1,15,buffer);
											return 0;
										}
										else
										{
											for(i=1;i<=DTC_now[0];i++)	//������
											{								  
												buffer[9+i*2-1] = (uint8_t)((DTC_now[i] & 0xFF00) >> 8);
												buffer[9+i*2] = (uint8_t)(DTC_now[i] & 0xFF);
											}
											i--;
											buffer[7] = 0;
											buffer[8] = i*2+1;
											buffer[10+i*2] = '\n';
										}	
										SCI_Transmit(1,11+i*2,buffer);
									break;
		default:return 0;
	}
	return 0;
}

void BLEAutoBack(void)
{
	if((car_state > 1)&&(sysMode != NONE))
	{
		//if((btMsg.tx[0][0] != 0x41)||(btMsg.tx[0][1] == 0x00))//SID > 1 or PID = 0
		//	return;	
		BtSend();
	}
}

/*******************************************************************************
* Function Name  : BtSend
* Input          : None
* Output         : None
* Return         : None
* Description    : ����OBD����ظ�
*******************************************************************************/
void BtSend(void)
{
    uint8_t i;
    uint8_t j;
		uint8_t header[6] = {"$OBD  "};
		uint8_t sum='\n';
    
    btMsg.state=COMM_SEND;							//����������״̬Ϊ������    
    for(i=0;i<btMsg.ECUNumber;i++)					//����ECU��Ŀ�ظ�
    {
        if(btMsg.BtTxState[i] == 1)
        {
            if(btMsg.FrameNum[i] == 1)				//��֡����
            {						
								header[5] = i+1;
								SCI_Transmit(1,6,header);					//����ͷ
								SCI_Transmit(1,1,&btMsg.txLen[i]);						//����
								SCI_Transmit(1,btMsg.txLen[i],&btMsg.tx[i][0]);	//����+����
								SCI_Transmit(1,1,&sum);							//У��
												
								btMsg.BtTxState[i] = 0;

                btMsg.state=COMM_SEND_OK;
						}
						else if(btMsg.FrameNum[i] > 1)//��֡
						{	
								if((sysMode>=3)&&(sysMode<=5))//K-line
								{
									for(j=2;j<btMsg.txLen[i];j++)
											btMsg.tx[i][j] = btMsg.tx[i][j+1];//����ͷ֡����ţ�ֻ�����ݺ�PID
									btMsg.txLen[i] -= 1;
								}		

								header[5] = i+1;
								SCI_Transmit(1,6,header);					//����ͷ
								SCI_Transmit(1,1,&btMsg.txLen[i]);	//����
								SCI_Transmit(1,btMsg.txLen[i],&btMsg.tx[i][0]);	//PID,��һ֡����		 
								SCI_Transmit(1,1,&sum);							//У��
								btMsg.state=COMM_SEND_OK;
            }//END (btMsg.FrameNum[i]>1)
        }//END (btMsg.BtTxState[i]==1)
    }//END (i=0;i<btMsg.ECUNumber;i++)      
}


/*******************************************************************************
* Function Name  : OilCount
* Input          : None
* Output         : None
* Return         : None
* Description    : ˲���ͺļ���
*******************************************************************************/
void OilCount(void)
{
	uint8_t Speed = 0,x;  //����   
  uint8_t AirFlow = 0; //�����ܾ���ѹ��
  uint8_t Temp = 0;    //�����¶�
	uint8_t oilsend_buffer[100]= {"$AT 05 "};
	
  uint16_t Oil_data = 0;     //˲���ͺ� 	
	uint16_t Oil_data1 = 0;    //������ƽ���ͺĵ�˲���ͺ�ֵ  
	uint16_t Quality = 0;  //������������
  uint16_t Espeed = 0;  //���ͻ�ת��
	uint16_t Ed = 0;   //���ڼ����ͺĵ�������  
	
	
	if(car_state >= IDLE)
	{
		if(car_state == IDLE)    //����ʱ�ͺ�Ϊ0
		{
			mem_copy(oilsend_buffer,"$AT 05 ");
			oilsend_buffer[7] = 0;
			oilsend_buffer[8] = 4;
			oilsend_buffer[9] = 0;
			oilsend_buffer[10] = 0;
			oilsend_buffer[11] = 0;
			oilsend_buffer[12] = 0;
			oilsend_buffer[13] = '\n';
			SCI_Transmit(1,14,oilsend_buffer);	
		}
		else if((supptemp.v0100 & 0x00010000) != 0)   //֧�ֽ�����������0110
		{
			Main_send(0x01,0x10);
			for(x=0;x<btMsg.ECUNumber;x++)
			{
        Quality = ((uint16_t)btMsg.tx[x][2]<<8) + btMsg.tx[x][3];				
			}
			Main_send(0x01,0x0D);
			for(x=0;x<btMsg.ECUNumber;x++)
			{
        Speed = btMsg.tx[x][2];				
			}
			Oil_data = (uint16_t)(Quality*3.378/Speed);
			Oil_data1 = (uint16_t)(Quality*33.78/Speed);
			Oil_ave1 += Oil_data1;
			Oilloop_time++;
			Oil_ave = (uint16_t)(Oil_ave1/(10*Oilloop_time));  //����ƽ���ͺ�
			Oil_this = (uint16_t)(Oil_ave*Mile_one/100000); //���κ�����
			mem_copy(oilsend_buffer,"$AT 05 ");
			oilsend_buffer[7] = 0;
			oilsend_buffer[8] = 4;
			oilsend_buffer[9] = (char)((Oil_data & 0xFF00) >> 8);
			oilsend_buffer[10] = (char)(Oil_data & 0xFF);
			oilsend_buffer[11] = (char)((Oil_ave & 0xFF00) >> 8);
			oilsend_buffer[12] = (char)(Oil_ave & 0xFF);
			oilsend_buffer[13] = '\n';
			SCI_Transmit(1,14,oilsend_buffer);			
		}
		else      //��֧�ֽ����������� 0110
		{
			Ex_Data[0] = 0x05;
			Ex_Data[1] = 0xDA;      //������������
			if((Ex_Data[0] == 0) && (Ex_Data[1] == 0))    //�����������û����У�����򷵻���������
			{                                                //Ed���ݣ���ʾ��λ������AT 12����
				mem_copy(oilsend_buffer,"$AT 05 00NOEdDATA\n");
				oilsend_buffer[7] = 0;
				oilsend_buffer[8] = 8;
				SCI_Transmit(1,18,oilsend_buffer);
			}
			else
			{
				Main_send(0x01,0x0B);
				for(x=0;x<btMsg.ECUNumber;x++)
				{
					AirFlow = btMsg.tx[x][2];				
				}
				Main_send(0x01,0x0C);
				for(x=0;x<btMsg.ECUNumber;x++)
				{
          Espeed = ((uint16_t)btMsg.tx[x][2]<<8) + btMsg.tx[x][3];					
				}
				Main_send(0x01,0x0D);
				for(x=0;x<btMsg.ECUNumber;x++)
				{
					Speed = btMsg.tx[x][2];				
				}
				Main_send(0x01,0x0F);   
				for(x=0;x<btMsg.ECUNumber;x++)
				{
					Temp = btMsg.tx[x][2]-40;				
				}
				Ed = ((uint16_t)Ex_Data[0]<<8) + Ex_Data[1];
				Oil_data = (uint16_t)(((AirFlow/(Temp+273.15))*Espeed*Ed)/(543.214*Speed));
				Oil_data1 = (uint16_t)(((AirFlow/(Temp+273.15))*Espeed*Ed)/(54.3214*Speed));
				Oil_ave1 += Oil_data1;
			  Oilloop_time++;
			  Oil_ave = (uint16_t)(Oil_ave1/(10*Oilloop_time));
				Oil_this = (uint16_t)(Oil_ave*Mile_one/100000); //���κ�����
				mem_copy(oilsend_buffer,"$AT 05 ");
				oilsend_buffer[7] = 0;
				oilsend_buffer[8] = 4;
				oilsend_buffer[9] = (char)((Oil_data & 0xFF00) >> 8);
				oilsend_buffer[10] = (char)(Oil_data & 0xFF);
				oilsend_buffer[11] = (char)((Oil_ave & 0xFF00) >> 8);
			  oilsend_buffer[12] = (char)(Oil_ave & 0xFF);
			  oilsend_buffer[13] = '\n';
			  SCI_Transmit(1,14,oilsend_buffer);
			}
		}			
	}
  else      //С�ڵ��ٵ��������Ϩ������ϵ�״̬ 
  {
		Oil_all += Oil_this;   //ȡÿһ��Ϩ��ǰһ�εı��κ������ۼӼ�Ϊ�ܺ����� 
		Oil_this_gprs = Oil_this;
		Oil_aveall = (uint16_t)(Oil_all*1000000/Mile_all);  //��ƽ���ͺ�
		Oil_this = 0;
		Oil_ave1 = 0;
		Oilloop_time = 0;
	}		
}


/*******************************************************************************
* Function Name  : Max_speed

* Input          : None
* Output         : None
* Return         : None
* Description    : ����ת���ж�
*******************************************************************************/
void Max_speed(void)
{
	RTC_timer time_espeed;
  RTC_timer time_vspeed;	
	
	uint16_t Espeed_this = 0;     //��ǰת��
	uint8_t Vspeed_this = 0,y;      //��ǰ����
	
	if(car_state >= IDLE)
	{
		Main_send(0x01,0x0C);
		for(y=0;y<btMsg.ECUNumber;y++)
		{
			Espeed_this = ((uint16_t)btMsg.tx[y][2]<<8) + btMsg.tx[y][3];					
		}
		if(Max_Espeed_this < Espeed_this)  //��¼���ת�ٷ���ʱ��
		{
			get_time(&time_espeed);
			Espeed_hour = (uint8_t)time_espeed.hour;
			Espeed_minute = (uint8_t)time_espeed.minute;
			Espeed_second = (uint8_t)time_espeed.second;
		}
		Max_Espeed_this = (Max_Espeed_this > Espeed_this) ? Max_Espeed_this:Espeed_this;
		Max_Espeed_all = (Max_Espeed_all > Max_Espeed_this) ? Max_Espeed_all:Max_Espeed_this;
		
		Main_send(0x01,0x0D);
		for(y=0;y<btMsg.ECUNumber;y++)
		{
			Vspeed_this = btMsg.tx[y][2];				
		}
		if(Max_Vspeed_this < Vspeed_this)  //��¼��߳��ٷ���ʱ��
		{
			get_time(&time_vspeed);
			Vspeed_hour = (uint8_t)time_vspeed.hour;
			Vspeed_minute = (uint8_t)time_vspeed.minute;
			Vspeed_second = (uint8_t)time_vspeed.second;
		}
		Max_Vspeed_this = (Max_Vspeed_this > Vspeed_this) ? Max_Vspeed_this:Vspeed_this;
		Max_Vspeed_all = (Max_Vspeed_all > Max_Vspeed_this) ? Max_Vspeed_all:Max_Vspeed_this;
	}
}




