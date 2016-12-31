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
/*******************************************小周*****************************************/
extern uint8_t Soft_Version;  //软件版本   
extern uint8_t Hard_Version;  //硬件版本   
extern uint8_t Mil_Data[3];  //里程数据
extern uint8_t Ex_Data[2];   //排气量  
extern uint8_t Max_Vspeed_this;           //本次最高车速
extern uint8_t Max_Vspeed_all;           //历史最高车速

extern uint16_t Oil_ave;     //本次平均油耗 
extern uint16_t Oil_this;    //本次耗油量
extern uint16_t Oil_this_gprs;  //用于GPRS上传的本次耗油量
extern uint16_t Oil_aveall;   //总平均油耗
extern uint16_t Oil_all;     //总耗油量
extern uint16_t Max_Espeed_this;            //本次最高转速
extern uint16_t Max_Espeed_all;            //历史最高转速

extern uint32_t Ter_Version; //终端编号  
extern uint32_t Oilloop_time;
extern uint32_t Oil_ave1;
//extern uint32_t Milestone[3];   //里程数组
extern uint32_t Mile_one;
extern uint32_t Mile_all;
//extern uint32_t Oil_all;     //总耗油量
extern uint32_t timelen_idle_one_at; //AT 15命令获取的本次怠速时长
extern uint32_t timelen_driving_one_at;  //AT 15命令获取的本次行驶时长

uint8_t Espeed_hour = 0;   //本次最高转速时间小时
uint8_t Espeed_minute = 0;
uint8_t Espeed_second = 0;
uint8_t Vspeed_hour = 0;   //本次最高车速发生时间 小时
uint8_t Vspeed_minute = 0;
uint8_t Vspeed_second = 0;

/*******************************************小周*****************************************/

/*******************************************************************************
* Function Name  : BLE_Main
* Input          : None
* Output         : None
* Return         : None
* Description    : 根据蓝牙命令采集车辆数据主程序
*******************************************************************************/
void BLE_Main(void)
{
	uint8_t frame,sid,i,j,k,len = 0;
	static uint8_t pos = 0, BLE_buffer[250];
	uint8_t send_buffer[200] = {"$OBD ECUOFF\n"};
	uint8_t file_buffer[50];
	uint16_t vol;
//	short aacx = 0,aacy = 0,aacz = 0;		//加速度传感器原始数据
//	short gyrox = 0,gyroy = 0,gyroz = 0;	//陀螺仪原始数据

//	OilCount();
	TIMELENGTH();
	Max_speed();
	mpu6050_send_idledata();
	mpu6050_judge();
//	MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
//	MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
//	mpu6050_send_data(aacx,aacy,aacz,gyrox,gyroy,gyroz);
	
	
	len=SCI_GetLen(1,0); //查看蓝牙有无数据缓冲
  if(len>0)            //只要收到数就将其拿出
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
					if(sysMode == NONE)   //车子未发动
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
					
					if((btMsg.rx[0] == 1)&&(btMsg.rx[1]	== 0x42)&&((supptemp.v0140 & 0x40000000) == 0))//电压
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
							ISO15765Main();//使用CAN则将数据转化为CAN数据包,并发送;同时将CAN接收的数据转化      
						else if((sysMode == K_14230_5) || (sysMode == K_14230_fast))
							ISO14230Main();	//同上
						else if(sysMode == K_9141_5)
							ISO9141Main();
					}
					btMsg.state=COMM_IDLE;
        						 
					if(proMsg.state==COMM_RX_OK) //车辆数据发送并接收完毕，准备返回蓝牙
					{		
            BtSend();			//蓝牙回复接收到的数据                   
            proMsg.state=COMM_IDLE;
					}
					else 					//车辆数据采集失败，即NO_DATA
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
			AT命令：
			$AT 01\n：OBD协议
			$AT 02\n：车辆状态
			$AT 03\n：支持码
			$AT 04\n：OBD命令（停止大循环）
			$AT 05\n：瞬间油耗
			$AT 06\n：尾气排放（PID01.cfg）
			$AT 07\n：北斗（主动）
			$AT 08\n：故障码（主动）
			$AT 09\n：偶发故障码（主动）
			$AT 10\n：文件传输（配置，升级）
			$AT 11 Len1 Len2 里程值\n：里程校正   
			$AT 12 Len1 Len2 排气量\n：排气量输入，用于油耗计算   
			$AT 13\n：终端信息查询
			$AT 15\n：发送本次怠速时长、本次行驶时长、本次行驶里程（主动）
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
					case 3:	mem_copy(send_buffer,"$AT 03 00");  	//支持码			//$AT 03 xx xx xx xx xx xx xx xx xx xx xx xx		
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
					case 4:	//$AT 04\n：停止OBD大循环
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
					case 5: //瞬间油耗			
									OilCount();
							break;
					case 6: //$AT 06\n：获取尾气排放的OBD数据
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
						case 13: mem_copy(send_buffer,"$AT 13 00");     //终端信息查询
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
			
			if((!mem_compare(&BLE_buffer[i], "$AT 11 "))&&(BLE_buffer[i+12] == '\n'))   //  里程校正
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
			
			if((!mem_compare(&BLE_buffer[i], "$AT 12 "))&&(BLE_buffer[i+11] == '\n'))   //排气量校正
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
* Description    : 文件传输与空中升级
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
		len=SCI_GetLen(1,0); //查看蓝牙有无数据缓冲
		if(len>0)            //只要收到数就将其拿出
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
					num = ((uint16_t)BLE_buffer[i+7]<<8) + BLE_buffer[i+8];//字节数
					now = ((uint16_t)BLE_buffer[i+9]<<8) + BLE_buffer[i+10];//当前序号
					all = ((uint16_t)BLE_buffer[i+11]<<8) + BLE_buffer[i+12];//总序号
					sum = ((uint16_t)BLE_buffer[i+13]<<8) + BLE_buffer[i+14];//校验
					
					if(BLE_buffer[i+num+9] == '\n')//数据完整
					{					
						temp = 0;
						for(j=0;j<num-6;j++)
							temp += BLE_buffer[j+i+15];
						if(sum != (uint16_t)(temp&0xFFFF))//校验出错
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
								res = f_write(&file, &BLE_buffer[i+15], num-6, &rc);//写入
								f_sync(&file);
								mem_copy(send_buffer,"$AT 10 00OK\n");
								BLE_buffer[i+11] = (uint8_t)((temp&0xFF00)>>8);
								BLE_buffer[i+12] = (uint8_t)(temp&0xFF);
								send_buffer[7] = 0;
								send_buffer[8] = 6;
								SCI_Transmit(1,9,send_buffer);
								SCI_Transmit(1,4,&BLE_buffer[i+9]);
								SCI_Transmit(1,3,&send_buffer[9]);
								if(now == all)//传输完成
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
* Description    : AT命令类主动发送（BLE）:
					故障0		$AT 08 00 0D 06 01 43 01 96 02 34 02 CD 03 57 0A 24\n	
					故障1		$AT 09 00 0D 06 01 43 01 96 02 34 02 CD 03 57 0A 24\n	
*******************************************************************************/
uint8_t AT_master_BLE(uint8_t cmd) 
{
	uint8_t i,buffer[100] = {"$AT 08 "};

	switch(cmd)
	{
		case 0:	buffer[5] = '8';
										buffer[9] = (uint8_t)DTC_now[0];//故障数量
										if(buffer[9] == 0)				//无故障
										{
											mem_copy(&buffer[9],"NODTC\n");
											buffer[7] = 0;
											buffer[8] = 5;
											SCI_Transmit(1,15,buffer);
											return 0;
										}
										else
										{
											for(i=1;i<=DTC_now[0];i++)	//故障码
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
										buffer[9] = (uint8_t)DTC_now[0];//故障数量
										if(buffer[9] == 0)				//无故障
										{
											mem_copy(&buffer[9],"NODTC\n");
											buffer[7] = 0;
											buffer[8] = 5;
											SCI_Transmit(1,15,buffer);
											return 0;
										}
										else
										{
											for(i=1;i<=DTC_now[0];i++)	//故障码
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
* Description    : 蓝牙OBD命令回复
*******************************************************************************/
void BtSend(void)
{
    uint8_t i;
    uint8_t j;
		uint8_t header[6] = {"$OBD  "};
		uint8_t sum='\n';
    
    btMsg.state=COMM_SEND;							//设置蓝牙的状态为发送中    
    for(i=0;i<btMsg.ECUNumber;i++)					//根据ECU数目回复
    {
        if(btMsg.BtTxState[i] == 1)
        {
            if(btMsg.FrameNum[i] == 1)				//单帧数据
            {						
								header[5] = i+1;
								SCI_Transmit(1,6,header);					//数据头
								SCI_Transmit(1,1,&btMsg.txLen[i]);						//长度
								SCI_Transmit(1,btMsg.txLen[i],&btMsg.tx[i][0]);	//命令+数据
								SCI_Transmit(1,1,&sum);							//校验
												
								btMsg.BtTxState[i] = 0;

                btMsg.state=COMM_SEND_OK;
						}
						else if(btMsg.FrameNum[i] > 1)//多帧
						{	
								if((sysMode>=3)&&(sysMode<=5))//K-line
								{
									for(j=2;j<btMsg.txLen[i];j++)
											btMsg.tx[i][j] = btMsg.tx[i][j+1];//消掉头帧的序号，只发数据和PID
									btMsg.txLen[i] -= 1;
								}		

								header[5] = i+1;
								SCI_Transmit(1,6,header);					//数据头
								SCI_Transmit(1,1,&btMsg.txLen[i]);	//长度
								SCI_Transmit(1,btMsg.txLen[i],&btMsg.tx[i][0]);	//PID,第一帧数据		 
								SCI_Transmit(1,1,&sum);							//校验
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
* Description    : 瞬间油耗计算
*******************************************************************************/
void OilCount(void)
{
	uint8_t Speed = 0,x;  //车速   
  uint8_t AirFlow = 0; //进气管绝对压力
  uint8_t Temp = 0;    //进气温度
	uint8_t oilsend_buffer[100]= {"$AT 05 "};
	
  uint16_t Oil_data = 0;     //瞬间油耗 	
	uint16_t Oil_data1 = 0;    //用于算平均油耗的瞬间油耗值  
	uint16_t Quality = 0;  //进气质量流量
  uint16_t Espeed = 0;  //发送机转速
	uint16_t Ed = 0;   //用于计算油耗的排气量  
	
	
	if(car_state >= IDLE)
	{
		if(car_state == IDLE)    //怠速时油耗为0
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
		else if((supptemp.v0100 & 0x00010000) != 0)   //支持进气质量流量0110
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
			Oil_ave = (uint16_t)(Oil_ave1/(10*Oilloop_time));  //本次平均油耗
			Oil_this = (uint16_t)(Oil_ave*Mile_one/100000); //本次耗油量
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
		else      //不支持进气质量流量 0110
		{
			Ex_Data[0] = 0x05;
			Ex_Data[1] = 0xDA;      //测试王经车用
			if((Ex_Data[0] == 0) && (Ex_Data[1] == 0))    //如果排气量还没更新校正，则返回无排气量
			{                                                //Ed数据，提示上位机输入AT 12命令
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
				Oil_this = (uint16_t)(Oil_ave*Mile_one/100000); //本次耗油量
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
  else      //小于怠速的情况，即熄火或者上电状态 
  {
		Oil_all += Oil_this;   //取每一次熄火前一次的本次耗油量累加即为总耗油量 
		Oil_this_gprs = Oil_this;
		Oil_aveall = (uint16_t)(Oil_all*1000000/Mile_all);  //总平均油耗
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
* Description    : 车速转速判断
*******************************************************************************/
void Max_speed(void)
{
	RTC_timer time_espeed;
  RTC_timer time_vspeed;	
	
	uint16_t Espeed_this = 0;     //当前转速
	uint8_t Vspeed_this = 0,y;      //当前车速
	
	if(car_state >= IDLE)
	{
		Main_send(0x01,0x0C);
		for(y=0;y<btMsg.ECUNumber;y++)
		{
			Espeed_this = ((uint16_t)btMsg.tx[y][2]<<8) + btMsg.tx[y][3];					
		}
		if(Max_Espeed_this < Espeed_this)  //记录最高转速发生时间
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
		if(Max_Vspeed_this < Vspeed_this)  //记录最高车速发生时间
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




