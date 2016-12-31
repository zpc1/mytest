/**
  ******************************************************************************
  * @file    iGPRS.c 
  * @author  PDAger iCar team
  * @version V0.5.0
  * @date    29-November-2011
  * @brief   OBD SIM900A loop.
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "iGPRS.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern CARprotocol  sysMode;
extern uint32_t KComRem;
extern MSG_COMM_BT btMsg;
extern MSG_COMM proMsg;
extern uint8_t timeInval;
extern uint32_t sysClock;
extern ECUstat car_state;
extern uint16_t time_m;
extern uint8_t pid_save;
extern uint8_t ID_CFG[5];
extern uint8_t mpu6050_buf[];
extern uint8_t WqData[];
extern uint8_t timetotal;
extern MSG_COMM_GPRS GPRS;
extern uint8_t led_flag;
extern PIDsupport support;
extern uint8_t GPS_databuffer[];
extern uint8_t DTC_num;
extern uint16_t DTC_now[];
extern vu16 ADC_ConvertedValue[];

extern TIMELEN timelen;
extern uint32_t timelen_idle_one_gprs;
extern uint32_t timelen_driving_one_gprs;
extern uint32_t Mile_one;
extern uint32_t Mile_all;
extern uint8_t Max_Vspeed_this;
extern uint8_t Vspeed_hour;   //本次最高车速发生时间 小时
extern uint8_t Vspeed_minute;
extern uint8_t Vspeed_second;
extern uint8_t Max_Vspeed_all;
extern uint16_t Max_Espeed_this;            //本次最高转速
extern uint8_t Espeed_hour;   //本次最高转速时间小时
extern uint8_t Espeed_minute;
extern uint8_t Espeed_second;
extern uint16_t Max_Espeed_all;
extern uint16_t Accela_this_times;         //本次急加速次数
extern uint32_t Accela_all_times;    //累积急加速次数
extern uint16_t Brake_this_times;         //本次急刹车次数
extern uint32_t Brake_all_times;     //累积急刹车次数
extern uint16_t Turn_this_times;
extern uint32_t Turn_all_times;
extern uint32_t Ignition_times;
extern uint16_t Oil_ave;      //本次平均油耗  
extern uint16_t Oil_this_gprs;     //本次耗油量 
extern uint16_t Oil_all;
extern uint16_t Oil_aveall;    //总平均油耗
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : GPRS_Main
* Input          : None
* Output         : None
* Return         : None
* Description    : GPRS动作主程序，根据GPRS标识进行相应动作：连接、等待连接、发数、重启等
*******************************************************************************/
void GPRS_Main(void)
{
#ifdef GPRS_ON
	switch(GPRS.step)
	{
		case CONNECTING:GPRS_connect();		 //GPRS连接
			break;
				
		case CHECK: 		GPRS_check_connect();//等待连接成功（SIM900A专有）
			break;

		case DATA:			GPRS_Data();		 //GPRS接收、发送数据
			break;

		case REBOOT: 		GPRS_reset();		//GPRS硬件复位
			break;

		default:printf("gprsflag = %d\n",GPRS.step);
				GPRS.step = CHECK;
			break;	
	}
#endif						 		 
}

/*******************************************************************************
* Function Name  : GPRS_Data
* Input          : None
* Output         : None
* Return         : None
* Description    : GPRS发数主程序，动态数据和熄火数据
*******************************************************************************/
void GPRS_Data(void)
{
	uint16_t i;
	uint8_t j,temp;
	RTC_timer timegprs;
	static uint8_t obd_buffer[30];
	uint8_t save_buffer[83];
	static uint8_t send_buffer[200];
	static uint8_t gps_buffer[200];
	static uint8_t TimeRem_GPRS = 0, GPRS_flag = 1,fail_flag = 0,wq_first = 1;
	static uint32_t TimeRem_GPRS1 = 0;
	static uint32_t TimeRem_GPRS2 = 0;
	static uint32_t TimeRem_GPRS3 = 0;
	static uint32_t TimeRem_SENDOK = 0;
	static uint8_t gps_num = 0,TimeRem_GPS = 0;
	uint16_t vol;
   	
	if(car_state < 1) 
	{
		if(TimeRem_GPRS)//熄火帧&电量检测
		{
			printf("iGPRS: send xihuo buffer\n");
			mem_copy(save_buffer,"$AT 05 ");
			save_buffer[7] = 0;
			save_buffer[8] = 19;
			for(i=9;i<13;i++)
				save_buffer[i] = ID_CFG[i-9];
			get_time(&timegprs);			
			save_buffer[13] = timegprs.hour;
			save_buffer[14] = timegprs.minute;
			save_buffer[15] = timegprs.second;
			save_buffer[16] = 0x41;
			save_buffer[17] = 0x42;
			vol = (uint32_t)ADC_ConvertedValue[0]*46/10+700;
			save_buffer[18] = (uint8_t)(vol>>8);
			save_buffer[19] = (uint8_t)vol&0xFF;
			for(i=4;i<12;i++)
				save_buffer[16+i] = GPS_databuffer[i];
			save_buffer[28] = '\n';		
			GPRS_Send_CMD(29);
			DelayMs(100);									
			SCI_Transmit(3,29,save_buffer);	//发送数据
			DelayMs(5000);
			GPRS_check_status();
			IWDG_ReloadCounter();//喂狗	
			mem_copy(save_buffer,"$AT 14 ");//驾驶习惯
			save_buffer[7] = 0;
			save_buffer[8] = 75;
			get_time(&timegprs);			
			save_buffer[13] = timegprs.hour;
			save_buffer[14] = timegprs.minute;
			save_buffer[15] = timegprs.second;
			mem_copy32(&save_buffer[16],timelen_idle_one_gprs);
			mem_copy32(&save_buffer[20],timelen_driving_one_gprs);
			mem_copy32(&save_buffer[24],timelen.idle_all);
			mem_copy32(&save_buffer[28],timelen.driving_all);
			mem_copy32(&save_buffer[32],Mile_one);
			mem_copy32(&save_buffer[36],Mile_all);
			save_buffer[40] = Max_Vspeed_this;
			save_buffer[41] = Vspeed_hour;
			save_buffer[42] = Vspeed_minute;
			save_buffer[43] = Vspeed_second;
			save_buffer[44] = Max_Vspeed_all;
			mem_copy16(&save_buffer[45],Max_Espeed_this);
			save_buffer[47] = Espeed_hour;
			save_buffer[48] = Espeed_minute;
			save_buffer[49] = Espeed_second;
			mem_copy16(&save_buffer[50],Max_Espeed_all);
			mem_copy16(&save_buffer[52],Accela_this_times);
			mem_copy32(&save_buffer[54],Accela_all_times);
			mem_copy16(&save_buffer[58],Brake_this_times);
			mem_copy32(&save_buffer[60],Brake_all_times);
			mem_copy16(&save_buffer[64],Turn_this_times);
			mem_copy32(&save_buffer[66],Turn_all_times);
			mem_copy32(&save_buffer[70],Ignition_times);
			mem_copy16(&save_buffer[74],Oil_ave);
			mem_copy16(&save_buffer[76],Oil_this_gprs);
			mem_copy16(&save_buffer[78],Oil_all);
			mem_copy16(&save_buffer[80],Oil_aveall);
			save_buffer[82] = '\n';
			GPRS_Send_CMD(83);
			DelayMs(100);									
		//	SCI_Transmit(1,83,save_buffer);	//发送数据
			SCI_Transmit(3,83,save_buffer);	//发送数据
			DelayMs(5000);
			IWDG_ReloadCounter();//喂狗	
			TimeRem_GPRS = 0;	
			GPRS_check_status();
			
			PWRKEY_DOWN
			DelayMs(2000);
			PWRKEY_UP//关机
			DelayMs(10000);
			IWDG_ReloadCounter();//喂狗	
			for(i=0;i<30;i++)
				mpu6050_buf[i] = 0;
			GPRS.sendok = 1;
			timelen_idle_one_gprs = 0;
			timelen_driving_one_gprs = 0;
				
			printf("iGPRS: xihuo buffer end\n");
		}
		
		if(mpu6050_buf[8] != 0)//只要有数就发
		{
			printf("iGPRS: mpu6050 buf\n");
			if((GPRS_flag == 1)&&(!GPRS_check_status()))
			{
				GPRS_Send_CMD(30);
				DelayMs(100);									
				SCI_Transmit(3,30,mpu6050_buf);	//发送数据
				GPRS.sendok = 0;
				for(i=0;i<30;i++)
					mpu6050_buf[i] = 0;
				DelayMs(5000);
				IWDG_ReloadCounter();//喂狗	
				GPRS_check_status();
				PWRKEY_DOWN
				DelayMs(2000);
				PWRKEY_UP//关机
				DelayMs(10000);		
				printf("iGPRS: mpu6050 buf end\n");
				GPRS.sendok = 1;
			}
			else
			{
				printf("iGPRS: reboot\n");
				GPRS.step = REBOOT;
			}
		}
		
		TimeRem_GPRS1 = sysClock;
		TimeRem_GPRS2 = sysClock;
	}
	else if((car_state > 1)&&(sysMode > NONE))
	{		
		TimeRem_GPRS = 1;
		GPRSRx();		
		
	/*	temp = GPRS_check_status();
		if(temp > 1)//判断GPRS连接状态
		{	
			GPRS.sendok = 1;
			GPRS.step = REBOOT;		  //失去连接，直接硬启
			return;
		}
		else if(temp == 1)
		{
			GPRS.sendok = 1;
			GPRS.step = CONNECTING;
			return;
		}
		if(GPRS.sendok == 0)//确认前次数据发送成功
		{
			if(sysClock - TimeRem_SENDOK > 6000)
			{
			}
			fail_flag++;
			if((fail_flag >= 100)||(0 != GPRS_check_status()))	  //连发10次失败，准备重启SIM900A
			{
				printf("iGPRS: check send failed\n");
				GPRS.step = REBOOT;
			}
		}
		else
			TimeRem_SENDOK = sysClock;*/
			
			
		if((btMsg.tx[0][0] == 0x41)&&(btMsg.tx[0][1] == 0x01))//0101
		{
			for(i=14;i<20;i++)
				obd_buffer[i] = btMsg.tx[0][i-14];
		}
		if((btMsg.tx[0][0] == 0x41)&&(btMsg.tx[0][1] == 0x21))//0121
		{
			for(i=20;i<24;i++)
				obd_buffer[i] = btMsg.tx[0][i-20];
		}
		if(((sysClock - TimeRem_GPRS1) > 30000)&&(GPRS.sendok == 1))//&&(obd_buffer[21] != 0)&&(obd_buffer[15] != 0))//30s发OBD
		{
			printf("iGPRS: send OBD 30s\n");
			led_flag = 3;
			TimeRem_GPRS1 = sysClock;
			mem_copy(obd_buffer,"$OBD ");
			obd_buffer[5] = 0;
			obd_buffer[6] = 17;
			for(i=7;i<11;i++)
				obd_buffer[i] = ID_CFG[i-7];
			get_time(&timegprs);			
			obd_buffer[11] = timegprs.hour;
			obd_buffer[12] = timegprs.minute;
			obd_buffer[13] = timegprs.second;
			obd_buffer[24] = '\n';
			
			if(!GPRS_check_status())
				GPRS_flag = 1;
			else
			{
				GPRS.step = REBOOT;
				return;
			}	
			
			for(i=0;i<290;i++)
				GPRS.msg[i] = 0;
			GPRS_Send_CMD(25);
			DelayMs(100);									
			SCI_Transmit(3,25,obd_buffer);	//发送数据		
			GPRS.sendok = 0;
			for(i=0;i<25;i++)
				obd_buffer[i] = 0;
		}
		if(((wq_first)||((sysClock - TimeRem_GPRS2) > 1800000))&&(GPRS.sendok == 1))//30min发尾气
		{		
			TimeRem_GPRS2 = sysClock;			
			if(WqData[0] > 0)
			{
				printf("iGPRS: send weiqi 30min:%d\n",WqData[0]);
				wq_first = 0;
				led_flag = 3;
				mem_copy(send_buffer,"$AT 06 ");
				send_buffer[7] = 0;
				send_buffer[8] = WqData[0]+7;
				for(i=9;i<13;i++)
					send_buffer[i] = ID_CFG[i-9];
				get_time(&timegprs);			
				send_buffer[13] = timegprs.hour;
				send_buffer[14] = timegprs.minute;
				send_buffer[15] = timegprs.second;
				for(i=0;i<WqData[0];i++)
					send_buffer[i+16] = WqData[i+1];
				send_buffer[WqData[0]+16] = '\n';
				
				if(!GPRS_check_status())
					GPRS_flag = 1;
				else
				{
					GPRS.step = REBOOT;
					return;
				}	
				
				for(i=0;i<290;i++)
					GPRS.msg[i] = 0;
				GPRS_Send_CMD(WqData[0]+17);
				DelayMs(100);									
				SCI_Transmit(3,WqData[0]+17,send_buffer);	//发送数据
				GPRS.sendok = 0;	
			}
		}
		if(TimeRem_GPS != GPS_databuffer[0])//攒15次发定位数据
		{
			if(gps_num < 15)
			{
				TimeRem_GPS = GPS_databuffer[0];
				gps_num++;
				for(i=1;i<13;i++)
					gps_buffer[12+i+(gps_num-1)*12] = GPS_databuffer[i];
			}
			else 
			{
				if(GPRS.sendok == 1)
				{
					printf("iGPRS: send GPS\n");
					led_flag = 3;
					gps_num = 0;
					mem_copy(gps_buffer,"$AT 07 ");
					gps_buffer[7] = 0;
					gps_buffer[8] = 184;
					for(i=9;i<13;i++)
						gps_buffer[i] = ID_CFG[i-9];
					gps_buffer[193] = '\n';
					
					if(!GPRS_check_status())
						GPRS_flag = 1;
					else
					{
						GPRS.step = REBOOT;
						return;
					}
					
					for(i=0;i<290;i++)
						GPRS.msg[i] = 0;
					GPRS_Send_CMD(194);
					DelayMs(100);									
					SCI_Transmit(3,194,gps_buffer);	//发送数据
					GPRS.sendok = 0;					
				}
			}
		}
		if(((sysClock - TimeRem_GPRS3) > 60000)&&(GPRS.sendok == 1)&&(DTC_now[0] > 0))//1min发故障码
		{
			led_flag = 3;
			TimeRem_GPRS3 = sysClock;
			switch(DTC_num)
			{
				case 1:mem_copy(send_buffer,"$AT 08 ");
					break;
				case 2:mem_copy(send_buffer,"$AT 09 ");
					break;
				default:return;
			}
			printf("iGPRS: send DTC\n");
			for(i=9;i<13;i++)
			send_buffer[i] = ID_CFG[i-9];
			get_time(&timegprs);			
			send_buffer[13] = timegprs.hour;
			send_buffer[14] = timegprs.minute;
			send_buffer[15] = timegprs.second;
			send_buffer[16] = (uint8_t)DTC_now[0];//故障数量
			for(i=1;i<=DTC_now[0];i++)	//故障码
			{								  
				send_buffer[16+i*2-1] = (uint8_t)((DTC_now[i] & 0xFF00) >> 8);
				send_buffer[16+i*2] = (uint8_t)(DTC_now[i] & 0xFF);
			}								
			send_buffer[7] = 0;
			send_buffer[8] = i*2+6;
			send_buffer[16+i*2-1] = '\n';
			
			if(!GPRS_check_status())
				GPRS_flag = 1;
			else
			{
				GPRS.step = REBOOT;
				return;
			}
			
			GPRS_Send_CMD(16+i*2);
			DelayMs(100);									
			SCI_Transmit(3,16+i*2,send_buffer);	//发送数据
			for(i=0;i<290;i++)
				GPRS.msg[i] = 0;
			GPRS.sendok = 0;
		}
	}
}

/*******************************************************************************
* Function Name  : GPRSRx
* Input          : None
* Output         : None
* Return         : None
* Description    : GPRS收数主程序，处理AT命令或OBD命令，可参考iCar.c
*******************************************************************************/
void GPRSRx(void)//接收命令，处理成OBD指令发给ECU并接收数据，发回GPRS
{
	uint8_t i,j,k,len = 0;
	uint8_t send_buffer[100] = {"$OBD ECUOFF\n"};
    
  len=SCI_GetLen(3,0);
  if(len>0)        
  {   	
		if(GPRS.pos >= 250)
			GPRS.pos = 0;
		SCI_Receive(3,len,&GPRS.msg[GPRS.pos]);
		GPRS.pos+=len;
		
		for(i=0;i<245;i++)
		{
			if((!mem_compare(&GPRS.msg[i], "$AT "))&&(GPRS.msg[i+6] == '\n'))//GPRS-AT 	
			{
				switch((GPRS.msg[i+4] - '0')*10+(GPRS.msg[i+5] - '0'))
				{
					default:break;
				}
			}
			if(!mem_compare(&GPRS.msg[i], "SEND OK")) 		//SEND OK
			{
				printf("iGPRS:SEND OK\n");
				led_flag = 2;
				GPRS.sendok = 1;
			}
		}
	}
}

/*******************************************************************************
* Function Name  : GPRSSend
* Input          : None
* Output         : None
* Return         : None
* Description    : GPRS被动发送程序（OBD数据）
*******************************************************************************/
void GPRSSend(void)
{
    uint8_t i;
    uint8_t j;
		uint8_t header[] = {0x5A,0xA5};
		uint8_t lenth,sum;
		uint32_t temp = 0;

		if (led_flag == 2)
			led_flag = 3;
    
    btMsg.state=COMM_SEND;//设置蓝牙的状态为发送中    
    for(i=0;i<btMsg.ECUNumber;i++)
    {
        if((btMsg.BtTxState[i] == 1)&&(((btMsg.tx[i][1] == pid_save)&&(btMsg.tx[i][0] == 0x41))||((btMsg.tx[i][0] != 0x41))))
        {
            if(btMsg.FrameNum[i] == 1)//单帧数据
            {
								temp = 0;
								lenth = btMsg.txLen[i] + 5;//加上sum位和终端符
								for(j=0;j<btMsg.txLen[i];j++)
									temp += btMsg.tx[i][j];	
								temp += lenth;	
								sum = (uint8_t)(temp & 0xFF); //和校验值

								if(GPRS_Send_CMD(8+btMsg.txLen[i]) != 0)
									return;			
								
								SCI_Transmit(3,2,&header[0]);					//数据头0x5AA5
								SCI_Transmit(3,1,&lenth);						//长度	 
								SCI_Transmit(3,4,&ID_CFG[0]);
								SCI_Transmit(3,btMsg.txLen[i],&btMsg.tx[i][0]);	//命令+数据
								SCI_Transmit(3,1,&sum);							//校验
												
								btMsg.BtTxState[i] = 0;
								GPRS.sendok = 0;
                btMsg.state=COMM_SEND_OK;
            }
            else if(btMsg.FrameNum[i] > 1)//多帧
            {	
								if((sysMode >= K_9141_5)&&(sysMode <= K_14230_fast))//K-line
								{
									for(j=2;j<btMsg.txLen[i];j++)
											btMsg.tx[i][j] = btMsg.tx[i][j+1];//消掉头帧的序号，只发数据和PID
									btMsg.txLen[i] -= 1;
								}

								lenth = btMsg.txLen[i] + 5;
								for(j=0;j<lenth;j++)					
									temp+=btMsg.tx[i][j];
								temp += lenth;
								sum = (uint8_t)(temp & 0xFF); 					//和校验值				

								if(GPRS_Send_CMD(8+btMsg.txLen[i]) != 0)
									return;
								
								SCI_Transmit(1,2,&header[0]);					//数据头0x5AA5
								SCI_Transmit(1,1,&lenth);						//长度	
								SCI_Transmit(3,4,&ID_CFG[0]);
								SCI_Transmit(1,btMsg.txLen[i],&btMsg.tx[i][0]);	//PID,数据		 
								SCI_Transmit(1,1,&sum);							//校验
								GPRS.sendok = 0;
								btMsg.state=COMM_SEND_OK;
            }//END (btMsg.FrameNum[i]>1)
        }//END (btMsg.BtTxState[i]==1)
    }//END (i=0;i<btMsg.ECUNumber;i++)   
	
	if (led_flag == 3)
		led_flag = 2;   
}
