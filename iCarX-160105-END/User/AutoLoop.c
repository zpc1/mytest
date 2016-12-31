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

/*******************************************小周***************************************/
extern uint32_t Ignition_times;

/*******************************************小周***************************************/

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : OBD_Main
* Input          : None
* Output         : None
* Return         : None
* Description    : 主动采集循环，根据车辆状态进行动作并判断新的车辆状态
					熄火：无协议
					上电：有协议但发动机转速（PID：010C）为零
					怠速：发动机转速不为零但车速（PID：010D）为零
					行驶：车速不为零
*******************************************************************************/
extern uint32_t send,recv;
void OBD_Main(void)
{
	K_Reminder();		//K线心跳
	
	if(OBDLoopFlag == 0)
		return;
	
	switch(car_state)			//根据车辆状态动作
	{
		case ECU_OFF:				//车辆状态：熄火，准备搜寻汽车总线协议 
					if(time_m>=2)				   	//等待10s搜一次协议
					{						
						ProtocolSearch();		   	//搜索协议
						printf("AutoLoop: search sysMode = %d\n",sysMode);
						if(sysMode == NONE)			//没有搜到协议
						{
							printf("AutoLoop: search failed\n");
							//printf("send recv = %d, %d\n",send,recv);
							car_state = ECU_OFF;   //无协议，仍为熄火状态
							time_m = 0;			   //继续等待10s
						}
						else					   //搜到协议了
							car_state = ECU_ON;	   //熄火——>上电
					}	
				break;

		case ECU_ON:				//车辆状态：上电，准备收集汽车ECU支持的PID（支持码） 
					if((support.v0100 == 0)&&(support.v0120 == 0)&&(support.v0140 == 0)&&(loop_time == 0))	
						first = 0;			//第一次开机，重新搜索支持码		
					if(loop_time <= 2)		//搜集标识：loop_time：0->1->2->3
						PID_check();		//寻找支持数据
					else
						car_state = state_judge();//支持码搜索完毕，开始判断车辆状态（主要是发动机转速及车速）	
					if(car_state >= IDLE)
					{
						WqFlag = 0;
						Ignition_times++;
					}		
				break;
		case IDLE:								//IDLE或DRIVING处理基本相同，都是采集、保存数据
		case DRIVING:
					 if((support.SID_now == 0)&&(support.PID_now == 0))//防止PID值异常（0101~05FF）
					 {
					 	support.SID_now = 1; //重置SID
						support.PID_now = 1; //重置PID
					 }					 
					 Cycle_Main(support.SID_now,support.PID_now);//动态数据与车辆状态循环					 
				break;
		default: car_state = ECU_OFF;
				break;
	}	
}
/*******************************************************************************
* Function Name  : Cycle_Main
* Input          : SID，PID
* Output         : None
* Return         : None
* Description    : 动态数据与车辆状态循环，调用采集、保存函数，判断车辆状态（怠速、行驶），保养提醒（王艳平）
*******************************************************************************/
void Cycle_Main(uint8_t SID,uint8_t PID)
{
	static uint8_t i,j,k;
	
	if(Main_send(SID,PID) == 0)//调用发送与接收函数，采集动态数据，若熄火则返回零
	{
		car_state = ECU_OFF;//熄火了
		sysMode = NONE;		//协议标识  
	}

	if((btMsg.tx[0][1] == 0x0C)&&(btMsg.tx[0][2] == 0)&&(btMsg.tx[0][3] == 0))//发动机转速（010C）为0
	{
		car_state = ECU_ON;//点火——>上电
	}	
	if((btMsg.tx[0][1] == 0x0D)&&(btMsg.tx[0][2] == 0)&&(car_state == DRIVING))//车速（010D）为0（行驶ing）
		car_state = IDLE;			//怠速
	if((btMsg.tx[0][1] == 0x0D)&&(btMsg.tx[0][2] > 0)&&(car_state == IDLE))//车速（010D）大于0（怠速ing）
		car_state = DRIVING;		//行驶											 
	if(car_state >= IDLE)//怠速以上时					
		DynInfCop();	 //PID处理，计算下一个PID（OBD或故障），并存储数据
	
	if(WqFlag == 1)//尾气检测
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
* Description    : 动态数据PID搜索、故障码与冻结帧PID搜索
*******************************************************************************/
void DynInfCop(void)	  
{	
	uint8_t i,j,ECU_flag,point,point_temp,freeze_PID_temp;

	if(support.SID_now == 0)		//即使知道没可能，还是写上了
		support.SID_now = 1;

	if(support.SID_now == 1)		//SID = 01，即动态数据循环
	{
		if(btMsg.tx[0][1] == 1)		//若是0101，则查找故障灯MIL
		{
			for(i=0;i<btMsg.ECUNumber;i++)		//已返回数据，开始解析
			{
				if(btMsg.tx[i][2] >= 0x80) 		//MIL为1，即有故障
				{
					support.SID_now = 3;		//准备发SID = 03（故障码循环）
					support.PID_now = 0;		//其实PID是不发的
					dtc_dog = 1;				//准备喂狗，防止突然间车辆熄火而死掉
				}
			}
			if(support.SID_now == 3)			//有故障
			{
				led_flag = 2;
				LED1_ON							//故障灯汪~
			}
			else								//无故障
			{
				DTC_num = 0;
				LED1_OFF						//故障灯咩~
			}  
		}
		if(support.SID_now == 1)				//无故障，则继续动态数据循环
		{
			dtc_dog = 0;						//不需要汪星人了
			support.PID_now = PID_chose(&support);//根据支持码与当前PID查找下一个PID
		}
	}//动态数据PID查找结束
	else if(support.SID_now == 7)//07偶发故障查询
	{
		point = 1;				//故障数量标识
		DTC_now[0] = 0;
		for(i=0;i<btMsg.ECUNumber;i++)//已经发过07，根据原始数据提取故障码
		{
			point_temp = point;//重置多ECU故障指针
			if(btMsg.tx[i][1] > 0)//43 06 0143 0196 0234 02CD 0357 0A24
			{					  
				if((sysMode >= CAN_STD_500) && (sysMode <= CAN_EXT_250))//CAN线提取
				{
					printf("AutoLoop: 07 dtc btMsg.len = %d,btMsg.tx = ",btMsg.txLen[i]);
					for(j=0;j<20;j++)
						printf("%1X ",btMsg.tx[i][j]);
					printf("\n");
					for(j = point_temp;j<100;j++)
					{  
						DTC_now[j] = ((uint16_t)btMsg.tx[i][1+(j-point_temp)*2] << 8) + btMsg.tx[i][2+(j-point_temp)*2];//原始故障码
						point++;								   //故障个数+1
						if((j - point_temp + 1) >= ((btMsg.txLen[i]-1)/2))//处理完了所有数据（btMsg.txLen[i]是数据长度）
							break; //跳出循环
					}
				}
				else if((sysMode >= K_9141_5) && (sysMode <= K_14230_fast))//K线，基本同上
				{
					for(j = point_temp;j<100;j++)
					{  
						DTC_now[j] = ((uint16_t)btMsg.tx[i][1+(j-point_temp)*2] << 8) + btMsg.tx[i][2+(j-point_temp)*2];//原始故障码
						point++;
						if((j - point_temp + 1) >= ((btMsg.txLen[i]-1)/2))
							break;
					}
				} 				
			}
			DTC_now[0] = point-1;	//故障数量
		}							
		if(DTC_now[0] > 0)//有偶发故障
		{					
			printf("AutoLoop: 07 DTC_now = ");
			for(j=0;j<20;j++)
				printf("%1X ",DTC_now[j]);
			printf("\n");		
			DTC_num = 2;
			AT_master_BLE(1);			//主动发送故障码						
		}
		else
			DTC_num = 0;

		support.PID_now = 1;//偶发故障搜索完毕，继续搜索动态数据
		support.SID_now = 1;
	}
	else 			//03永久故障码和冻结帧循环
	{
		if(support.SID_now == 3)//已经发过03，处理故障码，准备发020200采集冻结帧
		{
			point = 1;
			DTC_now[0] = 0;
			for(i=0;i<btMsg.ECUNumber;i++)//同偶发故障
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
							DTC_now[j] = ((uint16_t)btMsg.tx[i][1+(j-point_temp)*2] << 8) + btMsg.tx[i][2+(j-point_temp)*2];//原始故障码
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
							DTC_now[j] = ((uint16_t)btMsg.tx[i][1+(j-point_temp)*2] << 8) + btMsg.tx[i][2+(j-point_temp)*2];//原始故障码
							point++;
							if((j - point_temp + 1) >= ((btMsg.txLen[i]-1)/2))
								break;
						}
					}			
				} 
				DTC_now[0] = point - 1;
			}		
			if(DTC_now[0] > 0)//有故障
			{	
	/*			printf("AutoLoop: 03 DTC_now = ");
				for(j=0;j<20;j++)
					printf("%1X ",DTC_now[j]);
				printf("\n");*/
				//GPRS_DTC_message();
				AT_master_BLE(0);				//主动发送故障码
				DTC_num = 1;
				LED1_ON	//故障灯汪~										  
			}
			if(write_flag == 1)					//该故障是处哦，亲，需要继续插看冻结帧
			{	
				printf("AutoLoop: ready to collect freeze data\n");		
				support.SID_now = 2;
				support.PID_now = 2;			//020200，查看有无冻结帧的PID
			}
			else								//不是处，即已经存过冻结帧与故障码了，无视
			{
				dtc_dog = 0;					//都说了不是处了，狗狗回家吧
				support.SID_now = 1;			//回家继续采动态数据
				support.PID_now = 1;
				support.PID_now = PID_chose(&support);//刚采的0101，找0101后面那个PID吧
			}
		}
		else if(support.SID_now == 2)//已采完故障码并有冻结帧，开始采集冻结帧
		{
			if(support.PID_now == 2)		   //发过020200,查找有无冻结帧
			{
				for(i=0;i<btMsg.ECUNumber;i++) 
				{
					DTC_temp = ((uint16_t)btMsg.tx[i][3] << 8) + btMsg.tx[i][4];//引起冻结帧的故障码
					printf("AutoLoop: freeze dtc = %1X\n",DTC_temp);
					if(DTC_temp != 0)//有冻结帧
					{
						support.SID_now = 2;   //有冻结帧，发送020000采集冻结帧支持码
						support.PID_now = 0;
						ECU_flag = i;		  //存储ECU号
						break;
					}
					else
					{
						dtc_dog = 0;
						support.SID_now = 1;   //无冻结帧，继续采集动态数据
						support.PID_now = 0;
					}
				}	
			}
			else if((support.PID_now == 0)||(support.PID_now == 0x20)||(support.PID_now == 0x40))//发过020000,开始寻找冻结的PID
			{
				if(support.PID_now == 0)//冻结帧支持020300~021F00
				{
					frame_support.v0100 = ((btMsg.tx[ECU_flag][3] << 24)|(btMsg.tx[ECU_flag][4] << 16)|(btMsg.tx[ECU_flag][5] << 8)|(btMsg.tx[ECU_flag][6]));
					printf("AutoLoop: freeze support 00 = %1X\n",frame_support.v0100);
					if((frame_support.v0100 & 0x01) == 1)//支持022000
						support.PID_now = 0x20;
					else								   //不支持022000，不需要继续采集支持码了
						support.PID_now = 1;
				}
				else if(support.PID_now == 0x20)//冻结帧支持022000~023F00
				{
					frame_support.v0120 = ((btMsg.tx[ECU_flag][3] << 24)|(btMsg.tx[ECU_flag][4] << 16)|(btMsg.tx[ECU_flag][5] << 8)|(btMsg.tx[ECU_flag][6]));									
					printf("AutoLoop: freeze support 20 = %1X\n",frame_support.v0120);
					if((frame_support.v0120 & 0x01) == 1)//支持024000
						support.PID_now = 0x40;
					else
						support.PID_now = 1;
				}
				else if(support.PID_now == 0x40)//冻结帧支持024100~025F00
				{
					frame_support.v0140 = ((btMsg.tx[ECU_flag][3] << 24)|(btMsg.tx[ECU_flag][4] << 16)|(btMsg.tx[ECU_flag][5] << 8)|(btMsg.tx[ECU_flag][6]));									
					printf("AutoLoop: freeze support 40 = %1X\n",frame_support.v0140);
					support.PID_now = 1;
				}
				if(support.PID_now == 1)//02支持搜索完毕，开始正式采集冻结帧
				{
					frame_support.PID_now = 2; //初始化PID，从020200开始采集
					frame_support.SID_now = 2;
					support.PID_now = PID_chose(&frame_support);//下一个将要发送的冻结PID
					if(support.PID_now > 0x5F)//下一个PID超出范围，一般不会出现
					{
						support.PID_now = PID_chose(&support);//无冻结帧
						support.SID_now = 1;   
						return;
					}
				}
			}
			else  //冻结帧收集	 020100 ~ 025F00
			{	
				freeze_PID_temp = support.PID_now;				
				frame_support.PID_now = support.PID_now;
				support.PID_now = PID_chose(&frame_support);//下一个将要发送的冻结PID	
				printf("AutoLoop: freeze collect 02 %1X 00\n",support.PID_now);					   															
				if(support.PID_now < freeze_PID_temp)//已找完所有的冻结帧
				{
					support.SID_now = 1;   	//冻结帧采集完毕，继续采集动态数据
					support.PID_now = PID_chose(&support);
					dtc_dog = 0;		//喂狗完毕					
				}
			}									
		}
	}
}
/*******************************************************************************
* Function Name  : PID_check
* Input          : None
* Output         : 支持码
* Return         : None
* Description    : （车辆状态为上电时）支持码采集（动态数据or冻结帧）
					first：0->20->40->2->1
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
	if(first != 1)			 	//准备采集ECU支持的PID信息（first：支持码采集动作标识，为1则已经采集并查过有无值）
	{
		if(first != 2)		  		//先获得支持信息（为2则已经采集过了）		
		{
			loop_time = 1;			//循环采集标识（支持码循环两次已消除丢数影响）

			if(0 == Main_send(0x01,first))//发送0100,0120,0140，返回0即熄火了，不采了			
			{
				support.v0100 = 0;
				support.v0120 = 0;
				support.v0140 = 0;
				loop_time = 0;
				first = 0;
			}		
 
			for(i=0;i<btMsg.ECUNumber;i++)
			{
				if(btMsg.tx[i][1] == 0x00)//收到0100的回复，即0101~0120的支持码
				{					
					supptemp.v0100 |= ((btMsg.tx[i][2] << 24)|(btMsg.tx[i][3] << 16)|(btMsg.tx[i][4] << 8)|(btMsg.tx[i][5]));
					if((btMsg.tx[i][5] & 0x01) == 1)//有必要继续采集0120以后的？
						first = 0x20;
					else
						first = 2;
				}
				else if(btMsg.tx[i][1] == 0x20)//收到0120的回复，即0121~0140的支持码
				{
					supptemp.v0120 |= ((btMsg.tx[i][2] << 24)|(btMsg.tx[i][3] << 16)|(btMsg.tx[i][4] << 8)|(btMsg.tx[i][5]));
					if((btMsg.tx[i][5] & 0x01) == 1)//有必要继续采集0140以后的？
						first = 0x40;
					else
						first = 2;
				}
				else if(btMsg.tx[i][1] == 0x40)//收到0140的回复，即0141~015F的支持码
				{
					supptemp.v0140 |= ((btMsg.tx[i][2] << 24)|(btMsg.tx[i][3] << 16)|(btMsg.tx[i][4] << 8)|(btMsg.tx[i][5]));			
					first = 2;//全部采集完毕，准备查找是否全部有值			
				}
			}
		}
		else						   	//循环采集一次，查找有无值
		{	
			num = PID_chose2(&supptemp);	
			if((num == 1)||(loop_time == 1))//循环1次完毕
				loop_time++;
			if(loop_time > 3)	   //已经循环2次，查找完毕，退出
			{
				first = 1;		   //支持码全部ok，准备跳出
				loop_time = 0;
				return;	
       }
			if(0==Main_send(0x01,num))//从支持数据中获取可用PID并采集数据，返回0即熄火了
			{
			/*	support.v0100 = 0;
				support.v0120 = 0;
				support.v0140 = 0;
				loop_time = 0;
				first = 0;*/
			}
			for(i=0;i<btMsg.ECUNumber;i++)
			{
				if(btMsg.tx[i][0] != 0x7F)		//7F即无值，从支持码中delete
				{
					if(led_flag == 0)
						led_flag = 1;
					if(btMsg.tx[i][1] < 0x20)	//支持数据保存到support中
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
* Output         : 车辆状态
* Return         : ECUstat
* Description    : （上电时）判断车辆状态
*******************************************************************************/
ECUstat state_judge(void)
{
	uint32_t speed = 0;
	
	if(sysMode == NONE)
	 	return ECU_OFF;
	else
	{
		if(0==Main_send(0x01,0x0C))
			return ECU_OFF;	 //发送010C发动机转速
		speed = ((uint32_t)btMsg.tx[0][2] << 8)+btMsg.tx[0][3];	//发动机转速
		if(speed == 0)
			return ECU_ON;		 //发动机为0
		Main_send(0x01,0x0D);	 //发送010D车速
		speed = btMsg.tx[0][2];	 //车速
		if(speed == 0)
		{
			return IDLE;		 //发动机不为0，车速为0
		}
	}
	return DRIVING;
}
/*******************************************************************************
* Function Name  : PID_chose
* Input          : PID
* Output         : PID
* Return         : PID
* Description    : 根据当前PID和支持码选择下一个PID（动态数据or冻结帧）
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

	count = supp->PID_now;	//当前PID			

	if((supp->v0100 == 0)&&(supp->v0120 == 0)&&(supp->v0140 == 0)&&(supp->SID_now == 1))//压根儿没采过支持码或采失误（断电）了
	{
		car_state = ECU_OFF;
		return 0;
	}

retry:	
	temp = count;
	num = 0;
	while(num == 0)//循环查找，貌似用链表会更好？
	{
		count++;				//01~5A
		if((count == 0x20)||(count == 0x40))//跳过0120和0140
			count++;
		if(count > 0x5A)					//不会超过015A
		{
			count = 0;
			//supp->SID_now = 7;	//发07采集偶发故障 
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
		if(count < temp) //循环一次
		{
			PID_times++;
			if(PID_times > 8)
				PID_times = 0;
		}
	
		if(PID_map[PID_cfg[count]][PID_times] == 0)	  //未到采集频率
			goto retry;
	}

	return count;
}

uint8_t PID_chose2(PIDsupport *supp)
{
	static uint8_t first_read,PID_times,temp;
	uint8_t count = 0,num = 0;

	count = supp->PID_now;	//当前PID			

	temp = count;
	num = 0;
	while(num == 0)//循环查找，貌似用链表会更好？
	{
		count++;				//01~5A
		if((count == 0x20)||(count == 0x40))//跳过0120和0140
			count++;
		if(count > 0x5A)					//不会超过015A
		{
			count = 0;
			//supp->SID_now = 7;	//发07采集偶发故障 
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
* Input          : SID，PID
* Output         : None
* Return         : status
* Description    : 根据PID发送命令函数
*******************************************************************************/
uint8_t Main_send(uint8_t SID,uint8_t PID)//发送接收函数
{
	uint8_t i;

    //状态复位
    for(i=0;i<8;i++)
    {
        btMsg.btHeadLen[i]=0;
        btMsg.BtTxState[i]=0;
        btMsg.FrameNum[i]=0;
        btMsg.txLen[i]=0;
    }
    btMsg.ECUNumber=0;
		
	btMsg.rx[0] = SID;			 //btMsg.rx存入要采集的SID和PID号，在ISO15765Main或ISO14230Main中调用发送
	btMsg.rx[1]	= PID;
	btMsg.rxLen = 2;			 //数据长度2

	if((SID == 3)||(SID == 7))
	{
    	btMsg.rxLen = 1;	//03 or 07，数据长度1
		btMsg.rx[1]	= 0;
	}
	else if(SID == 2)
    	btMsg.rxLen = 3;	//020200 or 020000 or 02xx00   数据长度3
	else if(SID == 1)
    	btMsg.rxLen = 2;	//01 xx	数据长度2
 
    if(sysMode == NONE)		//根据车辆协议发送与采集
		car_state = ECU_OFF;
    else if((CAN_STD_500 <= sysMode) && (sysMode <= CAN_EXT_250)) //CAN
	{		
		return ISO15765Main();      //发送与接收      
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
* Input          : SID，PID
* Output         : None
* Return         : status
* Description    : PID变频文档备份
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
	if(res == FR_OK)				//进入配置文件夹
	{
		res = f_open(&file, PIDnum, FA_OPEN_EXISTING | FA_READ);//打开后台配置文件
		while((res != FR_OK)||(file.fsize < 10))
		{
			f_open(&file, PID, FA_CREATE_NEW | FA_WRITE);//打开后台配置文件
			f_write(&file, PID_data, 91, &rc);
			f_close(&file);
			f_open(&file, PID01, FA_CREATE_NEW | FA_WRITE);//打开后台配置文件
			f_write(&file, PID01_data, 91, &rc);
			f_close(&file);
			printf("AutoLoop: PID file restore ok\n");	
			res = f_open(&file, PIDnum, FA_OPEN_EXISTING | FA_READ);
		}
		f_read(&file, &PID_temp, file.fsize, &rc);//读文件				
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
