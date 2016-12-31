#include "UM220.h"
//#include "mpu6050.h"

extern ECUstat car_state;
extern CARprotocol sysMode;
extern MSG_COMM_BT btMsg;
extern uint32_t sysClock;
//extern uint32_t Milestone[3];
extern uint32_t Mile_one;
extern uint32_t Mile_all;
extern uint8_t led_flag;
extern uint8_t ID_CFG[];
extern uint8_t GPS_databuffer[];
extern uint8_t velocity;

void GPS_time(uint8_t *aaaa);

void BD_Main(void)
{
	uint16_t len,i,j,k;
	uint8_t sum,sum1[3];
	uint8_t header[10] = {"$AT 07 00"};
	static uint8_t beidou_buffer[800];
	static uint16_t pos = 0;
	uint32_t latitude,altitude,speed,speed_mile = 0;
	
	OilCount();
	
	len=SCI_GetLen(4,0); //查看蓝牙有无数据缓冲
  if(len>0)            //只要收到数就将其拿出
  {   	
		if(pos >= 800)
			pos = 0;
		SCI_Receive(4,len,&beidou_buffer[pos]);
		//SCI_Transmit(1,len,&beidou_buffer[pos]);
		pos+=len;

		for(i=0;i<750;i++)
		{
				if(!mem_compare(&beidou_buffer[i], "$GPRMC")) 	
				{
					for(j=i+5;j<750;j++)
					{
						if(beidou_buffer[j] == '*')//i+17为A/V位置，j记录了*的位置
						{
							if((j+3-i) < 50)								//数据完整性Step1
								return;
							for(k=i+1;k<j;k++)
								if(beidou_buffer[k] == '$')		//数据完整性Step2
									return;
							/*sum = 0;
							for(k=i+1;k<j;k++)
								sum = sum^beidou_buffer[k];
							sum1 = IntToChar(sum);
							if((sum1[0] == beidou_buffer[j+1])&&(sum1[1] == beidou_buffer[j+2]))			
							*/{													
								sum1[0] = 0;		
								if(((beidou_buffer[i+17] == 'V')||(beidou_buffer[i+17] == 'A')))
								//	&&((beidou_buffer[j-3] == 'E')||(beidou_buffer[j-3] == 'W')))//数据完整性Step3
								{
									//puts(&beidou_buffer[i]);
									//printf("\r\n");
									for(k=i+17;k<j-3;k++)//检查是否有齐全的位置标识
									{
											if(beidou_buffer[k] == 'N')
												sum1[0]++;
											else if(beidou_buffer[k] == 'E')
												sum1[0]++;
											else if(beidou_buffer[k] == 'S')
												sum1[0]++;
											else if(beidou_buffer[k] == 'W')
												sum1[0]++;
									}
									if(sum1[0] != 2)				//数据完整性Step4
										return;
									
									sum1[0] = 0;
									for(k=i+17;k<j-3;k++)//k记录了最后一个","的位置，即年月日开头，航向角结尾 
									{
										if((beidou_buffer[k] == 'E')||(beidou_buffer[k] == 'W'))
											sum1[0] = 1;
										if(sum1[0] > 0)
										{
											if(beidou_buffer[k] == ',')
												sum1[0]++;
											if(sum1[0] == 4)
												break;
										}
									}
																	
									header[7] = 0;
									header[8] = k-i-15;//发送的数据从经纬度起始到航向角后面的逗号结束
									header[9] = '\n';
									SCI_Transmit(1,9,header);
									SCI_Transmit(1,6,&beidou_buffer[i+7]);
									SCI_Transmit(1,k-i-16,&beidou_buffer[i+17]);
									SCI_Transmit(1,1,&beidou_buffer[j-1]);//定位模式
									SCI_Transmit(1,1,&header[9]);
									
									GPS_time(&beidou_buffer[i+7]);		
									
									if((beidou_buffer[i+17] == 'A')&&(beidou_buffer[i+23] == '.')&&(beidou_buffer[i+37] == '.')&&(beidou_buffer[i+45] == ','))
									{
										if(GPS_databuffer[0] >= 200)
											GPS_databuffer[0] = 0;
										else
											GPS_databuffer[0]++;
										latitude = (uint32_t)(beidou_buffer[i+19] - '0')*1000000 + 
																(uint32_t)(beidou_buffer[i+20] - '0')*100000 +									
																((uint32_t)(beidou_buffer[i+21] - '0')*1000000 +
																(uint32_t)(beidou_buffer[i+22] - '0')*100000 +
																(uint32_t)(beidou_buffer[i+24] - '0')*10000 +
																(uint32_t)(beidou_buffer[i+25] - '0')*1000 +
																(uint32_t)(beidou_buffer[i+26] - '0')*100 +
																(uint32_t)(beidou_buffer[i+27] - '0')*10 + (uint32_t)(beidou_buffer[i+28] - '0'))/60;
										altitude = (uint32_t)(beidou_buffer[i+32] - '0')*10000000 + 
																(uint32_t)(beidou_buffer[i+33] - '0')*1000000 + 
																(uint32_t)(beidou_buffer[i+34] - '0')*100000 + 
																((uint32_t)(beidou_buffer[i+35] - '0')*1000000 +
																(uint32_t)(beidou_buffer[i+36] - '0')*100000 +
																(uint32_t)(beidou_buffer[i+38] - '0')*10000 +
																(uint32_t)(beidou_buffer[i+39] - '0')*1000 +
																(uint32_t)(beidou_buffer[i+40] - '0')*100 +
																(uint32_t)(beidou_buffer[i+41] - '0')*10 + (uint32_t)(beidou_buffer[i+42] - '0'))/60;
										if((beidou_buffer[i+47] == '.')&&(beidou_buffer[i+51] == ','))
										{
//											speed = ((uint32_t)(beidou_buffer[i+46]-'0')*1000 +
//																(uint32_t)(beidou_buffer[i+48]-'0')*100 +
//																(uint32_t)(beidou_buffer[i+49]-'0')*10 +
//																(uint32_t)(beidou_buffer[i+50]-'0'))/540;
										    speed = ((uint32_t)(beidou_buffer[i+46]-'0')*1000 +
																(uint32_t)(beidou_buffer[i+48]-'0')*100 +
																(uint32_t)(beidou_buffer[i+49]-'0')*10 +
																(uint32_t)(beidou_buffer[i+50]-'0'))/1852;
											  speed_mile = ((uint32_t)(beidou_buffer[i+46]-'0')*1000 +
																     (uint32_t)(beidou_buffer[i+48]-'0')*100 +
																     (uint32_t)(beidou_buffer[i+49]-'0')*10 +
																     (uint32_t)(beidou_buffer[i+50]-'0'))*100000/1852;    //小数点保留5位
										}											
										else
										{
											speed = 0;
											speed_mile = 0;
										}																				
									//	velocity = speed*10/36;
									//	velocity = 10;
										velocity = speed_mile/360000;
										if(car_state == DRIVING)
										{
											Mile_all += velocity;
			                Mile_one += velocity;
										}
			              else if(car_state == ECU_OFF)
				               Mile_one = 0;
										
										GPS_databuffer[1] = ((beidou_buffer[i+7] - '0')*10 + (beidou_buffer[i+8] - '0'))+8;
										GPS_databuffer[2] = (beidou_buffer[i+9] - '0')*10 + (beidou_buffer[i+10] - '0');
										GPS_databuffer[3] = (beidou_buffer[i+11] - '0')*10 + (beidou_buffer[i+12] - '0');
										mem_copy32(&GPS_databuffer[4],latitude);
										mem_copy32(&GPS_databuffer[8],altitude);
										GPS_databuffer[12] = (uint8_t)speed;
									}
									else
									{
										velocity = 0;
										Mile_one += velocity;
									}										
	
									for(k=0;k<800;k++)
										beidou_buffer[k] = 0;
								}																		
								return;
							}	
							//else
								//return;						
						}
					}
				}
			}
		}
}

void GPS_time(uint8_t *aaaa)
{
	static uint8_t cache = 0;
	uint8_t i;
	RTC_timer time;
	
	get_time(&time);
	
	if((time.date == cache)||(aaaa[11] == 'V'))
		return;
		
	time.hour = (aaaa[0] - '0')*10 + (aaaa[1] - '0') + 8;
	time.minute = (aaaa[2] - '0')*10 + (aaaa[3] - '0');
	time.second = (aaaa[4] - '0')*10 + (aaaa[5] - '0');
	
	for(i=0;i<100;i++)
	{
		if(aaaa[i] == '*')
			break;
	}
	time.year = (aaaa[i-6] - '0')*10 + (aaaa[i-5] - '0');
	time.month = (aaaa[i-8] - '0')*10 + (aaaa[i-7] - '0');
	time.date = (aaaa[i-10] - '0')*10 + (aaaa[i-9] - '0');
	
	if((time.hour > 24)||(time.hour < 1))
		return;
	if((time.minute > 60)||(time.minute < 1))
		return;
	if((time.second > 60)||(time.second < 1))
		return;
	if((time.year > 50)||(time.year < 15))
		return;
	if((time.month > 12)||(time.month < 1))
		return;
	if((time.date > 31)||(time.date < 1))
		return;
	
	cache = time.date;
	set_time(&time);
	display_time();
}
