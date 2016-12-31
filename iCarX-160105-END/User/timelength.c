#include "timelength.h"


TIMELEN timelen={0,0,0,0};

RTC_timer time_idle_begin;         //怠速开始时刻
RTC_timer time_idle_stop;          //怠速结束时刻
RTC_timer time_driving_begin;      //行驶开始时刻
RTC_timer time_driving_stop;       //行驶结束时刻

uint32_t times_idle_begin;         //以秒为单位的怠速开始时刻
uint32_t times_idle_stop;          //以秒为单位的怠速结束时刻
uint32_t times_driving_begin;      //以秒为单位的行驶开始时刻
uint32_t times_driving_stop;      //以秒为单位的行驶结束时刻
uint32_t timelen_idle_one_at;     //AT 命令获取的本次怠速时长
uint32_t timelen_driving_one_at;   //AT 命令获取的本次行驶时长
uint32_t timelen_idle_one_gprs;   //GPRS驾驶数据流获取的本次累积怠速时长
uint32_t timelen_driving_one_gprs;  //GPRS驾驶数据流获取的本次累积行驶时长
uint32_t timelen_idle_one_bt;      //蓝牙端获取的本次累积怠速时长
uint32_t timelen_driving_one_bt;   //蓝牙端获取的本次累积行驶时长

uint8_t flag_idle=0;               //怠速状态标志位，0代表本次怠速已经结束
uint8_t flag_driving=0;            //行驶状态标志位，0代表本次行驶已经结束 

extern ECUstat car_state;
//extern uint32_t Milestone[3];   //里程数组
extern uint32_t Mile_one;
//extern uint32_t Mile_all;

void TIMELENGTH(void)
{
	uint8_t timesend_buffer[50] = {"$AT 15 "};
	
	if(car_state>=IDLE)   //只有怠速和行驶状态才进入本函数,统计时长
	{
		if(car_state==IDLE)               //怠速状态
		{
			timelen.driving_all += timelen.driving_one;       //总行驶时长 
			timelen_driving_one_gprs += timelen.driving_one;   
			flag_driving = 0;                //行驶状态标志复位，意味着上次行驶状态结束
			if(flag_idle==0)              //判断怠速状态标志位是否为0，即本次怠速阶段怠速开始时刻不再允许改变
			{
				get_time(&time_idle_begin);//获取怠速开始时刻
		        times_idle_begin=time_idle_begin.hour*3600+time_idle_begin.minute*60+time_idle_begin.second;//将时间转换为秒
				flag_idle=1;                //怠速状态标志置位，本次怠速阶段不变
			}
			get_time(&time_idle_stop);      //获取怠速结束时刻
			if(time_idle_stop.hour<time_idle_begin.hour)
				times_idle_stop=(time_idle_stop.hour+24)*3600+time_idle_stop.minute*60+time_idle_stop.second;
			else
				times_idle_stop=time_idle_stop.hour*3600+time_idle_stop.minute*60+time_idle_stop.second;
			timelen.idle_one = (times_idle_stop-times_idle_begin);//本次怠速时长 
      timelen_idle_one_bt = timelen_idle_one_gprs + timelen.idle_one;
      timelen_idle_one_at = timelen_idle_one_bt;			
			
			mem_copy(timesend_buffer,"$AT 15 ");
			timesend_buffer[7] = 0;
			timesend_buffer[8] = 12;
			timesend_buffer[9] = (char)((timelen_idle_one_bt & 0xFF000000) >> 24);
			timesend_buffer[10] = (char)((timelen_idle_one_bt & 0xFF0000) >> 16);
			timesend_buffer[11] = (char)((timelen_idle_one_bt & 0xFF00) >> 8);
			timesend_buffer[12] = (char)(timelen_idle_one_bt & 0xFF);
			timesend_buffer[13] = (char)((timelen_driving_one_gprs & 0xFF000000) >> 24);
			timesend_buffer[14] = (char)((timelen_driving_one_gprs & 0xFF0000) >> 16);
			timesend_buffer[15] = (char)((timelen_driving_one_gprs & 0xFF00) >> 8);
			timesend_buffer[16] = (char)(timelen_driving_one_gprs & 0xFF);
			timesend_buffer[17] = (char)((Mile_one & 0xFF000000) >> 24);
			timesend_buffer[18] = (char)((Mile_one & 0xFF0000) >> 16);
      timesend_buffer[19] = (char)((Mile_one & 0xFF00) >> 8);
      timesend_buffer[20] = (char)(Mile_one & 0xFF);
      timesend_buffer[21] = '\n';
      SCI_Transmit(1,22,timesend_buffer);			
			timelen.driving_one = 0;			
		}
		else if(car_state==DRIVING)          //行驶状态
		{
			timelen.idle_all += timelen.idle_one;             //总怠速时长 
			timelen_idle_one_gprs += timelen.idle_one;
			flag_idle=0;               //怠速状态标志复位，意味着上次怠速状态结束
			if(flag_driving==0)           //判断行驶状态标志位是否为0，即本次行驶阶段行驶开始时刻不再允许改变
			{
				get_time(&time_driving_begin);//获取行驶开始时刻
		        times_driving_begin=time_driving_begin.hour*3600+time_driving_begin.minute*60+time_driving_begin.second;//将时间转换为秒
				flag_driving=1;               //行驶状态标志置位，本次行驶阶段不变
			}
			get_time(&time_driving_stop);//获取行驶结束时刻
			if(time_driving_stop.hour<time_driving_begin.hour)
				times_driving_stop=(time_driving_stop.hour+24)*3600+time_driving_stop.minute*60+time_driving_stop.second;
			else
				times_driving_stop=time_driving_stop.hour*3600+time_driving_stop.minute*60+time_driving_stop.second;
			timelen.driving_one = (times_driving_stop-times_driving_begin);//本次行驶时长 
			timelen_driving_one_bt = timelen_driving_one_gprs + timelen.driving_one;
			timelen_driving_one_at = timelen_driving_one_bt;
			
			mem_copy(timesend_buffer,"$AT 15 ");
			timesend_buffer[7] = 0;
			timesend_buffer[8] = 12;
			timesend_buffer[9] = (char)((timelen_idle_one_gprs & 0xFF000000) >> 24);
			timesend_buffer[10] = (char)((timelen_idle_one_gprs & 0xFF0000) >> 16);
			timesend_buffer[11] = (char)((timelen_idle_one_gprs & 0xFF00) >> 8);
			timesend_buffer[12] = (char)(timelen_idle_one_gprs & 0xFF);
			timesend_buffer[13] = (char)((timelen_driving_one_bt & 0xFF000000) >> 24);
			timesend_buffer[14] = (char)((timelen_driving_one_bt & 0xFF0000) >> 16);
			timesend_buffer[15] = (char)((timelen_driving_one_bt & 0xFF00) >> 8);
			timesend_buffer[16] = (char)(timelen_driving_one_bt & 0xFF);
			timesend_buffer[17] = (char)((Mile_one & 0xFF000000) >> 24);
			timesend_buffer[18] = (char)((Mile_one & 0xFF0000) >> 16);
      timesend_buffer[19] = (char)((Mile_one & 0xFF00) >> 8);
      timesend_buffer[20] = (char)(Mile_one & 0xFF);
      timesend_buffer[21] = '\n';
      SCI_Transmit(1,22,timesend_buffer);	
			timelen.idle_one = 0;
		}
	}
  if(car_state < 1)	
	{
		timelen_idle_one_at = 0;
		timelen_driving_one_at = 0;
	}
}




