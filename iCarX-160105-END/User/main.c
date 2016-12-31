/**
  ******************************************************************************
  * @file    main.c 
  * @author  PDAger iCar team
  * @version V0.5.0
  * @date    12-August-2011
  * @brief   主要宏都在BSP.h中
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/
/*******************************************系统*****************************************/
FIL file;					//动态数据文件句柄
FATFS fs;					//文件系统句柄
DIR dir;					//目录句柄
CanTxMsg 	TxMessage;		//CAN报文发送结构体
CanRxMsg 	RxMessage;		//CAN报文接收结构体
/*******************************************系统*****************************************/
/*******************************************李丕茂*****************************************/
MSG_COMM 	proMsg;			//OBD数据结构体
MSG_COMM_BT btMsg;			//蓝牙数据结构体   
CARprotocol sysMode = NONE;	//记录OBD车辆协议状态

uint8_t timeInval;
uint8_t FraNum[8];			//记录各ECU返回的数据的总帧数
uint32_t sysClock = 0; 		//运行时间计数器，1ms
uint32_t KComRem;			//记录K线运行时间，防止K线中断
uint32_t TimeRem;			//记录当前的时间，即当前的sysClock的值
uint32_t TimeRem_K;
/*******************************************李丕茂*****************************************/
/*******************************************赵阳*****************************************/
ECUstat 	car_state = ECU_OFF;	//车辆状态枚举：ECU_OFF,ECU_ON,IDLE,DRIVING
PIDsupport 	support, supptemp, frame_support;	//动态数据、冻结帧支持码
MSG_COMM_GPRS 	GPRS;				//GPRS相关结构体
/*MSG_DTC		DTC;
LED_ACT		led_flag = ALL_FLASH; */

uint8_t loop_time = 0;		//采集支持码的循环变量
uint8_t first = 0;			//采集支持码的顺序变量
uint8_t pid_save = 0;		//上位机OBD命令缓存
uint8_t multi_flag = 0;		//多帧标识
uint8_t led_flag = 0;		//LED动作标识
uint8_t write_flag = 0;		//表征是否需要存储故障码并采集冻结帧
uint8_t dtc_dog = 0;		//采集故障码时的狗狗，防止ECUOFF后死循环
uint8_t IPR_flag = 0;		//初始化SIM900A波特率
uint8_t GPS_databuffer[13];
uint8_t DTC_num = 0;
uint8_t velocity = 0;
uint8_t ID_CFG[5] = "9999";	//硬件版本
uint8_t AddressBuff[8];
uint8_t PID_cfg[100], WqData[200], WqFlag = 0;

uint16_t first_go;			//记录是否第一次运行，创建文件用
uint16_t time_m = 5;		//记录运行时间间隔，ECU_OFF时搜索协议用
uint16_t DTC_temp;			//引起冻结帧的故障码
uint16_t DTC_now[20];		//故障码缓冲
uint16_t Soft_version[2] = {1,0};	//软件版本
vu16 ADC_ConvertedValue[1];

uint32_t TimeRem_SIM;
uint32_t TimeWq, TimeWqLimit = 1800000, TimesWqLimit = 40;
/*******************************************赵阳*****************************************/

/*******************************************小周*****************************************/
uint8_t Soft_Version = 0x10;  //软件版本   
uint8_t Hard_Version = 0x0A;  //硬件版本
uint8_t Mil_Data[3];       //里程数据
uint8_t Ex_Data[2];     //排气量 
uint8_t Max_Vspeed_this = 0;           //本次最高车速
uint8_t Max_Vspeed_all = 0;           //历史最高车速 
uint8_t mpu6050_buf[30] = {"$AT 16 "};

uint16_t Oil_ave = 0;      //本次平均油耗  
uint16_t Oil_this = 0;     //本次耗油量 
uint16_t Oil_this_gprs = 0;   //用于GPRS上传的本次耗油量
uint16_t Oil_aveall = 0;    //总平均油耗
uint16_t Oil_all = 0;       //总耗油量
uint16_t Max_Espeed_this = 0;            //本次最高转速
uint16_t Max_Espeed_all = 0;            //历史最高转速
uint16_t Accela_this_times = 0;         //本次急加速次数
uint16_t Brake_this_times = 0;         //本次急刹车次数
uint16_t Turn_this_times = 0;         //本次急转弯次数

uint32_t Ter_Version = 0x00000001;   //终端编号  
uint32_t Oilloop_time = 0;           //用于本次平均油耗算式的循环变量
uint32_t Oil_ave1 = 0;      //用于算本次平均油耗的累加值
//uint32_t Oil_all = 0;      //总耗油量
uint32_t Mile_one =0;        //本次行驶里程
uint32_t Mile_all = 0;        //总里程
uint32_t Mile_store = 0;      //用于总里程存储
uint32_t Ignition_times = 0;  //总点火次数
uint32_t Accela_all_times = 0;    //累积急加速次数
uint32_t Brake_all_times = 0;     //累积急刹车次数
uint32_t Turn_all_times = 0;      //累积急转弯次数
/*******************************************小周***************************************/

/*******************************************NEW*****************************************/
bool OBDLoopFlag = 1;
/*******************************************NEW*****************************************/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
int main(void)
{	
	MainConfig();

	while(1)
  {
		OBD_Main();			//主动查询，采集动态数据，包含协议搜索和故障诊断 
    BD_Main();			//北斗主动发数	
    GPRS_Main();  	//被动查询及AT，由后台发起		
    BLE_Main();			//被动查询及AT，由上位机发起		
		ParamConfig();	//系统参数处理
		IWDG_ReloadCounter();//喂狗	
  } 
}

void DelayMs(uint16_t numMs)	
{	
    register u32 numClock;
    do							 
	{
	    for(numClock=0;numClock <= 7200;numClock++)
	        __nop();      
  	}while(--numMs);
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{ 
  while (1)
  {
  }
}

#endif

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

