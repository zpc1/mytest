/**
  ******************************************************************************
  * @file    main.c 
  * @author  PDAger iCar team
  * @version V0.5.0
  * @date    12-August-2011
  * @brief   ��Ҫ�궼��BSP.h��
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/
/*******************************************ϵͳ*****************************************/
FIL file;					//��̬�����ļ����
FATFS fs;					//�ļ�ϵͳ���
DIR dir;					//Ŀ¼���
CanTxMsg 	TxMessage;		//CAN���ķ��ͽṹ��
CanRxMsg 	RxMessage;		//CAN���Ľ��սṹ��
/*******************************************ϵͳ*****************************************/
/*******************************************��اï*****************************************/
MSG_COMM 	proMsg;			//OBD���ݽṹ��
MSG_COMM_BT btMsg;			//�������ݽṹ��   
CARprotocol sysMode = NONE;	//��¼OBD����Э��״̬

uint8_t timeInval;
uint8_t FraNum[8];			//��¼��ECU���ص����ݵ���֡��
uint32_t sysClock = 0; 		//����ʱ���������1ms
uint32_t KComRem;			//��¼K������ʱ�䣬��ֹK���ж�
uint32_t TimeRem;			//��¼��ǰ��ʱ�䣬����ǰ��sysClock��ֵ
uint32_t TimeRem_K;
/*******************************************��اï*****************************************/
/*******************************************����*****************************************/
ECUstat 	car_state = ECU_OFF;	//����״̬ö�٣�ECU_OFF,ECU_ON,IDLE,DRIVING
PIDsupport 	support, supptemp, frame_support;	//��̬���ݡ�����֧֡����
MSG_COMM_GPRS 	GPRS;				//GPRS��ؽṹ��
/*MSG_DTC		DTC;
LED_ACT		led_flag = ALL_FLASH; */

uint8_t loop_time = 0;		//�ɼ�֧�����ѭ������
uint8_t first = 0;			//�ɼ�֧�����˳�����
uint8_t pid_save = 0;		//��λ��OBD�����
uint8_t multi_flag = 0;		//��֡��ʶ
uint8_t led_flag = 0;		//LED������ʶ
uint8_t write_flag = 0;		//�����Ƿ���Ҫ�洢�����벢�ɼ�����֡
uint8_t dtc_dog = 0;		//�ɼ�������ʱ�Ĺ�������ֹECUOFF����ѭ��
uint8_t IPR_flag = 0;		//��ʼ��SIM900A������
uint8_t GPS_databuffer[13];
uint8_t DTC_num = 0;
uint8_t velocity = 0;
uint8_t ID_CFG[5] = "9999";	//Ӳ���汾
uint8_t AddressBuff[8];
uint8_t PID_cfg[100], WqData[200], WqFlag = 0;

uint16_t first_go;			//��¼�Ƿ��һ�����У������ļ���
uint16_t time_m = 5;		//��¼����ʱ������ECU_OFFʱ����Э����
uint16_t DTC_temp;			//���𶳽�֡�Ĺ�����
uint16_t DTC_now[20];		//�����뻺��
uint16_t Soft_version[2] = {1,0};	//����汾
vu16 ADC_ConvertedValue[1];

uint32_t TimeRem_SIM;
uint32_t TimeWq, TimeWqLimit = 1800000, TimesWqLimit = 40;
/*******************************************����*****************************************/

/*******************************************С��*****************************************/
uint8_t Soft_Version = 0x10;  //����汾   
uint8_t Hard_Version = 0x0A;  //Ӳ���汾
uint8_t Mil_Data[3];       //�������
uint8_t Ex_Data[2];     //������ 
uint8_t Max_Vspeed_this = 0;           //������߳���
uint8_t Max_Vspeed_all = 0;           //��ʷ��߳��� 
uint8_t mpu6050_buf[30] = {"$AT 16 "};

uint16_t Oil_ave = 0;      //����ƽ���ͺ�  
uint16_t Oil_this = 0;     //���κ����� 
uint16_t Oil_this_gprs = 0;   //����GPRS�ϴ��ı��κ�����
uint16_t Oil_aveall = 0;    //��ƽ���ͺ�
uint16_t Oil_all = 0;       //�ܺ�����
uint16_t Max_Espeed_this = 0;            //�������ת��
uint16_t Max_Espeed_all = 0;            //��ʷ���ת��
uint16_t Accela_this_times = 0;         //���μ����ٴ���
uint16_t Brake_this_times = 0;         //���μ�ɲ������
uint16_t Turn_this_times = 0;         //���μ�ת�����

uint32_t Ter_Version = 0x00000001;   //�ն˱��  
uint32_t Oilloop_time = 0;           //���ڱ���ƽ���ͺ���ʽ��ѭ������
uint32_t Oil_ave1 = 0;      //�����㱾��ƽ���ͺĵ��ۼ�ֵ
//uint32_t Oil_all = 0;      //�ܺ�����
uint32_t Mile_one =0;        //������ʻ���
uint32_t Mile_all = 0;        //�����
uint32_t Mile_store = 0;      //��������̴洢
uint32_t Ignition_times = 0;  //�ܵ�����
uint32_t Accela_all_times = 0;    //�ۻ������ٴ���
uint32_t Brake_all_times = 0;     //�ۻ���ɲ������
uint32_t Turn_all_times = 0;      //�ۻ���ת�����
/*******************************************С��***************************************/

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
		OBD_Main();			//������ѯ���ɼ���̬���ݣ�����Э�������͹������ 
    BD_Main();			//������������	
    GPRS_Main();  	//������ѯ��AT���ɺ�̨����		
    BLE_Main();			//������ѯ��AT������λ������		
		ParamConfig();	//ϵͳ��������
		IWDG_ReloadCounter();//ι��	
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

