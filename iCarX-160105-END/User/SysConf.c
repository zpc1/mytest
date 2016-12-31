#include "SysConf.h"
#include "timelength.h"
#include "mpu6050.h"

extern FIL file;				
extern FATFS fs;				
extern DIR dir;			
extern MSG_COMM_GPRS 	GPRS;
//extern uint32_t Milestone[];
/*******************************************С��***************************************/
extern uint32_t Mile_one;
extern uint32_t Mile_all;
extern uint32_t Mile_store;
extern uint32_t Ignition_times;
extern uint32_t Accela_all_times;
extern uint32_t Brake_all_times;
extern uint32_t Turn_all_times;
extern ECUstat car_state;
extern TIMELEN timelen;
extern uint8_t Max_Vspeed_all;
extern uint16_t Max_Espeed_all;
extern uint16_t Oil_aveall;
extern uint16_t Oil_all;
/*******************************************С��***************************************/

/*******************************************************************************
* Function Name  : MainConfig
* Input          : None
* Output         : None
* Return         : None
* Description    : 
*******************************************************************************/
void MainConfig(void)
{
	UINT br,bw;	
	static FIL file,filebak;
	uint16_t res;
	uint8_t buffer[513];
	
	/*************************Ӳ�����ʼ��*************************/	
	clock_init();		 	//����ʱ��
	gpio_init();		 	//IO�˿�
	IWDG_Configuration();//���Ź�  
	SPI_Flash_Init();	
	f_mount(&fs,"/",1);	
	usart1_init();	  	//����1������)	
	RTC_init();					//RTC��PCF8563��  
	MPU_Init();         //mpu6050
	usart3_init();			//����3��GPRSģ�飩
	uart4_init();				//����4������ģ�飩
	tim_init();		  	 	//��ʱ������ʱ�жϣ�
	AD_init();
	DMA_Config();
	NVIC_init();		 		//�жϣ���ʱ�жϣ�	
	printf("\nsystem start!\n");
	display_time();			//��ӡʱ��	

/************************GPRS���ݳ�ʼ��************************/
	GPRS.step = REBOOT;			  //GPRS״̬��׼������
	GPRS.sendok = 1;			  //GPRS���ͱ�ʶ
	GPRS.point = GPRS_DATA_START; //GPRS���ݻ�������ʼ
	GPRS.at_after_data = 0;		  //����&AT����˳��

/************************�̼�������ʼ��************************/
	f_chdir("/");		
	res = f_open(&file, "Update.tmp", FA_OPEN_EXISTING | FA_READ);
	if(res == FR_OK)//�̼������ɹ���ɾ��Update.tmp������UpOK.tmp������Update.bin
	{
		res = f_open(&filebak, "UpOK.tmp", FA_CREATE_NEW | FA_WRITE);
		if(res == FR_OK)
		{
			f_close(&file);
			f_close(&filebak);
			
			res = f_open(&file, "Update.bin", FA_OPEN_EXISTING | FA_READ);
			if(res == FR_OK)
			{
				f_unlink("Upback.bin");
				res = f_open(&filebak, "Upback.bin", FA_CREATE_NEW | FA_WRITE);
				if(res == FR_OK)
				{				
					while(1)//����Update.bin
					{
						res = f_read(&file, buffer, 512, &br);
						if (res || br == 0) break;   
						res = f_write(&filebak, buffer, br, &bw);
						if (res || bw < br) break; 
					}
					f_close(&file);
					f_close(&filebak);
					f_unlink("Update.tmp");
					f_unlink("Update.bin");
				}
			}			
		}
	}
	else//��������������UpOK.tmp��������һ������
	{
		res = f_open(&filebak, "UpOK.tmp", FA_CREATE_NEW | FA_WRITE);
		if(res == FR_OK)
			f_close(&filebak);
	}

	FileConfig();
}

/*******************************************************************************
* Function Name  : ParamConfig
* Input          : None
* Output         : None
* Return         : None
* Description    : 
*******************************************************************************/
void ParamConfig(void)
{
	UINT bw;	
	static FIL file;
	uint16_t res;
	uint8_t buffer[10];
	
	if(car_state == ECU_OFF)
	{
		/************************��̴洢************************/
		if(Mile_all != Mile_store)
		{
			f_chdir("/");
			f_chdir("BACKUP");
			Mile_store = Mile_all;
			buffer[0] = (uint8_t)(Mile_store >> 24);
			buffer[1] = (uint8_t)(Mile_store >> 16);
			buffer[2] = (uint8_t)(Mile_store >> 8);
			buffer[3] = (uint8_t)(Mile_store & 0xFF);
			
			res = f_open(&file, "Mile.cfg", FA_OPEN_EXISTING | FA_WRITE);
			if(res == FR_OK)
			{
				f_lseek(&file,0);
				f_write(&file, buffer, 4, &bw);
			}
			else
			{
				res = f_open(&file, "Mile.cfg", FA_CREATE_NEW | FA_WRITE);
				if(res == FR_OK)
					res = f_write(&file, buffer, 4, &bw);
				f_close(&file);
			}
			f_close(&file);
			f_chdir("/");
		}
	/************************�������洢************************/
		if(Ignition_times != 0)
		{
			f_chdir("/");
			f_chdir("BACKUP");
			buffer[0] = (uint8_t)(Ignition_times >> 24);
			buffer[1] = (uint8_t)(Ignition_times >> 16);
			buffer[2] = (uint8_t)(Ignition_times >> 8);
			buffer[3] = (uint8_t)Ignition_times;
			res = f_open(&file, "Igni.cfg", FA_OPEN_EXISTING | FA_WRITE);
			if(res == FR_OK)
			{
				res = f_lseek(&file, 0);
				res = f_write(&file, buffer, 4, &bw);
			}
			else
			{
				res = f_open(&file, "Igni.cfg", FA_CREATE_NEW | FA_WRITE);
				if(res == FR_OK)
					res = f_write(&file, buffer, 4, &bw);
				f_close(&file);
			}
			f_close(&file);
			f_chdir("/");
		}
		/************************�ۼƵ���ʱ���洢************************/
		if(timelen.idle_all != 0)
		{
			f_chdir("/");
			f_chdir("BACKUP");
			buffer[0] = (uint8_t)(timelen.idle_all >> 24);
			buffer[1] = (uint8_t)(timelen.idle_all >> 16);
			buffer[2] = (uint8_t)(timelen.idle_all >> 8);
			buffer[3] = (uint8_t)timelen.idle_all;
			res = f_open(&file, "Idle.cfg", FA_WRITE);
			if(res == FR_OK)
			{
				f_lseek(&file, 0);
				f_write(&file, buffer, 4, &bw);
			}
      else
      {
				res = f_open(&file, "Idle.cfg", FA_CREATE_NEW | FA_WRITE);
				if(res == FR_OK)
				{
					f_write(&file, buffer, 4, &bw);
					f_close(&file);
				}
			}
      f_close(&file);
      f_chdir("/");			
		}
		/************************�ۼ���ʻʱ���洢************************/
		if(timelen.driving_all != 0)
		{
			f_chdir("/");
			f_chdir("BACKUP");
			buffer[0] = (uint8_t)(timelen.driving_all >> 24);
			buffer[1] = (uint8_t)(timelen.driving_all >> 16);
			buffer[2] = (uint8_t)(timelen.driving_all >> 8);
			buffer[3] = (uint8_t)timelen.driving_all;
			res = f_open(&file, "Driv.cfg", FA_WRITE);
			if(res == FR_OK)
			{
				f_lseek(&file, 0);
				f_write(&file, buffer, 4, &bw);
			}
			else
			{
				f_open(&file, "Driv.cfg", FA_CREATE_NEW | FA_WRITE);
				if(res == FR_OK)
				{
					f_write(&file, buffer, 4, &bw);
					f_close(&file);
				}
			}
			f_close(&file);
			f_chdir("/");	
		}
		/************************��ʷ��߳��ٴ洢************************/
		if(Max_Vspeed_all != 0)
		{
			f_chdir("/");
			f_chdir("BACKUP");
			buffer[0] = Max_Vspeed_all;
			res = f_open(&file, "Vspe.cfg", FA_WRITE);
			if(res == FR_OK)
			{
				f_lseek(&file, 0);
				f_write(&file, buffer, 1, &bw);
			}
			else
			{
				f_open(&file, "Vspe.cfg", FA_CREATE_NEW | FA_WRITE);
				if(res == FR_OK)
				{
					f_write(&file, buffer, 1, &bw);
					f_close(&file);
				}
			}
			f_close(&file);
			f_chdir("/");	
		}
		/************************��ʷ���ת�ٴ洢************************/
		if(Max_Espeed_all != 0)
		{
			f_chdir("/");
			f_chdir("BACKUP");
			buffer[0] = (uint8_t)(Max_Espeed_all >> 8);
			buffer[1] = (uint8_t)Max_Espeed_all;
			res = f_open(&file, "Espe.cfg", FA_WRITE);
			if(res == FR_OK)
			{
				f_lseek(&file, 0);
				f_write(&file, buffer, 2, &bw);
			}
			else
			{
				f_open(&file, "Espe.cfg", FA_CREATE_NEW | FA_WRITE);
				if(res == FR_OK)
				{
					f_write(&file, buffer, 2, &bw);
					f_close(&file);
				}
			}
			f_close(&file);
			f_chdir("/");	
		}
		/************************�ۼƼ����ٴ����洢************************/
		if(Accela_all_times != 0)
		{
			f_chdir("/");
			f_chdir("BACKUP");
			buffer[0] = (uint8_t)(Accela_all_times >> 24);
			buffer[1] = (uint8_t)(Accela_all_times >> 16);
			buffer[2] = (uint8_t)(Accela_all_times >> 8);
			buffer[3] = (uint8_t)Accela_all_times;
			res = f_open(&file, "Acce.cfg", FA_WRITE);
			if(res == FR_OK)
			{
				res = f_lseek(&file, 0);
				res = f_write(&file, buffer, 4, &bw);
				f_close(&file);
			}
			else
			{
				res = f_open(&file, "Acce.cfg", FA_CREATE_NEW | FA_WRITE);
				if(res == FR_OK)
					res = f_write(&file, buffer, 4, &bw);
				f_close(&file);
			}
			f_chdir("/");
		}
		/************************�ۼƼ�ɲ�������洢************************/
		if(Brake_all_times != 0)
		{
			f_chdir("/");
			f_chdir("BACKUP");
			buffer[0] = (uint8_t)(Brake_all_times >> 24);
			buffer[1] = (uint8_t)(Brake_all_times >> 16);
			buffer[2] = (uint8_t)(Brake_all_times >> 8);
			buffer[3] = (uint8_t)Brake_all_times;
			res = f_open(&file, "Brak.cfg", FA_WRITE);
			if(res == FR_OK)
			{
				res = f_lseek(&file, 0);
				res = f_write(&file, buffer, 4, &bw);
				f_close(&file);
			}
			else
			{
				res = f_open(&file, "Brak.cfg", FA_CREATE_NEW | FA_WRITE);
				if(res == FR_OK)
					res = f_write(&file, buffer, 4, &bw);
				f_close(&file);
			}
			f_chdir("/");
		}
		/************************�ۼƼ�ת������洢************************/
		if(Turn_all_times != 0)
		{
			f_chdir("/");
			f_chdir("BACKUP");
			buffer[0] = (uint8_t)(Turn_all_times >> 24);
			buffer[1] = (uint8_t)(Turn_all_times >> 16);
			buffer[2] = (uint8_t)(Turn_all_times >> 8);
			buffer[3] = (uint8_t)Turn_all_times;
			res = f_open(&file, "Turn.cfg", FA_WRITE);
			if(res == FR_OK)
			{
				res = f_lseek(&file, 0);
				res = f_write(&file, buffer, 4, &bw);
				f_close(&file);
			}
			else
			{
				res = f_open(&file, "Turn.cfg", FA_CREATE_NEW | FA_WRITE);
				if(res == FR_OK)
					res = f_write(&file, buffer, 4, &bw);
				f_close(&file);
			}
			f_chdir("/");
		}
		/************************�ܺ������洢************************/
		if(Oil_all != 0)
		{
			f_chdir("/");
			f_chdir("BACKUP");
			buffer[0] = (uint8_t)(Oil_all >> 8);
			buffer[1] = (uint8_t)Oil_all;
			res = f_open(&file, "Oil.cfg", FA_WRITE);
			if(res == FR_OK)
			{
				res = f_lseek(&file, 0);
				res = f_write(&file, buffer, 2, &bw);
				f_close(&file);
			}
			else
			{
				res = f_open(&file, "Oil.cfg", FA_CREATE_NEW | FA_WRITE);
				if(res == FR_OK)
					res = f_write(&file, buffer, 2, &bw);
				f_close(&file);
			}
			f_chdir("/");
		}
		/************************��ƽ���ͺĴ洢************************/
		if(Oil_aveall != 0)
		{
			f_chdir("/");
			f_chdir("BACKUP");
			buffer[0] = (uint8_t)(Oil_aveall >> 8);
			buffer[1] = (uint8_t)Oil_aveall;
			res = f_open(&file, "Oila.cfg", FA_WRITE);
			if(res == FR_OK)
			{
				res = f_lseek(&file, 0);
				res = f_write(&file, buffer, 2, &bw);
				f_close(&file);
			}
			else
			{
				res = f_open(&file, "Oila.cfg", FA_CREATE_NEW | FA_WRITE);
				if(res == FR_OK)
					res = f_write(&file, buffer, 2, &bw);
				f_close(&file);
			}
			f_chdir("/");
		}
	}
}


/*******************************************************************************
* Function Name  : FileConfig
* Input          : None
* Output         : None
* Return         : None
* Description    : �ļ�����
*******************************************************************************/
void FileConfig(void)
{
	uint16_t res;
	UINT bw;	
	FILINFO info;
	FIL file;
	uint8_t IP[]="\"UDP\",\"202.189.1.43\",\"2001\"";
	uint8_t ID[]="0001";
	uint8_t buffer[513];
	
	f_chdir("/");
	
	res = f_chdir("BACKUP");
	if(res != FR_OK)
	{
		f_mkdir("BACKUP");
		f_chdir("BACKUP");
	}
/************************************IP.cfg*******************************************/	
	res = f_open(&file, "IP.cfg", FA_OPEN_EXISTING | FA_WRITE);
	if(res == FR_NO_FILE)
	{
		res = f_open(&file, "IP.cfg", FA_CREATE_NEW | FA_WRITE);
		if(res == FR_OK)
		{
			f_write(&file, IP, 27, &bw);
			f_close(&file);
		}
	}
	else if(res == FR_OK)
	{
		f_stat("IP.cfg",&info);
		if(info.fsize < 10)
			f_write(&file, IP, 27, &bw);
	}
	f_close(&file);
/************************************ID.cfg*******************************************/	
	res = f_open(&file, "ID.cfg", FA_OPEN_EXISTING | FA_WRITE);
	if(res == FR_NO_FILE)
	{
		res = f_open(&file, "ID.cfg", FA_CREATE_NEW | FA_WRITE);
		if(res == FR_OK)
		{
			f_write(&file, ID, 4, &bw);
			f_close(&file);
		}
	}
	else if(res == FR_OK)
	{
		f_stat("ID.cfg",&info);
		if(info.fsize < 4)
			f_write(&file, ID, 4, &bw);
	}
	f_close(&file);
/************************************Mile.cfg*******************************************/	
	Mile_one = 0;
	res = f_open(&file, "Mile.cfg", FA_OPEN_EXISTING | FA_READ);
	if(res != FR_OK)
	{
		Mile_all = 0;
		buffer[0] = 0;buffer[1] = 0;buffer[2] = 0;buffer[3] = 0;
		res = f_open(&file, "Mile.cfg", FA_CREATE_NEW | FA_WRITE);
		if(res == FR_OK)
			res = f_write(&file, buffer, 4, &bw);
		f_close(&file);
	}
	else
	{
		res = f_read(&file, buffer, 4, &bw);
		if(res == FR_OK)
			Mile_all = ((uint32_t)buffer[0] << 24) + ((uint32_t)buffer[1] << 16) + ((uint32_t)buffer[2] << 8) + (uint32_t)buffer[3];
		f_close(&file);
	}
/************************************Ignition.cfg*******************************************/
  res = f_open(&file, "Igni.cfg", FA_OPEN_EXISTING | FA_READ);
  if(res != FR_OK)
  {
		buffer[0] = 0;buffer[1] = 0;buffer[2] = 0;buffer[3] = 0;
		res = f_open(&file, "Igni.cfg", FA_CREATE_NEW | FA_WRITE);
		if(res == FR_OK)
			res = f_write(&file, buffer, 4, &bw);
		f_close(&file);
	}
  else
  {
		res = f_read(&file, buffer, 4, &bw);
		if(res == FR_OK)
			Ignition_times = ((uint32_t)buffer[0] << 24) + ((uint32_t)buffer[1] << 16) + ((uint32_t)buffer[2] << 8) + (uint32_t)buffer[3];
		f_close(&file);
	}
/************************************Idle.cfg*******************************************/
  res = f_open(&file, "Idle.cfg", FA_OPEN_EXISTING | FA_READ);
  if(res != FR_OK)
  {
		buffer[0] = 0;buffer[1] = 0;buffer[2] = 0;buffer[3] = 0;
		res = f_open(&file, "Idle.cfg", FA_CREATE_NEW | FA_WRITE);
		if(res == FR_OK)
			res = f_write(&file, buffer, 4, &bw);
		f_close(&file);
	}
  else
  {
		res = f_read(&file, buffer, 4, &bw);
		if(res == FR_OK)
			timelen.idle_all = ((uint32_t)buffer[0] << 24) + ((uint32_t)buffer[1] << 16) + ((uint32_t)buffer[2] << 8) + (uint32_t)buffer[3];
		f_close(&file);
	}
/************************************Driv.cfg*******************************************/
  res = f_open(&file, "Driv.cfg", FA_OPEN_EXISTING | FA_READ);
  if(res != FR_OK)
  {
		buffer[0] = 0;buffer[1] = 0;buffer[2] = 0;buffer[3] = 0;
		res = f_open(&file, "Driv.cfg", FA_CREATE_NEW | FA_WRITE);
		if(res == FR_OK)
			res = f_write(&file, buffer, 4, &bw);
		f_close(&file);
	}
  else
  {
		res = f_read(&file, buffer, 4, &bw);
		if(res == FR_OK)
			timelen.driving_all = ((uint32_t)buffer[0] << 24) + ((uint32_t)buffer[1] << 16) + ((uint32_t)buffer[2] << 8) + (uint32_t)buffer[3];
		f_close(&file);
	}
/************************************Vspe.cfg*******************************************/
  res = f_open(&file, "Vspe.cfg", FA_OPEN_EXISTING | FA_READ);
  if(res != FR_OK)
  {
		buffer[0] = 0;
		res = f_open(&file, "Vspe.cfg", FA_CREATE_NEW | FA_WRITE);
		if(res == FR_OK)
			res = f_write(&file, buffer, 1, &bw);
		f_close(&file);
	}
  else
  {
		res = f_read(&file, buffer, 1, &bw);
		if(res == FR_OK)
			Max_Vspeed_all = buffer[0];
		f_close(&file);
	}
/************************************Espe.cfg*******************************************/
  res = f_open(&file, "Espe.cfg", FA_OPEN_EXISTING | FA_READ);
  if(res != FR_OK)
  {
		buffer[0] = 0;buffer[1] = 0;
		res = f_open(&file, "Espe.cfg", FA_CREATE_NEW | FA_WRITE);
		if(res == FR_OK)
			res = f_write(&file, buffer, 2, &bw);
		f_close(&file);
	}
  else
  {
		res = f_read(&file, buffer, 2, &bw);
		if(res == FR_OK)
			Max_Espeed_all = ((uint16_t)buffer[0] << 8) + (uint16_t)buffer[1];
		f_close(&file);
	}
/************************************Acce.cfg*******************************************/
  res = f_open(&file, "Acce.cfg", FA_OPEN_EXISTING | FA_READ);
  if(res != FR_OK)
  {
		buffer[0] = 0;buffer[1] = 0;buffer[2] = 0;buffer[3] = 0;
		res = f_open(&file, "Acce.cfg", FA_CREATE_NEW | FA_WRITE);
		if(res == FR_OK)
			res = f_write(&file, buffer, 4, &bw);
		f_close(&file);
	}
  else
  {
		res = f_read(&file, buffer, 4, &bw);
		if(res == FR_OK)
			Accela_all_times = ((uint32_t)buffer[0] << 24) + ((uint32_t)buffer[1] << 16) + ((uint32_t)buffer[2] << 8) + (uint32_t)buffer[3];
		f_close(&file);
	}
/************************************Brak.cfg*******************************************/
  res = f_open(&file, "Brak.cfg", FA_OPEN_EXISTING | FA_READ);
  if(res != FR_OK)
  {
		buffer[0] = 0;buffer[1] = 0;buffer[2] = 0;buffer[3] = 0;
		res = f_open(&file, "Brak.cfg", FA_CREATE_NEW | FA_WRITE);
		if(res == FR_OK)
			res = f_write(&file, buffer, 4, &bw);
		f_close(&file);
	}
  else
  {
		res = f_read(&file, buffer, 4, &bw);
		if(res == FR_OK)
			Brake_all_times = ((uint32_t)buffer[0] << 24) + ((uint32_t)buffer[1] << 16) + ((uint32_t)buffer[2] << 8) + (uint32_t)buffer[3];
		f_close(&file);
	}
/************************************Turn.cfg*******************************************/
  res = f_open(&file, "Turn.cfg", FA_OPEN_EXISTING | FA_READ);
  if(res != FR_OK)
  {
		buffer[0] = 0;buffer[1] = 0;buffer[2] = 0;buffer[3] = 0;
		res = f_open(&file, "Turn.cfg", FA_CREATE_NEW | FA_WRITE);
		if(res == FR_OK)
			res = f_write(&file, buffer, 4, &bw);
		f_close(&file);
	}
  else
  {
		res = f_read(&file, buffer, 4, &bw);
		if(res == FR_OK)
			Turn_all_times = ((uint32_t)buffer[0] << 24) + ((uint32_t)buffer[1] << 16) + ((uint32_t)buffer[2] << 8) + (uint32_t)buffer[3];
		f_close(&file);
	}
/************************************Oil.cfg*******************************************/
  res = f_open(&file, "Oil.cfg", FA_OPEN_EXISTING | FA_READ);
  if(res != FR_OK)
  {
		buffer[0] = 0;buffer[1] = 0;
		res = f_open(&file, "Oil.cfg", FA_CREATE_NEW | FA_WRITE);
		if(res == FR_OK)
			res = f_write(&file, buffer, 2, &bw);
		f_close(&file);
	}
  else
  {
		res = f_read(&file, buffer, 2, &bw);
		if(res == FR_OK)
			Oil_all = ((uint16_t)buffer[0] << 8) + (uint16_t)buffer[1];
		f_close(&file);
	}
/************************************Oila.cfg*******************************************/
  res = f_open(&file, "Oila.cfg", FA_OPEN_EXISTING | FA_READ);
  if(res != FR_OK)
  {
		buffer[0] = 0;buffer[1] = 0;
		res = f_open(&file, "Oila.cfg", FA_CREATE_NEW | FA_WRITE);
		if(res == FR_OK)
			res = f_write(&file, buffer, 2, &bw);
		f_close(&file);
	}
  else
  {
		res = f_read(&file, buffer, 2, &bw);
		if(res == FR_OK)
			Oil_aveall = ((uint16_t)buffer[0] << 8) + (uint16_t)buffer[1];
		f_close(&file);
	}	
}
