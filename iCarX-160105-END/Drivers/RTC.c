/**
  ******************************************************************************
  * @file    RTC.c 
  * @author  PDAger iCar team
  * @version V0.5.0
  * @date    12-August-2011
  * @brief   timer initialize.
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "RTC.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/


/****************PCF8563 �Ĵ����ӿڶ���*****************/
#define PCF8563T_GET_ADR      0xA3              //PCF8563��ͳһ��ַ��I2C��ַ��
#define PCF8563T_SEND_ADR     0xA2              //PCF8563дͳһ��ַ
#define PCF8563T_SYS1         0x00              //PCF8563״̬�Ĵ�����ַ1
#define PCF8563T_SYS2         0x01              //PCF8563״̬�Ĵ�����ַ2
#define PCF8563T_S            0x02              //��
#define PCF8563T_MIN          0x03              //��
#define PCF8563T_OUR          0x04              //ʱ
#define PCF8563T_DAT          0x05              //��
#define PCF8563T_             0x06              //����
#define PCF8563T_MATH         0x07              //��
#define PCF8563T_YER          0x08              //��
#define PCF8563T_BJ1          0x09               //���ӱ���
#define PCF8563T_BJ2          0x0A               //Сʱ����
#define PCF8563T_BJ3          0x0B               //�ձ���
#define PCF8563T_BJ4          0x0C               //���ڱ���
#define PCF8563T_CLK          0x0D              //ʱ�Ӳ���
#define PCF8563T_CT           0x0E              //��ʱ��������ʽ
#define PCF8563T_COUNT        0x0F              //��ʱ��������ʽ
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void PCF8563T_DAT_PIN(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_SetBits(GPIOA, GPIO_Pin_0);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;					 
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;	
  	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void PCF8563T_DAT_POUT(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_SetBits(GPIOA, GPIO_Pin_0);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;					 
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;	
  	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/****************************************************
                    ��ʱ����
**************************************************/
void dealy(uint16_t x)
{
 	while(x--); 
}
/*****************************************

        ��������ת��16����ת��BCD��

******************************************/
uint8_t Hex_BCD(uint8_t dat)
{   
   uint8_t Bcd_dat=0;	   
   uint8_t DH,DL;			   		   
		   DH =(dat/10)<<4;		
		   DL =dat%10;		
		   Bcd_dat=DH+DL;		   
		   return (Bcd_dat); 
}
/*****************************************

���ܣ�  ��������ת��16����ת��BCD��

******************************************/

uint8_t BCD_Hex(uint8_t dat)
{   
   uint8_t Bcd_dat=0;	   
   uint8_t DH,DL;			   		   
		   DH =((dat>>4)&0x0F)*10;		
		   DL =(dat&0x0F);		
		   Bcd_dat=DH+DL;		   
		   return (Bcd_dat); 
} 
/****************************************************

����˵����˽�к�����I2Cר�ã�����i2c����
���÷�ʽ��i2c_start ();

**************************************************/
void  i2c_start (void)
{
   __disable_irq();
   PCF8563T_SDA_ON;  
   dealy(1);
   PCF8563T_SCL_ON;               
   dealy(5);                 
   PCF8563T_SDA_OFF;
   dealy(2);
   PCF8563T_SCL_OFF;   
   dealy(2);                                                 
}

/****************************************************

����˵����˽�к�����I2Cר�ã�ֹͣ�ر�i2c����
���÷�ʽ��i2c_stop ();

**************************************************/
void  i2c_stop (void)
{
   PCF8563T_SCL_OFF;  dealy(2);
   PCF8563T_SDA_OFF;  dealy(4); 
   PCF8563T_SCL_ON;   dealy(2);            
   PCF8563T_SDA_ON;   
   __enable_irq();
}
/******************************************************

����˵����˽�к�����I2Cר�ã��ȴ����������շ���Ӧ��
���÷�ʽ��i2c_WaitAck();

******************************************************/
uint8_t  i2c_WaitAck(void) 
{
     uint16_t err=500;
     PCF8563T_SDA_ON;  dealy(4);
     PCF8563T_DAT_PIN(); 
     PCF8563T_SCL_ON;  dealy(10);
       //�ı������߶˿ڷ���Ϊ����״̬
     while(PCF8563T_SDA_PIN)
     {
        err--; 
        if(err==0)
        {
          i2c_stop ();
          return 0;
        }
      }
     PCF8563T_SCL_OFF; 
     PCF8563T_DAT_POUT();  //�ı������߶˿ڷ���Ϊ���״̬
    return 1;                                                                                  
}
/******************************************************

����˵����˽�к�����I2Cר�ã�������Ϊ���շ���������Ϊ���ͷ�ʱ��������Ӧ���źš�
���÷�ʽ��i2c_SendAck();

******************************************************/
void  i2c_SendAck(void) 
{
     
     PCF8563T_SDA_OFF;  dealy(1);
     PCF8563T_SCL_ON;   dealy(4);
     PCF8563T_SCL_OFF;                                                                         
}
/******************************************************

����˵����˽�к�����I2Cר�ã�������Ϊ���շ���������Ϊ���ͷ�ʱ����Ӧ���źš�
���÷�ʽ��i2c_Send_notAck();

******************************************************/
void  i2c_Send_notAck(void) 
{ 
     PCF8563T_SDA_ON;  dealy(1);
     PCF8563T_SCL_ON;  dealy(4);
     PCF8563T_SCL_OFF;                                                                         
}
/******************************************************

����˵����˽�к�����I2Cר�ã�����������һ�ֽ�����
          datҪ���͵�����;
���÷�ʽ��i2c_SendByte(uint8_t dat);

******************************************************/
void  i2c_SendByte(uint8_t dat) 
{
      uint8_t j;
     
      for(j=0;  j<8;   j++)
      {    
        if((dat&0x80)>0)  
        PCF8563T_SDA_ON; 
        else             
        PCF8563T_SDA_OFF; 
        dealy(8);
        dat<<=1;
       PCF8563T_SCL_ON;  
       dealy(5); 
       PCF8563T_SCL_OFF;       
      }       
}
/******************************************************

����˵����˽�к�����I2Cר�ã�����������һ�ֽ�����
         
���÷�ʽ��i2c_ReceiveByte();

******************************************************/
uint8_t  i2c_ReceiveByte  (void) 
{
      uint8_t j;
      uint8_t dat=0;
      PCF8563T_DAT_PIN();    //�ı������߶˿ڷ���Ϊ����״̬
      for(j=0;j<8;j++)
      {
        dat<<=1;
        PCF8563T_SCL_OFF; 
        dealy(5);
        PCF8563T_SCL_ON;        
        if(PCF8563T_SDA_PIN)
        dat|=1;   
      } 
      PCF8563T_SCL_OFF; 
      PCF8563T_DAT_POUT();  //�ָ������߶˿ڷ���Ϊ���״̬
      return dat;
}

//����IIC��ʼ�ź�
void IIC_Start(void)
{
	PCF8563T_DAT_POUT();     //sda�����
	PCF8563T_SDA_ON;	  	  
	PCF8563T_SCL_ON;
	dealy(4);
 	PCF8563T_SDA_OFF;//START:when CLK is high,DATA change form high to low 
	dealy(4);
	PCF8563T_SCL_OFF;//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  
//����IICֹͣ�ź�
void IIC_Stop(void)
{
	PCF8563T_DAT_POUT();//sda�����
	PCF8563T_SCL_OFF;
	PCF8563T_SDA_ON;//STOP:when CLK is high DATA change form low to high
 	dealy(4);
	PCF8563T_SCL_ON; 
	PCF8563T_SDA_ON;//����I2C���߽����ź�
	dealy(4);							   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	PCF8563T_DAT_PIN();      //SDA����Ϊ����  
	PCF8563T_SDA_ON;dealy(1);	   
	PCF8563T_SCL_ON;dealy(1);	 
	while(PCF8563T_SDA_PIN)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	PCF8563T_SCL_OFF;//ʱ�����0 	   
	return 0;  
} 
//����ACKӦ��
void IIC_Ack(void)
{
	PCF8563T_SCL_OFF;
	PCF8563T_DAT_POUT();
	PCF8563T_SDA_OFF;
	dealy(2);
	PCF8563T_SCL_ON;
	dealy(2);
	PCF8563T_SCL_OFF;
}
//������ACKӦ��		    
void IIC_NAck(void)
{
	PCF8563T_SCL_OFF;
	PCF8563T_DAT_POUT();
	PCF8563T_SDA_ON;
	dealy(2);
	PCF8563T_SCL_ON;
	dealy(2);
	PCF8563T_SCL_OFF;
}					 				     
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void IIC_Send_Byte(uint8_t dat)
{                        
    uint8_t j;   
	  PCF8563T_DAT_POUT(); 	    
    PCF8563T_SCL_OFF;//����ʱ�ӿ�ʼ���ݴ���
    for(j=0;  j<8;   j++)
      {    
        if((dat&0x80)>0)  
        PCF8563T_SDA_ON; 
        else             
        PCF8563T_SDA_OFF; 
        dealy(8);
        dat<<=1;
       PCF8563T_SCL_ON;  
       dealy(5); 
       PCF8563T_SCL_OFF;       
      } 	 
}
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	PCF8563T_DAT_PIN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        PCF8563T_SCL_OFF; 
        dealy(2);
		    PCF8563T_SCL_ON;
        receive<<=1;
        if(PCF8563T_SDA_PIN)receive++;   
		      dealy(1); 
  }					 
    if (!ack)
        IIC_NAck();//����nACK
    else
        IIC_Ack(); //����ACK   
    return receive;
}


/******************************************************
PCF8563T��������
����˵������ʼ��ʱ��оƬPCF8563
        
���÷�ʽ��RTC_init(uint8_t*p);

******************************************************/

void  RTC_init(void) 
{
      i2c_start ();  
      i2c_SendByte(PCF8563T_SEND_ADR);       i2c_WaitAck(); //����д�Ĵ���ָ��
      i2c_SendByte(PCF8563T_SYS1);           i2c_WaitAck(); //����״̬�Ĵ���1
      i2c_SendByte(0x00);                    i2c_WaitAck(); //����״̬�Ĵ���1
      i2c_SendByte(0x00);                    i2c_WaitAck(); //����״̬�Ĵ���2�����ж�
      i2c_stop ();  
      dealy(5);
      i2c_start ();   //��������
      i2c_SendByte(PCF8563T_SEND_ADR);       i2c_WaitAck(); //����д�Ĵ���ָ��
      i2c_SendByte(PCF8563T_BJ1);            i2c_WaitAck(); //���ñ����Ĵ�����ʼ��ַ
      i2c_SendByte(0x80);                    i2c_WaitAck(); //���ñ�������0x09��0x80 = disable
      i2c_SendByte(0x80);                    i2c_WaitAck(); //���ñ�������0x0a
      i2c_SendByte(0x80);                    i2c_WaitAck(); //���ñ�������0x0b
      i2c_SendByte(0x80);                    i2c_WaitAck(); //���ñ�������0x0c
      i2c_SendByte(0x00);                    i2c_WaitAck(); //����clkʱ�������ʽ0x0d
      i2c_SendByte(0x00);                    i2c_WaitAck(); //���ö�ʱ��,disable
      i2c_stop (); 
      dealy(5);
}

/******************************************************
PCF8563T��������
����˵������ȡʱ��оƬPCF8563��ʱ�䣬һ�ΰ�ʱ���ȡ��*p��
˳���ǣ� 0x02:��/0x03:��/0x04:Сʱ/0x05:��/0x06:����/0x07:��(����)/0x08:��         
���÷�ʽ��get_pcf8563t(uint8_t*p);

******************************************************/
void  get_pcf8563t(uint8_t*p) 
{
      uint8_t j;
      PCF8563T_SDA_ON;  
      i2c_start ();
      i2c_SendByte(PCF8563T_SEND_ADR);       i2c_WaitAck(); //����д�Ĵ���ָ��
      i2c_SendByte(PCF8563T_S);              i2c_WaitAck();	//���ַ0x02
      i2c_start ();
      i2c_SendByte(PCF8563T_GET_ADR);        i2c_WaitAck();
      for(j=0;j<7;j++)
      {
         *(p+j)=i2c_ReceiveByte();
         if(j!=6)
         i2c_SendAck(); //�����һ���ֽ��⣬������Ҫ��Ӧ��
      }
      i2c_Send_notAck();   
      i2c_stop ();    
}
/******************************************************
PCF8563T��������
����˵��������ʱ��оƬPCF8563��ʱ�䣬һ�ΰ�ʱ��*pд��
˳���ǣ� 0x02:��/0x03:��/0x04:Сʱ/0x05:��/0x06:����/0x07:��(����)/0x08:��         
���÷�ʽ��get_pcf8563t(uint8_t*p);

******************************************************/
void set_pcf8563t(uint8_t*p) 
{
      uint8_t j; 
      i2c_start ();
      i2c_SendByte(PCF8563T_SEND_ADR);       
	  i2c_WaitAck();
      i2c_SendByte(PCF8563T_S);              
	  i2c_WaitAck();//
      for(j=0;j<7;j++)
      {
       i2c_SendByte(*(p+j));
       i2c_WaitAck();
      }
      i2c_stop ();    
}

void set_time(RTC_timer *ttimer)
{
	uint8_t time[8];

	time[0] = Hex_BCD(ttimer->second);
	time[1] = Hex_BCD(ttimer->minute);
	time[2] = Hex_BCD(ttimer->hour);
	time[3] = Hex_BCD(ttimer->date);
	time[4]	= 1;
	time[5] = Hex_BCD(ttimer->month);
	time[6] = Hex_BCD(ttimer->year);

	set_pcf8563t(&time[0]);
}

void get_time(RTC_timer *ttimer)
{
	uint8_t time[8];

	get_pcf8563t(&time[0]);
	ttimer->second = BCD_Hex((time[0] & 0x7F));
	ttimer->minute = BCD_Hex((time[1] & 0x7F));
	ttimer->hour = BCD_Hex((time[2] & 0x3F));
	ttimer->date = BCD_Hex((time[3] & 0x3F));
	ttimer->month = BCD_Hex((time[5] & 0x1F));
	ttimer->year = BCD_Hex(time[6]);
}

void timer_selfcheck(void)		//8563�Լ���򣬷�ֹʱ��������ʧ
{
	static uint8_t firstgo = 0;
	static RTC_timer oldtimer;
	static char time_name[] = "8563err.rec";
	static FIL file_time;
	UINT rc;
	RTC_timer newtimer;
	uint32_t newsecond,oldsecond;
	uint16_t newmonth,oldmonth;
	uint16_t res;
	uint8_t savetemp[14];

	if(firstgo == 0)
	{
		firstgo = 1;
		get_time(&oldtimer);
		return;
	}
	get_time(&newtimer);

	newsecond = newtimer.second + newtimer.minute*60 + newtimer.hour*3600 + newtimer.date*86400;//��ʱ����
	oldsecond = oldtimer.second + oldtimer.minute*60 + oldtimer.hour*3600 + oldtimer.date*86400;

	if(oldsecond > newsecond) //�ɵ���ʱ��������µģ��ж��Ƿ��»���
	{
		newmonth = newtimer.year*12 + newtimer.month;//����
		oldmonth = oldtimer.year*12 + oldtimer.month;
		if(oldmonth > newmonth)	//�ɵ�������ʱ��������µģ�׼�����³�ʼ��һ��PCF8563����ʱ
		{
			RTC_init();
			set_time(&oldtimer);
			savetemp[0] = 0x5A;
			savetemp[1] = 0xA5;
			savetemp[2] = (char)(oldtimer.year);
			savetemp[3] = (char)(oldtimer.month);
			savetemp[4] = (char)(oldtimer.date);
			savetemp[5] = (char)(oldtimer.hour);
			savetemp[6] = (char)(oldtimer.minute);
			savetemp[7] = (char)(oldtimer.second); 
			savetemp[8] = (char)(newtimer.year);
			savetemp[9] = (char)(newtimer.month);
			savetemp[10] = (char)(newtimer.date);
			savetemp[11] = (char)(newtimer.hour);
			savetemp[12] = (char)(newtimer.minute);
			savetemp[13] = (char)(newtimer.second);

			res = f_open(&file_time, time_name, FA_OPEN_EXISTING | FA_WRITE | FA_READ);
			if(res == FR_NO_FILE)	 
				res = f_open(&file_time, time_name, FA_CREATE_NEW | FA_WRITE | FA_READ);
			f_lseek(&file_time,file_time.fsize);
			f_write(&file_time, &savetemp[0], 14, &rc);
			f_close(&file_time);

			return;
		}			
	}

	oldtimer.year = newtimer.year;		//�ɵ�������ʱ����С���µģ���ȷ����
	oldtimer.month = newtimer.month;
	oldtimer.date = newtimer.date;
	oldtimer.hour = newtimer.hour;
	oldtimer.minute = newtimer.minute;
	oldtimer.second = newtimer.second;
}

void display_time(void)
{
	RTC_timer timer;

	get_time(&timer);
	if((timer.year < 12)||(timer.year > 99))
	{
		set_time_init();
		get_time(&timer);			
	}
	printf("20%d-%d-%d, ",timer.year,timer.month,timer.date);
	if(timer.hour<10)
		printf("0%d:",timer.hour);
	else
		printf("%d:",timer.hour);
	if(timer.minute<10)
		printf("0%d:",timer.minute);
	else
		printf("%d:",timer.minute);
	if(timer.second<10)
		printf("0%d\n",timer.second);
	else
		printf("%d\n",timer.second);
}

void set_time_init(void)
{
	uint8_t time[7];

	time[0] = 0x00;
	time[1] = 0x00;
	time[2] = 0x00;
	time[3] = 0x01;
	time[4]	= 1;
	time[5] = 0x01;
	time[6] = 0x15;

	set_pcf8563t(&time[0]);
}


