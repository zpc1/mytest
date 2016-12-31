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


/****************PCF8563 寄存器接口定义*****************/
#define PCF8563T_GET_ADR      0xA3              //PCF8563读统一地址（I2C地址）
#define PCF8563T_SEND_ADR     0xA2              //PCF8563写统一地址
#define PCF8563T_SYS1         0x00              //PCF8563状态寄存器地址1
#define PCF8563T_SYS2         0x01              //PCF8563状态寄存器地址2
#define PCF8563T_S            0x02              //秒
#define PCF8563T_MIN          0x03              //分
#define PCF8563T_OUR          0x04              //时
#define PCF8563T_DAT          0x05              //日
#define PCF8563T_             0x06              //星期
#define PCF8563T_MATH         0x07              //月
#define PCF8563T_YER          0x08              //年
#define PCF8563T_BJ1          0x09               //分钟报警
#define PCF8563T_BJ2          0x0A               //小时报警
#define PCF8563T_BJ3          0x0B               //日报警
#define PCF8563T_BJ4          0x0C               //星期报警
#define PCF8563T_CLK          0x0D              //时钟测试
#define PCF8563T_CT           0x0E              //定时器工作方式
#define PCF8563T_COUNT        0x0F              //定时器工作方式
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
                    延时函数
**************************************************/
void dealy(uint16_t x)
{
 	while(x--); 
}
/*****************************************

        数据类型转换16进制转成BCD码

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

功能：  数据类型转换16进制转成BCD码

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

函数说明：私有函数，I2C专用，启动i2c总线
调用方式：i2c_start ();

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

函数说明：私有函数，I2C专用，停止关闭i2c总线
调用方式：i2c_stop ();

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

函数说明：私有函数，I2C专用，等待从器件接收方的应答
调用方式：i2c_WaitAck();

******************************************************/
uint8_t  i2c_WaitAck(void) 
{
     uint16_t err=500;
     PCF8563T_SDA_ON;  dealy(4);
     PCF8563T_DAT_PIN(); 
     PCF8563T_SCL_ON;  dealy(10);
       //改变数据线端口方向为输入状态
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
     PCF8563T_DAT_POUT();  //改变数据线端口方向为输出状态
    return 1;                                                                                  
}
/******************************************************

函数说明：私有函数，I2C专用，主器件为接收方，从器件为发送方时，主器件应答信号。
调用方式：i2c_SendAck();

******************************************************/
void  i2c_SendAck(void) 
{
     
     PCF8563T_SDA_OFF;  dealy(1);
     PCF8563T_SCL_ON;   dealy(4);
     PCF8563T_SCL_OFF;                                                                         
}
/******************************************************

函数说明：私有函数，I2C专用，主器件为接收方，从器件为发送方时，非应答信号。
调用方式：i2c_Send_notAck();

******************************************************/
void  i2c_Send_notAck(void) 
{ 
     PCF8563T_SDA_ON;  dealy(1);
     PCF8563T_SCL_ON;  dealy(4);
     PCF8563T_SCL_OFF;                                                                         
}
/******************************************************

函数说明：私有函数，I2C专用，主器件发送一字节数。
          dat要发送的数据;
调用方式：i2c_SendByte(uint8_t dat);

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

函数说明：私有函数，I2C专用，主器件接收一字节数。
         
调用方式：i2c_ReceiveByte();

******************************************************/
uint8_t  i2c_ReceiveByte  (void) 
{
      uint8_t j;
      uint8_t dat=0;
      PCF8563T_DAT_PIN();    //改变数据线端口方向为输入状态
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
      PCF8563T_DAT_POUT();  //恢复数据线端口方向为输出状态
      return dat;
}

//产生IIC起始信号
void IIC_Start(void)
{
	PCF8563T_DAT_POUT();     //sda线输出
	PCF8563T_SDA_ON;	  	  
	PCF8563T_SCL_ON;
	dealy(4);
 	PCF8563T_SDA_OFF;//START:when CLK is high,DATA change form high to low 
	dealy(4);
	PCF8563T_SCL_OFF;//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void IIC_Stop(void)
{
	PCF8563T_DAT_POUT();//sda线输出
	PCF8563T_SCL_OFF;
	PCF8563T_SDA_ON;//STOP:when CLK is high DATA change form low to high
 	dealy(4);
	PCF8563T_SCL_ON; 
	PCF8563T_SDA_ON;//发送I2C总线结束信号
	dealy(4);							   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	PCF8563T_DAT_PIN();      //SDA设置为输入  
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
	PCF8563T_SCL_OFF;//时钟输出0 	   
	return 0;  
} 
//产生ACK应答
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
//不产生ACK应答		    
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
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void IIC_Send_Byte(uint8_t dat)
{                        
    uint8_t j;   
	  PCF8563T_DAT_POUT(); 	    
    PCF8563T_SCL_OFF;//拉低时钟开始数据传输
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
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	PCF8563T_DAT_PIN();//SDA设置为输入
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
        IIC_NAck();//发送nACK
    else
        IIC_Ack(); //发送ACK   
    return receive;
}


/******************************************************
PCF8563T驱动程序
函数说明：初始化时钟芯片PCF8563
        
调用方式：RTC_init(uint8_t*p);

******************************************************/

void  RTC_init(void) 
{
      i2c_start ();  
      i2c_SendByte(PCF8563T_SEND_ADR);       i2c_WaitAck(); //发送写寄存器指令
      i2c_SendByte(PCF8563T_SYS1);           i2c_WaitAck(); //设置状态寄存器1
      i2c_SendByte(0x00);                    i2c_WaitAck(); //设置状态寄存器1
      i2c_SendByte(0x00);                    i2c_WaitAck(); //设置状态寄存器2，无中断
      i2c_stop ();  
      dealy(5);
      i2c_start ();   //启动总线
      i2c_SendByte(PCF8563T_SEND_ADR);       i2c_WaitAck(); //发送写寄存器指令
      i2c_SendByte(PCF8563T_BJ1);            i2c_WaitAck(); //设置报警寄存器起始地址
      i2c_SendByte(0x80);                    i2c_WaitAck(); //设置报警内容0x09，0x80 = disable
      i2c_SendByte(0x80);                    i2c_WaitAck(); //设置报警内容0x0a
      i2c_SendByte(0x80);                    i2c_WaitAck(); //设置报警内容0x0b
      i2c_SendByte(0x80);                    i2c_WaitAck(); //设置报警内容0x0c
      i2c_SendByte(0x00);                    i2c_WaitAck(); //设置clk时钟输出方式0x0d
      i2c_SendByte(0x00);                    i2c_WaitAck(); //设置定时器,disable
      i2c_stop (); 
      dealy(5);
}

/******************************************************
PCF8563T驱动程序
函数说明：读取时钟芯片PCF8563的时间，一次把时间读取到*p中
顺序是： 0x02:秒/0x03:分/0x04:小时/0x05:日/0x06:星期/0x07:月(世纪)/0x08:年         
调用方式：get_pcf8563t(uint8_t*p);

******************************************************/
void  get_pcf8563t(uint8_t*p) 
{
      uint8_t j;
      PCF8563T_SDA_ON;  
      i2c_start ();
      i2c_SendByte(PCF8563T_SEND_ADR);       i2c_WaitAck(); //发送写寄存器指令
      i2c_SendByte(PCF8563T_S);              i2c_WaitAck();	//秒地址0x02
      i2c_start ();
      i2c_SendByte(PCF8563T_GET_ADR);        i2c_WaitAck();
      for(j=0;j<7;j++)
      {
         *(p+j)=i2c_ReceiveByte();
         if(j!=6)
         i2c_SendAck(); //除最后一个字节外，其他都要发应答。
      }
      i2c_Send_notAck();   
      i2c_stop ();    
}
/******************************************************
PCF8563T驱动程序
函数说明：设置时钟芯片PCF8563的时间，一次把时间*p写入
顺序是： 0x02:秒/0x03:分/0x04:小时/0x05:日/0x06:星期/0x07:月(世纪)/0x08:年         
调用方式：get_pcf8563t(uint8_t*p);

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

void timer_selfcheck(void)		//8563自检程序，防止时钟重启丢失
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

	newsecond = newtimer.second + newtimer.minute*60 + newtimer.hour*3600 + newtimer.date*86400;//日时分秒
	oldsecond = oldtimer.second + oldtimer.minute*60 + oldtimer.hour*3600 + oldtimer.date*86400;

	if(oldsecond > newsecond) //旧的日时分秒大于新的，判断是否换月或年
	{
		newmonth = newtimer.year*12 + newtimer.month;//年月
		oldmonth = oldtimer.year*12 + oldtimer.month;
		if(oldmonth > newmonth)	//旧的年月日时分秒大于新的，准备重新初始化一遍PCF8563并授时
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

	oldtimer.year = newtimer.year;		//旧的年月日时分秒小于新的，正确无误
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


