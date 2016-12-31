#ifndef _RTC_H_
#define _RTC_H_
#include <time.h>
#include "stm32f10x_bkp.h"
#include "common.h"
#include "BSP.h"


/****************PCF8563 I2C�ӿڶ���*****************/
#define PCF8563T_SDA_ON      GPIO_SetBits(GPIOA, GPIO_Pin_0)	//i2c�����߸ߵ�ƽ
#define PCF8563T_SDA_OFF     GPIO_ResetBits(GPIOA, GPIO_Pin_0) //i2c�����ߵ͵�ƽ
#define PCF8563T_SCL_ON      GPIO_SetBits(GPIOA, GPIO_Pin_1)   //i2cʱ���߸ߵ�ƽ	 
#define PCF8563T_SCL_OFF     GPIO_ResetBits(GPIOA, GPIO_Pin_1) //i2cʱ���ߵ͵�ƽ	
#define PCF8563T_SDA_PIN     GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)//i2c�����������ƽ


typedef struct
{
  uint16_t year;
  uint16_t month;
  uint16_t date;
  uint16_t hour;
  uint16_t minute;
  uint16_t second;   
}RTC_timer;

uint8_t Hex_BCD(uint8_t dat);
void RTC_init(void);	
void set_pcf8563t(uint8_t*p); 
void set_time(RTC_timer *ttimer);
void get_time(RTC_timer *ttimer);
void timer_selfcheck(void);
void display_time(void);
void set_time_init(void);

void PCF8563T_DAT_PIN(void);
void PCF8563T_DAT_POUT(void);
void dealy(uint16_t x);
void  i2c_start(void);
void  i2c_stop(void);
uint8_t  i2c_WaitAck(void);
void  i2c_SendAck(void);
void  i2c_Send_notAck(void);
void  i2c_SendByte(uint8_t dat);
uint8_t  i2c_ReceiveByte  (void);

//IIC���в�������
//void IIC_Init(void);                //��ʼ��IIC��IO��				 
void IIC_Start(void);				//����IIC��ʼ�ź�
void IIC_Stop(void);	  			//����IICֹͣ�ź�
void IIC_Send_Byte(u8 txd);			//IIC����һ���ֽ�
u8 IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
u8 IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack(void);					//IIC����ACK�ź�
void IIC_NAck(void);				//IIC������ACK�ź�


#endif


