/**
  ******************************************************************************
  * @file    INT.c 
  * @author  PDAger iCar team
  * @version V0.5.0
  * @date    12-August-2011
  * @brief   interrupt Handlers.
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "INT.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern uint32_t sysClock; 
extern uint16_t time_m;
extern uint8_t led_flag;
extern uint8_t dtc_dog;
extern PIDsupport support;
extern MSG_COMM_GPRS GPRS;
//extern uint32_t Milestone[];
//extern uint8_t velocity;
//extern ECUstat car_state;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void NMI_Handler(void)
{
}
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}
void SVC_Handler(void)
{
}
void DebugMon_Handler(void)
{
}
void PendSV_Handler(void)
{
}
void USART1_IRQHandler(void)
{
}
void SysTick_Handler(void)
{
}
void RTC_IRQHandler(void)
{
}

void TIM1_UP_IRQHandler(void)	//500us
{	
	static uint8_t stClock = 0;	  
	static uint16_t time_s = 0;
//	static uint16_t time_v = 0;
	static uint16_t a = 0;
	static uint16_t dtc_timer = 0;
	static uint16_t time_temp = 0;
	static uint32_t gprs_time = 0;

  stClock++;
  if(2==stClock)
  {
    stClock=0;
    sysClock++;     	//1ms
		time_s++;
//		time_v++;
//		if(time_v >= 1000)
//		{
//			time_v = 0;
//			Milestone[0] += velocity;
//			Milestone[1] += velocity;
//			if(car_state == ECU_OFF)
//				Milestone[1] = 0;
//		}
		if(time_s >= 5000)
		{
			time_s = 0;
			time_temp++;
			time_m++;
		} 	

		if(sysClock > 0xFFFFFFFE) 				//即将溢出，约49天
			sysClock = 0;	
		if((GPRS.sendok == 0)&&(GPRS.step != 0xFF))
		{
			gprs_time++;
			if(gprs_time >= 120000)//2min
			{
				gprs_time = 0;
				GPRS.sendok = 1;
			}	
		}
		else if((GPRS.sendok == 1)&&(GPRS.step != 0xFF))
		{
			gprs_time = 0;
		}

		if(dtc_dog == 1)
		{
			dtc_timer++;
			if(dtc_timer >= 10000)//10s
			{
				dtc_timer = 0;
				dtc_dog = 0;
				support.SID_now = 1;//超过10s没采完冻结帧，强制退出
				support.PID_now	= 1;
			}
		} 
		
		switch(led_flag)	   //控制LED闪烁，确认协议搜索或动态数据发送
		{
			case 0: a++;
					#ifndef ICAR_DOCK			
					if(a == 500)
					{
						LED1_ON
						LED2_OFF
					}
					if(a==1000)
					{
						a = 0;
						LED1_OFF
						LED2_ON
					}
					#else
					if(a == 250)
					{
						LED1_ON
						LED2_OFF
						LED3_OFF
						LED4_OFF
					}
					if(a == 500)
					{
						LED1_OFF
						LED2_ON
						LED3_OFF
						LED4_OFF
					}
					if(a == 750)
					{
						LED1_OFF
						LED2_OFF
						LED3_ON
						LED4_OFF
					}
					if(a == 1000)
					{
						a = 0;
						LED1_OFF
						LED2_OFF
						LED3_OFF
						LED4_ON
					}
					#endif
				break;
			case 1: LED1_OFF
					LED2_OFF
					#ifdef ICAR_DOCK
					LED3_OFF
					LED4_OFF
					#endif
					a = 0;
					led_flag = 2;
				break;
			#ifdef ICAR_DOCK
			case 2:	LED4_OFF;
					a = 0;
				break;
			case 3: a++;
					if(a == 1)
						LED4_ON
					if(a == 101)
						LED4_OFF
					if(a == 201)
						a = 0;
				break;
			#else
			case 2:	LED2_OFF;
					a = 0;
				break;
			case 3: a++;
					if(a == 1)
						LED2_ON
					if(a == 51)
						LED2_OFF
					if(a >= 101)
						a = 0;
				break;
			#endif	
			default:break;
		}
	}		
		
  SCI_MainFunction();    
	TIM_ClearITPendingBit(TIM1, TIM_FLAG_Update); //清中断
}

void UART4_IRQHandler(void)
{
	printf("1234\n");
}

void TIM4_IRQHandler(void)  //J1850+输出比较中断TIM4_CH1
{ 
	uint16_t capture;

    if(TIM_GetITStatus(TIM4, TIM_IT_CC1) == SET) 
    { 
        TIM_ClearITPendingBit(TIM4, TIM_IT_CC1);
		capture = TIM_GetCapture1(TIM4); 
		TIM_SetCompare1(TIM4, capture + 2000);
    } 
}

void TIM3_IRQHandler(void) //vPW输入捕获中断TIM3_CH3
{	
	if((TIM_GetITStatus(TIM3, TIM_IT_CC3) == SET)&&(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0) == SET)) 
    {         
		//count = TIM_GetCapture3(TIM3);
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);
    }
}							   
