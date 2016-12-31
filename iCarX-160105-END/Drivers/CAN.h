#ifndef __CAN_H
#define __CAN_H

#include "stm32f10x.h"
#include "stm32f10x_can.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rtc.h"
#include "stm32f10x_usart.h" 
#include "main.h"

#define CAN_STD_ID CAN_ID_STD
#define CAN_EXT_ID CAN_ID_EXT

typedef unsigned char UINT8;

void can_init(const u32 Baud,int mode);
uint8_t TaskCanRx(CanRxMsg *RxMessage);	

#endif
