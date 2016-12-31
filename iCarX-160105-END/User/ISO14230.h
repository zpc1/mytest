#ifndef __ISO14230_H
#define __ISO14230_H

#include "stm32f10x.h"
#include "stm32f10x_can.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rtc.h"
#include "stm32f10x_usart.h" 
#include "main.h"

#define     TIME_LIMIT_K      200                             //never set TIME_LIMIT more than 1000

//0--正常模式从车上获取数据,1--测试模式
#define     CONF_TEST_MODE_K  0

uint8_t ISO14230Main(void);
uint8_t get14230(void);
uint8_t dock14230(void);
void KwpToSci(void);
void SciToKwp(void);
void KwpRx(void);

#endif
