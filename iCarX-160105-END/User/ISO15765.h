#ifndef __ISO15765_H
#define __ISO15765_H

#include "stm32f10x.h"
#include "stm32f10x_can.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rtc.h"
#include "stm32f10x_usart.h" 
#include "INT.h"
#include "main.h"
#include "CAN.h"
#include "ISO14230.h"

#define TIME_LIMIT 50

typedef unsigned char UINT8;

uint8_t ISO15765Main(void);
void send15765(void);
uint8_t get15765(void);
void CanToSci(void);
uint8_t SciToCan(void);

#endif
