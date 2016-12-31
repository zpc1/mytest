#ifndef __BSP_H_
#define __BSP_H_

#include "stm32f10x.h"
#include "stm32f10x_can.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rtc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_pwr.h"
#include "stm32f10x_flash.h"
#include "stm32f10x_iwdg.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_adc.h"

#include "misc.h"
#include "main.h"
#include "common.h"
#include "ff.h"
#include "CAN.h"
#include "USART.h"
#include "ISO14230.h"
#include "ISO15765.h"
#include "autoloop.h"
#include "NRF24L01.h"
#include "ADC.h"
#include "UM220.h"
#include "timelength.h"
#include <stdio.h>

#define GPRS_ON			//iCar-1000 or iCar-2000
//#undef GPRS_ON

#define ICAR_DOCK		//iCar-2000 or iCar-DOCK
#undef ICAR_DOCK

#define TEST_GPRS		//GPRS test data offline
#undef TEST_GPRS 

#define DEBUG			//user printf to debug
#undef DEBUG

typedef enum {
	ALL_FLASH = 0,
	ALL_STOP,
	DATA_STOP,
	DATA_FLASH,
}LED_ACT;

#ifdef ICAR_DOCK
#define LED1_OFF	GPIO_SetBits(GPIOA, GPIO_Pin_3);
#define LED2_OFF	GPIO_SetBits(GPIOA, GPIO_Pin_4);
#define LED3_OFF	GPIO_SetBits(GPIOB, GPIO_Pin_0);
#define LED4_OFF	GPIO_SetBits(GPIOB, GPIO_Pin_1);
#define LED1_ON		GPIO_ResetBits(GPIOA, GPIO_Pin_3);
#define LED2_ON		GPIO_ResetBits(GPIOA, GPIO_Pin_4);
#define LED3_ON		GPIO_ResetBits(GPIOB, GPIO_Pin_0);
#define LED4_ON		GPIO_ResetBits(GPIOB, GPIO_Pin_1);
#define PWRKEY_DOWN	GPIO_SetBits(GPIOA, GPIO_Pin_14);
#define PWRKEY_UP	GPIO_ResetBits(GPIOA, GPIO_Pin_14);
#else
#define LED1_OFF	GPIO_SetBits(GPIOB, GPIO_Pin_0);
#define LED2_OFF	GPIO_SetBits(GPIOB, GPIO_Pin_1);
#define LED1_ON		GPIO_ResetBits(GPIOB, GPIO_Pin_0);
#define LED2_ON		GPIO_ResetBits(GPIOB, GPIO_Pin_1);
#define PWRKEY_DOWN	GPIO_SetBits(GPIOB, GPIO_Pin_7);
#define PWRKEY_UP	GPIO_ResetBits(GPIOB, GPIO_Pin_7);
#endif

void clock_init(void);
void AD_init(void);
void DMA_Config(void);
void NVIC_init(void);
void IWDG_Configuration(void);
void gpio_init(void);
void tim_init(void);
void usart1_init(void);
void usart2_init(void);
void usart3_init(void);
void uart4_init(void);
void SPI_Flash_Init(void);
uint8_t CharToInt(uint8_t c);
uint8_t * IntToChar(const uint8_t Hex);
uint8_t IntCharH(const uint8_t Hex) ;
uint8_t IntCharL(const uint8_t Hex);
uint8_t mem_compare(uint8_t *data, uint8_t *p);
uint8_t mem_copy(uint8_t *data, uint8_t *p);
void mem_copy32(uint8_t *data, uint32_t p);
void mem_copy16(uint8_t *data, uint16_t p);

#endif
