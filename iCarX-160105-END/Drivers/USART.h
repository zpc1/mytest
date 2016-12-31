#ifndef __USART_H
#define __USART_H

#define SCI_BufferSize	250
#define SCI_BufferSize2 800

#include "stm32f10x.h"
#include "stm32f10x_can.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rtc.h"
#include "stm32f10x_usart.h" 
#include "main.h"

typedef unsigned char UINT8;

struct SCIBuffer
{
    UINT8 RxBuffer[SCI_BufferSize];
    UINT8 RxLen;
    UINT8 RxPos;//指向当前被读取位置，该位置的值没被读取，范围为0-(SCI_BufferSize-1)
    UINT8 TxLen;
    UINT8 TxPos;
    UINT8 State;//Remember to set these two state whenever you do some related operations
    UINT8 TxBuffer[SCI_BufferSize];
};

struct SCIBuffer2
{
    uint8_t RxBuffer[SCI_BufferSize2];
    uint16_t RxLen;
    uint16_t RxPos;//指向当前被读取位置，该位置的值没被读取，范围为0-(SCI_BufferSize-1)
    uint16_t TxLen;
    uint16_t TxPos;
    uint8_t State;//Remember to set these two state whenever you do some related operations
    uint8_t TxBuffer[SCI_BufferSize2];
};

uint16_t SCI_Transmit(uint8_t controller,uint16_t len,const uint8_t *txData);
uint16_t SCI_Receive(uint8_t controller,uint16_t len,uint8_t *rxData);
uint16_t SCI3_Receive(uint8_t turn,uint16_t len,uint8_t *rxData);
void SetState(uint8_t controller,uint8_t state);
uint16_t SCI_GetLen(uint8_t controller, uint8_t RorT);
void SCI_MainFunction(void);

#endif
