#ifndef __ADC_H
#define __ADC_H

#include "main.h"
#include "stm32f10x.h"
#include "stm32f10x_adc.h"

void ADC_init(void);
uint16_t ADC_Temp(uint16_t mode);

#endif
