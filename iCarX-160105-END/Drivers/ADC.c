/**
  ******************************************************************************
  * @file    ADC.c 
  * @author  PDAger iCar team
  * @version V0.5.0
  * @date    9-November-2012
  * @brief   ADC.
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "ADC.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/	

void ADC_init(void)
{
	ADC_InitTypeDef ADC_InitStructure;

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  	ADC_InitStructure.ADC_NbrOfChannel = 1;
  	ADC_Init(ADC1, &ADC_InitStructure);	

	ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 1, ADC_SampleTime_55Cycles5); 
	
	ADC_TempSensorVrefintCmd(ENABLE);

	ADC_Cmd(ADC1, ENABLE);

	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));

  	ADC_StartCalibration(ADC1);
  	while(ADC_GetCalibrationStatus(ADC1));    

  	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
} 

uint16_t ADC_Temp(uint16_t mode)
{
	uint32_t AD_value = 0;
	static uint16_t avg_25 = 1430;

	if(mode == 0)
	{
		AD_value = ADC_GetConversionValue(ADC1);
		AD_value = (uint16_t)(((avg_25 - (AD_value*3300/4096))*100/43)+250);
		printf("ADC: temperature = %d\n",AD_value);
	}
	else if(mode == 1)
	{
		AD_value = ADC_GetConversionValue(ADC1);
		printf("ADC: Data = %d\n",AD_value);
	}
	else
	{
		avg_25 = mode;
	}
	
	return (uint16_t)AD_value;
} 
