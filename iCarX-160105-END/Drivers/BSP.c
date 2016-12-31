/**
  ******************************************************************************
  * @file    BSP.c 
  * @author  PDAger iCar team
  * @version V0.5.0
  * @date    12-August-2011
  * @brief   Driver initialize.
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "BSP.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define ADC1_DR_Address    ((u32)0x4001244C)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
ADC_InitTypeDef ADC_InitStructure;
DMA_InitTypeDef DMA_InitStructure;
GPIO_InitTypeDef GPIO_InitStructure;
EXTI_InitTypeDef EXTI_InitStructure;
TIM_TimeBaseInitTypeDef  TIM_BaseInitStructure;
ErrorStatus HSEStartUpStatus;
static uint8_t HexToCharBuff[2];
extern struct SCIBuffer SCI1,SCI2;
extern struct SCIBuffer2 SCI3,SCI4;
extern uint8_t IPR_flag;
extern vu16 ADC_ConvertedValue[];
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void clock_init(void)
{
	SystemInit();

	  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	
  	RCC_APB2PeriphClockCmd( RCC_APB2Periph_USART1
													| RCC_APB2Periph_TIM1 
													| RCC_APB2Periph_GPIOA 
													| RCC_APB2Periph_GPIOB
													| RCC_APB2Periph_GPIOC
													| RCC_APB2Periph_AFIO
													| RCC_APB2Periph_ADC1
													, ENABLE);					 
  
  	RCC_APB1PeriphClockCmd( RCC_APB1Periph_USART3
													| RCC_APB1Periph_UART4
													| RCC_APB1Periph_CAN1
													| RCC_APB1Periph_WWDG
													| RCC_APB1Periph_SPI2
													, ENABLE);
}

void NVIC_init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
						 
	NVIC_SetVectorTable(0x08000000,0x8000);			//中断向量表偏移地址

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3); //3组,最高3位用于指定抢占式优先级，最低1位用于指定响应优先级
  
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;    		//更新（向上溢出）中断：0.5ms
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;   //抢占优先级0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;          //响应优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             //允许中断
	NVIC_Init(&NVIC_InitStructure);                             //写入设置
}

void IWDG_Configuration(void) 
{ 
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);  //使能看门狗，50s喂一次
  IWDG_SetPrescaler(IWDG_Prescaler_256);         //40K/256=156HZ(6.4ms) 

  IWDG_SetReload(7810);     // 喂狗时间 50s/6.4ms=7810 
  IWDG_ReloadCounter(); 	// 喂狗 
  IWDG_Enable(); 		 	// 使能 
} 


void tim_init(void)
{
	TIM_DeInit(TIM1);											//溢出中断初始化
  	/* Time Base configuration */
	TIM_BaseInitStructure.TIM_Period = 50;			    		//计数峰值 = 10us × 50 = 0.5ms
	TIM_BaseInitStructure.TIM_Prescaler = 719;				   	//预分频   = 72MHz / (719 + 1) = 10us
	TIM_BaseInitStructure.TIM_ClockDivision = 0;
	TIM_BaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_BaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_BaseInitStructure);
	
	TIM_ClearFlag(TIM1, TIM_FLAG_Update);						//清中断标志
	TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE);					//允许更新(向上溢出)中断
  	/* TIM1 counter enable */
  	TIM_Cmd(TIM1,ENABLE);	
}

void gpio_init(void)
{
	GPIO_AFIODeInit();
	GPIO_DeInit(GPIOA);
	GPIO_DeInit(GPIOB);
	GPIO_DeInit(GPIOC);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;     //mpu6050中断引脚
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
//  GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;	        //SIM900A复位脚：PB7
  GPIO_Init(GPIOB, &GPIO_InitStructure);		  
	GPIO_ResetBits(GPIOB, GPIO_Pin_7);  		       
	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;	        //Ublox7M复位脚：PC7
  GPIO_Init(GPIOC, &GPIO_InitStructure);		    	 
	GPIO_ResetBits(GPIOC, GPIO_Pin_7);  		       

/******************************************* RTC ****************************************************/
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;					 
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;	
  	GPIO_Init(GPIOA, &GPIO_InitStructure);

/******************************************* LED ****************************************************/
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;					 
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;	//LED	 B0 B1
  	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB, GPIO_Pin_0|GPIO_Pin_1); 
/******************************************* BlueTooth ****************************************************/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;	        //USART1 TX	—— PA9
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;    	//复用推挽输出
  	GPIO_Init(GPIOA, &GPIO_InitStructure);		    	//A端口 
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;	        //USART1 RX	—— PA10
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//复用开漏输入
  	GPIO_Init(GPIOA, &GPIO_InitStructure);		        //A端口
/******************************************* SIM900A ****************************************************/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;	        //USART3 TX	—— PB10
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;    	//复用推挽输出
  	GPIO_Init(GPIOB, &GPIO_InitStructure);		    	//A端口 
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;	        //USART3 RX	—— PB11
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//复用开漏输入
  	GPIO_Init(GPIOB, &GPIO_InitStructure);		        //A端口
/******************************************* UM220III ****************************************************/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;	        //UART4 TX	—— PC10
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;    	//复用推挽输出
  	GPIO_Init(GPIOC, &GPIO_InitStructure);		    	//A端口 
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;	        //UART4 RX	—— PC11
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//复用开漏输入
  	GPIO_Init(GPIOC, &GPIO_InitStructure);		        //A端口
/******************************************* CAN AFIO ****************************************************/
	/* Configure CAN pin: RX */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;		   	//CAN_RX重定义到PB8
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  	/* Configure CAN pin: TX */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;			//CAN_TX重定义到PB9
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  	GPIO_PinRemapConfig(GPIO_Remap1_CAN1 , ENABLE);
}

void DMA_Config(void)
{
 	DMA_InitTypeDef DMA_InitStructure;//定义DMA初始化结构体
 	DMA_DeInit(DMA1_Channel1);//复位DMA通道1

 	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address; //定义 DMA通道外设基地址=ADC1_DR_Address
 	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADC_ConvertedValue; //定义DMA通道存储器地址
 	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//指定外设为源地址
 	DMA_InitStructure.DMA_BufferSize = 1;//定义DMA缓冲区大小9
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//当前外设寄存器地址不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//当前存储器地址不变
 	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//定义外设数据宽度16位
 	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; //定义存储器数据宽度16位
 	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//DMA通道操作模式位环形缓冲模式
 	DMA_InitStructure.DMA_Priority = DMA_Priority_High;//DMA通道优先级高
 	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;//禁止DMA通道存储器到存储器传输
 	DMA_Init(DMA1_Channel1, &DMA_InitStructure);//初始化DMA通道1

 	DMA_Cmd(DMA1_Channel1, ENABLE); //使能DMA通道1
}

void AD_init(void)
{
	/* ADC1 configuration ------------------------------------------------------*/
  	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  	ADC_InitStructure.ADC_NbrOfChannel = 1;
		ADC_Init(ADC1, &ADC_InitStructure);
   	
  	/* ADC1 regular channel 0~8 configuration */ 
  	ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 1, ADC_SampleTime_28Cycles5);
  
  	/* Enable ADC1 DMA */
  	ADC_DMACmd(ADC1, ENABLE);
	
	/* Enable ADC1 */
  	ADC_Cmd(ADC1, ENABLE);

  	/* Enable ADC1 reset calibaration register */   
  	ADC_ResetCalibration(ADC1);
  	/* Check the end of ADC1 reset calibration register */
  	while(ADC_GetResetCalibrationStatus(ADC1));

  	/* Start ADC1 calibaration */
  	ADC_StartCalibration(ADC1);
  	/* Check the end of ADC1 calibration */
  	while(ADC_GetCalibrationStatus(ADC1));
  	   
  	/* Start ADC1 Software Conversion */ 
  	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void usart1_init(void)
{
	USART_InitTypeDef USART_InitStructure;

	USART_DeInit(USART1);

	USART_InitStructure.USART_BaudRate = 9600;
  	USART_InitStructure.USART_WordLength = USART_WordLength_8b;		//0x00
  	USART_InitStructure.USART_StopBits = USART_StopBits_1;		   //0x0000
  	USART_InitStructure.USART_Parity = USART_Parity_No;			   //0x0000
  	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //0x0000
  	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  //0x000C
  	/* Configure USART1 */
  	USART_Init(USART1, &USART_InitStructure);
	//USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);	
  	/* Enable the USART1 */
  	USART_Cmd(USART1, ENABLE);

	SCI1.State = 1;
}

void usart2_init(void)
{	
	USART_InitTypeDef USART_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	USART_InitStructure.USART_BaudRate = 10400;
  	USART_InitStructure.USART_WordLength = USART_WordLength_8b;		//0x00
  	USART_InitStructure.USART_StopBits = USART_StopBits_1;		   //0x0000
  	USART_InitStructure.USART_Parity = USART_Parity_No;			   //0x0000
  	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //0x0000
  	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  //0x000C
  	/* Configure USART2 */
  	USART_Init(USART2, &USART_InitStructure);	
  	/* Enable the USART2 */
  	USART_Cmd(USART2, ENABLE);
	
	SCI2.State=1;	  
}

void uart4_init(void)
{	
		USART_InitTypeDef USART_InitStructure;

		USART_InitStructure.USART_BaudRate = 9600;
  	USART_InitStructure.USART_WordLength = USART_WordLength_8b;		//0x00
  	USART_InitStructure.USART_StopBits = USART_StopBits_1;		   //0x0000
  	USART_InitStructure.USART_Parity = USART_Parity_No;			   //0x0000
  	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //0x0000
  	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  //0x000C
  	/* Configure USART4 */
  	USART_Init(UART4, &USART_InitStructure);	
		//USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
  	/* Enable the USART4 */
  	USART_Cmd(UART4, ENABLE);
	
		SCI4.State=1;	  
}

void usart3_init(void)
{
#ifdef GPRS_ON

	USART_InitTypeDef USART_InitStructure; 
	uint16_t res;
	char backup_IPR[] = "IPR.tmp";//第一次运行时放入，用于SIM900A的波特率初始化（115200->9600）
	static FIL file;
	
	USART_DeInit(USART3);
 	
	f_chdir("/");
	res = f_chdir("BACKUP");
	if(res == FR_OK)			   
	{
		res = f_open(&file, backup_IPR, FA_OPEN_EXISTING | FA_READ);
		if(res == FR_OK)
		{		
			USART_InitStructure.USART_BaudRate = 115200;
			IPR_flag = 1;
			printf("usart3 115200\n");
		}
		else
		{			
			USART_InitStructure.USART_BaudRate = 9600;
			IPR_flag = 0;
			//printf("usart3 9600\n");
		}
	}
	else
	{
		USART_InitStructure.USART_BaudRate = 9600;
		IPR_flag = 0;
		//printf("usart3 9600 default\n");
	}
	f_chdir("/");

  	USART_InitStructure.USART_WordLength = USART_WordLength_8b;		//0x00
  	USART_InitStructure.USART_StopBits = USART_StopBits_1;		   //0x0000
  	USART_InitStructure.USART_Parity = USART_Parity_No;			   //0x0000
  	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //0x0000
  	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  //0x000C
  	/* Configure USART3 */
  	USART_Init(USART3, &USART_InitStructure);	
  	/* Enable the USART3 */
  	USART_Cmd(USART3, ENABLE);

	SCI3.State = 1;

#endif
}									  

void SPI_Flash_Init(void)
{
	SPI_InitTypeDef SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;	
	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB,GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);
	SPI_I2S_DeInit(SPI2);
	W25X_FLASH_CS_HIGH();
	 
	SPI_Cmd(SPI2, DISABLE);
  SPI_InitStructure.SPI_Direction =SPI_Direction_2Lines_FullDuplex; 
  SPI_InitStructure.SPI_Mode =SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize =SPI_DataSize_8b; 
  SPI_InitStructure.SPI_CPOL =SPI_CPOL_High; 
  SPI_InitStructure.SPI_CPHA =SPI_CPHA_2Edge; 
  SPI_InitStructure.SPI_NSS =SPI_NSS_Soft; 
  SPI_InitStructure.SPI_BaudRatePrescaler =SPI_BaudRatePrescaler_4;
  SPI_InitStructure.SPI_FirstBit =SPI_FirstBit_MSB; 
  SPI_InitStructure.SPI_CRCPolynomial =7; 
  SPI_Init(SPI2, &SPI_InitStructure);
  SPI_Cmd(SPI2, ENABLE);
	 
	SPI_Flash_SendByte(0xff);
}

uint8_t CharToInt(uint8_t c)
{
    if( ('A'<=c) && (c<='F') )//A-F
    {
        return c-'A'+10;
    }
    else if( ('a'<=c) && (c<='f'))
    {
        return c-'a'+10;
    }
    else if( ('0'<=c) && (c<='9') ) 
    {
        return c-'0';
    }
    else
    {
        return 0;
    }
}

//二进制数向字符串的转化
uint8_t * IntToChar(const uint8_t Hex)   
{
    uint8_t i;
    uint8_t HLf[2];//依次用于存放高四位数据和低四位数据
    
    HLf[0]=(uint8_t)(Hex>>4);
    HLf[1]=(uint8_t)(Hex%16);
    
    for(i=0;i<2;i++)
    {
        if(HLf[i] <= 9)
        {
            HexToCharBuff[i]=HLf[i]+'0';//0-9
        }
        else if((10<=HLf[i])&&(HLf[i]<=15))
        {
            HexToCharBuff[i]=HLf[i]+'a'-10;//a-f
        }
    }
    return &HexToCharBuff[0];
}

uint8_t IntCharH(const uint8_t Hex)   
{
    uint8_t HLf[2];//依次用于存放高四位数据和低四位数据
    							   
    HLf[0]=(uint8_t)(Hex/10);	   

    return HLf[0]+0x30;
}

uint8_t IntCharL(const uint8_t Hex)   
{
    uint8_t HLf[2];//依次用于存放高四位数据和低四位数据
    
    HLf[1]=(uint8_t)(Hex%10);	   

    return HLf[1]+0x30;
}

uint8_t mem_compare(uint8_t *data, uint8_t *p)
{
	while(*p)
	{
		if(*data != *p)
			return 1;
		p++;
		data++;
	}
	return 0;
}

uint8_t mem_copy(uint8_t *data, uint8_t *p)
{
	while(*p)
	{
		*data = *p;
		p++;
		data++;
	}
	return 0;
}

void mem_copy32(uint8_t *data, uint32_t p)
{
	data[0] = (uint8_t)(p >> 24);
	data[1] = (uint8_t)(p >> 16);
	data[2] = (uint8_t)(p >> 8);
	data[3] = (uint8_t)p;
}

void mem_copy16(uint8_t *data, uint16_t p)
{
	data[0] = (uint8_t)(p >> 8);
	data[1] = (uint8_t)p;
}

int fputc(int ch, FILE *f)
{
#ifdef DEBUG
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(USART1, (uint8_t) ch);

  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
  {}
#endif
		
  return ch;
}
