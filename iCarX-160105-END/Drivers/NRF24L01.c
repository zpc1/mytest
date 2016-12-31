/**
  ******************************************************************************
  * @file    NRF20L01.c 
  * @author  PDAger iCar team
  * @version V0.5.0
  * @date    19-December-2011
  * @brief   SPI NRF20L01 driver.
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "NRF24L01.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t  TX_ADDRESS[TX_ADR_WIDTH]= {0xE7,0xE7,0xE7,0xE7,0xE7};	//本地地址
uint8_t  RX_ADDRESS[RX_ADR_WIDTH]= {0xE7,0xE7,0xE7,0xE7,0xE7};	//接收地址
extern struct SCIBuffer SCI4;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void SPI2_init(void)
{
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
   
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
  
  	/* Configure SPI2 pins: SCK, MISO and MOSI */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);
  	
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);//禁用JTAG以便使用PB4
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5; //CE,CSN
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);

	SCI4.State = 0;
	SPI_I2S_DeInit(SPI2);
  	
  	/* SPI2 configuration */ 
  	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		//SPI_CPOL_High=模式3，时钟空闲为高 //SPI_CPOL_Low=模式0，时钟空闲为低
  	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;	//SPI_CPHA_2Edge;//SPI_CPHA_1Edge, SPI_CPHA_2Edge;
  	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//SPI_NSS_Soft;//SPI_NSS_Hard
  	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;//SPI_BaudRatePrescaler_2=18M;//SPI_BaudRatePrescaler_4=9MHz
  	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;//数据从高位开始发送
  	SPI_InitStructure.SPI_CRCPolynomial = 7;  	
  	SPI_Init(SPI2, &SPI_InitStructure);
  	/* Enable SPI2  */
  	SPI_Cmd(SPI2, ENABLE);
}

uint8_t NRF24SPI_Send_Byte(uint8_t dat)
{
  /* Loop while DR register in not emplty */
  while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);

  /* Send byte through the SPI2 peripheral */
  SPI_I2S_SendData(SPI2, dat);

  /* Wait to receive a byte */
  while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);

  /* Return the byte read from the SPI bus */
  return SPI_I2S_ReceiveData(SPI2);
}

void NRF24L01_init(void)
{ 
#ifdef ICAR_DOCK
	uint8_t i;
	
	LED1_ON
	LED2_ON
	LED3_ON
	LED4_ON	
	
	SPI2_init();

	CE_L();    // chip enable
	CSN_H();   // Spi disable
	
	printf("nrf2401 init\n");	  	
	
	SPI_WR_Reg(WRITE_REG2401 + EN_AA, 0); //不允许自动应答	
	SPI_WR_Reg(WRITE_REG2401 + EN_RXADDR, 0x01);  //允许接收地址只有频道0
	SPI_WR_Reg(WRITE_REG2401 + SETUP_RETR, 0x03); //禁止自动重发
	SPI_WR_Reg(WRITE_REG2401 + RF_CH, 40);        //设置信道工作为2.4GHZ，收发必须一致
	SPI_WR_Reg(WRITE_REG2401 + RF_SETUP, 0x07);	  //1M速率
	SPI_WR_Reg(WRITE_REG2401 + RX_PW_P0, RX_PLOAD_WIDTH);//接收通道有效数据宽度最大
	SPI_WR_Reg(WRITE_REG2401 + CONFIG, 0x71);
		
	SPI_Write_Buf(WRITE_REG2401 + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);    // 写本地地址	
	SPI_Write_Buf(WRITE_REG2401 + RX_ADDR_P0, RX_ADDRESS, RX_ADR_WIDTH); // 写接收端地址 	  

	NRF_POWOFF_Mode();
	DelayMs(100);

	for(i=0;i<250;i++)
	{
		SCI4.RxBuffer[i] = 0;
		SCI4.TxBuffer[i] = 0;
	}	
	NRF_RX_Mode();
	DelayMs(100);
	SCI4.State = 1;

#endif
}

uint8_t SPI_RD_Reg(uint8_t reg)
{
	uint8_t reg_val;
	
	CSN_L();                // CSN low, initialize SPI communication...
	NRF24SPI_Send_Byte(reg);            // Select register to read from..
	reg_val = NRF24SPI_Send_Byte(0);    // ..then read registervalue
	CSN_H();                // CSN high, terminate SPI communication
	
	return(reg_val);        // return register value
}

uint8_t SPI_WR_Reg(uint8_t reg, uint8_t value)
{
	uint8_t status;
	
	CSN_L();                   // CSN low, init SPI transaction
	status = NRF24SPI_Send_Byte(reg);// select register
	NRF24SPI_Send_Byte(value);             // ..and write value to it..
	CSN_H();                   // CSN high again
	
	return(status);            // return nRF24L01 status uint8_t
}

uint8_t SPI_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t Len)
{
	uint8_t status,i;
	
	CSN_L();                    		// Set CSN low, init SPI tranaction
	status = NRF24SPI_Send_Byte(reg);       		// Select register to write to and read status uint8_t
	
  	for(i=0;i<Len;i++)
     	pBuf[i] = NRF24SPI_Send_Byte(0);
	
	CSN_H();                           
	
	return(status);                    // return nRF24L01 status uint8_t
}

uint8_t SPI_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t Len)
{
	uint8_t status,i;
	
	CSN_L();            //SPI使能       
	status = NRF24SPI_Send_Byte(reg);   
	for(i=0;i<Len;i++) 
	{
		NRF24SPI_Send_Byte(*pBuf);
		pBuf ++;
	}
	CSN_H();           //关闭SPI
	return(status);    
}

void NRF_RX_Mode(void)
{
	uint8_t PRIM_RX = SPI_RD_Reg(CONFIG);
	
	if((PRIM_RX & 0x01) == 0) //发射模式
	{
		CE_L();	
		SPI_WR_Reg(WRITE_REG2401 + CONFIG, 0x73);//0x0F);
		//SPI_WR_Reg(FLUSH_RX,0xFF);
		CE_H();
		//printf("rx mode...................\n");
	}		
}

void NRF_TX_Mode(void)
{
	uint8_t PRIM_RX = SPI_RD_Reg(CONFIG);

	if((PRIM_RX & 0x01) == 1) //接收模式
	{
		CE_L();	
		SPI_WR_Reg(WRITE_REG2401 + CONFIG, 0x72);//0x0E);
		//SPI_WR_Reg(FLUSH_TX,0xFF);
		CE_H();	
		//printf("tx mode...................\n");
	}
}

void NRF_POWOFF_Mode(void)
{
	uint8_t PRIM_RX = SPI_RD_Reg(CONFIG);	
 	
	if((PRIM_RX & 0x02) == 0x02) //上电模式
	{
		CE_L();	
		SPI_WR_Reg(WRITE_REG2401 + CONFIG, 0x70);//0x70);
		CE_H();	
	}
	else if((PRIM_RX & 0x02) == 0) //掉电模式
	{
		CE_L();	
		SPI_WR_Reg(WRITE_REG2401 + CONFIG, 0x72);//0x72);
		CE_H();	
	}
}

void nRF24L01_TxPacket(uint8_t * tx_buf)
{
	CE_L();			//StandBy I模式	
	SPI_Write_Buf(WRITE_REG2401 + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); // 装载接收端地址
	SPI_Write_Buf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH); 			 // 装载数据	
//	SPI_WR_Reg(WRITE_REG2401 + CONFIG, 0x0e);   		 // IRQ收发完成中断响应，16位CRC，主发送
	CE_H();		 //置高CE，激发数据发送
	//Delay_us(10);
}

uint8_t nRF24L01_RxPacket(uint8_t* rx_buf)
{
	uint8_t flag=0;
 	uint8_t status;
 	
	status=SPI_RD_Reg(NRFRegSTATUS);	// 读取状态寄存其来判断数据接收状况
	
	if(status != 0x0E)				// 判断是否接收到数据
	{
	   //CE_L(); 			//SPI使能
		 SPI_Read_Buf(RD_RX_PLOAD,rx_buf,TX_PLOAD_WIDTH);// read receive payload from RX_FIFO buffer
		 flag =1;			//读取数据完成标志
	}
	SPI_WR_Reg(WRITE_REG2401+NRFRegSTATUS, status);   //接收到数据后RX_DR,TX_DS,MAX_RT都置高为1，通过写1来清楚中断标志
	return flag;
}

void NRF_Send(void)
{
	uint8_t TxBuf[32]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	static uint16_t aaaa = 0;  
   
   //TX_Mode();
	DelayMs(2000);
	aaaa++;
	TxBuf[0] = aaaa%256;
	TxBuf[1] = aaaa/256;
    nRF24L01_TxPacket(TxBuf);
	SPI_WR_Reg(WRITE_REG2401 + NRFRegSTATUS, 0x20);	  
}  
