#ifndef _NRF24L01_H_
#define _NRF24L01_H_

#include "main.h"
#include "stm32f10x.h"
#include "stm32f10x_spi.h"

#define CE_H()   GPIO_SetBits(GPIOB, GPIO_Pin_4)
#define CE_L()   GPIO_ResetBits(GPIOB, GPIO_Pin_4)
#define CSN_H()  GPIO_SetBits(GPIOB, GPIO_Pin_5)
#define CSN_L()  GPIO_ResetBits(GPIOB, GPIO_Pin_5)

///*********************************************NRF24L01*************************************
#define TX_ADR_WIDTH    5   	// 5 uints TX address width
#define RX_ADR_WIDTH    5   	// 5 uints RX address width

#define RX_PLOAD_WIDTH  32  	// 32 uints TX payload 
#define TX_PLOAD_WIDTH  32
///***************************************NRF24L01寄存器指令*******************************************************
#define READ_REG2401    0x00  	// 读寄存器指令基址
#define WRITE_REG2401   0x20 	// 写寄存器指令基址
#define RD_RX_PLOAD     0x61  	// 接收模式下读RX有效数据：1-32字节
#define WR_TX_PLOAD     0xA0  	// 发送模式下写TX有效数据：1-32字节
#define FLUSH_TX        0xE1 	// 发送模式下清TX FIFO指令
#define FLUSH_RX        0xE2  	// 接收模式下清RX FIFO指令
#define REUSE_TX_PL     0xE3  	// 定义重复装载数据指令
#define NOP             0xFF  	// 保留
///*************************************SPI(nRF24L01)寄存器地址****************************************************
#define CONFIG          0x00  // 配置收发状态，CRC校验模式以及收发状态响应方式
#define EN_AA           0x01  // 自动应答功能设置
#define EN_RXADDR       0x02  // 可用信道设置
#define SETUP_AW        0x03  // 收发地址宽度设置
#define SETUP_RETR      0x04  // 自动重发功能设置
#define RF_CH           0x05  // 工作频率设置
#define RF_SETUP        0x06  // 发射速率、功耗功能设置
#define NRFRegSTATUS    0x07  // 状态寄存器
#define OBSERVE_TX      0x08  // 发送监测功能
#define CD              0x09  // 地址检测           
#define RX_ADDR_P0      0x0A  // 频道0接收数据地址
#define RX_ADDR_P1      0x0B  // 频道1接收数据地址
#define RX_ADDR_P2      0x0C  // 频道2接收数据地址
#define RX_ADDR_P3      0x0D  // 频道3接收数据地址
#define RX_ADDR_P4      0x0E  // 频道4接收数据地址
#define RX_ADDR_P5      0x0F  // 频道5接收数据地址
#define TX_ADDR         0x10  // 发送地址寄存器
#define RX_PW_P0        0x11  // 接收频道0接收数据长度
#define RX_PW_P1        0x12  // 接收频道1接收数据长度
#define RX_PW_P2        0x13  // 接收频道2接收数据长度
#define RX_PW_P3        0x14  // 接收频道3接收数据长度
#define RX_PW_P4        0x15  // 接收频道4接收数据长度
#define RX_PW_P5        0x16  // 接收频道5接收数据长度
#define FIFO_STATUS     0x17  // FIFO栈入栈出状态寄存器设置

void SPI2_init(void);
uint8_t NRF24SPI_Send_Byte(uint8_t dat);
void NRF24L01_init(void);
uint8_t SPI_RD_Reg(uint8_t reg);
uint8_t SPI_WR_Reg(uint8_t reg, uint8_t value);
uint8_t SPI_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t Len);
uint8_t SPI_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t Len);
void NRF_RX_Mode(void);
void NRF_TX_Mode(void);
void NRF_POWOFF_Mode(void);

void nRF24L01_TxPacket(uint8_t * tx_buf);
uint8_t nRF24L01_RxPacket(uint8_t* rx_buf);
void NRF_Send(void);

#endif
