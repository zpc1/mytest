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
///***************************************NRF24L01�Ĵ���ָ��*******************************************************
#define READ_REG2401    0x00  	// ���Ĵ���ָ���ַ
#define WRITE_REG2401   0x20 	// д�Ĵ���ָ���ַ
#define RD_RX_PLOAD     0x61  	// ����ģʽ�¶�RX��Ч���ݣ�1-32�ֽ�
#define WR_TX_PLOAD     0xA0  	// ����ģʽ��дTX��Ч���ݣ�1-32�ֽ�
#define FLUSH_TX        0xE1 	// ����ģʽ����TX FIFOָ��
#define FLUSH_RX        0xE2  	// ����ģʽ����RX FIFOָ��
#define REUSE_TX_PL     0xE3  	// �����ظ�װ������ָ��
#define NOP             0xFF  	// ����
///*************************************SPI(nRF24L01)�Ĵ�����ַ****************************************************
#define CONFIG          0x00  // �����շ�״̬��CRCУ��ģʽ�Լ��շ�״̬��Ӧ��ʽ
#define EN_AA           0x01  // �Զ�Ӧ��������
#define EN_RXADDR       0x02  // �����ŵ�����
#define SETUP_AW        0x03  // �շ���ַ�������
#define SETUP_RETR      0x04  // �Զ��ط���������
#define RF_CH           0x05  // ����Ƶ������
#define RF_SETUP        0x06  // �������ʡ����Ĺ�������
#define NRFRegSTATUS    0x07  // ״̬�Ĵ���
#define OBSERVE_TX      0x08  // ���ͼ�⹦��
#define CD              0x09  // ��ַ���           
#define RX_ADDR_P0      0x0A  // Ƶ��0�������ݵ�ַ
#define RX_ADDR_P1      0x0B  // Ƶ��1�������ݵ�ַ
#define RX_ADDR_P2      0x0C  // Ƶ��2�������ݵ�ַ
#define RX_ADDR_P3      0x0D  // Ƶ��3�������ݵ�ַ
#define RX_ADDR_P4      0x0E  // Ƶ��4�������ݵ�ַ
#define RX_ADDR_P5      0x0F  // Ƶ��5�������ݵ�ַ
#define TX_ADDR         0x10  // ���͵�ַ�Ĵ���
#define RX_PW_P0        0x11  // ����Ƶ��0�������ݳ���
#define RX_PW_P1        0x12  // ����Ƶ��1�������ݳ���
#define RX_PW_P2        0x13  // ����Ƶ��2�������ݳ���
#define RX_PW_P3        0x14  // ����Ƶ��3�������ݳ���
#define RX_PW_P4        0x15  // ����Ƶ��4�������ݳ���
#define RX_PW_P5        0x16  // ����Ƶ��5�������ݳ���
#define FIFO_STATUS     0x17  // FIFOջ��ջ��״̬�Ĵ�������

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
