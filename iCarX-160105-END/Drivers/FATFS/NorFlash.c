/**
  ******************************************************************************
  * @file    NorFlash.c 
  * @author  Skylead iCarX team
  * @version V1.0
  * @date    8-December-2015
  * @brief   W25Q64 initialize.
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "NorFlash.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

//发送字节
uint8_t SPI_Flash_SendByte(uint8_t byte)
{
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(W25X_FLASH_SPI, byte);
 
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
	return SPI_I2S_ReceiveData(W25X_FLASH_SPI);
}
 
//读取字节
uint8_t SPI_Flash_ReadByte(void)
{
	return (SPI_Flash_SendByte(W25X_DUMMY_BYTE));
}

uint8_t W25X_Read_StatusReg(void)
{	
	u8 u8 = 0;
	W25X_FLASH_CS_LOW();
	SPI_Flash_SendByte(W25X_CMD_ReadStatusReg1);
	u8 = SPI_Flash_ReadByte();
	W25X_FLASH_CS_HIGH();
	return u8;
}
void SPI_Flash_Write_Enable(void)
{	
	W25X_FLASH_CS_LOW();
	SPI_Flash_SendByte(W25X_CMD_WriteEnable);
	W25X_FLASH_CS_HIGH();
}
void W25X_Write_Disable(void)
{	
	W25X_FLASH_CS_LOW();
	SPI_Flash_SendByte(W25X_CMD_WriteDisable);
	W25X_FLASH_CS_HIGH();
}
void SPI_Flash_WaitForWriteEnd(void)
{	
	while(W25X_Read_StatusReg() == 0x03)
		W25X_Read_StatusReg();
}

//读取设备ID
 
uint16_t SPI_Flash_ReadID(void)
{
	uint16_t Temp = 0, Temp0 = 0,Temp1 = 0;
 
	/*Select the FLASH: Chip Select low */
	W25X_FLASH_CS_LOW();
 
	/*! Send &quot;RDID &quot; instruction */
	SPI_Flash_SendByte(W25X_ManufactDeviveID);	
	SPI_Flash_SendByte(0x00);
	SPI_Flash_SendByte(0x00);
	SPI_Flash_SendByte(0x00);
	/*! Read a byte from the FLASH */
	Temp0 = SPI_Flash_SendByte(W25X_DUMMY_BYTE);
 
	/*!&lt; Read a byte from the FLASH */
	Temp1 = SPI_Flash_SendByte(W25X_DUMMY_BYTE);
	/*!&lt; Deselect the FLASH: Chip Select high */
	W25X_FLASH_CS_HIGH();
 
	Temp = (Temp0 << 8) | Temp1;
 
	return Temp;
}
 
//写一页256字节内。
void SPI_Flash_WritePage(const uint8_t* pBuffer, uint32_t WriteAddr)
{
	uint16_t i,j;

	SPI_Flash_Write_Enable();

	WriteAddr *= FLASH_SECTOR_SIZE; 	

	for(j=0;j<FLASH_PAGES_PER_SECTOR;j++)
	{
		W25X_FLASH_CS_LOW();
		
		SPI_Flash_SendByte(W25X_CMD_PageProgram);
		SPI_Flash_SendByte((u8)(WriteAddr >> 16));
		SPI_Flash_SendByte((u8)(WriteAddr >> 8));	
		SPI_Flash_SendByte((u8) WriteAddr);
		
		for(i=0;i<FLASH_PAGE_SIZE;i++)						
			SPI_Flash_SendByte(pBuffer[i]);
		
		pBuffer += FLASH_PAGE_SIZE;
		WriteAddr += FLASH_PAGE_SIZE;

		W25X_FLASH_CS_HIGH();
 
		SPI_Flash_WaitForWriteEnd();
	}
}

void W25QXX_Write_Page(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite)
{
 	uint16_t i;  
	
  SPI_Flash_Write_Enable();                 
	W25X_FLASH_CS_LOW();                          
  SPI_Flash_SendByte(W25X_CMD_PageProgram);     
  SPI_Flash_SendByte((u8)((WriteAddr)>>16));   
  SPI_Flash_SendByte((u8)((WriteAddr)>>8));   
  SPI_Flash_SendByte((u8)WriteAddr);   
  for(i=0;i<NumByteToWrite;i++)
		SPI_Flash_SendByte(pBuffer[i]);
	W25X_FLASH_CS_HIGH();
	SPI_Flash_WaitForWriteEnd();				   		
} 

void W25QXX_Write_NoCheck(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite)   
{ 			 		 
	u16 pageremain;	   
	pageremain=256-WriteAddr%256; 	 	 
		
	if(NumByteToWrite<=pageremain)
		pageremain=NumByteToWrite;
	while(1)
	{	   
		W25QXX_Write_Page(pBuffer,WriteAddr,pageremain);
		if(NumByteToWrite==pageremain)
			break;
	 	else 
		{
			pBuffer+=pageremain;
			WriteAddr+=pageremain;	

			NumByteToWrite-=pageremain;			  
			if(NumByteToWrite>256)
				pageremain=256; 
			else 
				pageremain=NumByteToWrite; 	 
		}
	}	    
} 
	 
void W25QXX_Write(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite)   
{ 
	u32 secpos;
	u16 secoff;
	u16 secremain;	   
 	u16 i;    
	u8 W25QXX_BUF[4096];	
	
  //W25QXX_BUF = W25QXX_BUFFER;	     
 	secpos=WriteAddr/4096;
	secoff=WriteAddr%4096;
	secremain=4096-secoff;
	
 	if(NumByteToWrite<=secremain)
		secremain=NumByteToWrite;
	while(1) 
	{	
		//while(1);
		W25QXX_Read(W25QXX_BUF,secpos*4096,4096);
		for(i=0;i<secremain;i++)
		{
			if(W25QXX_BUF[secoff+i]!=0XFF)
				break; 	  
		}
		if(i<secremain)
		{
			SPI_Flash_EraseSector(secpos);	
			for(i=0;i<secremain;i++)	   	
			{
				W25QXX_BUF[i+secoff]=pBuffer[i];	  
			}
			W25QXX_Write_NoCheck(W25QXX_BUF,secpos*4096,4096);

		}
		else 
			W25QXX_Write_NoCheck(pBuffer,WriteAddr,secremain);	
		
		if(NumByteToWrite==secremain)
			break;
		else
		{
			secpos++;
			secoff=0; 

		  pBuffer+=secremain;  			
			WriteAddr+=secremain;				
		  NumByteToWrite-=secremain;		
			if(NumByteToWrite>4096)
				secremain=4096;
			else 
				secremain=NumByteToWrite;	
		}	 
	}
}
//读
void SPI_Flash_ReadBuffer(uint8_t* pBuffer, uint32_t ReadAddr)
{
	uint16_t i;

	W25X_FLASH_CS_LOW();

	ReadAddr *= FLASH_SECTOR_SIZE;
 
	SPI_Flash_SendByte(W25X_CMD_ReadData);
	 
	SPI_Flash_SendByte((u8)(ReadAddr  >> 16));
	SPI_Flash_SendByte((u8)(ReadAddr >> 8));
	SPI_Flash_SendByte((u8) ReadAddr );
	 
	for(i=0;i<FLASH_SECTOR_SIZE;i++)	
	pBuffer[i] = SPI_Flash_ReadByte();
	 
	W25X_FLASH_CS_HIGH();
}
 
void W25QXX_Read(u8* pBuffer,u32 ReadAddr,u16 NumByteToRead)   
{ 
 	uint16_t i;
	
	W25X_FLASH_CS_LOW();
	
  SPI_Flash_SendByte(W25X_CMD_ReadData);      
  SPI_Flash_SendByte((u8)(ReadAddr>>16));  
  SPI_Flash_SendByte((u8)(ReadAddr>>8));   
  SPI_Flash_SendByte((u8)ReadAddr);   
  for(i=0;i<NumByteToRead;i++) 
        pBuffer[i]=SPI_Flash_ReadByte();   

	W25X_FLASH_CS_HIGH();  				    	      
}  

void SPI_Flash_EraseChip(void)
{
	SPI_Flash_Write_Enable();
	SPI_Flash_WaitForWriteEnd();
	W25X_FLASH_CS_LOW();
	SPI_Flash_SendByte(W25X_CMD_ChipErase);
	W25X_FLASH_CS_HIGH();
	SPI_Flash_WaitForWriteEnd();
}

void SPI_Flash_EraseSector(uint32_t nDest)
{
	nDest*=4096;
	
	SPI_Flash_Write_Enable();
	SPI_Flash_WaitForWriteEnd();
	W25X_FLASH_CS_LOW();
	SPI_Flash_SendByte(W25X_CMD_SectorErase);
	SPI_Flash_SendByte((u8)(nDest >> 16));
	SPI_Flash_SendByte((u8)(nDest >> 8));
	SPI_Flash_SendByte((u8)nDest);
	W25X_FLASH_CS_HIGH();
	SPI_Flash_WaitForWriteEnd();
}

