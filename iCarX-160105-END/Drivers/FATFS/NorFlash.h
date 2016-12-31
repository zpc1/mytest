#ifndef __NORFLASH_H_
#define __NORFLASH_H_

#include "BSP.h"

//W25Q64BV
#define W25Q64_DeviceID 0xEF16
 
#define W25X_CMD_WriteEnable 0x06 /*Write enable instruction */
#define W25X_CMD_WriteDisable 0x04 /*! Write to Memory Disable */
#define W25X_CMD_WriteStatusReg 0x01 /* Write Status Register instruction */
 
#define W25X_CMD_PageProgram 0x02 /* Write enable instruction */
#define W25X_CMD_QuadPageProgram 0x32 /* Write enable instruction */
 
#define W25X_CMD_BlockErase64 0xD8 /* Block 64k Erase instruction */
#define W25X_CMD_BlockErase32 0x52 /* Block 32k Erase instruction */
#define W25X_CMD_ChipErase 0xC7 /* Bulk Erase instruction */
#define W25X_CMD_SectorErase 0x20 /* Sector 4k Erase instruction */
#define W25X_CMD_EraseSuspend 0x75 /* Sector 4k Erase instruction */
#define W25X_CMD_EraseResume 0x7a /* Sector 4k Erase instruction */
 
#define W25X_CMD_ReadStatusReg1 0x05 /* Read Status Register instruction */
#define W25X_CMD_ReadStatusReg2 0x35 /* Read Status Register instruction */
 
#define W25X_CMD_High_Perform_Mode 0xa3
#define W25X_CMD_Conti_Read_Mode_Ret 0xff
 
#define W25X_WakeUp 0xAB
#define W25X_JedecDeviveID 0x9F /*Read identification */
#define W25X_ManufactDeviveID 0x90 /* Read identification */
#define W25X_ReadUniqueID 0x4B
 
#define W25X_Power_Down 0xB9 /*Sector 4k Erase instruction */
 
#define W25X_CMD_ReadData 0x03 /* Read from Memory instruction */
#define W25X_CMD_FastRead 0x0b /* Read from Memory instruction */
#define W25X_CMD_FastReadDualOut 0x3b /*Read from Memory instruction */
#define W25X_CMD_FastReadDualIO 0xBB /* Read from Memory instruction */
#define W25X_CMD_FastReadQuadOut 0x6b /* Read from Memory instruction */
#define W25X_CMD_FastReadQuadIO 0xeb /* Read from Memory instruction */
#define W25X_CMD_OctalWordRead 0xe3 /* Read from Memory instruction */
 
#define W25X_DUMMY_BYTE 0xff //0xA5
#define W25X_SPI_PAGESIZE 0x100

#define W25X_FLASH_SPI 		SPI2

#define W25X_FLASH_CS_LOW()			GPIO_ResetBits(GPIOB,GPIO_Pin_12)
#define W25X_FLASH_CS_HIGH()		GPIO_SetBits(GPIOB,GPIO_Pin_12)


//FLASH最大能够达到的地址，是8M
#define FLASH_MAX_ADDR FLASH_SECTOR_SIZE*FLASH_SECTOR_COUNT


#define FLASH_SECTOR_SIZE 	512			  

#define	FLASH_SECTOR_COUNT	16384
#define FLASH_BLOCK_SIZE   	8   

//FLASH页大小，256
#define FLASH_PAGE_SIZE 256

#define FLASH_PAGES_PER_SECTOR	FLASH_SECTOR_SIZE/FLASH_PAGE_SIZE

uint8_t SPI_Flash_SendByte(uint8_t byte);
void SPI_Flash_EraseSector(uint32_t nDest); 
void SPI_Flash_EraseChip(void);
void SPI_Flash_ReadBuffer(uint8_t* pBuffer, uint32_t ReadAddr);
void W25QXX_Read(u8* pBuffer,u32 ReadAddr,u16 NumByteToRead) ;
void SPI_Flash_WritePage(const uint8_t* pBuffer, uint32_t WriteAddr);
void W25QXX_Write(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite);
uint16_t SPI_Flash_ReadID(void);

#endif
