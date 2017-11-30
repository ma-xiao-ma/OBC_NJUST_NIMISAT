#ifndef _BSP_NOR_FLASH_H
#define _BSP_NOR_FLASH_H

#include <stdint.h>
#include "stm32f4xx.h"

#define NOR_FLASH_ADDR  			((uint32_t)0x64000000)

#define WORD_ADDR(addr)             ((vu16 *)(NOR_FLASH_ADDR + (2 * (addr))))
#define NOR_WRITE(Address, Data)    (*(vu16 *)(Address) = (Data))

#define BYTE_ADDR(addr)				((vu8 *)(NOR_FLASH_ADDR + ((addr))))
#define NOR_BWRITE(Address, Data)  	(*(vu8 *)(Address) = (Data))

#define NOR_SECTOR_SIZE				(128 * 1024)	/* 扇区大小 */
#define NOR_SECTOR_COUNT			128				/* 扇区数量 */
#define NOR_FLASH_SIZE				(NOR_SECTOR_SIZE * NOR_SECTOR_COUNT)

/*
	制造商ID: Spansion   0x01

	S29GL01GP	01 7E 28 01		1 Gigabit		128M字节
	S29GL512P	01 7E 23 01		512 Megabit		64M字节
	S29GL256P	01 7E 22 01		256 Megabit		32M字节
	S29GL128P	01 7E 21 01		128 Megabit		16M字节
*/

typedef enum
{
	S29GL128P = 0x017E2101,
	S29GL256P = 0x017E2201,
	S29GL512P = 0x017E2301,
	S29JL032H = 0x017E0A00,
	S29JL064J = 0x017E0201
}NOR_CHIP_ID;

/* NOR Status */
typedef enum
{
	NOR_SUCCESS = 0,
	NOR_ONGOING = 1,
	NOR_ERROR   = 2,
	NOR_TIMEOUT = 3
}NOR_STATUS;

void bsp_InitNorFlash(void);

void FSMC_NOR_ReadID(void);

NOR_STATUS USER_NOR_SectorErase(u8 SectorNum);
NOR_STATUS FSMC_NOR_EraseBlock(u32 BlockAddr);
NOR_STATUS FSMC_NOR_EraseChip(void);

uint16_t FSMC_NOR_ReadHalfWord(uint32_t ReadAddr);
void FSMC_NOR_ReadBuffer(u16* pBuffer, u32 ReadAddr, u32 NumHalfwordToRead);
NOR_STATUS FSMC_NOR_WriteHalfWord(uint32_t WriteAddr, uint16_t Data);
NOR_STATUS FSMC_NOR_WriteBuffer(u16* pBuffer, u32 WriteAddr, u32 NumHalfwordToWrite);

uint8_t NOR_ReadByte(uint32_t _uiWriteAddr);
uint8_t NOR_WriteByte(uint32_t _uiWriteAddr, uint8_t _usData);


#endif
