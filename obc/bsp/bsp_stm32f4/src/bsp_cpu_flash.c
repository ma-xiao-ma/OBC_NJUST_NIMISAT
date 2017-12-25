#include "bsp_cpu_flash.h"

#include "stm32f4xx_flash.h"

/**
 * 通过FLASH地址获取FLASH扇区号
 *
 * @param Address FLASH地址
 * @return 对应扇区号
 */
uint32_t bsp_GetSector(uint32_t Address)
{
	uint32_t sector = 0;

	if((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
	{
		sector = FLASH_Sector_0;
	}
	else if((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
	{
		sector = FLASH_Sector_1;
	}
	else if((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
	{
		sector = FLASH_Sector_2;
	}
	else if((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
	{
		sector = FLASH_Sector_3;
	}
	else if((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
	{
		sector = FLASH_Sector_4;
	}
	else if((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
	{
		sector = FLASH_Sector_5;
	}
	else if((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
	{
		sector = FLASH_Sector_6;
	}
	else if((Address < ADDR_FLASH_SECTOR_8) && (Address >= ADDR_FLASH_SECTOR_7))
	{
		sector = FLASH_Sector_7;
	}
	else if((Address < ADDR_FLASH_SECTOR_9) && (Address >= ADDR_FLASH_SECTOR_8))
	{
		sector = FLASH_Sector_8;
	}
	else if((Address < ADDR_FLASH_SECTOR_10) && (Address >= ADDR_FLASH_SECTOR_9))
	{
		sector = FLASH_Sector_9;
	}
	else if((Address < ADDR_FLASH_SECTOR_11) && (Address >= ADDR_FLASH_SECTOR_10))
	{
		sector = FLASH_Sector_10;
	}
	else	/*(Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_11))*/
	{
		sector = FLASH_Sector_11;
	}

	return sector;
}

void User_Write_FLASH(uint32_t start_addr, char *data, uint32_t len, uint32_t erase){

	uint32_t StartSector = 0, EndSector = 0, SectorCounter = 0, EndAddr = 0;

	EndAddr = start_addr + len;

	/* Get the number of the start and end sectors */
	StartSector = bsp_GetSector(start_addr);
	EndSector = bsp_GetSector(start_addr+len);

	if(erase){
		for (SectorCounter = StartSector; SectorCounter <= EndSector; SectorCounter += 8)
		{
			/* Device voltage range supposed to be [2.7V to 3.6V], the operation will
			   be done by word */
			if (FLASH_EraseSector(SectorCounter, VoltageRange_3) != FLASH_COMPLETE)
			{
			  /* Error occurred while sector erase. */
//			  printf("Erase flash error\r\n");

			  return;
			}
		}
	}
	FLASH_Unlock();

	/* Clear pending flags (if any) */
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
	                  FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);

	while (start_addr < EndAddr)
	{
		if (FLASH_ProgramByte(start_addr, *data++) == FLASH_COMPLETE)
		{
			start_addr = start_addr + 1;
		}
		else
		{
		  /* Error occurred while sector program. */
//		  printf("Program flash error\r\n");
		  FLASH_Lock();

		  return;
		}
	}

	FLASH_Lock();
}

/**
 * 片内FALSH读取API
 *
 * @param _ulFlashAddr 读取FALSH地址
 * @param _ucpDst 读取目的指针
 * @param _ulSize 读取字节数
 * @return 返回1为读取正确，0为错误
 */
uint8_t bsp_ReadCpuFlash(uint32_t _ulFlashAddr, uint8_t *_ucpDst, uint32_t _ulSize)
{
	uint32_t i;

	/* 判断读取的地址范围是否在flash的地址内 */
	if (_ulFlashAddr + _ulSize > FLASH_BASE_ADDR + FLASH_SIZE)
	{
		return 1;
	}

	/* 若读取长度为0，则认为错误 */
	if (_ulSize == 0)
	{
		return 1;
	}

	for (i = 0; i < _ulSize; i++)
	{
		*_ucpDst++ = *(uint8_t *)_ulFlashAddr++;
	}

	return 0;
}

/**
 * 比较待写入数据与FLASH中的原始数据，判断FLASH是否需要擦除
 *
 * @param _ulFlashAddr 待写入FLASH地址
 * @param _ucpBuf 待写入数据指针
 * @param _ulSize 待写入数据字节数
 * @return
 */
uint8_t bsp_CmpCpuFlash(uint32_t _ulFlashAddr, uint8_t *_ucpBuf, uint32_t _ulSize)
{
	uint32_t i;
	uint8_t ucIsEqu;
	uint8_t ucByte;

	/* 参数有效性判断*/
	if (_ulFlashAddr + _ulSize > FLASH_BASE_ADDR + FLASH_SIZE)
	{
		return FLASH_PARAM_ERR;
	}

	/* 参数有效性判断*/
	if (_ulSize == 0)
	{
		return FLASH_IS_EQU;
	}

	ucIsEqu = 1;
	for (i = 0; i < _ulSize; i++)
	{
		ucByte = *(uint8_t *)_ulFlashAddr;

		if (ucByte != *_ucpBuf)
		{
			if (ucByte != 0xFF)
			{
				return FLASH_REQ_ERASE;	 /*若FLASH中的数据与待写入的数据不一致且不等于0xFF，则需要擦除，才可写入*/
			}
			else
			{
				ucIsEqu = 0;
			}
		}

		_ulFlashAddr++;
		_ucpBuf++;
	}

	if (ucIsEqu == 1)
	{
		return FLASH_IS_EQU;	/* 若FLASH中的数据与待写入的数据一致，则返回EQU，无需写入*/
	}
	else
	{
		return FLASH_REQ_WRITE;	/* 若FLASH中的数据与待写入的数据不一致，且为0xFF则不需要擦除，直接写入即可*/
	}
}

/**
 * 片内FLASH写入
 *
 * @param _ulFlashAddr 待写入的片内FLASH地址
 * @param _ucpSrc 待写入数据指针
 * @param _ulSize 待写入字节数
 * @return 返回0写入成功， 返回1为擦除错误，返回2为编程错误
 */
uint8_t bsp_WriteCpuFlash(uint32_t _ulFlashAddr, uint8_t *_ucpSrc, uint32_t _ulSize)
{
	uint32_t i;
	uint8_t ucRet;

	/* 判断读取的地址范围是否在flash的地址内 */
	if (_ulFlashAddr + _ulSize > FLASH_BASE_ADDR + FLASH_SIZE)
	{
		return 1;
	}

	/* 若读取长度为0，则认为错误 */
	if (_ulSize == 0)
	{
		return 0;
	}

	/*比较待写入数据与FLASH中的原始数据，得出需不需要擦除*/
	ucRet = bsp_CmpCpuFlash(_ulFlashAddr, _ucpSrc, _ulSize);

	if (ucRet == FLASH_IS_EQU)
	{
		return 0;
	}

//	__set_PRIMASK(1);  /*屏蔽中断 */

	/* FLASH 解锁 */
	FLASH_Unlock();

	/*禁止数据缓存*/
	FLASH_DataCacheCmd(DISABLE);

  	/* Clear pending flags (if any) */
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                  FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);

	/* 若需要擦除 */
	if (ucRet == FLASH_REQ_ERASE)
	{
		if(FLASH_EraseSector(bsp_GetSector(_ulFlashAddr), VoltageRange_3) != FLASH_COMPLETE)
		{
			return 1; //擦除错误
		}
	}

	for (i = 0; i < _ulSize; i++)
	{
		if(FLASH_ProgramByte(_ulFlashAddr++, *_ucpSrc++) != FLASH_COMPLETE)
		{
			return 2; //编程错误
		}
	}

	/* 开启数据缓存 */
	FLASH_DataCacheCmd(ENABLE);
  	/* Flash 锁定 */
  	FLASH_Lock();

//  	__set_PRIMASK(0);  		/* ���ж� */

	return 0;
}

/**
 * 片内FLASH擦除某个扇区
 *
 * @param _ulFlashAddr 待擦除的FLASH地址
 * @return 返回0为擦除成功，返回1为擦除失败
 */
uint8_t bsp_EraseCpuFlash(uint32_t _ulFlashAddr)
{

//	__set_PRIMASK(1);

    /* Flash解锁 */
	FLASH_Unlock();

    /*禁止数据缓存*/
    FLASH_DataCacheCmd(DISABLE);

	/* Clear pending flags (if any) */
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
				  FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);

	if(FLASH_EraseSector(bsp_GetSector(_ulFlashAddr), VoltageRange_3) != FLASH_COMPLETE)
	{
		return 1;
	}

    /*开启数据缓存 */
    FLASH_DataCacheCmd(ENABLE);

    /* Flash锁定 */
	FLASH_Lock();

//	__set_PRIMASK(0);

	return 0;
}
