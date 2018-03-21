/*
 * cmd_norflash.c
 *
 *  Created on: 2016年6月14日
 *      Author: Ma Wenli
 */
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "driver_debug.h"
#include "command.h"
#include "console.h"
#include "bsp_nor_flash.h"
#include "obc_mem.h"

/**
 * norflash 字节写入测试函数
 *
 * @param context
 * @return
 */
int nor_byte_write_handler(struct command_context * context)
{
    char * args = command_args(context);
    static uint32_t Byte_Addr;
    static uint8_t u8data;

    if (sscanf(args, "%u %x", &Byte_Addr, &u8data)!= 2)
        return CMD_ERROR_SYNTAX;

    uint8_t nor_return = 1;
    printf("WRITE: byte address: %u, data: 0x%02x\r\n", Byte_Addr, u8data);

    nor_return = NOR_WriteByte(Byte_Addr, u8data);

    /* 返回0说明写入正常 */
    printf("return is :%u\n",nor_return);
    u8data  = 0;
    u8data = NOR_ReadByte(Byte_Addr);
    printf("READ: byte address: %u, data: 0x%02x\r\n", Byte_Addr, u8data);

    return CMD_ERROR_NONE;
}


   /*经测试前 8K half-word(16K Byte)无法写入 */
int nor_half_write_handler(struct command_context * context)
{
	char * args = command_args(context);
	static uint32_t Half_Addr; /* 实际地址会乘以2。例：往半字地址32768写入其实是往字节地址65536写入*/
	static uint16_t u16data;

	if (sscanf(args, "%u %x", &Half_Addr, &u16data)!= 2)
		return CMD_ERROR_SYNTAX;

	uint8_t nor_return = 1;
	printf("WRITE: half word address: %u, data: 0x%04x\r\n", Half_Addr, u16data);

	nor_return = FSMC_NOR_WriteHalfWord(Half_Addr, u16data);

	/* 返回0说明写入正常 */
	printf("return is :%u\n",nor_return);
	u16data  = 0;
	u16data = FSMC_NOR_ReadHalfWord(Half_Addr);
	printf("READ: half word address: %u, data: 0x%04x\r\n", Half_Addr, u16data);

	return CMD_ERROR_NONE;
}

/**
 * norflash字节访问模式，读取
 *
 * @param context
 * @return
 */
int nor_byte_read_handler(struct command_context * context)
{

    char * args = command_args(context);
    static uint32_t byte_addr;
    uint8_t u8data;

    if (sscanf(args, "%u", &byte_addr) != 1)
        return CMD_ERROR_SYNTAX;

//    FSMC_NOR_ReadID();
    u8data = NOR_ReadByte(byte_addr);

    printf("READ: byte address: %u, data: 0x%02x\r\n", byte_addr, u8data);

    return CMD_ERROR_NONE;
}

int nor_half_read_handler(struct command_context * context)
{

	char * args = command_args(context);
	static uint32_t half_addr;
	static uint16_t u16data;

	if (sscanf(args, "%u", &half_addr) != 1)
		return CMD_ERROR_SYNTAX;

	FSMC_NOR_ReadID();
	u16data = FSMC_NOR_ReadHalfWord(half_addr);

	printf("READ: half word address: %u, data: 0x%04x\r\n", half_addr, u16data);

	return CMD_ERROR_NONE;
}



int NorFlash_Sector_Erase_Handler(struct command_context * context) {

    char * args = command_args(context);
    uint32_t SectorNum, SectorAddr;
    NOR_STATUS EraseResult = NOR_ERROR;

    if (sscanf(args, "%u", &SectorNum) != 1)
        return CMD_ERROR_SYNTAX;

    if(SectorNum < 8)
        SectorAddr = SectorNum * (8*1024);
    else
        SectorAddr = (SectorNum - 7) * (64*1024);

    EraseResult = USER_NOR_SectorErase(SectorNum);

    if(EraseResult != NOR_SUCCESS)
        printf("Section Erase Failed Addr:%u Sector: %u\n",
                SectorAddr, SectorNum);
    else
        printf("Section Erase Success Addr:%u Sector: %u\n",
                SectorAddr, SectorNum);
    return CMD_ERROR_NONE;
}


int NorFlash_Chip_Erase_Handler(struct command_context * context __attribute__((unused)))
{
    NOR_STATUS EraseResult = NOR_ERROR;

    FSMC_NOR_EraseChip();

    printf("Chip Erasing ...\r\n");
    vTaskDelay(40000);
    printf("Erase Complete!!\n");

    return CMD_ERROR_NONE;
}


static uint8_t NorArray[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};

int NorFlash_WriteBuffer_Handler(struct command_context * context __attribute__((unused))) {

    NOR_STATUS EraseResult = NOR_ERROR;

    EraseResult = FSMC_NOR_WriteBuffer((uint16_t*)NorArray, 32768, sizeof(NorArray)/2);
    if(EraseResult != NOR_SUCCESS)
    {
        printf("Write Buffer Failed!\n");
    }
    else
    {
        printf("Write Buffer Success!\n");
    }

    return CMD_ERROR_NONE;
}

int NorFlash_Read_Buffer_Handler(struct command_context * context) {

    char * args = command_args(context);
    uint32_t ReadAddr, NumToRead;
    u16 * pbuffer,* pfree;
    NOR_STATUS EraseResult = NOR_ERROR;

    if (sscanf(args,"%u %u", &ReadAddr, &NumToRead) != 2)
        return CMD_ERROR_SYNTAX;

    if(NumToRead > 100)
        return CMD_ERROR_NOMEM;

    pbuffer = (uint16_t *)ObcMemMalloc(NumToRead * sizeof(unsigned short));
    pfree = pbuffer;

    if(pbuffer == NULL)
        return CMD_ERROR_NOMEM;

    FSMC_NOR_ReadBuffer(pbuffer, ReadAddr, NumToRead);


    while(NumToRead--)
    {
        printf("0x%04x ", *pbuffer++);
    }
    printf("\r\n");

    ObcMemFree(pfree);

    return CMD_ERROR_NONE;
}

command_t __sub_command read_nor_sub[] = {
    {
        .name = "byte",
        .help = "byte read",
        .usage = "<byte address>",
        .handler = nor_byte_read_handler,
    },
    {
        .name = "half",
        .help = "half word read",
        .usage = "<half word address>",
        .handler = nor_half_read_handler,
    }
};

command_t __sub_command write_nor_sub[] = {
    {
        .name = "byte",
        .help = "byte write",
        .usage = "<byte address><byte data>",
        .handler = nor_byte_write_handler,
    },
    {
        .name = "half",
        .help = "half word write",
        .usage = "<half word address><half data>",
        .handler = nor_half_write_handler,
    }
};

command_t __sub_command cmd_norflash_sub[] = {
	{
		.name = "read",
		.help = "read flash",
		.chain = INIT_CHAIN(read_nor_sub),
	},
	{
		.name = "write",
		.help = "write to flash",
		.chain = INIT_CHAIN(write_nor_sub),
	},
	{
        .name = "chip_erase",
        .help = "Erase Chip",
        .handler = NorFlash_Chip_Erase_Handler,
    },
    {
        .name = "sector_erase",
        .help = "Erase Sector",
        .usage = "<SectorNum>",
        .handler = NorFlash_Sector_Erase_Handler,
    },
    {
        .name = "bufferw",
        .help = "Buffer Write",
        .handler = NorFlash_WriteBuffer_Handler,
    },
    {
        .name = "bufferr",
        .help = "Buffer Read",
        .usage = "<address><number>",
        .handler = NorFlash_Read_Buffer_Handler,
    }
};

command_t __root_command cmd_norflash_master[] = {
	{
		.name = "norflash",
		.help = "flash test",
		.chain = INIT_CHAIN(cmd_norflash_sub),
	}
};

void cmd_norflash_setup(void) {
	command_register(cmd_norflash_master);
}
