/*
 * cmd_norflash.c
 *
 *  Created on: 2016年6月14日
 *      Author: Ma Wenli
 */

#include "bsp_nor_flash.h"

#include "driver_debug.h"
#include "command.h"
#include "console.h"


#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"


   /*经测试前8K无法写入*/
int norflash_write_handler(struct command_context * context) {

	char * args = command_args(context);
	uint32_t u32MyAddr;
	uint16_t u16data;

	if (sscanf(args, "%x %u", &u16data, &u32MyAddr)!= 2)
		return CMD_ERROR_SYNTAX;

	uint8_t nor_re = 1;
	printf("write addr: %u data: %u\n", u32MyAddr, u16data);

	nor_re = FSMC_NOR_WriteHalfWord(u32MyAddr, u16data);

	printf("return is :%u\n",nor_re);
	u16data  = 0;
	u16data = FSMC_NOR_ReadHalfWord(u32MyAddr);
	printf("read addr: %u data: %04x\n", u32MyAddr, u16data);
	return CMD_ERROR_NONE;
}

int norflash_read_handler(struct command_context * context) {

	char * args = command_args(context);
	uint32_t addr;
	uint16_t u16data;

	if (sscanf(args, "%u", &addr) != 1)
		return CMD_ERROR_SYNTAX;

	FSMC_NOR_ReadID();
	u16data = FSMC_NOR_ReadHalfWord(addr);

	printf("Read Address: %u Data: 0x%04x \n", addr, u16data);

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


int NorFlash_Chip_Erase_Handler(struct command_context * context __attribute__((unused))) {

    NOR_STATUS EraseResult = NOR_ERROR;

    EraseResult = FSMC_NOR_EraseChip();
    if(EraseResult != NOR_SUCCESS)
    {
        printf("Chip Erase Failed!\n");
    }
    else
    {
        printf("Chip Erase Success!\n");
    }

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

    pbuffer = (uint16_t *)qb50Malloc(NumToRead * sizeof(unsigned short));
    pfree = pbuffer;

    if(pbuffer == NULL)
        return CMD_ERROR_NOMEM;

    FSMC_NOR_ReadBuffer(pbuffer, ReadAddr, NumToRead);


    while(NumToRead--)
    {
        printf("0x%04x ", *pbuffer++);
    }
    printf("\r\n");

    qb50Free(pfree);

    return CMD_ERROR_NONE;
}

struct command cmd_norflash_sub[] = {
	{
		.name = "read",
		.help = "read flash",
		.usage = "<address>",
		.handler = norflash_read_handler,
	},{
		.name = "write",
		.help = "write to flash",
		.usage = "<data><address>",
		.handler = norflash_write_handler,
	},{
        .name = "chip_erase",
        .help = "Erase Chip",
        .handler = NorFlash_Chip_Erase_Handler,
    },{
        .name = "sector_erase",
        .help = "Erase Sector",
        .usage = "<SectorNum>",
        .handler = NorFlash_Sector_Erase_Handler,
    },{
        .name = "bufferw",
        .help = "Buffer Write",
        .handler = NorFlash_WriteBuffer_Handler,
    },{
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
