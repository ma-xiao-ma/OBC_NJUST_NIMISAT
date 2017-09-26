/*
 * cmd_test.c
 *
 *  Created on: 2016年7月27日
 *      Author: Administrator
 */

#include <stdint.h>
#include <stdio.h>

#include "command.h"

#include "bsp_ad7490.h"
#include "QB50_mem.h"
#include "contrl.h"
#include "ctrl_cmd_types.h"
#include "driver_debug.h"
#include "bsp_cpu_flash.h"
#include "crc.h"
//#include "bsp_camera.h"

#include "FreeRTOS.h"
#include "task.h"

//static uint8_t array_myfiles[64*1024] = {0};
//
//int save_files(struct command_context *ctx __attribute__((unused))){
//
//	bsp_EraseCpuFlash(ADDR_FLASH_SECTOR_8);
//
//	cup_flash_content_t * pointer = (cup_flash_content_t *)qb50Malloc(sizeof(cup_flash_content_t));
//
//	pointer->index = 0;
//	pointer->len = 91;
//	uint32_t j = 0;
//	uint16_t length = 100;
//	while(length --)
//	{
//		pointer->id = 0;
//		pointer->info = ((pointer->index << 12) | (pointer->id));
//		for (uint8_t i = 0; i < 91; i++)
//		{
//			pointer->data[i] = array_myfiles[j];
//			j ++;
//		}
//		pointer->id ++;
//		SavePermanentAudioFiles(pointer);
//	}
//	return CMD_ERROR_NONE;
//}



int write_flansh(struct command_context *ctx ){

	uint32_t index, id;
	uint32_t  len;

	char * args = command_args(ctx);

	cup_flash_content_t * pointer = NULL;
	pointer = (cup_flash_content_t *)qb50Malloc(sizeof(cup_flash_content_t));
	if(sscanf(args, "%u %u %u", &index, &id, &len) != 3)
		return CMD_ERROR_SYNTAX;

	pointer->id = id;
	pointer->len = len;
	pointer->index = index;

	for(uint8_t i=0;i<(pointer->len);i++)
		pointer->data[i] = i;
	SavePermanentAudioFiles(pointer);

	qb50Free(pointer);

	return CMD_ERROR_NONE;
}


int continuously_write_Permanent_flansh(struct command_context *ctx ){

	uint32_t index, id, len, framesize;
	cup_flash_content_t * pointer = NULL;
	char * args = command_args(ctx);

	pointer = (cup_flash_content_t *)qb50Malloc(sizeof(cup_flash_content_t));
	if(sscanf(args, "%u %u %u %u", &index, &id, &len, &framesize) != 4)
		return CMD_ERROR_SYNTAX;
	if((index < 0) || (index > 3)){
		driver_debug(DEBUG_FLASH, "index input error\r\n");
		return 1;
	}
	if((id < 0) || (id > 345)){
		driver_debug(DEBUG_FLASH, "id input error\r\n");
		return 2;
	}
	if((len < 0) || (len > 91)){
		driver_debug(DEBUG_FLASH, "len input error\r\n");
		return 3;
	}
	if(((id + framesize) < 0) || ((id + framesize) > 345)){
		driver_debug(DEBUG_FLASH, "framesize input error\r\n");
		return 4;
	}


	for(uint16_t i = 0;i < framesize;i ++)
	{
		pointer->info= ((index << 12) | id);
//		pointer->index = index;
//		pointer->id = id;
		id ++;
		pointer->len = len;

		for(uint16_t i=0;i<(pointer->len);i++)
			pointer->data[i] = i;
		SavePermanentAudioFiles(pointer);
	}

	qb50Free(pointer);
	return CMD_ERROR_NONE;
}



int continuously_write_new_flansh(struct command_context *ctx ){

	uint32_t index, id, len, framesize;
	cup_flash_content_t * pointer = NULL;
	char * args = command_args(ctx);

	pointer = (cup_flash_content_t *)qb50Malloc(sizeof(cup_flash_content_t));
	if(sscanf(args, "%u %u %u %u", &index, &id, &len, &framesize) != 4)
		return CMD_ERROR_SYNTAX;
	if((index < 0) || (index > 3)){
		driver_debug(DEBUG_FLASH, "index input error\r\n");
		return 1;
	}
	if((id < 0) || (id > 345)){
		driver_debug(DEBUG_FLASH, "id input error\r\n");
		return 2;
	}
	if((len < 0) || (len > 91)){
		driver_debug(DEBUG_FLASH, "len input error\r\n");
		return 3;
	}
	if(((id + framesize) < 0) || ((id + framesize) > 345)){
		driver_debug(DEBUG_FLASH, "framesize input error\r\n");
		return 4;
	}


	for(uint16_t i = 0;i < framesize;i ++)
	{
		pointer->info= ((index << 12) | id);
		id++;
		pointer->len = len;

		for(uint16_t i=0;i<(pointer->len);i++)
			pointer->data[i] = i;
		SaveNewAudioFiles(pointer);
	}

	qb50Free(pointer);
	return CMD_ERROR_NONE;
}



int read_Permanent_flansh(struct command_context *ctx )
{
	char * args = command_args(ctx);
	uint32_t index, id, len;
	cup_flash_cmd_t * pointer = NULL;

	pointer = (cup_flash_cmd_t *)qb50Malloc(sizeof(cup_flash_cmd_t));

	if(sscanf(args, "%u %u %u", &index, &id, &len) != 3)
			return CMD_ERROR_SYNTAX;

	pointer->info = ((index << 12) | id);
//	pointer->index = index;
//	pointer->id = id;
	pointer->len = len;

	xTaskCreate(DownloadSavedAudioFiles, "DownloadSavedAudioFiles", configMINIMAL_STACK_SIZE * 2,
			pointer, tskIDLE_PRIORITY + 4, NULL);

	qb50Free(pointer);
	return CMD_ERROR_NONE;

}


int read_new_flansh(struct command_context *ctx )
{
	char * args = command_args(ctx);
	uint32_t index, id, len;
	cup_flash_cmd_t * pointer = NULL;

	pointer = (cup_flash_cmd_t *)qb50Malloc(sizeof(cup_flash_cmd_t));

	if(sscanf(args, "%u %u %u", &index, &id, &len) != 3)
			return CMD_ERROR_SYNTAX;

	pointer->info = ((index << 12) | id);
//	pointer->index = index;
//	pointer->id = id;
	pointer->len = len;

	xTaskCreate(DownloadNewAudioFiles, "DownloadNewAudioFiles", configMINIMAL_STACK_SIZE * 2,
			pointer, tskIDLE_PRIORITY + 4, NULL);

	qb50Free(pointer);
	return CMD_ERROR_NONE;

}

int I2C_SendToADChip(struct command_context * context __attribute__((unused))){
	uint8_t SendData = 0x10;
	uint8_t ReceiveData[2];
	i2c_master_transaction(0,0x52,&SendData,1,NULL,0,0);
	vTaskDelay(1000);
	if(i2c_master_transaction(0,0x53,NULL,0,ReceiveData,2,20) == -1){
		for(uint8_t i = 0; i < 2; i++){
			driver_debug(DEBUG_I2C,"0x%02x \r\n", ReceiveData[i]);
		}
	}
	return CMD_ERROR_NONE;
}

int crc16_generate(struct command_context * context __attribute__((unused))){
	uint8_t* pdata = qb50Malloc(21);
	pdata[0] = 0x42;
	pdata[1] = 0x55;
	pdata[2] = 0x53;
	pdata[3] = 0x41;
	pdata[4] = 0x54;
	pdata[5] = 0x42;
	pdata[6] = 0x4a;
	pdata[7] = 0x42;
	pdata[8] = 0x55;
	pdata[9] = 0x53;
	pdata[10] = 0x41;
	pdata[11] = 0x54;
	pdata[12] = 0x42;
	pdata[13] = 0x4a;
	pdata[14] = 0x03;
	pdata[15] = 0xf0;
	pdata[16] = 0x00;
	pdata[17] = 0x03;
	pdata[18] = 0x05;
	pdata[19] = 0x01;
	pdata[20] = 0x0b;
	uint16_t mycrc = crc_citt_value(pdata, 21);

	uint8_t mycrchighbyte = (uint8_t)(mycrc >> 8);
	uint8_t mycrclowbyte = (uint8_t)mycrc;

	printf("%02x %02x\r\n",mycrchighbyte,mycrclowbyte);

	qb50Free(pdata);
	return CMD_ERROR_NONE;
}

static uint8_t test[4];

int ts_cam(struct command_context * context __attribute__((unused))){
    uint32_t test1 = 0x12345678;
    *(uint32_t *)test = test1;

    for(int i=0; i<4; i++)
        printf("%#02x ", test[i]);
    printf("\r\n");
    printf("%#08x\n", *(uint32_t *)test);

    memset(test, 0, (size_t)4);
    test[0] = 0x78;
    test[1] = 0x56;
    test[2] = 0x34;
    test[3] = 0x12;
    printf("%#08x\n", *(uint32_t *)test);

    memset(test, 0, (size_t)4);
    test[0] = 0x12;
    test[1] = 0x34;
    test[2] = 0x56;
    test[3] = 0x78;
    printf("%#08x\n", *(uint32_t *)test);

    return CMD_ERROR_NONE;
}

//int Camera_Test (struct command_context * context __attribute__((unused)))
//{
//    ImageQuality MyQuality = highQuality;
//    ImageResolution MyResolution = highResolution;
////    Camera_Synchronize();
////    Camera_Set_Image_Quality(lowQuality);
////    Camera_Set_Image_Resolution(highResolution);
////    Camera_Imaging(0);
////    Camera_Image_Info_Read(0);
////    uint32_t DataRemaining = 0;
////    DataRemaining = ImageSize;
////    for (int i=0; i<HowManyPack; i++, DataRemaining-=506)
////    {
////        if(DataRemaining > 506)
////        {
////            Camera_Get_Image_Packet(i, 506);
////            Camera_Image_Packet_Store(0, i, 506);
////        }
////        else
////        {
////            Camera_Get_Image_Packet(i, DataRemaining);
////            Camera_Image_Packet_Store(0, i, DataRemaining);
////        }
////    }
//    Camera_JPG_Store(0, MyQuality, MyResolution);
//    return 0;
//}

int dtb_tc(struct command_context *ctx )
{
    char * args = command_args(ctx);
    uint8_t cmd;
    if(sscanf(args, "%u", &cmd) != 1)
        return CMD_ERROR_SYNTAX;

    if(!xDTBTeleControlSend(cmd, 1000))
    {
        printf("DTB tc send success!\r\n");
    }

    return CMD_ERROR_NONE;
}

int dtb_tm(struct command_context * context __attribute__((unused)))
{
    static uint8_t tm_data[17];

    if(xDTBTelemetryGet(tm_data, 1000) > 0)
    {
        printf("DTB tm receive success!\r\n");
    }

    return CMD_ERROR_NONE;
}



struct command test_subcommands[] = {
	{
		.name = "wflansh",
		.help = "write flansh sector_8",
		.usage = "<index><id><w_len>",
		.handler = I2C_SendToADChip,
	},
//	{
//		.name = "savefiles",
//		.help = "save the audio files in flash sector_8",
//		.handler = save_files,
//	},
	{
		.name = "rpflansh",
		.help = "read flansh sector_8",
		.usage = "<index><id><r_len>",
		.handler = read_Permanent_flansh,
	},
	{
		.name = "rnflansh",
		.help = "read flansh sector_9",
		.usage = "<index><id><r_len>",
		.handler = read_new_flansh,
	},
	{
		.name = "cwpflansh",
		.help = "continuously write flansh sector_8",
		.usage = "<index><id><w_len><Number of frames>",
		.handler = continuously_write_Permanent_flansh,
	},
	{
		.name = "cwnflansh",
		.help = "continuously write flansh sector_9",
		.usage = "<index><id><w_len><Number of frames>",
		.handler = continuously_write_new_flansh,
	},
	{
		.name = "crc16",
		.help = "CameraSynchronize",
		.handler = crc16_generate,
	},
    {
        .name = "ts",
        .help = "just a test",
        .handler = ts_cam,
    },
//    {
//        .name = "camera",
//        .help = "generate crc-citt",
//        .handler = Camera_Test,
//    }
};

struct command __root_command test_commands_master[] =
{
	{
		.name = "test",
		.help = "test commands",
		.chain = INIT_CHAIN(test_subcommands),
	},
};

void cmd_test_setup(void)
{
	command_register(test_commands_master);
}
