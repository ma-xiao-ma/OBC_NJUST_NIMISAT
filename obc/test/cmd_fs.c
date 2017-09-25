/*
 * cmd_fs.c
 *
 *  Created on: 2015��11��29��
 *      Author: hp3
 */

#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "stm32f4xx.h"

#include "ff.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#include "command.h"
#include "error.h"

#include "bsp_pca9665.h"
#include "bsp_cpu_flash.h"
#include "bsp_reset.h"
#include "bsp_ds1302.h"

char cur_ls_path[40];

extern FIL fd;

int cmd_flash_program(struct command_context *ctx) {

	FIL myfile;
	UINT byteread;
	char   buffer;
	char * args = command_args(ctx);
	char * filename;
	uint32_t StartSector, EndSector, SectorCounter, StartAddr, EndAddr;

	if (sscanf(args, "%s", filename) != 1)
		return CMD_ERROR_SYNTAX;

	StartSector = 0;
	EndSector = 0;
	SectorCounter = 0;
	StartAddr = 0;
	EndAddr = 0;

	int result = f_open(&myfile,filename,FA_READ | FA_OPEN_EXISTING);
	if(result != FR_OK){
		printf("the filename is not existing\r\n");
		printf("open file error ,result is  :%u\r\n",result);

		return CMD_ERROR_FAIL;
	}

	StartAddr = 0x08000000;
	EndAddr = StartAddr + myfile.fsize*4;

	/* Get the number of the start and end sectors */
	StartSector = bsp_GetSector(StartAddr);
	EndSector = bsp_GetSector(EndAddr);

	printf("Program flash...\r\n");

	FLASH_Unlock();
	/* Clear pending flags (if any) */
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
					  FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);

	vPortEnterCritical();
	for (SectorCounter = StartSector; SectorCounter < EndSector; SectorCounter += 8)
	{
		/* Device voltage range supposed to be [2.7V to 3.6V], the operation will
		   be done by word */
		if(FLASH_EraseSector(SectorCounter, VoltageRange_3) != FLASH_COMPLETE)
		{
		  /* Error occurred while sector erase. */
		  printf("Erase flash error\r\n");
		  vPortExitCritical();
		  FLASH_Lock();

		  return CMD_ERROR_FAIL;
		}
	}
//	vPortExitCritical();

	int i = 0;

	f_lseek(&myfile, 0);

//	vPortEnterCritical();
	for(;;){
		result = f_read(&myfile,&buffer,1,&byteread);
		if(result == FR_OK){
			if(byteread == 0) break;
			i += 1;
			if(FLASH_ProgramByte(StartAddr, buffer) == FLASH_COMPLETE){
				StartAddr += 1;
			}else{
				printf("Program flash error\r\n");

				goto myreturn;
			}
		}else{
			printf("read file failed\r\n");
			printf("read error ,result is :%u\r\n",result);

			goto myreturn;
		}
	}
	printf("read byte number is: %d\r\n",i);

myreturn:

	vPortExitCritical();
	f_close(&myfile);
	FLASH_Lock();

//	__asm volatile(
//					"ldr sp, =0x20020000 \n"
//					"ldr r0, =0x08000004 \n"
//					"ldr r0, [r0] 		 \n"
//					"mov pc, r0 		 \n"
//					);

//	register char * stack_ptr asm ("sp");
//	stack_ptr = (char *) 0x20020000;
//	void (*jump) (void) = (void *) (0x08000000);
//	jump();

	return CMD_ERROR_NONE;
}

int cmd_fat_setdir(struct command_context *ctx __attribute__((unused))) {

	memset(cur_ls_path, 0, 40);
	strcpy(cur_ls_path,"0:");

	printf("current path is : %s\r\n", cur_ls_path);

	return CMD_ERROR_NONE;
}

int cmd_fat_ls(struct command_context *ctx __attribute__((unused))) {

	FILINFO fno;
	DIR dir;
	int i = 0;

	char lfname[25];
	uint32_t lfname_length = 25;

	printf("current path is : %s\r\n", cur_ls_path);

	int result = f_opendir(&dir,cur_ls_path);
	if(result == FR_OK){
		fno.lfname = lfname;
		fno.lfsize = lfname_length;
		for(;;){
			result = f_readdir(&dir,&fno);
			if(result != FR_OK){
				printf("read directory failed\r\n");
				printf("list error ,result is :%u\r\n",result);

				return CMD_ERROR_FAIL;
			}
			i++;
			if(fno.fname[0] == 0) break;
			if(fno.fname[0] == '.') continue;

			printf("%s\n",((uint8_t)fno.lfname[0])?fno.lfname:fno.fname);
		}
	}
	if(i <= 1) printf("no sub directory\r\n");

	return CMD_ERROR_NONE;

}

int cmd_fat_cd(struct command_context *ctx) {

	DIR dir;
	char * args = command_args(ctx);
	char * path;
	char  pre_path[40];

	if (sscanf(args, "%s", path) != 1)
		return CMD_ERROR_SYNTAX;

	strcpy(pre_path,cur_ls_path);
	strcat(cur_ls_path,"/");
	strcat(cur_ls_path,path);

	int result = f_opendir(&dir,cur_ls_path);
	if(result == FR_OK){
		printf("current path is : %s\r\n", cur_ls_path);
	}
	else {
		printf("the directory is not existing\r\n");
		strcpy(cur_ls_path,pre_path);

		return CMD_ERROR_FAIL;
	}

	return CMD_ERROR_NONE;
}

int cmd_fat_cat(struct command_context *ctx) {

	FIL myfile;
	UINT byteread;
	char   buffer;
	char * args = command_args(ctx);
	char * filename;
	char pre_path[40];

	if (sscanf(args, "%s", filename) != 1)
		return CMD_ERROR_SYNTAX;

	strcpy(pre_path,cur_ls_path);
	strcat(cur_ls_path,"/");
	strcat(cur_ls_path,filename);

	printf("the filename is: %s\r\n",cur_ls_path);

	int result = f_open(&myfile,cur_ls_path,FA_READ | FA_OPEN_EXISTING);
	if(result != FR_OK){
		printf("the filename is not existing\r\n");
		printf("open file error ,result is :%u\r\n",result);
		strcpy(cur_ls_path,pre_path);

		return CMD_ERROR_FAIL;
	}

	int i = 0;

	for(;;){
		result = f_read(&myfile,&buffer,1,&byteread);
		if(result == FR_OK){
			i++;
			if(isprint(buffer)){
				printf("%c",buffer);
			}
			else{
				printf("*");
			}
			if(byteread == 0) break;
		}
		else{
			printf("read failed\r\n");
			printf("read error ,result is :%u\r\n",result);
			f_close(&myfile);
			strcpy(cur_ls_path,pre_path);

			return CMD_ERROR_FAIL;
		}
	}
	printf("\r\n");
	printf("read byte number is: %d\r\n",i);
	f_close(&myfile);
	strcpy(cur_ls_path,pre_path);

	return CMD_ERROR_NONE;

}

int cmd_fat_mkfs(struct command_context *ctx) {

	/* Get args */
	char * args = command_args(ctx);
	unsigned int dev;
	if (sscanf(args, "%u", &dev) != 1)
		return CMD_ERROR_SYNTAX;

	printf("Formatting drive %u\r\n", (BYTE) dev);

	int result;

	vPortEnterCritical();

	result = f_mkfs((BYTE)dev, (BYTE)0, (BYTE)0);

	vPortExitCritical();

	printf("Format Result %u\r\n", result);

	return CMD_ERROR_NONE;

}

int cmd_fat_free(struct command_context *ctx __attribute__((unused))) {

	FATFS * fs;
	unsigned long int freeclusters;
	int result = f_getfree("/", &freeclusters, &fs);
	printf("Wee %u freeclusters %lu fs %p\r\n", result, freeclusters, fs);
	if (result == FR_OK) {
		printf("Clusters free %lu sectors per cluster %u, sector size %u\r\n", freeclusters, fs->csize, fs->fsize);
		printf("Bytes free: %lu of %lu\r\n", freeclusters * fs->csize * fs->fsize, (fs->n_fatent - 2) * fs->csize * fs->fsize);
	}

	return CMD_ERROR_NONE;

}

int cmd_fat_mkdir(struct command_context *ctx) {

	char * args = command_args(ctx);
	char * path;

	if (sscanf(args, "%s", path) != 1)
		return CMD_ERROR_SYNTAX;

	printf("path is %s\r\n", path);

	int result = f_mkdir(path);
	if (result == FR_OK)
	{
		printf("successful ,result is :%u\r\n",result);
	}
	else
		printf("error ,result is :%u\r\n",result);

	return CMD_ERROR_NONE;

}

int cmd_fat_rm(struct command_context *ctx) {

	char * args = command_args(ctx);
	char * fileordir;
	char pre_path[40];

	if (sscanf(args, "%s", fileordir) != 1)
		return CMD_ERROR_SYNTAX;

	strcpy(pre_path,cur_ls_path);
	strcat(cur_ls_path,"/");
	strcat(cur_ls_path,fileordir);

	int result = f_unlink(cur_ls_path);
	if (result == FR_OK)
		printf("remove successful ,result is :%u\r\n",result);
	else
		printf("remove error ,result is :%u\r\n",result);
	strcpy(cur_ls_path,pre_path);

	return CMD_ERROR_NONE;
}

int cmd_fat_write(struct command_context *ctx) {

	char * args = command_args(ctx);
	char * filename;
	char * string;

	if (sscanf(args, "%s %s", filename,string) != 2)
		return CMD_ERROR_SYNTAX;

	FIL myfile;
	UINT byterwrite;
	char pre_path[40];

	strcpy(pre_path,cur_ls_path);
	strcat(pre_path,"/");
	strcat(pre_path,filename);

	printf("the filename is: %s\r\n",filename);

	int result = f_open(&myfile,pre_path,FA_WRITE | FA_OPEN_ALWAYS);
	if(result != FR_OK){
		printf("the filename is not existing\r\n");
		printf("open file error ,result is :%u\r\n",result);

		return CMD_ERROR_FAIL;
	}

	result = f_write(&myfile,string,strlen(string),&byterwrite);

	f_close(&myfile);

	printf("written num: %u\r\n",byterwrite);
	if(result == FR_OK){
		printf("write %s to %s\r\n",string,filename);
	}

	return CMD_ERROR_NONE;
}

int cmd_fat_rmall(struct command_context *ctx) {

	char * args = command_args(ctx);

	FILINFO fno;
	DIR dir;
	char lfname[25];
	uint32_t lfname_length = 25;
	char path[40];

	memcpy(path, "0:hk", 5);
	printf("rm all hk\r\n");

	int result = f_opendir(&dir,path);

	if(result == FR_OK){

		strcat(path,"/");
		fno.lfname = lfname;
		fno.lfsize = lfname_length;

		portENTER_CRITICAL();

		f_close(&fd);

		for(;;){
			result = f_readdir(&dir,&fno);
			if(result != FR_OK){
				printf("read directory failed\r\n");
				printf("list error ,result is :%u\r\n",result);

				portEXIT_CRITICAL();

				return CMD_ERROR_FAIL;
			}
			if(fno.fname[0] == 0) break;
			if(fno.fname[0] == '.') continue;

			strcat(path, (((uint8_t)fno.lfname[0])?fno.lfname:fno.fname));
			f_unlink(path);

			memcpy(path, "0:hk/", 6);
		}

		cpu_reset();

		portEXIT_CRITICAL();
	}

	return CMD_ERROR_NONE;
}

int cmd_reset_all(struct command_context *ctx __attribute__((unused))) {

	char * args = command_args(ctx);

	FILINFO fno;
	DIR dir;
	char lfname[25];
	uint32_t lfname_length = 25;
	char path[40];

	memcpy(path, "0:hk", 5);
	printf("rm all hk\r\n");

	vTaskSuspendAll();

	FRESULT result = f_opendir(&dir,path);

	for(;;){
		result = f_readdir(&dir,&fno);
		if(result != FR_OK){
			printf("read directory failed\r\n");
			printf("list error ,result is :%u\r\n",result);

			portEXIT_CRITICAL();

			return CMD_ERROR_FAIL;
		}
		if(fno.fname[0] == 0) break;
		if(fno.fname[0] == '.') continue;

		strcat(path, (((uint8_t)fno.lfname[0])?fno.lfname:fno.fname));
		f_unlink(path);
		printf("rm file:%s\r\n", path);

		memcpy(path, "0:hk/", 6);
	}

	while(1);

	xTaskResumeAll();
	portEXIT_CRITICAL();

	return CMD_ERROR_NONE;
}

struct command fat_subcommands[] = {
	{
		.name = "reset",
		.help = "FAT reset dirptr to 0:",
		.handler = cmd_fat_setdir,
	},
/*	{
		.name = "mkfs",
		.help = "format file system",
		.usage = "<dev>",
		.handler = cmd_fat_mkfs,
	},*/
	{
		.name = "ls",
		.help = "FAT file list",
		.handler = cmd_fat_ls,
	},
	{
		.name = "cd",
		.help = "FAT change directory",
		.usage = "<path>",
		.handler = cmd_fat_cd,
	},
	{
		.name = "cat",
		.help = "FAT read file",
		.usage = "<filename>",
		.handler = cmd_fat_cat,
	},
/*	{
		.name = "program",
		.help = "program flash",
		.usage = "<filename>",
		.handler = cmd_flash_program,
	},*/
	{
		.name = "free",
		.help = "FAT show free space",
		.handler = cmd_fat_free,
	},
	{
		.name = "mkdir",
		.help = "FAT make directory",
		.usage = "<path>",
		.handler = cmd_fat_mkdir,
	},
	{
		.name = "rm",
		.help = "FAT remove file or directory",
		.usage = "<fileordir>",
		.handler = cmd_fat_rm,
	},
	{
		.name = "write",
		.help = "FAT write file",
		.usage = "<filename><string>",
		.handler = cmd_fat_write,
	},
	{
		.name = "rmall",
		.help = "FAT write file",
		.usage = "<opt>",
		.handler = cmd_fat_rmall,
	},
	{
		.name = "rst",
		.help = "reset board",
		.handler = cmd_reset_all,
	},
};

struct command __root_command fat_commands[] = {
	{
		.name = "fat",
		.help = "FAT filesystem commands",
		.chain = INIT_CHAIN(fat_subcommands),  //	struct chain {GOSH_CMD_STRUCT *list; unsigned int count;	}
	}
};

void cmd_fs_setup(void) {
	memset(cur_ls_path, 0, 40);
	strcpy(cur_ls_path,"0:");
	/* If memory was loaded okay, add plugins */
	command_register(fat_commands);

}
