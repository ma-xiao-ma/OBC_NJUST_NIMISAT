
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <FreeRTOS.h>
#include <task.h>

#include "hk_arg.h"

#include "bsp_nor_flash.h"

#include "crc.h"

#include "sensor/flash_sd.h"

uint8_t write_flash(void){


	uint8_t res = flash_program(0, (uint8_t *)&store_info, sizeof(flash_store_t), 1);

	return res;
}

uint8_t read_flash(void){
	uint8_t res = 1;

	NOR_ReadBuffer(0, (uint8_t *)&store_info, sizeof(flash_store_t));

	return res;
}
