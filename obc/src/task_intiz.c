/*
 * task_intiz.c
 *
 *  Created on: 2015年11月16日
 *      Author: iceyuyu
 */
#include "bsp_usart.h"
#include "console.h"
#include "command.h"

#include "bsp_adc.h"
#include "bsp_switch.h"
#include "bsp_watchdog.h"
#include "bsp_pca9665.h"
#include "bsp_spi.h"
#include "bsp_sdio_sd.h"
#include "bsp_nor_flash.h"
#include "bsp_fsmc_sram.h"
#include "bsp_ds1302.h"
#include "bsp_ad7490.h"
#include "bsp_intadc.h"
//#include "bsp_camera.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#include "ff.h"

#include "obc_argvs_save.h"
#include "hk.h"
#include "contrl.h"
#include "switches.h"
#include "camera_805.h"
#include "downlink_interface.h"
#include "driver_debug.h"

static FATFS fs; /* Work area (file system object) for logical drives */

//extern unsigned char driver_debug_switch[DEBUG_ENUM_MAX+1];

//void task_initz(void *pvParameters __attribute__((unused)))
void task_initz(void)
{

	Console_Usart_init(115200);

#if USE_SERIAL_PORT_DOWNLINK_INTERFACE

	vSerialInterfaceInit();

#endif

	Camera_805_Init();

	int_adc_init();

	spi_init_dev();
//	AD7490_Init();
	/*power related and switches*/
	bsp_InitSPI1();
	bsp_InitSwitch();
	/*on-chip clock RTC*/
	bsp_InitDS1302();
	/* Initialise RTC */
	struct ds1302_clock clock;
	timestamp_t timestamp;

	/* Get time from RTC */
	ds1302_clock_read_burst(&clock);
	ds1302_clock_to_time((time_t *) &timestamp.tv_sec, &clock);
	timestamp.tv_nsec = 0;

	/* Set time in lib-c system time*/
	clock_set_time(&timestamp);

	/*command initialize*/
	command_init();

extern unsigned char driver_debug_switch[DEBUG_ENUM_MAX+1];
	driver_debug_switch[DEBUG_HK] = 1;
	/*Initialize SD card*/
	SD_NVIC_Configuration();
	f_mount(0,&fs);

	/*use cpu flash to store the info*/
	obc_argvs_recover();

	/*house-keeping store to SD card*/
	hk_list_init(&hk_list);
	hk_list_recover();
	vTelemetryFileManage(&hk_list);

	/*I2C(PCA9665) initialize*/
	PCA9665_IO_Init();
	//driver_debug_switch[DEBUG_I2C] = 1;
	i2c_init(0, I2C_MASTER, 0x1A, 40, 5, 5, i2c_rx_callback); //frequency = 40Kbit/s
	i2c_init(1, I2C_MASTER, 0x08, 60, 5, 5, i2c_rx_callback);
	pca9665_isr_init();

	/*FSMC SRAM 初始化
	 *  容量：2MB*/
	bsp_InitExtSRAM();
	/*FSMC NorFlash 初始化
	 * 容量：4MB*/
	bsp_InitNorFlash();

	extern void cmd_eps_setup(void);
	cmd_eps_setup();
	extern void cmd_dfl_setup(void);
	cmd_dfl_setup();
	extern void cmd_fs_setup(void);
	cmd_fs_setup();
	extern void cmd_i2c_setup(void);
	cmd_i2c_setup();
	extern void cmd_nor_setup(void);
	cmd_nor_setup();
	extern void cmd_sram_setup(void);
	cmd_sram_setup();
	extern void cmd_rtc_setup(void);
	cmd_rtc_setup();
	extern void cmd_ad7490_setup(void);
	cmd_ad7490_setup();
	extern void cmd_pwr_setup(void);
	cmd_pwr_setup();
	extern void cmd_switches_setup(void);
	cmd_switches_setup();
	extern void cmd_intadc_setup(void);
	cmd_intadc_setup();
	extern void cmd_ants_setup(void);
	cmd_ants_setup();
	extern void cmd_test_setup(void);
	cmd_test_setup();
	extern void cmd_isis_setup(void);
	cmd_isis_setup();
	extern void cmd_norflash_setup(void);
	cmd_norflash_setup();
	extern void cmd_camera_setup(void);
	cmd_camera_setup();

//	vTaskDelete(NULL);
}
