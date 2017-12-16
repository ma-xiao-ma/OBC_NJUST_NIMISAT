/*
 * task_intiz.c
 *
 *  Created on: 2015年11月16日
 *      Author: iceyuyu
 */
#include <if_downlink_serial.h>
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
#include "bsp_temp175.h"
//#include "bsp_camera.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#include "ff.h"

#include "route.h"
#include "router_io.h"
#include "obc_argvs_save.h"
#include "hk.h"
#include "contrl.h"
#include "switches.h"
#include "camera_805.h"
#include "driver_debug.h"
#include "task_monitor.h"


FATFS fs; /* Work area (file system object) for logical drives */

void task_initz(void)
{
//    extern uint8_t driver_debug_switch[DEBUG_ENUM_MAX+1];
//
//    driver_debug_switch[DEBUG_HK] = 1;

    /** 给SD卡挂载FATFS文件系统 */
    f_mount(0,&fs);

    /* 任务监视器初始化, 最大超时时间15s */
    supervisor_init(15000);

    /*控制开关IO口初始化*/
    bsp_InitSwitch();
    /*姿控上电*/
    EpsOutSwitch(OUT_EPS_S0, ENABLE);

	Console_Usart_init(115200);

#if USE_SERIAL_PORT_DOWNLINK_INTERFACE

	vSerialInterfaceInit();

#endif

//	Camera_805_Init();

	extern void Camera_Enlai_Usart_Init(uint32_t baudrate);
	Camera_Enlai_Usart_Init(115200);

	int_adc_init();

	spi_init_dev();
	AD7490_Init();
	/*power related and switches*/
	bsp_InitSPI1();

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

	/*use cpu flash to store the info*/
	obc_argvs_recover();

	/*house-keeping store to SD card*/
	hk_list_init(&hk_list);
	hk_list_recover();
//	vTelemetryFileManage(&hk_list);  /* 此函数会导致文件系统崩溃， 需查原因 */

#if USE_ROUTE_PROTOCOL

    router_init(1, 5);

    /*创建服务器任务*/
    server_start_task(512, 2);
    /*创建发送处理任务*/
    send_processing_start_task(256, 2);
    /*创建路由任务*/
    router_start_task(256, 1);

#endif

    /*I2C(PCA9665) initialize*/
    PCA9665_IO_Init();
    //driver_debug_switch[DEBUG_I2C] = 1;
    i2c_init(0, I2C_MASTER, 0x1A, 40, 5, 5, NULL); //frequency = 40Kbit/s
    i2c_init(1, I2C_MASTER, 0x08, 60, 5, 5, i2c_rx_callback);
    pca9665_isr_init();

	/*采温芯片初始化*/
	temp175_init();

	cmd_eps_setup();
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
	extern void cmd_vu_setup(void);
	cmd_vu_setup();
	extern void cmd_norflash_setup(void);
	cmd_norflash_setup();
	extern void cmd_camera_setup(void);
	cmd_camera_setup();

	cmd_ina_temp_setup();

//	vTaskDelete(NULL);
}
