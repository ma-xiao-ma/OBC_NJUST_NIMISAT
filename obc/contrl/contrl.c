#include <if_downlink_serial.h>
#include "contrl.h"

#include "hk_arg.h"

#include "cube_com.h"

#include "sensor/flash_sd.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "portmacro.h"

#include "ctrl_cmd_types.h"
#include "error.h"
#include "driver_debug.h"
#include "switches.h"
#include "if_adcs.h"
#include "obc_mem.h"
#include "hk.h"
#include "crc.h"
#include "command.h"
#include "if_downlink_vu.h"

#include "bsp_ds1302.h"
#include "bsp_switch.h"
#include "bsp_pca9665.h"
#include "bsp_cis.h"
#include "bsp_cpu_flash.h"
#include "router_io.h"
#include "task_monitor.h"

#include "csp.h"

#include <stdint.h>
#include <time.h>
#include <string.h>


#define OpenAntenna_Time 		(10*60)   //
#define OpenBattery_Time 		(15*60)   //

#define Sleep_Battery 			7.2
#define Normal_Battery   		7.5		//
#define Abnormal_Battery   		1.0		//


uint8_t adcs_pwr_sta 	= 0;
uint8_t up_cmd_adcs_pwr	= 1;

///////////////////////////////////////local function////////////////////////////

//static uint8_t downloadopt = 0;
//static uint8_t cnt_to_save = 0;
uint8_t IsRealTelemetry = 1;	//1为实时遥测   0为延时遥测
///work mode variable
uint8_t mode = SLEEP_MODE;
static uint16_t down_cnt = 0; //遥测下行时间控制变量
static uint8_t down_cmd_enable = 0;
/////////////////////////////////////////////////////////////////////////////////


void taskdelaytime(uint32_t *time) {
	downtimeset = *time;
}

void downdelaytimes(uint32_t *times) {
    StorageIntervalCount = *times;
}

void chcontinuetimes(uint32_t *times) {
	Stop_Down_Time = *times;
}

/**
 * 下行普通数据接口
 *
 * @param dst 目的地址
 * @param src 源地址
 * @param type 消息类型
 * @param pdata 待发送数据指针
 * @param len 待发送数据长度
 */
void ProtocolSendDownCmd( uint8_t dst, uint8_t src, uint8_t type, void *pdata, uint32_t len )
{

#if USE_SERIAL_PORT_DOWNLINK_INTERFACE
    ProtocolSerialSend( dst, src, type, pdata, len );
#endif

#if CONFIG_USE_ISIS_VU
    vu_isis_send( dst, src, type, pdata, len );
#endif

#if CONFIG_USE_JLG_VU
    vu_jlg_send( dst, src, type, pdata, len );
#endif

}


void NormalWorkMode(void)
{

	if (mode == SLEEP_MODE)
	{
		mode = NORMAL_MODE;
	}

	adcs_send_mode(mode);
}

void SleepWorkMode(void)
{

	if (mode == NORMAL_MODE)
	{
		mode = SLEEP_MODE;
	}

	adcs_send_mode(mode);
}

typedef struct
{
    uint32_t vu_idle_state;
} control_para;

static control_para control_task;

#define VU_IDLE_ON      (15 * 60) /*通信机空闲状态连续发射开 持续时间    单位：秒*/

#define CONTROL_CYCLE   5000      /*控制周期  单位：毫秒*/

/**
 * 将秒数转换成在控制任务中的控制次数
 * @param pvParameters 秒数
 */
#define JUDGMENT(x)  ( x / (CONTROL_CYCLE / configTICK_RATE_HZ) )

void ControlTask(void * pvParameters __attribute__((unused)))
{

	portTickType xLastWakeTime = xTaskGetTickCount(); //for the 10s timer task

	vTaskDelayUntil(&xLastWakeTime, (10000 / portTICK_RATE_MS));

	vu_isis_hk_t *vu_tm = (vu_isis_hk_t *)ObcMemMalloc(sizeof(vu_isis_hk_t));
	if (vu_tm == NULL)
	    while(1);

	while (1)
	{
	    ttc_hk_get_peek(vu_tm);

	    if (vu_tm->tx_state.IdleState == RemainOn)
	        control_task.vu_idle_state++;
	    else
	        control_task.vu_idle_state = 0;

	    if (control_task.vu_idle_state++ >= JUDGMENT(VU_IDLE_ON))
	    {
	        vu_transmitter_set_idle_state(TurnOff);
	        control_task.vu_idle_state = 0;
	    }

//		switch (Battery_Task())
//		{
//            case 0:
//                SleepWorkMode();
//                break;
//            case 1:
//                NormalWorkMode();
//                break;
//            case 2:
//                break;
//            default:
//                break;
//		}

		vTaskDelayUntil(&xLastWakeTime, (CONTROL_CYCLE / portTICK_RATE_MS));
	}
}

void hk_collect_task(void *pvParameters __attribute__((unused)))
{

    eps_start();
    hk_collect_task_init();
    /* 等待个分系统启动 */
    vTaskDelay(5000);

    while (1)
    {
        task_report_alive(Collect);

        obc_hk_task();

        eps_hk_task();

        ttc_hk_task();

        /* 若数传上电，则获取遥测值 */
        if (OUT_SW_DTB_5V_PIN())
            dtb_hk_task();

        /* 若相机上电，则获取遥测值 */
        if (OUT_SW_CAMERA_10W_PIN() && OUT_SW_CAMERA_5W_PIN())
            cam_hk_task();

        /* 若姿控上电，则获取遥测值 */
        if (SW_EPS_S0_PIN())
            adcs_hk_task();

        vTaskDelay(2000);
    }
}

void OpenAntenna_Task(void* param __attribute__((unused))) {

	portTickType CurTime = xTaskGetTickCount();

	if(obc_boot_count <= 5)
	{
		vTaskDelayUntil(&CurTime, OpenAntenna_Time * (1000 / portTICK_RATE_MS));
	}
	else
	{
		vTaskDelayUntil(&CurTime, 120 * (1000 / portTICK_RATE_MS));
	}

//	else if (antenna_status == 2)
//	{
//		vTaskDelete(NULL);
//	}

	enable_antspwr(0,0);

	vTaskDelay(1000 / portTICK_RATE_MS);

	while (1)
	{
	    /*若天线全部展开*/
		if (get_antenna_status_nopara() == 2)
		{
		    /*置展开标志为2*/
			antenna_status = 2;
			disable_antspwr(0,0);
			vTaskDelete(NULL);
		}

		if (open_antenna() != 0)
		{
			vTaskDelay(20000 / portTICK_RATE_MS);

			if (get_antenna_status_nopara() == 1)
			{
				antenna_status = 1;
			}
			else if (get_antenna_status_nopara() == 2)
			{
				antenna_status = 2;
				disable_antspwr(0,0);
				vTaskDelete(NULL);
			}
			else
			{
				vTaskDelay(2000 / portTICK_RATE_MS);
			}
		}
		else
		{
			vTaskDelay(2000 / portTICK_RATE_MS);
		}
	}
}

/////////////////////////////////////////////
void OpenPanel_Task(void* param __attribute__((unused))) {

	if (obc_boot_count > 5)
	{
		vTaskDelete(NULL);
	}

	portTickType CurTime = xTaskGetTickCount();

	if(obc_boot_count <= 3)
	{
		vTaskDelayUntil(&CurTime, OpenBattery_Time * (1000 / portTICK_RATE_MS));
	}
	else
	{
		vTaskDelayUntil(&CurTime, 180 * (1000 / portTICK_RATE_MS));
	}

	while (1)
	{
		enable_panel(0,0);

		openpanel_times++;

		if (openpanel_times >= 2)
		{
			disable_panel(0,0);
			vTaskDelete(NULL);
		}

		vTaskDelay(10000 / portTICK_RATE_MS);
	}
}

int Battery_Task(const EpsAdcValue_t *eps_hk)
{

	float BatteryVoltage = 0.0;

	BatteryVoltage = eps_hk->Out_BusV * 0.001;

	if (BatteryVoltage > Normal_Battery || BatteryVoltage < Abnormal_Battery)
		return 1;
	else if (BatteryVoltage < Sleep_Battery)
		return 0;
	else
		return 2;
}

void vContrlStopDownload(void)
{
    up_hk_down_cmd = 0;

    vTaskDelay(100);

    down_cnt = 0;
    down_cmd_enable = 0;
    IsRealTelemetry = 1;
}

void down_save_task(void * pvParameters __attribute__((unused)))
{
    /*初始化主帧FIFO和辅帧FIFO*/
	HK_fifoInit(&hk_main_fifo);
	HK_fifoInit(&hk_append_fifo);

	uint16_t hk_down_counter = 0, hk_save_counter = 0, beacon_counter = 0;

	/*等待系统稳定*/
	vTaskDelay(2000);

	hk_store_init();

	/*存储时间间隔15秒*/
	StorageIntervalCount = (HK_STORAGE_INTERVAL * 1000)/downtimeset;

	/*下行时间间隔downtimeset毫秒， 一共下行10分钟，也就是600000毫秒*/
	Stop_Down_Time = (10 * 60 * 1000)/downtimeset;

	TickType_t xLastWakeTime = xTaskGetTickCount ();

	while (1)
	{
	    vTaskDelayUntil( &xLastWakeTime, downtimeset / portTICK_RATE_MS );

	    task_report_alive(DownSave);

		//if ((up_hk_down_cmd == 1 || PassFlag == 1) && down_cnt <= 60) {
		if ((up_hk_down_cmd == 1 ) && down_cnt <= 60)
		{
			up_hk_down_cmd = 0;
			down_cmd_enable = 1;
//			PassFlag = 0;
		}
		/* 如果开始下行标志置1，则下行遥测 */
		if (down_cmd_enable == 1)
		{
			hk_down_proc_task();

			if (++down_cnt >= Stop_Down_Time)  //下行次数 = 600000/downtimeset 默认下载200次
			{
				down_cnt = 0;
				down_cmd_enable = 0;
				IsRealTelemetry = 1;
			}
		}
		/* 否则保存遥测 */
		else
		{
			if (++down_cnt >= StorageIntervalCount)  //存储间隔15秒
			{
				down_cnt = 0;
				up_hk_down_cmd = 0;
//				PassFlag = 0;

				hk_data_save_task();
			}
		}

//		vTaskDelay( downtimeset / portTICK_RATE_MS );
	}
}

void hk_down_proc_task(void)
{
    /*如果是实时遥测数据下行*/
	if (IsRealTelemetry == 1)
	{
	    /* 遥测收集 */
        hk_collect_no_store();

        ProtocolSendDownCmd( GND_ROUTE_ADDR, OBC_ROUTE_ADDR, OBC_TELEMETRY,
                &hk_frame.main_frame, sizeof(HK_Main_t) );

        vTaskDelay(10 / portTICK_RATE_MS);

        ProtocolSendDownCmd( GND_ROUTE_ADDR, ADCS_ROUTE_ADDR, ADCS_TELEMETRY,
                &hk_frame.append_frame, sizeof(HK_Append_t) );
	}

	/*如果是延时遥测下行*/
	else
	{
	    /*若选择TF卡延时遥测下行*/
        if(hk_select == HK_SDCARD)
        {
//              int result = f_open(&hkfile, hk_sd_path, FA_READ | FA_OPEN_EXISTING);
//              if(result != FR_OK){
//                  driver_debug(DEBUG_HK,"the filename is not existing\r\n");
//                  driver_debug(DEBUG_HK,"open file error ,result is :%u\r\n",result);
//              }
            vPortEnterCritical();
            int result = f_lseek(&hkfile, hkleek);
            if(result != FR_OK)
            {
                f_close(&hkfile);
                hkleek      = 0;
                IsRealTelemetry = 1;
                vPortExitCritical();
                return;
            }

            result = f_read(&hkfile, &hk_old_frame, sizeof(HK_Store_t), &hkrbytes);
            if(result == FR_OK)
            {
                if(hkrbytes != sizeof(HK_Store_t))
                {
                    f_close(&hkfile);
                    hkleek      = 0;
                    IsRealTelemetry = 1;
                    vPortExitCritical();
                    return;
                }
                hkleek += sizeof(HK_Store_t);
            }
            else
            {
                f_close(&hkfile);
                hkleek      = 0;
                IsRealTelemetry = 1;
            }
            vPortExitCritical();

            ProtocolSendDownCmd( GND_ROUTE_ADDR, OBC_ROUTE_ADDR, OBC_TELEMETRY,
                    &hk_old_frame.main_frame, sizeof(HK_Main_t) );

            vTaskDelay(10 / portTICK_RATE_MS);

            ProtocolSendDownCmd( GND_ROUTE_ADDR, ADCS_ROUTE_ADDR, ADCS_TELEMETRY,
                    &hk_old_frame.append_frame, sizeof(HK_Append_t) );
        }

        /* 若选择SRAM延时遥测下行 */
        if(hk_select == HK_SRAM)
        {
            memcpy(&hk_old_frame.main_frame, hk_main_fifo.frame[hk_sram_index], sizeof(HK_Main_t));

            ProtocolSendDownCmd( GND_ROUTE_ADDR, OBC_ROUTE_ADDR, OBC_TELEMETRY,
                    &hk_old_frame.main_frame, sizeof(HK_Main_t) );

            memcpy(&hk_old_frame.append_frame, hk_append_fifo.frame[hk_sram_index], sizeof(HK_Append_t));

            ProtocolSendDownCmd( GND_ROUTE_ADDR, ADCS_ROUTE_ADDR, ADCS_TELEMETRY,
                    &hk_old_frame.append_frame, sizeof(HK_Append_t) );

            if((hk_sram_index++)%HK_FIFO_BUFFER_CNT == hk_main_fifo.rear)
            {
                IsRealTelemetry   = 1;
                hk_sram_index = 0;
            }
        }
	}
}

void hk_down_store_task(void)
{
	if (antenna_status != 0)
	{
		hk_collect();

        ProtocolSendDownCmd( GND_ROUTE_ADDR, OBC_ROUTE_ADDR, OBC_TELEMETRY,
                &hk_frame.main_frame, sizeof(HK_Main_t) );

		vTaskDelay(10 / portTICK_RATE_MS);

        ProtocolSendDownCmd( GND_ROUTE_ADDR, ADCS_ROUTE_ADDR, ADCS_TELEMETRY,
                &hk_frame.append_frame, sizeof(HK_Append_t) );
	}
}

void hk_data_save_task(void)
{
	hk_collect();

    ProtocolSendDownCmd( GND_ROUTE_ADDR, OBC_ROUTE_ADDR, OBC_TELEMETRY,
            &hk_frame.main_frame, sizeof(HK_Main_t) );

    vTaskDelay(10 / portTICK_RATE_MS);

    ProtocolSendDownCmd( GND_ROUTE_ADDR, ADCS_ROUTE_ADDR, ADCS_TELEMETRY,
            &hk_frame.append_frame, sizeof(HK_Append_t) );

	hk_store_add();
}


void adcs_pwr_task(void *pvParameters __attribute__((unused)))
{
	EpsOutSwitch(OUT_EPS_S0, ENABLE);  //enable ADCS power

	while (1)
	{
		EpsOutSwitch(OUT_EPS_S0, ENABLE);  //enable ADCS power
		if(SW_EPS_S0_PIN()) {
			vTaskDelete(NULL);
		}
		vTaskDelay(200);
		EpsOutSwitch(OUT_EPS_S0, ENABLE);  //enable ADCS power
		if(SW_EPS_S0_PIN()) {
			vTaskDelete(NULL);
		}
		vTaskDelay(200);
	}

}
