#include <stdint.h>
#include <time.h>
#include <string.h>

#include "contrl.h"
#include "hk_arg.h"
#include "cube_com.h"
#include "sensor/flash_sd.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "portmacro.h"

#include "if_downlink_serial.h"
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
#include "obc_argvs_save.h"
#include "bsp_nor_flash.h"

#include "bsp_ds1302.h"
#include "bsp_switch.h"
#include "bsp_pca9665.h"
#include "bsp_cis.h"
#include "bsp_cpu_flash.h"
#include "router_io.h"
#include "task_monitor.h"

#include "csp.h"




#define OpenAntenna_Time 		(10*60)   //
#define OpenBattery_Time 		(15*60)   //

#define Sleep_Battery 			7.2
#define Normal_Battery   		7.5		//
#define Abnormal_Battery   		1.0		//


uint8_t adcs_pwr_sta 	= 0;
uint8_t up_cmd_adcs_pwr	= 1;

///////////////////////////////////////local function////////////////////////////

uint8_t IsJLGvuWorking = 0;     /*JLG通信机开启标志，开启为1， 关闭为0*/
bool PassFlag = false;          /*过境标志*/

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

void NormalWorkMode(void)
{
	if( mode == SLEEP_MODE )
	{
	    adcs_send_mode(mode);
		mode = NORMAL_MODE;
	}
}

void SleepWorkMode(void)
{
	if( mode == NORMAL_MODE )
	{
	    adcs_send_mode(mode);
		mode = SLEEP_MODE;
	}
}

typedef struct
{
    uint32_t vu_idle_state;
    uint32_t vu_jlg_switch_on;
    uint32_t passing;
    uint32_t time_valid;
} control_para;

static control_para control_task;

#define VU_IDLE_ON       (10 * 60) /*通信机空闲状态连续发射开 持续时间    单位：秒*/
#define JLG_VU_SWITCH_ON (10 * 60) /*解理工通信机备份机切换     持续时间    单位：秒*/
#define PASSIN_LAST      (10 * 60) /*过境时间*/
#define TIME_SYSN        ( 5 * 60) /*时间同步间隔*/

#define CONTROL_CYCLE    5000      /*控制周期  单位：毫秒*/

/**
 * 将秒数转换成在控制任务中的控制次数
 * @param pvParameters 秒数
 */
#define JUDGMENT(x)  ( x / (CONTROL_CYCLE / configTICK_RATE_HZ) )

void ControlTask(void * pvParameters __attribute__((unused)))
{
	vu_isis_hk_t *vu_tm = (vu_isis_hk_t *)ObcMemMalloc(sizeof(vu_isis_hk_t));
	obc_hk_t *obc_tm = (obc_hk_t *)ObcMemMalloc(sizeof(obc_hk_t));
//	eps_hk_t *eps_tm = (eps_hk_t *)ObcMemMalloc(sizeof(eps_hk_t));

	if (vu_tm == NULL || obc_tm == NULL)
	    cpu_reset();

	portTickType xLastWakeTime = xTaskGetTickCount();

	while (1)
	{
	    task_report_alive(Control);

        vTaskDelayUntil(&xLastWakeTime, (CONTROL_CYCLE / portTICK_RATE_MS));

	    obc_hk_get_peek(obc_tm);
	    ttc_hk_get_peek(vu_tm);
//	    eps_hk_get_peek(eps_tm);

	    /**如果通信机空闲状态连续发射已经开启*/
	    if (vu_tm->tx_state.IdleState == RemainOn)
	        control_task.vu_idle_state++;
	    else
	        control_task.vu_idle_state = 0;

	    /**如果备份通信机是开机状态，则计数器累加*/
	    if ( ((obc_switch_t *)&obc_tm->on_off_status)->jlg_vu_on == true ||
	            ((obc_switch_t *)&obc_tm->on_off_status)->switch_vu_on == true)
	        control_task.vu_jlg_switch_on++;
	    else
	        control_task.vu_jlg_switch_on = 0;

        /**如果过境标志已经置位*/
        if (PassFlag == true)
            control_task.passing++;
        else
            control_task.passing = 0;

        /**若星上时间有效则每5分钟向姿控系统同步时间信息*/
        if (obc_tm->utc_time > 1514856690 && obc_tm->utc_time < 1577836800)
            control_task.time_valid ++;
        else
            control_task.time_valid = 0;

	    /* 如果通信机空闲状态连续发射连续开启超过15分钟，或者备份通信机启动时，则自动关闭连续发射 */
	    if ( control_task.vu_idle_state > JUDGMENT(VU_IDLE_ON) || IsJLGvuWorking )
	    {
	        /* 若连续发射已经关闭，则不再执行 */
	        if (control_task.vu_idle_state != TurnOff)
	            vu_transmitter_set_idle_state(TurnOff);
	    }

        /* 如果备份通信机开启超过15分钟，则自动切换为主份 */
        if ( control_task.vu_jlg_switch_on > JUDGMENT(JLG_VU_SWITCH_ON) )
        {
            /***此处应将通信机切换回ISIS通信机, 除能备份通信机加电, 并将信道控制板断电*/
            vu_backup_switch_off();
        }

        /* 每次过境不会超过15分钟，因此超过15分钟清除过境标志*/
        if ( control_task.passing > JUDGMENT(PASSIN_LAST) )
        {
            PassFlag = false;
        }

        /* 星上时间每5min跟姿控同步一次 */
        if ( control_task.time_valid > JUDGMENT(TIME_SYSN) )
        {
            adcstimesync( clock_get_time_nopara() );
            control_task.time_valid = 0;
        }

//        /**功耗控制函数*/
//		switch( Battery_Task(eps_tm) )
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
	}
}

void hk_collect_task(void *pvParameters __attribute__((unused)))
{

    eps_start();
    hk_collect_task_init();
    /* 等待个分系统启动 */
    vTaskDelay(2000);

    while (1)
    {
        task_report_alive(Collect);

        obc_hk_task();

        eps_hk_task();

        ttc_hk_task();

//        /* 若数传上电，则获取遥测值 */
//        if (OUT_SW_DTB_5V_PIN())
//            dtb_hk_task();

        if (IsJLGvuWorking)
            jlg_hk_task();

        /* 若相机上电，则获取遥测值 */
        if (OUT_SW_CAMERA_10W_PIN() && OUT_SW_CAMERA_5W_PIN())
            cam_hk_task();

        /* 若姿控上电，则获取遥测值 */
        if (SW_EPS_S0_PIN())
            adcs_hk_task();

        vTaskDelay(2000);
    }
}

void OpenAntenna_Task(void* param __attribute__((unused)))
{

	portTickType CurTime = xTaskGetTickCount();

	if(obc_boot_count < 5)
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

	enable_antspwr(1000,0);

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

/**
 * 太阳能电池阵展开任务
 *
 * @param param
 */
void OpenPanel_Task(void* param __attribute__((unused)))
{

	portTickType CurTime = xTaskGetTickCount();

	if(obc_boot_count < 5)
		vTaskDelayUntil(&CurTime, OpenBattery_Time * (1000 / portTICK_RATE_MS));
	else
	    vTaskDelete(NULL);

	while (1)
	{
		enable_panel(6000);

		Solar_Array_Unfold(10);

		openpanel_times++;

		if( IN_SW_PAL_STATUS_1_PIN() && IN_SW_PAL_STATUS_2_PIN() )
		{
		    disable_panel(0,0);
            vTaskDelete(NULL);
		}

		if (openpanel_times >= 4)
		{
			disable_panel(0,0);
			vTaskDelete(NULL);
		}

		vTaskDelay(10000 / portTICK_RATE_MS);
	}
}

int Battery_Task(const eps_hk_t *eps_hk)
{

	float BatteryVoltage = 0.0;

	BatteryVoltage = eps_hk->out_BusV * 0.001;

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

				hk_data_save_task(); /*250分钟存满一个文件，一个文件1000调遥测*/
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

	if ( !PassFlag ) /* 非过境时每个存储周期下行遥测 */
	{
        /**
         * 遥测值信标
         */
        ProtocolSendDownCmd( GND_ROUTE_ADDR, OBC_ROUTE_ADDR, OBC_TELEMETRY,
                &hk_frame.main_frame, sizeof(HK_Main_t) );

        vTaskDelay(10 / portTICK_RATE_MS);

        ProtocolSendDownCmd( GND_ROUTE_ADDR, ADCS_ROUTE_ADDR, ADCS_TELEMETRY,
                &hk_frame.append_frame, sizeof(HK_Append_t) );
	}

	hk_store_add();
}


void adcs_pwr_task( void *pvParameters __attribute__((unused)) )
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

/**
 * 延时处理函数
 *
 * @param para
 */
void __attribute__((weak))Delay_Task_Mon(void *para)
{
    delay_task_t *task_para = (delay_task_t *)para;

    while(1)
    {
        vTaskDelay(282);

        /* 若当前时间还没有到指令执行时间 */
        if (clock_get_time_nopara() < task_para->execution_utc)
            continue;

        DelayTask_UnPacket(task_para);

        break;
    }

    vTaskDelete(NULL);
}

/**
 * 延时处理任务创建
 *
 * @param para 延时任务参数
 * @return
 */
int Delay_Task_Mon_Start(delay_task_t *para)
{
    extern obc_save_t obc_save;

    if ( para->execution_utc < clock_get_time_nopara() )
        return E_INVALID_PARAM;

    /*片内FALSH 读、改、写*/
//    if ( bsp_ReadCpuFlash( OBC_STORE_ADDR, (uint8_t*)&obc_save, sizeof(obc_save) ) != 0 )
//        return E_FLASH_ERROR;

    FSMC_NOR_ReadBuffer( (uint16_t *)&obc_save, OBC_STORE_NOR_ADDR, sizeof(obc_save_t)/2 );

    /* 给任务恢复结构体赋值 */
    obc_save.delay_task_recover[0].task_function = Delay_Task_Mon;
    strcpy(obc_save.delay_task_recover[0].task_name, "MON");
    memcpy( obc_save.delay_task_recover[0].task_para, para, 60);

    if (USER_NOR_SectorErase(0) != NOR_SUCCESS)
    {
        printf("ERROR: NorFalsh erase sector 0 fail!\r\n");
        return E_FLASH_ERROR;
    }

    if (FSMC_NOR_WriteBuffer((uint16_t *)&obc_save, OBC_STORE_NOR_ADDR, sizeof(obc_save_t)/2) != NOR_SUCCESS)
    {
        printf("ERROR: NorFalsh write sector 0 fail!\r\n");
        return E_FLASH_ERROR;
    }

    int ret = xTaskCreate( Delay_Task_Mon, "MON", 256,
            obc_save.delay_task_recover[0].task_para, 4, NULL );

    if (ret != pdTRUE)
        return E_NO_BUFFER;

    return E_NO_ERR;
}


void vWatchDogTask1( void *pvParameters __attribute__((unused)))
{
    for(;;)
    {
        vTaskDelay(100);
        bsp_WatchDogToggle();
    }
//  xTaskCreate( vWatchDogTask1, "Watchdog", configMINIMAL_STACK_SIZE,
//          ( void * ) NULL, tskIDLE_PRIORITY + 3, ( TaskHandle_t * ) NULL );
}
