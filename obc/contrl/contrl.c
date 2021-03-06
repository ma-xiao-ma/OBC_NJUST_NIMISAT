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
#include "camera_805.h"
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
#include "bsp_watchdog.h"
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
bool down_cmd_enable = false;  //遥测下行使能标志
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
    uint32_t cam_power_on_counter;
    uint32_t dtb_power_on_counter;
    uint32_t passing;
    uint32_t time_valid;
} control_para;

static control_para control_task;

#define VU_IDLE_ON       (10 * 60) /*通信机空闲状态连续发射开 持续时间    单位：秒*/
#define JLG_VU_SWITCH_ON (10 * 60) /*解理工通信机备份机切换     持续时间    单位：秒*/
#define CAM_WORK_TIME    (6 * 60) /*相机工作时间控制  单位：秒*/
#define DTB_WORK_TIME    (11 * 60) /*数传回放时间控制  单位：秒*/
#define PASSIN_LAST      (10 * 60) /*过境时间  单位：秒*/
#define TIME_SYSN        ( 5 * 60) /*时间同步间隔  单位：秒*/

#define CONTROL_CYCLE    1200      /*控制周期  单位：毫秒*/

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

	    /** 如果通信机空闲状态连续发射已经开启 */
	    if (vu_tm->tx_state.IdleState == RemainOn)
	        control_task.vu_idle_state++;
	    else
	        control_task.vu_idle_state = 0;

	    /** 如果备份通信机信道是开启状态，则计数器累加 */
	    if (((obc_switch_t *)&obc_tm->on_off_status)->switch_vu_on == true)
	        control_task.vu_jlg_switch_on++;
	    else
	        control_task.vu_jlg_switch_on = 0;

        /** 如果相机是开机状态，则计数器累加 */
        if (((obc_switch_t *)&obc_tm->on_off_status)->cam_5w_5v_pwr == true ||
                ((obc_switch_t *)&obc_tm->on_off_status)->cam_10w_5v_pwr == true)
            control_task.cam_power_on_counter++;
        else
            control_task.cam_power_on_counter = 0;

        /**如果数传机是开机状态，则计数器累加*/
        if (((obc_switch_t *)&obc_tm->on_off_status)->dtb_5v_pwr == true ||
                ((obc_switch_t *)&obc_tm->on_off_status)->dtb_12v_pwr == true)
            control_task.dtb_power_on_counter++;
        else
            control_task.dtb_power_on_counter = 0;

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

	    /* 如果通信机空闲状态连续发射连续开启超过10分钟，或者备份通信机启动时，则自动关闭连续发射 */
	    if ( control_task.vu_idle_state > JUDGMENT(VU_IDLE_ON) || IsJLGvuWorking )
	    {
	        /* 若连续发射已经关闭，则不再执行 */
	        if (control_task.vu_idle_state != TurnOff)
	            vu_transmitter_set_idle_state(TurnOff);
	    }

        /* 如果备份通信机开启超过10分钟，则自动切换为主份 */
        if ( control_task.vu_jlg_switch_on > JUDGMENT(JLG_VU_SWITCH_ON) )
        {
            /***此处应将通信机切换回ISIS通信机, 除能备份通信机加电, 并将信道控制板断电*/
            vu_backup_switch_off();
        }

        /* 如果相机开启超过10分钟，则切断相机电源 */
        if ( control_task.cam_power_on_counter > JUDGMENT(CAM_WORK_TIME) )
        {
            cam_ctl_t cam_ctl_mode = {0, 0, 0};
            Camera_Work_Mode_Set(cam_ctl_mode);
            Camera_Power_Off();
        }

        /* 如果数传开启超过10分钟，则切断数传电源 */
        if ( control_task.dtb_power_on_counter > JUDGMENT(DTB_WORK_TIME) )
        {
            xDTBTeleControlSend(MemStop, 100);
            xDTBTeleControlSend(ShutDown, 100);
            dtb_power_off();
        }


        /* 每次过境不会超过10分钟，因此超过10分钟清除过境标志*/
        if ( control_task.passing > JUDGMENT(PASSIN_LAST) )
        {
            PassFlag = false;
            down_cmd_enable = false;
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

#define COLLECT_CYCLE 2000 //采集任务周期 单位：毫秒
/**
 * 将存储时间间隔转换成下行任务中的计数器判断次数
 * @param pvParameters 秒数
 */
#define COLLECT_TASK_JUDGMENT(x)    ( x / (COLLECT_CYCLE / configTICK_RATE_HZ) )

#define HK_SAVE_INTERVAL            60 //存储间隔60秒  须为COLLECT_CYCLE的倍数
#define HK_DOWN_INTERVAL_OUTSIDE    12 //境外遥测下行间隔， 单位：秒
#define ADCS_BOOT_WAITING           2  //姿控系统启动等待时间， 单位：秒
#define DTB_BOOT_WAITING            4  //数传启动等待时间， 单位：秒
#define CAM_BOOT_WAITING            10 //相机启动等待时间， 单位：秒
#define VU_BOOT_WAITING             4  //备份通信机启动等待时间， 单位：秒

void hk_collect_task(void *pvParameters __attribute__((unused)))
{
    uint32_t hk_save_counter = 0, hk_down_counter = 0, adcs_boot_waiting = 0,
            dtb_boot_waiting = 0, cam_boot_waiting = 0, vu_boot_waiting = 0;

    eps_start();
    hk_collect_task_init();

    /*初始化主帧FIFO和辅帧FIFO*/
    HK_fifoInit(&hk_main_fifo);
    HK_fifoInit(&hk_append_fifo);

    /* 创建遥测文件文件 */
    hk_store_init();

    TickType_t xLastWakeTime = xTaskGetTickCount ();
    while (1)
    {
        vTaskDelayUntil( &xLastWakeTime, COLLECT_CYCLE / portTICK_RATE_MS );

        task_report_alive(Collect);

        obc_hk_task();

        eps_hk_task();

        ttc_hk_task();

        /* 若数传上电，则获取遥测值 */
        if( OUT_SW_DTB_5V_PIN() )
        {
            if( ++dtb_boot_waiting >= COLLECT_TASK_JUDGMENT(DTB_BOOT_WAITING) )
                dtb_hk_task();
        }
        else
            dtb_boot_waiting = 0;

        /* 备份通信机工作，等待后开始采集 */
//        if( IsJLGvuWorking )
//        {
//            if( ++vu_boot_waiting >= COLLECT_TASK_JUDGMENT(VU_BOOT_WAITING) )
                jlg_hk_task();
//        }
//        else
//            vu_boot_waiting = 0;

        /* 若相机上电，则获取遥测值 */
        if( OUT_SW_CAMERA_5W_PIN() && OUT_SW_CAMERA_10W_PIN() )
        {
            if( ++cam_boot_waiting >= COLLECT_TASK_JUDGMENT(CAM_BOOT_WAITING) )
                cam_hk_task();
        }
        else
            cam_boot_waiting = 0;

        /* 若姿控上电，则获取遥测值 */
        if( SW_EPS_S0_PIN() )
            if( ++adcs_boot_waiting >= COLLECT_TASK_JUDGMENT(ADCS_BOOT_WAITING) )
                adcs_hk_task();
        else
            adcs_boot_waiting = 0;

        /* 如果开始下行标志置1，则下行遥测 */
        if( down_cmd_enable == true )
            hk_down_proc_task();
        else if( ++hk_down_counter >= COLLECT_TASK_JUDGMENT(HK_DOWN_INTERVAL_OUTSIDE) && !PassFlag) /* 非过境时 */
        {
            hk_down_counter = 0;
            hk_down_proc_task();
        }

        /* 否则保存遥测 */
        if( ++hk_save_counter >= COLLECT_TASK_JUDGMENT(HK_SAVE_INTERVAL) )  //存储间隔60秒
        {
            hk_save_counter = 0;
            hk_data_save_task(); /*240分钟存满一个文件，一个文件240条遥测*/
        }
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
    down_cmd_enable = false;
    IsRealTelemetry = 1;
}


void hk_down_proc_task(void)
{
    /*如果是实时遥测数据下行*/
	if (IsRealTelemetry == 1)
	{
	    /* 遥测收集 */
        hk_collect_no_store();

        /* 解决首帧丢失问题 */
        ProtocolSendDownCmd( GND_ROUTE_ADDR, OBC_ROUTE_ADDR, OBC_TELEMETRY,
                &hk_frame.main_frame, sizeof(HK_Main_t) );

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

            /* 解决首帧丢失问题 */
            ProtocolSendDownCmd( GND_ROUTE_ADDR, OBC_ROUTE_ADDR, OBC_TELEMETRY,
                    &hk_old_frame.main_frame, sizeof(HK_Main_t) );

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
void __attribute__((weak))Delay_Task(void *para)
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

uint8_t delay_task_name[DELAY_TASK_NUM][4] = {"Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun"};

/**
 * 延时处理任务创建
 *
 * @param para 延时任务参数
 * @return
 */
int Delay_Task_Mon_Start(delay_task_t *para)
{
    int i;
    extern obc_save_t obc_save;

    /* 如果上行指令执行时间比当前星上时间小，或者晚于2030年1月1日0时0分0秒，则认为上行参数有误 */
    if ( para->execution_utc < clock_get_time_nopara() || para->execution_utc > 1893427200 )
        return E_INVALID_PARAM;

    /*片外NOR_FALSH SECTOR_0 读、改、写*/
    FSMC_NOR_ReadBuffer( (uint16_t *)&obc_save, OBC_STORE_NOR_ADDR, sizeof(obc_save_t) / 2 );

    for( i = 0; i < DELAY_TASK_NUM; i++ )
    {
        /* 寻找空闲的或者失效的存储体 */
        if( ((delay_task_t *)obc_save.delay_task_recover[i].task_para)->execution_utc == 0xFFFFFFFF
                || ((delay_task_t *)obc_save.delay_task_recover[i].task_para)->execution_utc <
                clock_get_time_nopara() )
            break;
    }

    /* 若没有查找到空闲存储结构  */
    if( i == DELAY_TASK_NUM )
        return E_NO_BUFFER;

    /* 给任务恢复结构体赋值 */
    obc_save.delay_task_recover[i].task_function = Delay_Task;
    strcpy(obc_save.delay_task_recover[i].task_name, &delay_task_name[i][0]);
    memcpy( obc_save.delay_task_recover[i].task_para, para, 60);


    /* 擦除NOR_FLASH系统变量存储区 */
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

    int ret = xTaskCreate( obc_save.delay_task_recover[i].task_function, obc_save.delay_task_recover[i].task_name,
            256, obc_save.delay_task_recover[i].task_para, 1, NULL );

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
