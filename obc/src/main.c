
#include "stm32f4xx.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#include "bsp_usart.h"
#include "console.h"
#include "bsp_watchdog.h"
#include "task_user.h"
#include "if_downlink_vu.h"
#include "contrl.h"
#include "task_monitor.h"



void vWatchDogTask1( void *pvParameters __attribute__((unused)))
{
	for(;;)
	{
		vTaskDelay(100);
		bsp_WatchDogToggle();
	}
//	xTaskCreate( vWatchDogTask1, "Watchdog", configMINIMAL_STACK_SIZE,
//			( void * ) NULL, tskIDLE_PRIORITY + 3, ( TaskHandle_t * ) NULL );
}


int main(void)
{

    task_initz();

    //创建监视任务
    xTaskCreate( supervisor_task, "SV", 128, NULL, configMAX_PRIORITIES - 1, NULL );

    //创建调试台任务
    xTaskCreate( debug_console, "CONSOLE", 1024, NULL, 4, NULL );

    //创建控制任务
    xTaskCreate( ControlTask, "CONTROL", 256, NULL, 3, NULL );

    //创建分系统遥测采集任务
    xTaskCreate( hk_collect_task, "COLLECT", 512, NULL, 1, NULL );

#if USE_SERIAL_PORT_DOWNLINK_INTERFACE
#else
    //创建上行ISIS通信板解包任务
    xTaskCreate( vu_isis_uplink_task, "UPLINK", 512, NULL, 3, NULL );
#endif

//    //创建上行解理工通信板解包任务
//    xTaskCreate( vu_jlg_uplink_task, "JLG", 512, NULL, 3, NULL );

    //遥测下行和保存任务
    xTaskCreate( down_save_task, "DOWN", 512, NULL, 2, NULL);

//    //展电池阵任务
//    xTaskCreate( OpenPanel_Task, "BATT", 256, NULL, 3, NULL);
//
//    //展天线任务
//    xTaskCreate( OpenAntenna_Task, "ANTS", 256, NULL, 4, NULL);


    vTaskStartScheduler();  //开启任务调度器

    while(1){
        reset();
    }

    return 0;
}


