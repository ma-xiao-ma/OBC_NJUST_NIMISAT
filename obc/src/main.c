#include <if_downlink_serial.h>
#include "stm32f4xx.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#include "bsp_usart.h"
#include "console.h"
#include "command.h"

#include "bsp_adc.h"
#include "bsp_switch.h"
#include "bsp_watchdog.h"
#include "bsp_reset.h"
#include "if_downlink_vu.h"

#include "task_user.h"

#include "cube_com.h"
#include "contrl.h"
#include "hk.h"

/***************开始任务********************/
//任务优先级
#define START_TASK_PRIO     1
//任务堆栈大小
#define START_STK_SIZE      128
//任务句柄
TaskHandle_t StartTask_Handler;
//任务函数
void start_task(void *pvParameters);
/***************调试台任务********************/
//任务优先级
#define CONSOLE_TASK_PRIO     (tskIDLE_PRIORITY + 4)
//任务堆栈大小
#define CONSOLE_STK_SIZE      (configMINIMAL_STACK_SIZE * 4)
//任务句柄
TaskHandle_t ConsoleTask_Handler;

/****************电源信息采集控制任务********************/
//任务优先级
#define EPS_TASK_PRIO     (tskIDLE_PRIORITY + 1)
//任务堆栈大小
#define EPS_STK_SIZE      (configMINIMAL_STACK_SIZE * 2)
//任务句柄
TaskHandle_t EpsTask_Handler;

/*****************调试上行串口解包任务*******************/
//任务优先级
#define USART2_REVEIVE_TASK_PRIO     (tskIDLE_PRIORITY + 3)
//任务堆栈大小
#define USART2_REVEIVE_STK_SIZE      (configMINIMAL_STACK_SIZE)
//任务句柄
TaskHandle_t Usart2ReceiveTask_Handler;

/****************上行ISIS通信板解包任务******************/
//任务优先级
#define ISIS_READ_TASK_PRIO     (tskIDLE_PRIORITY + 3)
//任务堆栈大小
#define ISIS_READ_STK_SIZE      (configMINIMAL_STACK_SIZE * 2)
//任务句柄
TaskHandle_t ISISReadTask_Handler;

/***************遥测下行和保存任务*******************/
//任务优先级
#define DOWN_SAVE_TASK_PRIO     (tskIDLE_PRIORITY + 2)
//任务堆栈大小
#define DOWN_SAVE_STK_SIZE      (configMINIMAL_STACK_SIZE*2)
//任务句柄
TaskHandle_t DownSaveTask_Handler;

/***************展电池阵任务********************/
//任务优先级
#define OPEN_PANEL_TASK_PRIO     (tskIDLE_PRIORITY + 3)
//任务堆栈大小
#define OPEN_PANEL_STK_SIZE      (configMINIMAL_STACK_SIZE)
//任务句柄
TaskHandle_t OpenPanelTask_Handler;

/***************展天线任务************************/
//任务优先级
#define OPEN_ANTENNA_TASK_PRIO     (tskIDLE_PRIORITY + 4)
//任务堆栈大小
#define OPEN_ANTENNA_STK_SIZE      (configMINIMAL_STACK_SIZE)
//任务句柄
TaskHandle_t OpenAntennaTask_Handler;




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

    /* 创建开始任务 */
    xTaskCreate((TaskFunction_t )start_task,            //任务函数
                (const char*    )"start_task",          //任务名
                (uint16_t       )START_STK_SIZE,        //任务堆栈大小
                (void*          )NULL,                  //传递给任务的参数
                (UBaseType_t    )START_TASK_PRIO,       //任务优先级
                (TaskHandle_t*  )&StartTask_Handler);   //任务句柄

    vTaskStartScheduler();  //开启任务调度器

    while(1){
        reset();
    }

    return 0;
}

//开始任务任务函数
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();           //进入临界区

    //创建调试台任务
    xTaskCreate((TaskFunction_t )debug_console,
                (const char*    )"Console",
                (uint16_t       )CONSOLE_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )CONSOLE_TASK_PRIO,
                (TaskHandle_t*  )&ConsoleTask_Handler);

    //创建电源信息采集控制任务
    xTaskCreate((TaskFunction_t )eps_task,
                (const char*    )"Eps",
                (uint16_t       )EPS_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )EPS_TASK_PRIO,
                (TaskHandle_t*  )&EpsTask_Handler);

#if USE_SERIAL_PORT_DOWNLINK_INTERFACE
    #ifndef USE_ROUTE_PROTOCOL
    //创建调试上行串口解包任务
    xTaskCreate((TaskFunction_t )USART2_Receive_Task,
                (const char*    )"USART2_RX",
                (uint16_t       )USART2_REVEIVE_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )USART2_REVEIVE_TASK_PRIO,
                (TaskHandle_t*  )&Usart2ReceiveTask_Handler);
    #endif
#else
    //创建上行ISIS通信板解包任务
    xTaskCreate((TaskFunction_t )isis_read_task,
                (const char*    )"ISIS",
                (uint16_t       )ISIS_READ_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )ISIS_READ_TASK_PRIO,
                (TaskHandle_t*  )&ISISReadTask_Handler);

#endif

//    //创建上行ISIS通信板解包任务
//    xTaskCreate((TaskFunction_t )vu_jlg_uplink_task,
//                (const char*    )"JLG",
//                (uint16_t       )ISIS_READ_STK_SIZE,
//                (void*          )NULL,
//                (UBaseType_t    )ISIS_READ_TASK_PRIO,
//                (TaskHandle_t*  )&ISISReadTask_Handler);

//    //遥测下行和保存任务
//    xTaskCreate((TaskFunction_t )down_save_task,
//                (const char*    )"down",
//                (uint16_t       )DOWN_SAVE_STK_SIZE,
//                (void*          )NULL,
//                (UBaseType_t    )DOWN_SAVE_TASK_PRIO,
//                (TaskHandle_t*  )&DownSaveTask_Handler);

//    //展电池阵任务
//    xTaskCreate((TaskFunction_t )OpenPanel_Task,
//                (const char*    )"Batt",
//                (uint16_t       )OPEN_PANEL_STK_SIZE,
//                (void*          )NULL,
//                (UBaseType_t    )OPEN_PANEL_TASK_PRIO,
//                (TaskHandle_t*  )&OpenPanelTask_Handler);
//
//    //展天线任务
//    xTaskCreate((TaskFunction_t )OpenAntenna_Task,
//                (const char*    )"ants",
//                (uint16_t       )OPEN_ANTENNA_STK_SIZE,
//                (void*          )NULL,
//                (UBaseType_t    )OPEN_ANTENNA_TASK_PRIO,
//                (TaskHandle_t*  )&OpenAntennaTask_Handler);

    vTaskDelete(StartTask_Handler); //删除开始任务

    taskEXIT_CRITICAL();            //退出临界区
}

