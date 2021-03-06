/*
 * route.c
 *
 *  Created on: 2017年9月23日
 *      Author: Ma Wenli
 */
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "obc_mem.h"
#include "driver_debug.h"
#include "cube_com.h"
#include "router_io.h"
#include "stm32f4xx.h"
#include "task_monitor.h"
#include "error.h"

#include "route.h"

/**
 * 路由核心队列句柄
 */
static xQueueHandle route_queue; // 路由队列
static xQueueHandle server_queue; // 接收处理任务队列
static xQueueHandle send_processing_queue; // 发送处理任务队列

/** 路由软件定义地址 */
static uint8_t router_my_address;

/**
 * 路由地址设置函数
 *
 * @param addr 路由地址设置值
 */
void router_set_address(uint8_t addr)
{
    router_my_address = addr;
}

/**
 * 获取路由地址设置值
 *
 * @return 路由地址
 */
uint8_t router_get_my_address(void)
{
    return router_my_address;
}

/**
 * 路由队列初始化 ，创建路由队列
 *
 * @param queue_len_router 路由队列深度
 * @return 返回E_NO_ERR（-1）为创建成功
 */
static int route_queue_init(uint32_t queue_len_router)
{
    if(route_queue == NULL)
    {
        route_queue = xQueueCreate(queue_len_router, sizeof(route_packet_t *));
        if(!route_queue)
            return E_OUT_OF_MEM;
    }
    return E_NO_ERR;
}

/**
 * 接收处理任务队列初始化 ，创建接收处理任务队列
 *
 * @param queue_len_server 队列深度
 * @return 返回E_NO_ERR（-1）为创建成功
 */
static int server_queue_init(uint32_t queue_len_server)
{
    if(server_queue == NULL)
    {
        server_queue = xQueueCreate(queue_len_server, sizeof(route_packet_t *));
        if(!server_queue)
            return E_OUT_OF_MEM;
    }
    return E_NO_ERR;
}

/**
 * 发送处理任务队列初始化 ，创建发送处理任务队列
 *
 * @param queue_len_server 队列深度
 * @return 返回E_NO_ERR（-1）为创建成功
 */
static int send_processing_queue_init(uint32_t queue_len_server)
{
    if(send_processing_queue == NULL)
    {
        send_processing_queue = xQueueCreate(queue_len_server, sizeof(route_packet_t *));
        if(!send_processing_queue)
            return E_OUT_OF_MEM;
    }
    return E_NO_ERR;
}

/**
 * 服务队列读出函数
 *
 * @param packet 读出的路由数据包
 * @return 返回E_NO_ERR（-1）表示执行正确
 */
static int server_queue_read(route_packet_t ** packet)
{
    if (server_queue == NULL)
        return E_NO_DEVICE;

    if (xQueueReceive(server_queue, packet, SERVER_QUEUE_READ_TIMEOUT) == pdFALSE)
        return E_TIMEOUT;

    return E_NO_ERR;
}

/**
 * 服务队列写入函数，接口链路层和网络层的媒介
 *
 * @param packet 路由数据包指针
 * @param pxTaskWoken 在任务中调用时次值应置为NULL空指针，在中断中应指向有效地址
 */
static void server_queue_wirte(route_packet_t *packet, portBASE_TYPE *pxTaskWoken)
{
    int result;

    if(packet == NULL)
    {
        printf("server_queue_wirte called with NULL packet\r\n");
        return;
    }

    if (server_queue == NULL)
    {
        printf("server queue not initialized!\r\n");
        ObcMemFree(packet);
        return;
    }

    if(pxTaskWoken == NULL)
        result = xQueueSendToBack(server_queue, &packet, 0);
    else
        result = xQueueSendToBackFromISR(server_queue, &packet, pxTaskWoken);

    if(result != pdTRUE)
    {
        printf("ERROR: Server queue is FULL. Dropping packet.\r\n");
        ObcMemFree(packet);
        printf("Cleaning up server queue...\r\n");
        route_queue_clean(server_queue, NULL);
    }
}

/**
 * 接收处理任务
 *
 * @param param
 */
static void server_task(void *param __attribute__((unused)))
{
    static route_packet_t *packet;

    while (1)
    {
        task_report_alive(Server);

        packet = NULL;

        if (server_queue_read(&packet) != E_NO_ERR)
        {
            driver_debug(DEBUG_ROUTER, "SERVER Task Running!\r\n");
            continue;
        }

        if (packet == NULL)
        {
            printf("Server task detected with NULL packet.\r\n");
            continue;
        }

        if (packet->dst != router_get_my_address())
        {
            printf("Server task packet error. Dropping packet.\r\n");
            ObcMemFree(packet);
            continue;
        }

        router_unpacket(packet);
    }
}

/**
 * 接收处理任务创建
 *
 * @param task_stack_size 任务堆栈大小
 * @param priority 任务优先级
 * @return 任务创建成功返回E_NO_ERR（-1）
 */
int server_start_task(uint16_t task_stack_size, uint32_t priority)
{
    int ret = xTaskCreate(server_task, "SERVER", task_stack_size, NULL, priority, NULL);

    if (ret != pdPASS)
    {
        printf("Failed to start server task!\r\n");
        return E_OUT_OF_MEM;
    }

    return E_NO_ERR;
}

/**
 * 发送处理队列读出函数
 *
 * @param packet 读出的路由数据包
 * @return 返回E_NO_ERR（-1）表示执行正确
 */
static int send_processing_queue_read(route_packet_t ** packet)
{
    if (send_processing_queue == NULL)
        return E_NO_DEVICE;

    if (xQueueReceive(send_processing_queue, packet, SEND_QUEUE_READ_TIMEOUT) == pdFALSE)
        return E_TIMEOUT;

    return E_NO_ERR;
}

/**
 * 发送处理队列写入函数，由发送接口函数调用
 *
 * @param packet 路由数据包指针
 * @param pxTaskWoken 在任务中调用时次值应置为NULL空指针，在中断中应指向有效地址
 */
static void send_processing_queue_wirte(route_packet_t *packet, portBASE_TYPE *pxTaskWoken)
{
    int result;

    if (packet == NULL)
    {
        printf("send_processing_queue_wirte called with NULL packet\r\n");
        return;
    }

    if (send_processing_queue == NULL)
    {
        printf("send processing queue not initialized!\r\n");
        ObcMemFree(packet);
        return;
    }

    if (pxTaskWoken == NULL)
        result = xQueueSendToBack(send_processing_queue, &packet, 0);
    else
        result = xQueueSendToBackFromISR(send_processing_queue, &packet, pxTaskWoken);

    if (result != pdTRUE)
    {
        printf("ERROR: Send processing queue is FULL. Dropping packet.\r\n");
        ObcMemFree(packet);
        printf("Cleaning up send queue...\r\n");
        route_queue_clean(send_processing_queue, NULL);
    }
}


static void send_processing_task(void *param __attribute__((unused)))
{
    static route_packet_t *packet;

    while (1)
    {
        task_report_alive(Send);

        packet = NULL;

        if (send_processing_queue_read(&packet) != E_NO_ERR)
        {
            driver_debug(DEBUG_ROUTER, "SEND Task Running!\r\n");
            continue;
        }

        if (packet == NULL)
        {
            printf("Send processing  task detected with NULL packet.\r\n");
            continue;
        }

        if (packet->dst == router_get_my_address())
        {
            printf("Send processing task packet error. Dropping packet.\r\n");
            ObcMemFree(packet);
            continue;
        }

        driver_debug(DEBUG_ROUTER, "OTP: S %u, D %u, Tp 0x%02X, Sz %u\r\n",
                packet->src, packet->dst, packet->typ, packet->len);

        router_send_to_other_node(packet);
    }
}

/**
 * 路由器发送处理任务创建
 *
 * @param task_stack_size 任务堆栈大小
 * @param priority 任务优先级
 * @return 返回E_NO_ERR（-1）表示创建成功
 */
int send_processing_start_task(uint16_t task_stack_size, uint32_t priority)
{
    int ret = xTaskCreate(send_processing_task, "SEND", task_stack_size, NULL, priority, NULL);

    if (ret != pdPASS)
    {
        printf ("Failed to start send processing task!\r\n");
        return E_OUT_OF_MEM;
    }

    return E_NO_ERR;
}

/**
 * 路由队列读出函数
 *
 * @param packet 读出的路由数据包
 * @return 返回E_NO_ERR（-1）表示执行正确
 */
static int route_queue_read(route_packet_t ** packet)
{
    if (route_queue == NULL)
        return E_NO_DEVICE;

    if (xQueueReceive(route_queue, packet, ROUTE_QUEUE_READ_TIMEOUT) != pdTRUE)
        return E_TIMEOUT;

    return E_NO_ERR;
}

/**
 * 路由队列写入函数，接口链路层和网络层的媒介
 *
 * @param packet 路由数据包指针
 * @param pxTaskWoken 在任务中调用时次值应置为NULL空指针，在中断中应指向有效地址
 */
void route_queue_wirte(route_packet_t *packet, portBASE_TYPE *pxTaskWoken)
{
    int result;

    if(packet == NULL)
    {
        printf("route_queue_wirte called with NULL packet\r\n");
        return;
    }

    if (route_queue == NULL)
    {
        printf("route queue not initialized!\r\n");
        ObcMemFree(packet);
        return;
    }

    if(pxTaskWoken == NULL)
        result = xQueueSendToBack(route_queue, &packet, 0);
    else
        result = xQueueSendToBackFromISR(route_queue, &packet, pxTaskWoken);

    if(result != pdTRUE)
    {
        printf("ERROR: Routing queue is FULL. Dropping packet.\r\n");
        ObcMemFree(packet);
        printf("Cleaning up route queue...\r\n");
        route_queue_clean(route_queue, NULL);
    }
}

/**
 * 路由任务，星务计算机的所有接口接收到的数据都应送入路由队列中
 *
 * @param param
 */
static void router_task(void *param __attribute__((unused)))
{
    static route_packet_t *packet;
    uint16_t len    = 0;

    while (1)
    {
        /* 软件看门狗喂狗 */
        task_report_alive(Router);

        packet = NULL;

        if (route_queue_read(&packet) != E_NO_ERR)
        {
            driver_debug(DEBUG_ROUTER, "ROUTER Task Running!\r\n");
            continue;
        }

        if (packet == NULL)
        {
            printf("router task detected with NULL packet\r\n");
            continue;
        }

        driver_debug(DEBUG_ROUTER, "INP: S %u, D %u, Tp 0x%02X, Sz %u\r\n",
                packet->src, packet->dst, packet->typ, packet->len);

        /**
         * 如果数据包是发给其他节点的，则发送包到发送处理任务队列
         *
         */
        if (packet->dst != router_get_my_address())
        {
            send_processing_queue_wirte(packet, NULL);
        }
        else if( packet->src == ADCS_ROUTE_ADDR &&
                packet->typ == INS_OBC_GET_ADCS_HK)
        {
//            adcs_queue_wirte( packet, NULL );
            extern QueueHandle_t adcs_hk_queue;
            xQueueOverwrite(adcs_hk_queue, packet->dat);
            ObcMemFree(packet);
        }
        else
        {
            /**
             * 如果数据包是给本节点的,则发送到接收处理任务队列
             *
             */
            server_queue_wirte(packet, NULL);
        }
    }
}

/**
 * 路由任务创建
 *
 * @param task_stack_size 任务堆栈大小
 * @param priority 任务优先级
 * @return 任务创建成功返回E_NO_ERR（-1）
 */
int router_start_task(uint16_t task_stack_size, uint32_t priority)
{
    int ret = xTaskCreate(router_task, "RTE", task_stack_size, NULL, priority, NULL);

    if (ret != pdPASS)
    {
        printf ("Failed to start router task!\r\n");
        return E_OUT_OF_MEM;
    }

    return E_NO_ERR;
}

/**
 * 路由器初始化函数，设置路由地址，创建相关队列
 *
 * @param address 路由地址设置值
 * @param router_queue_len 路由队列深度
 * @return
 */
int router_init(uint8_t address, uint32_t router_queue_len)
{
    int ret;

    router_set_address(address);

    ret = route_queue_init(router_queue_len);
    if(ret != E_NO_ERR)
        return ret;

    ret = server_queue_init(SERVER_QUEUE_LEN);
    if(ret != E_NO_ERR)
        return ret;

    ret = send_processing_queue_init(SEND_PROCESSING_QUEUE_LEN);
    if(ret != E_NO_ERR)
        return ret;


    ret = route_user_init();
    if(ret != E_NO_ERR)
        return ret;

    return E_NO_ERR;
}

/**
 * 路由器队列清理函数，清除已满的路由器队列元素，释放内存
 *
 * @param queue 队列句柄
 * @param pxTaskWoken 在任务中调用此项为0，在中断中调用时必须为一个有效指针
 */
void route_queue_clean(QueueHandle_t queue, portBASE_TYPE *pxTaskWoken)
{
    route_packet_t *packet;

    UBaseType_t QueueItemNum = uxQueueMessagesWaiting(queue);

    do {

        if (pxTaskWoken == NULL)
            xQueueReceive(queue, &packet, 0);
        else
            xQueueReceiveFromISR(queue, &packet, pxTaskWoken);

        if (packet != NULL)
            ObcMemFree(packet);

    }while (--QueueItemNum);
}
