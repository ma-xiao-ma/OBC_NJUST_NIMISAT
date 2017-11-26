/*
 * task_monitor.c
 *
 *  Created on: 2017年11月7日
 *      Author: Ma Wenli
 */

#include <stdio.h>

#include "FreeRTOS.h"
#include "event_groups.h"
#include "semphr.h"

#include "bsp_reset.h"
#include "bsp_iwdg.h"
#include "inttypes.h"
#include "obc_mem.h"

#include "task_monitor.h"

#define BYTETOBINARYPATTERN "%d%d%d%d%d%d%d%d"

#define BYTETOBINARY(byte)  \
  (byte & 0x80 ? 1 : 0), \
  (byte & 0x40 ? 1 : 0), \
  (byte & 0x20 ? 1 : 0), \
  (byte & 0x10 ? 1 : 0), \
  (byte & 0x08 ? 1 : 0), \
  (byte & 0x04 ? 1 : 0), \
  (byte & 0x02 ? 1 : 0), \
  (byte & 0x01 ? 1 : 0)

#define COLLECT_TASK_BIT    ( 1 << 0 )
#define DOWN_SAVE_TASK_BIT  ( 1 << 1 )
#define ROUTE_TASK_BIT      ( 1 << 2 )
#define SERVER_TASK_BIT     ( 1 << 3 )
#define SEND_TASK_BIT       ( 1 << 4 )
#define MONITOR_ALL_BIT     ( COLLECT_TASK_BIT | DOWN_SAVE_TASK_BIT | ROUTE_TASK_BIT | SERVER_TASK_BIT | SEND_TASK_BIT )


EventGroupHandle_t task_status;
TickType_t  monitor_window = 30000; // 超时时间默认30秒

/**
 *任务监视器初始化
 *
 * @param timeout 任务报告超时时间
 */
void supervisor_init(uint32_t timeout)
{
    /* 创建时间标志组 */
    task_status = xEventGroupCreate();

    /* 任务监视器超时时间 */
    monitor_window = timeout;
}

/**
 *线程监视任务，任务优先级需设为最高
 *
 * @param para
 */
void supervisor_task(void *para)
{
    EventBits_t EventValue;

    if ( task_status == NULL )
        supervisor_init( monitor_window );

    /* 硬件看门狗超时间20s */
    IWDG_Init(20000);

    while(1)
    {
        EventValue = xEventGroupWaitBits( task_status, (EventBits_t)MONITOR_ALL_BIT, pdTRUE, pdTRUE, monitor_window );

        IWDG_Feed();

//        printf("EventValue :"BYTETOBINARYPATTERN"\r\n", BYTETOBINARY((uint8_t)EventValue));

        if ( (EventValue & MONITOR_ALL_BIT) != MONITOR_ALL_BIT )
        {
            /**
             * 随后可以添加线程爆炸处理，暂时用计算机复位代替
             */
            if ( (EventValue & (EventBits_t)COLLECT_TASK_BIT) == 0)
                printf("Collect Task Already Exploded!!!\n");

            if ( (EventValue & (EventBits_t)DOWN_SAVE_TASK_BIT) == 0)
                printf("Down_Save Task Already Exploded!!!\n");

            if ( (EventValue & (EventBits_t)ROUTE_TASK_BIT) == 0)
                printf("Router Task Already Exploded!!!\n");

            if ( (EventValue & (EventBits_t)SERVER_TASK_BIT) == 0)
                printf("Server Task Already Exploded!!!\n");

            if ( (EventValue & (EventBits_t)SEND_TASK_BIT) == 0)
                printf("Send Task Already Exploded!!!\n");

            /* 计算机复位 */
            cpu_reset();
        }
    }
}

/**
 * 任务需要上报任务状态给监视器
 *
 * @param monitor_task_bit
 */
void task_report_alive(monitor_bit monitor_task_bit)
{
    if ( task_status == NULL )
        return;

    xEventGroupSetBits( task_status,  (EventBits_t)monitor_task_bit);
}


typedef struct sv_element_s {
    char * name;                /**< Task name */
    unsigned int id;            /**< Task id */
    uint32_t timer;             /**< Task timer value */
    uint32_t timeout;           /**< Task timeout value */
    struct sv_element_s * next; /**< Pointer to next task in list */
} sv_task_t;

static uint32_t interval = 1000;
static int sv_next_id = 0;
static xSemaphoreHandle sv_sem;

static sv_task_t * sv_head = NULL;
static sv_task_t * sv_tail = NULL;

/* Calculate greatest common divisor of a and b
 * using the Euclidean algorithm */
uint32_t gcd(uint32_t a, uint32_t b) {
    uint32_t t;
    while (b) {
        t = b;
        b = a % b;
        a = t;
    }
    return a;
}

/* Return current time */
uint32_t time_now(void) {
    return (uint32_t)(xTaskGetTickCount() * (1000/configTICK_RATE_HZ));
}

void sv_task(void * param) {

    uint32_t now, last = time_now();
    while (1) {

        /* MCU硬件看门狗喂狗 */
        IWDG_Feed();;

        /* Check timeouts */
        if (xSemaphoreTake(sv_sem, 1 * configTICK_RATE_HZ) == pdPASS) {
            sv_task_t * t = sv_head;
            while (t) {
                now = time_now();
                t->timer += now - last;
                if (t->timer >= t->timeout) {
                    printf("Supervisor timeout for %s - Rebooting system!\r\n", t->name);
                    cpu_reset();
                }
                t = t->next;
            }
            xSemaphoreGive(sv_sem);
        }

        /* Sleep quarter of interval */
        last = time_now();
        vTaskDelay((interval * configTICK_RATE_HZ / 4) / 1000);
    }
}

int sv_init(uint32_t initial_timeout) {

    /* Init list semaphore */
    vSemaphoreCreateBinary(sv_sem);
    if (sv_sem == NULL) {
        printf("Failed to create supervisor semaphore\r\n");
        return -1;
    }

    /* Set maximum timeout */
    interval = initial_timeout;

    /* Start task */
    if (xTaskCreate(&sv_task, (signed char *)"SV", 1024, NULL, configMAX_PRIORITIES - 1, NULL) != pdPASS) {
        printf("Failed to create supervisor task\r\n");
        return -1;
    }

    return 0;

}

int sv_add(char * name, uint32_t timeout) {
    sv_task_t * t = ObcMemMalloc(sizeof(sv_task_t));
    if (t == NULL) {
        printf("Failed to allocate memory for supervisor entry\r\n");
        return -1;
    }

    if (xSemaphoreTake(sv_sem, 1 * configTICK_RATE_HZ) != pdPASS) {
        ObcMemFree(t);
        return -1;
    }

    /* Set task properties */
    t->name = name;
    t->id = sv_next_id++;
    t->timer = 0;
    t->timeout = timeout;
    t->next = NULL;

    /* Adjust interval */
    interval = gcd(interval, timeout);

    if (interval < 100) {
        printf("GCD of intervals too small, changed to 100 ms\r\n");
        interval = 100;
    }

    /* 设置硬件看门狗超时间 */
    IWDG_Init(interval);

    /* Insert task */
    if (sv_head == NULL && sv_tail == NULL) {
        sv_head = t;
        sv_tail = t;
    } else {
        sv_tail->next = t;
        sv_tail = t;
    }

    xSemaphoreGive(sv_sem);

    return t->id;
}

int sv_reset(unsigned int id) {

    if (xSemaphoreTake(sv_sem, 1 * configTICK_RATE_HZ) != pdPASS)
        return -1;

    /* Loop task list */
    sv_task_t * i = sv_head;
    while (i) {
        if (i->id == id) {
            i->timer = 0;
            break;
        }
        i = i->next;
    }

    xSemaphoreGive(sv_sem);

    return 0;
}

int sv_print(struct command_context *ctx) {

    /* Loop task list */
    sv_task_t * i = sv_head;
    if (i) {
        printf("ID\tTask\tTimer\tTimeout\r\n");
        while (i) {
            printf("%u\t%s\t%"PRIu32"\t%"PRIu32"\r\n",
                    i->id, i->name, i->timer, i->timeout);
            i = i->next;
        }
    } else {
        printf("No tasks being supervised\r\n");
    }

    return CMD_ERROR_NONE;

}
