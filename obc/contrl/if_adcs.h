/*
 * if_adcs.h
 *
 *  Created on: 2017年9月29日
 *      Author: Ma Wenli
 */

#ifndef CONTRL_IF_ADCS_H_
#define CONTRL_IF_ADCS_H_

#include "FreeRTOS.h"
#include "route.h"

#define OBC_TO_ADCS_HANDLE      0x01
#define ADCS_DELAY              1000U

#define ADCS_I2C_ADDR           ADCS_I2C1_ADDR

#define ADCS_I2C0_ADDR          0x06
#define ADCS_I2C1_ADDR          0x05

/**
 * 创建姿控分系统消息接收队列
 *
 * @return 返回E_NO_ERR（-1）表示创建正确
 */
int adcs_queue_init(void);

/**
 * 姿控系统消息队列写入函数，server进程调用
 *
 * @param packet 路由数据包指针
 * @param pxTaskWoken 在任务中调用时次值应置为NULL空指针，在中断中应指向有效地址
 */
void adcs_queue_wirte(route_packet_t *packet, portBASE_TYPE *pxTaskWoken);

/**
 * 直接调用I2C接口函数进行发送，响应收到I2C RX queue
 *
 * @param type 消息类型（以前的命令字）
 * @param txbuf 发送内容指针
 * @param txlen 发送内容长度（字节）
 * @param rxbuf 接收缓冲区指针
 * @param rxlen 接收长度
 * @param timeout 读接收队列超时时间
 * @return E_NO_ERR为正常
 */
int adcs_transaction(uint8_t type, void * txbuf, size_t txlen, void * rxbuf, size_t rxlen, uint16_t timeout);

/**
 * 获取姿控分系统遥测值
 *
 * @param hk 接收指针
 * @param timeout 超时值设置
 * @return E_NO_ERR（-1）为正常
 */
int adcs_get_hk(void *hk, uint16_t timeout);

/**
 * 发送电源信息到姿控计算机
 *
 * @param eps_mode 正常模式或者休眠模式
 * @return E_NO_ERR为正常
 */
int adcs_send_mode(uint8_t eps_mode);

/**
 * 姿控时间同步
 *
 * @param secs utc时间
 * @return E_NO_ERR为正常
 */
int adcstimesync(uint32_t secs);

#endif /* CONTRL_IF_ADCS_H_ */
