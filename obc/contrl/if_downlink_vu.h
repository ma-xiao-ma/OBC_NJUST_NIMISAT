/*
 * if_downlink_vu.h
 *
 *  Created on: 2017年10月5日
 *      Author: Ma Wenli
 */
#ifndef CONTRL_IF_DOWNLINK_VU_H_
#define CONTRL_IF_DOWNLINK_VU_H_

#include "route.h"
#include "ff.h"

#define CONFIG_USE_ISIS_VU  0
#define CONFIG_USE_JLG_VU   1

#define DOWNLINK_CRC_SIZE   0
#define DOWNLINK_OVERHEAD (ROUTE_HEAD_SIZE + DOWNLINK_CRC_SIZE)
#define DOWNLINK_MTU (ISIS_MTU - DOWNLINK_OVERHEAD)

#define IMAGE_DOWNLINK_OVERHEAD (DOWNLINK_OVERHEAD + IMAGE_PACK_HEAD_SIZE)

#define MAX_UPLINK_CONTENT_SIZE  104
/*obc给trxvu数据包的时间间隔*/
#define PACK_DOWN_INTERVAL (0 / portTICK_PERIOD_MS)

/*当发射机缓冲区满时等待的时间*/
#define MS_WAIT_TRANS_FREE_BUFF (5000 / portTICK_PERIOD_MS)

extern uint16_t vu_isis_rx_count; //ISISvu 通信机接收上行遥控帧计数
extern uint16_t vu_jlg_rx_count;  //JLGvu 通信机接收上行遥控帧计数

typedef struct
{
    uint8_t tpye;
    void * pdata;
    uint32_t data_len;
    uint16_t start_pack;
    FIL * file;
    char * file_name;
} downlink_request;

/**
 * 通用路由协议下行数据接口
 *
 * @param dst 目的地址
 * @param src 源地址
 * @param type 消息类型
 * @param pdata 待发送数据指针
 * @param len 待发送数据长度
 */
int ProtocolSendDownCmd( uint8_t dst, uint8_t src, uint8_t type, void *pdata, uint32_t len );

/**
 * obc地面指令响应函数
 *
 * @param type 响应指令类型
 * @param result 指令执行结果
 */
void obc_cmd_ack(uint8_t type, uint8_t result);

/**
 * ISIS通信机下行接口函数，由OBC本地调用
 *
 * @param type 下行消息类型
 * @param pdata 下行数据指针
 * @param len 下行数据字节数
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_obc_downlink(uint8_t type, void *pdata, uint32_t len);

/**
 * 通过vu无线接口发送数据
 *
 * @param dst 目的地址
 * @param src 源地址
 * @param type 消息类型
 * @param pdata 待发送数据指针
 * @param len 待发送数据长度
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_isis_send( uint8_t dst, uint8_t src, uint8_t type, void *pdata, uint32_t len );

/**
 *解理工通信机下行加协议接口
 *
 * @param dst 目的地址
 * @param src 源地址
 * @param type 消息类型
 * @param pdata 待发送数据指针
 * @param len 待发送数据长度
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_jlg_send( uint8_t dst, uint8_t src, uint8_t type, void *pdata, uint32_t len );

/**
 * 路由器中下行接口调用，接受一个路由包
 *
 * @param packet 送到路由器的待下行的数据包
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_router_downlink(route_packet_t *packet);

/**
 * 整个文件下行接口函数
 *
 * @param file 文件句柄指针
 * @param file_name 文件名指针
 * @return E_NO_ERR 正常，任务创建成功
 */
int file_whole_download(FIL *file, char *file_name);

/**
 * 恩来星文件下行接口
 *
 * @param file_id 图谱按文件编号
 * @return E_NO_ERR 正常，任务创建成功
 */
int enlai_file_down(uint32_t file_id);

/**
 * 下行整幅图片接口函数
 *
 * @param pdata 图像数据指针
 * @param len 图像数据字节数
 * @param start_pack 图像起始包号
 * @return E_NO_ERR 正常
 */
int image_whole_download(void *pdata, uint32_t len, uint16_t start_pack);

/**
 * 解理工通信机上行接受任务
 *
 * @param para 没用
 */
void vu_jlg_uplink_task(void *para __attribute__((unused)));

/**
 * ISIS通信机上行轮询任务
 *
 * @param para 任务参数，没有用到
 */
void vu_isis_uplink_task(void *para __attribute__((unused)));

#endif /* CONTRL_IF_DOWNLINK_VU_H_ */
