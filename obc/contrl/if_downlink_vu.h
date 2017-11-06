/*
 * if_downlink_vu.h
 *
 *  Created on: 2017年10月5日
 *      Author: Ma Wenli
 */
#ifndef CONTRL_IF_DOWNLINK_VU_H_
#define CONTRL_IF_DOWNLINK_VU_H_

#define DOWNLINK_CRC_SIZE   4
#define DOWNLINK_OVERHEAD (ROUTE_HEAD_SIZE + DOWNLINK_CRC_SIZE)
#define DOWNLINK_MTU (ISIS_MTU - DOWNLINK_OVERHEAD)

#define IMAGE_DOWNLINK_OVERHEAD (DOWNLINK_OVERHEAD + IMAGE_PACK_HEAD_SIZE)

#define MAX_UPLINK_CONTENT_SIZE  64
/*obc给trxvu数据包的时间间隔*/
#define PACK_DOWN_INTERVAL (1 / portTICK_PERIOD_MS)

/*当发射机缓冲区满时等待的时间*/
#define MS_WAIT_TRANS_FREE_BUFF (5000 / portTICK_PERIOD_MS)

typedef struct
{
    uint8_t tpye;
    void * pdata;
    uint32_t data_len;
    uint16_t start_pack;
} downlink_request;

/**
 * ISIS通信机下行接口函数，由OBC本地调用
 *
 * @param type 下行消息类型
 * @param pdata 下行数据指针
 * @param len 下行数据字节数
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_isis_downlink(uint8_t type, void *pdata, uint32_t len);

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
