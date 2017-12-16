/*
 * if_jlgvu.h
 *
 *  Created on: 2017年10月12日
 *      Author: Ma Wenli
 */

#ifndef CONTRL_IF_JLGVU_H_
#define CONTRL_IF_JLGVU_H_

/*设备定义*/
#define JLG_VU_I2C_HANDLE           0
#define JLG_VU_I2C_ADDR             0x18

/*超时时间*/
#define JLG_VU_TIMEOUT              400
/*最大传输单元*/
#define JLG_VU_TX_MTU               235
#define JLG_VU_RX_MTU               200

/* 接收机指令定义  */
#define WATCHDOG_RESET              0xCC
#define SOFTWARE_RESET              0xAA
#define SEND_FRAME_DEFAULT          0x10
#define SEND_FRAME_NEW_CALLSIGN     0x11
#define SET_BEACON                  0x14
#define SET_BEACON_NEW_CALLSIGN     0x15
#define MEASURE_ALL_TELEMETRY       0x1A
#define CLEAR_BEACON                0x1F
#define GET_FRAME_NUM               0x21
#define GET_FRAME                   0x22
#define REMOVE_FRAME                0x24
#define SET_DEFAULT_TO_CALL         0x32
#define SET_DEFAULT_FROM_CALL       0x33
#define SET_DILE_STATE              0x34
#define SET_BITRATE                 0x38
#define REPORT_UPTIME               0x40
#define TRANSMITTER_STATE           0x41

#define FM_FORWARDING_ON            0x51
#define FM_FORWARDING_OFF           0x52
#define VU_POWER_ON                 0x53
#define VU_POWER_OFF                0x54

/*获取vu遥测响应结构体*/
typedef  struct __attribute__((packed)) {
    uint16_t ReflectedPower;
    uint16_t ForwardPower;
    uint16_t DopplerOffset;
    uint16_t RSSI;
    uint16_t BusVoltage;
    uint16_t TotalCurrent;
    uint16_t AmplifierTemp;
    uint16_t OscillatorTemp;
} rsp_vu_tm;

/**
 * 接收单元看门狗复位
 *
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_watchdog_reset(void);

/**
 * 接收单元软件复位
 *
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_software_reset(void);

/**
 * 获取通信机射频接收缓冲区，帧计数
 *
 * @param pnum 帧计数指针
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_get_frame_num(uint16_t *pnum);

/**
 * 获取通信机 射频接收缓冲区最早收到的一帧
 *
 * @param frame 数据帧指针
 * @param content_size 带获取数据帧中帧内容字节数
 * @return  E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_get_frame(rsp_frame *frame, uint8_t content_size);

/**
 * 获取接收机缓冲区最早的一帧，送入路由队列，路由协议的星地接口
 *
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_router_get_frame(void);

/**
 * 从通信机接收缓冲中清除最早的一帧
 *
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_remove_frame(void);

/**
 * 获取vu所有遥测
 *
 * @param receiver_tm 遥测数据指针
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_measure_all_tm(rsp_vu_tm *receiver_tm);

/**
 * 获取vu自上次复位以来运行时间
 *
 * @param uptime 运行时间指针
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_get_uptime(uint32_t *uptime);

/**
 * 发射单元发送AX.25数据帧，使用默认源呼号和目的呼号
 *
 * @param frame 数据帧指针
 * @param framelen 数据帧字节数，最大235个字节
 * @param rsp 发射机缓冲区剩余空间
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_send_frame(void *frame, uint8_t framelen, uint16_t *rsp);

/**
 * 发射单元发送AX.25数据帧，使用设置 的源呼号和目的呼号覆盖默认呼号
 *
 * @param frame 带呼号数据帧指针
 * @param framelen 数据帧字节数
 * @param rsp 发射机缓冲区剩余空间
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_send_frame_new_callsigns(par_frame_new_call *frame, uint8_t content_len, uint8_t *rsp);

/**
 *设置信标
 *
 * @param beacon 信标设置结构体指针
 * @param content_len 信标内容字节数
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_beacon_set(par_beacon_set *beacon, uint8_t content_len);

/**
 *设置覆盖默认呼号的信标
 *
 * @param beacon 信标设置结构体指针
 * @param content_len 信标内容字节数
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_beacon_new_call_set(par_beacon_new_call_set *beacon, uint8_t content_len);

/**
 * 清除信标
 *
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_clear_beacon(void);

/**
 * 设置发射机默认呼号
 *
 * @param callsigns 呼号设置结构体指针
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_set_callsigns(par_default_call_set *callsigns);

/**
 * 设置发射机空闲模式（即发送缓冲区为空时）是停止发射机工作，还是任然继续发送空序列，以便让地面锁定
 *
 * @param state 空闲状态，枚举数据类型，应为TurnOff或RemainOn其中之一
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_set_idle_state(par_idle_state state);

/**
 * 设置发射机波特率
 *
 * @param bitrate 发射波特率，枚举类型bps1200，bps2400，bps4800，bps9600其中之一
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_set_bitrate(par_transmission_bitrate bitrate);

/**
 * 获取发射机工作状态
 *
 * @param state 工作状态结构体指针
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_get_state(rsp_transmitter_state *state);

/**
 * fm转发开
 *
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_fm_on(void);

/**
 * fm转发关
 *
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_fm_off(void);

/**
 * 收发机上电
 *
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_power_on(void);

/**
 * 收发机断电
 *
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_power_off(void);


#endif /* CONTRL_IF_JLGVU_H_ */
