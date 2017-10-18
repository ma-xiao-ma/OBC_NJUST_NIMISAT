/*
 * if_trxvu.h
 *
 *  Created on: 2017年10月02日
 *      Author: Ma Wenli
 */
#ifndef CONTRL_IF_TRXVU_H_
#define CONTRL_IF_TRXVU_H_


#include <stdint.h>
#include <inttypes.h>
#include <stddef.h>

#include "stm32f4xx.h"

/*设备定义*/
#define ISIS_I2C_HANDLE		        0
/*地址定义*/
#define RECEIVER_I2C_ADDR           0x60
#define TRANSMITTER_I2C_ADDR        0x61
/*超时时间*/
#define ISIS_TIMEOUT	            1000
/*最大传输单元*/
#define ISIS_MTU                    235
#define ISIS_RX_MTU                 200


#define SCALL			"BI4ST-0"
#define DCALL			"NJUST-1"
#define ISIS_STAT_SET	0x08

#define ISIS_DELAY_TO_READ      1234


typedef enum {
    Receiver = 0x60,
    Transmitter
} vu_i2c_addr;

/*地址定义*/
#define	RECEIVER_I2C_ADDR		        0x60
#define	TRANSMITTER_I2C_ADDR		    0x61


/* 接收机指令定义  */
#define RECEIVER_WATCHDOG_RESET         0xCC
#define RECEIVER_SOFTWARE_RESET         0xAA
#define RECEIVER_HARDWARE_RESET         0xAB
#define RECEIVER_GET_FRAME_NUM			0x21
#define RECEIVER_GET_FRAME			    0x22
#define RECEIVER_REMOVE_FRAME		    0x24
#define MEASURE_RECEIVER_TELEMETRY      0x1A
#define REPORT_RECEIVER_UPTIME		    0x40

/* 发射机指令定义  */
#define TRANSMITTER_WATCHDOG_RESET      0xCC
#define TRANSMITTER_SOFTWARE_RESET      0xAA
#define TRANSMITTER_HARDWARE_RESET      0xAB
#define TRANSMITTER_SEND_FRAME_DEFAULT  0x10
#define SEND_FRAME_NEW_CALLSIGN 	    0x11
#define TRANSMITTER_SET_BEACON			0x14
#define SET_BEACON_NEW_CALLSIGN	        0x15
#define TRANSMITTER_CLEAR_BEACON	    0x1F
#define SET_DEFAULT_TO_CALLSIGN	        0x22
#define SET_DEFAULT_FROM_CALLSIGN	    0x23
#define SET_TRANSMITTER_DILE_STATE	    0x24
#define MEASURE_TRANSMITTER_TELEMETRY   0x25
#define GET_TRANSMITTER_TELEMETRY_LAST	0x26
#define SET_TRANSMISSION_BITRATE	    0x28
#define REPORT_TRANSMITTER_UPTIME	    0x40
#define REPORT_TRANSMITTER_STATE	    0x41

/*带参数命令结构体*/
typedef  struct __attribute__((packed)) {
    uint8_t command;
    uint8_t parameter[0];
} cmd_with_para;

/*获取帧响应结构体*/
typedef  struct __attribute__((packed)) {
    uint16_t DateSize;
    uint16_t DopplerOffset;
    uint16_t RSSI;
    uint8_t  Data[0];
} rsp_frame;

/*获取接收单元遥测响应结构体*/
typedef  struct __attribute__((packed)) {
    uint16_t DopplerOffset;
    uint16_t TotalCurrent;
    uint16_t BusVoltage;
    uint16_t OscillatorTemp;
    uint16_t AmplifierTemp;
    uint16_t RSSI;
} rsp_rx_tm;

/*接收机收到上行数据时的遥测*/
typedef  struct __attribute__((packed)) {
    uint16_t DopplerOffset;
    uint16_t RSSI;
} receiving_tm;

/*发送新呼号数据帧 命令结构体 */
typedef  struct __attribute__((packed)) {
    char	DstCall[7];
    char	SrcCall[7];
	uint8_t	FrameContent[0];
} par_frame_new_call;

/*信标设置命令结构体定义  */
typedef struct __attribute__((packed))  {
	uint16_t RepeatInterval;
	uint8_t	 BeaconContent[0];
}par_beacon_set;

/*信标覆盖默认呼号设置命令结构体定义  */
typedef struct __attribute__((packed))  {
    uint16_t RepeatInterval;
    char DstCall[7];
    char SrcCall[7];
    uint8_t  BeaconContent[0];
}par_beacon_new_call_set;

/*发射单元AX.25默认呼号设置命令结构体*/
typedef struct __attribute__((packed))  {
    char DstCall[7];
    char SrcCall[7];
}par_default_call_set;

typedef enum {
    TurnOff = 0,
    RemainOn
} par_idle_state;

/*获取发射单元遥测响应结构体*/
typedef  struct __attribute__((packed)) {
    uint16_t ReflectedPower;
    uint16_t ForwardPower;
    uint16_t BusVoltage;
    uint16_t TotalCurrent;
    uint16_t AmplifierTemp;
    uint16_t OscillatorTemp;
} rsp_tx_tm;

typedef enum {
    bps1200 = 1,
    bps2400 = 2,
    bps4800 = 4,
    bps9600 = 8,
} par_transmission_bitrate;

/*获取发射单元遥测响应结构体*/
typedef  struct __attribute__((packed)) {
    uint8_t IdleState: 1;    // bit0
    uint8_t BeaconAct: 1;    // bit1
    uint8_t BitRate: 2;      // bit2~bit3
    uint8_t FM_On: 1;        // bit4
    uint8_t padding: 3;
} rsp_transmitter_state;




#define STATE_IDLE_MASK     0x01 //0000 000x
#define STATE_BEACON_MASK   0x02 //0000 00x0
#define STATE_RATE_MASK     0x0C //0000 xx00

#define IDLE_STATE(state) (state&STATE_IDLE_MASK)
#define BEACIN_ACTIVE(state) ((state&STATE_BEACON_MASK)>>1)
#define BIT_RATE(state) ((state&STATE_RATE_MASK)>>2)


//end

/* 函数声明*/

/**************************接收单元指令函数****************************/
/**
 * 接收单元看门狗复位
 *
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_receiver_watchdog_reset(void);

/**
 * 接收单元软件复位
 *
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_receiver_software_reset(void);

/**
 * 接收单元硬件复位
 *
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_receiver_hardware_reset(void);

/**
 * 获取通信机射频接收缓冲区，帧计数
 *
 * @param pnum 帧计数指针
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_receiver_get_frame_num(uint16_t *pnum);

/**
 * 获取通信机 射频接收缓冲区最早收到的一帧
 *
 * @param frame 数据帧指针
 * @param content_size 带获取数据帧中帧内容字节数
 * @return  E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_receiver_get_frame(rsp_frame *frame, uint8_t content_size);

/**
 * 获取接收机缓冲区最早的一帧，送入路由队列，路由协议的星地接口
 *
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_receiver_router_get_frame(void);

/**
 * 从通信机接收缓冲中清除最早的一帧
 *
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_receiver_remove_frame(void);

/**
 * 获取接收单元所有遥测
 *
 * @param receiver_tm 遥测数据指针
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_receiver_measure_tm(rsp_rx_tm *receiver_tm);

/**
 * 获取接收单元接收上一帧时的遥测
 *
 * @param tm 接收缓冲区指针
 * @return pdTRUE为正常，pdFALSE不正常
 */
int vu_isis_get_receiving_tm(receiving_tm *tm);

/**
 * 获取接收单元自上次复位以来运行时间
 *
 * @param uptime 运行时间指针
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_receiver_get_uptime(uint32_t *uptime);

/**************************发射单元指令函数****************************/

/**
 * 发射单元看门狗复位
 *
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_transmitter_watchdog_reset(void);

/**
 * 发射单元软件复位
 *
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_transmitter_software_reset(void);

/**
 * 发射单元硬件复位
 *
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_transmitter_hardware_reset(void);

/**
 * 发射单元发送AX.25数据帧，使用默认源呼号和目的呼号
 *
 * @param frame 数据帧指针
 * @param framelen 数据帧字节数
 * @param rsp 发射机缓冲区剩余空间
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_transmitter_send_frame(void *frame, uint8_t framelen, uint8_t *rsp);

/**
 * 发射单元发送AX.25数据帧，使用设置 的源呼号和目的呼号覆盖默认呼号
 *
 * @param frame 带呼号数据帧指针
 * @param framelen 数据帧字节数
 * @param rsp 发射机缓冲区剩余空间
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_transmitter_send_frame_new_callsigns(par_frame_new_call *frame, uint8_t content_len, uint8_t *rsp);

/**
 *设置信标
 *
 * @param beacon 信标设置结构体指针
 * @param content_len 信标内容字节数
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_transmitter_beacon_set(par_beacon_set *beacon, uint8_t content_len);

/**
 *设置覆盖默认呼号的信标
 *
 * @param beacon 信标设置结构体指针
 * @param content_len 信标内容字节数
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_transmitter_beacon_new_call_set(par_beacon_new_call_set *beacon, uint8_t content_len);

/**
 * 清除信标
 *
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_transmitter_clear_beacon(void);

/**
 * 设置发射机默认呼号
 *
 * @param callsigns 呼号设置结构体指针
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_transmitter_set_callsigns(par_default_call_set *callsigns);

/**
 * 设置发射机空闲模式（即发送缓冲区为空时）是停止发射机工作，还是任然继续发送空序列，以便让地面锁定
 *
 * @param state 空闲状态，枚举数据类型，应为TurnOff或RemainOn其中之一
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_transmitter_set_idle_state(par_idle_state state);

/**
 * 发射单元获取所有遥测
 *
 * @param transmitter_tm 发送单元遥测结构体指针
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_transmitter_measure_tm(rsp_tx_tm *transmitter_tm);

/**
 * 发射单元获取上次发射时遥测
 *
 * @param last_tm 发送单元遥测结构体指针
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_transmitter_get_last_tm(rsp_tx_tm *last_tm);

/**
 * 设置发射机波特率
 *
 * @param bitrate 发射波特率，枚举类型bps1200，bps2400，bps4800，bps9600其中之一
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_transmitter_set_bitrate(par_transmission_bitrate bitrate);

/**
 * 获取发射单元自上次复位以来运行时间
 *
 * @param uptime 运行时间指针
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_transmitter_get_uptime(uint32_t *uptime);

/**
 * 获取发射机工作状态
 *
 * @param state 工作状态结构体指针
 * @return
 */
int vu_transmitter_get_state(rsp_transmitter_state *state);


#endif /* CONTRL_IF_TRXVU_H_ */
