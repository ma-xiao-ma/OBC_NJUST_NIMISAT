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
#define JLG_VU_TIMEOUT                1000
/*最大传输单元*/
#define JLG_VU_TX_MTU                 235
#define JLG_VU_RX_MTU                 200

/* 接收机指令定义  */
#define WATCHDOG_RESET          0xCC
#define SOFTWARE_RESET          0xAA
#define SEND_FRAME_DEFAULT      0x10
#define SEND_FRAME_NEW_CALLSIGN 0x11
#define SET_BEACON              0x14
#define SET_BEACON_NEW_CALLSIGN 0x15
#define MEASURE_ALL_TELEMETRY   0x1A
#define CLEAR_BEACON            0x1F
#define GET_FRAME_NUM           0x21
#define GET_FRAME               0x22
#define REMOVE_FRAME            0x24
#define SET_DEFAULT_TO_CALL     0x32
#define SET_DEFAULT_FROM_CALL   0x33
#define SET_DILE_STATE          0x34
#define SET_BITRATE             0x38
#define REPORT_UPTIME           0x40
#define TRANSMITTER_STATE       0x41

#define FM_FORWARDING_ON        0x51
#define FM_FORWARDING_OFF       0x52

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




#endif /* CONTRL_IF_JLGVU_H_ */
