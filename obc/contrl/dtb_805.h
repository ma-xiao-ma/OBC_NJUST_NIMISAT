/*
 * dtb_805.h
 *
 *  Created on: 2017年9月18日
 *      Author: Ma Wenli
 *  描述：Data
 */

#ifndef CONTRL_DTB_805_H_
#define CONTRL_DTB_805_H_


typedef struct __attribute__((__packed__))
{
        /**数传发射机本振锁定遥测*/
        uint8_t TM_STA; //W0
        /**数传发射机本振锁定遥测*/
        uint8_t AF_PWR; //W1
        /**数传发射机固放温度遥测*/
        uint8_t AF_TEMP; //W2
        /*************************************/
        /**下位机复位计数*/
        uint8_t RS_CNT:4; //W3B3～B0
        /**下位机“看门狗”定时计数*/
        uint8_t WD_CNT:3; //W3B6～B4
        /**下位机总线状态遥测*/
        uint8_t IS_CAN:1; //W3B7
//        uint8_t WD_CNT:3; //W3B6～B4
//        uint8_t RS_CNT:4; //W3B3～B0
        /*************************************/
        /**下位机I2C总线复位计数*/
        uint8_t IIC_RS_CNT:4; //W4B3～B0
        /**下位机CAN总线复位计数*/
        uint8_t CAN_RS_CNT:4; //W4B7～B4
//        uint8_t IIC_RS_CNT:4; //W4B3～B0
        /*************************************/
        /**数传下传码速率状态*/
        uint8_t DOWN_RATE:3; //W5B2～B0
        /**数传发射机开关机状态*/
        uint8_t TRANS_ON:1; //W5B3
        /**下位机间接指令计数*/
        uint8_t RX_INS_CNT:4; //W5B7～B4
//        uint8_t TRANS_ON:1; //W5B3
//        uint8_t DOWN_RATE:3; //W5B2～B0
        /*************************************/
        /**备用*/
        uint8_t PADDING:1; //W6B0
        /**数传工作模式*/
        uint8_t WORK_MODE:3; //W6B3～B1
        /**数据处理记录数据帧头遥测*/
        uint8_t RECORD_CORRECT:1; //W6B4
        /**数据处理回放正确遥测*/
        uint8_t BACK_CORRECT:1; //W6B5
        /**数据处理伪码状态遥测*/
        uint8_t PSD_CODE_ON:1; //W6B6
        /**数据处理+3.3V电源遥测*/
        uint8_t PWR_3V3_ON:1; //W6B7
//        uint8_t PSD_CODE_ON:1; //W6B6
//        uint8_t BACK_CORRECT:1; //W6B5
//        uint8_t RECORD_CORRECT:1; //W6B4
//        uint8_t WORK_MODE:3; //W6B3～B1
//        uint8_t PADING:1; //W6B0
        /*************************************/
        /**下位机总线状态遥测*/
        uint8_t MEM4_STA:2; //W7B1～B0
        /**下位机总线状态遥测*/
        uint8_t MEM3_STA:2; //W7B3～B2
        /**下位机总线状态遥测*/
        uint8_t MEM2_STA:2; //W7B5～B4
        /**下位机总线状态遥测*/
        uint8_t MEM1_STA:2; //W7B7～B6
//        uint8_t MEM2_STA:2; //W7B5～B4
//        uint8_t MEM3_STA:2; //W7B3～B2
//        uint8_t MEM4_STA:2; //W7B1～B0
        /*************************************/
        /**存储器4区状态遥测*/
        uint8_t MEM4_MARGIN:2; //W8B1～B0
        /**存储器3区状态遥测*/
        uint8_t MEM3_MARGIN:2; //W8B3～B2
        /**存储器2区状态遥测*/
        uint8_t MEM2_MARGIN:2; //W8B5～B4
        /**存储器1区状态遥测*/
        uint8_t MEM1_MARGIN:2; //W8B7～B6
//        uint8_t MEM2_MARGIN:2; //W8B5～B4
//        uint8_t MEM3_MARGIN:2; //W8B3～B2
//        uint8_t MEM4_MARGIN:2; //W8B1～B0
        /*************************************/
        /**存储器1区记录数据量计数*/
        uint8_t MEM1_RECORD_CNT; //W9
        /**存储器2区记录数据量计数*/
        uint8_t MEM2_RECORD_CNT; //W10
        /**存储器3区记录数据量计数*/
        uint8_t MEM3_RECORD_CNT; //W11
        /**存储器4区记录数据量计数*/
        uint8_t MEM4_RECORD_CNT; //W12
        /**存储器1区回放数据量计数*/
        uint8_t MEM1_BACK_CNT; //W13
        /**存储器2区回放数据量计数*/
        uint8_t MEM2_BACK_CNT; //W14
        /**存储器3区回放数据量计数*/
        uint8_t MEM3_BACK_CNT; //W15
        /**存储器4区回放数据量计数*/
        uint8_t MEM4_BACK_CNT; //W16
} dtb_tm_pack;

/**数传控制命令*/
typedef enum __attribute__((__packed__))
{
    Boot = 1,
    ShutDown,
    MemReset,
    Mem1Record,
    Mem2Record,
    Mem3Record,
    Mem4Record,
    MemStop,
    Mem1Back,
    Mem2Back,
    Mem3Back,
    Mem4Back,
    Mem1Erase,
    Mem2Erase,
    Mem3Erase,
    Mem4Erase,
    PseudoOn,
    PseudoOff,
    Rate1Mbps,
    Rate2Mbps,
    Rate4Mbps
} dtb_cmd;


int xDTBTelemetryGet(uint8_t *pRxData, uint16_t Timeout);

int xDTBTeleControlSend(uint8_t Cmd, uint16_t Timeout);

void dtb_tm_print(dtb_tm_pack *tm);

#endif /* CONTRL_DTB_805_H_ */
