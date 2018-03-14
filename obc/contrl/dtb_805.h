/*
 * dtb_805.h
 *
 *  Created on: 2017年9月18日
 *      Author: Ma Wenli
 *  描述：Data
 */

#ifndef CONTRL_DTB_805_H_
#define CONTRL_DTB_805_H_

/* 数传遥测结构 */
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
    /*************************************/
    /**下位机I2C总线复位计数*/
    uint8_t IIC_RS_CNT:4; //W4B3～B0
    /**下位机CAN总线复位计数*/
    uint8_t CAN_RS_CNT:4; //W4B7～B4
    /*************************************/
    /**数传下传码速率状态*/
    uint8_t DOWN_RATE:3; //W5B2～B0
    /**数传发射机开关机状态*/
    uint8_t TRANS_ON:1; //W5B3
    /**下位机间接指令计数*/
    uint8_t RX_INS_CNT:4; //W5B7～B4
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
    /*************************************/
    /**下位机总线状态遥测*/
    uint8_t MEM4_STA:2; //W7B1～B0
    /**下位机总线状态遥测*/
    uint8_t MEM3_STA:2; //W7B3～B2
    /**下位机总线状态遥测*/
    uint8_t MEM2_STA:2; //W7B5～B4
    /**下位机总线状态遥测*/
    uint8_t MEM1_STA:2; //W7B7～B6
    /*************************************/
    /**存储器4区状态遥测*/
    uint8_t MEM4_MARGIN:2; //W8B1～B0
    /**存储器3区状态遥测*/
    uint8_t MEM3_MARGIN:2; //W8B3～B2
    /**存储器2区状态遥测*/
    uint8_t MEM2_MARGIN:2; //W8B5～B4
    /**存储器1区状态遥测*/
    uint8_t MEM1_MARGIN:2; //W8B7～B6
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
    uint8_t MEM1_BACK_CNT;  //W13
    /**存储器2区回放数据量计数*/
    uint8_t MEM2_BACK_CNT;  //W14
    /**存储器3区回放数据量计数*/
    uint8_t MEM3_BACK_CNT;  //W15
    /**存储器4区回放数据量计数*/
    uint8_t MEM4_BACK_CNT;  //W16
} dtb_tm_pack;

/** 数传机固存区域  固存1-4*/
typedef enum
{
    MemOne = 1,
    MemTwo,
    MemThree,
    MemFour
} mem_region;

/** 数传机下行码速率选择*/
typedef enum
{
    bps_1_M = 1,
    bps_2_M,
    bps_4_M,
    bps_8_M
} data_rate;

/** 数传机下行码速率选择*/
typedef enum
{
    tr_sw_off = 0,
    tr_sw_on
} tr_sw_status;

/** 数传控制命令 指令字*/
typedef enum
{
    Boot = 1,       //发射机开机
    ShutDown,       //发射机关机
    MemReset,       //固存复位
    Mem1Record,     //固存一区记录
    Mem2Record,     //固存二区记录
    Mem3Record,     //固存三区记录
    Mem4Record,     //固存四区记录
    MemStop,        //固存停止
    Mem1Back,       //固存一区回放
    Mem2Back,       //固存二区回放
    Mem3Back,       //固存三区回放
    Mem4Back,       //固存四区回放
    Mem1Erase,      //固存一区擦除
    Mem2Erase,      //固存二区擦除
    Mem3Erase,      //固存三区擦除
    Mem4Erase,      //固存四区擦除
    PseudoOn,       //伪码开
    PseudoOff,      //伪码关
    Rate1Mbps,      //1Mbps速率
    Rate2Mbps,      //2Mbps速率
    Rate4Mbps = 22, //4Mbps速率
    Rate8Mbps       //8Mbps速率
} dtb_cmd;

/**
 * IO控制数传机上电
 *
 * @return 返回E_NO_ERR（-1）为正常
 */
int dtb_power_on(void);

/**
 * IO控制数传机断电
 *
 */
int dtb_power_off(void);

/**
 * 数传板供电控制函数
 *
 * @return 返回E_NO_ERR（-1）为正常
 */
int dtb_power_switch( tr_sw_status power_sw );

/**
 * 数传遥测获取指令
 *
 * @param pRxData 遥测获取指针
 * @param Timeout I2C超时时间
 * @return
 */
int xDTBTelemetryGet(uint8_t *pRxData, uint16_t Timeout);

/**
 * 数传I2C遥控指令函数
 *
 * @param Cmd 指令字
 * @param Timeout 超时时间
 * @return 返回E_NO_ERR（-1）为正常
 */
int xDTBTeleControlSend(uint8_t Cmd, uint16_t Timeout);

/**
 * 数传固存擦除
 *
 * @param mem_num 固存编号
 * @return 返回E_NO_ERR（-1）为正常
 */
int mem_erase( mem_region mem_num );

/**
 * 数传固存记录
 *
 * @param mem_num 固存编号
 * @return 返回E_NO_ERR（-1）为正常
 */
int mem_record( mem_region mem_num );

/**
 * 数传固存回放
 *
 * @param mem_num 固存编号
 * @return 返回E_NO_ERR（-1）为正常
 */
int mem_back( mem_region mem_num );

/**
 * 数传机射频端开关函数
 *
 * @param select 开为1，关为0
 * @return 函数执行成功返回E_NO_ERR(-1)
 */
int dtb_rf_switch( tr_sw_status select );

/**
 * 数传伪码开关函数
 *
 * @param select 开为1，关为0
 * @return 函数执行成功返回E_NO_ERR(-1)
 */
int dtb_pseudo_switch( tr_sw_status select );

/**
 * 数传机擦除加记录
 *
 * @param mem_num 即将向存储区几存储
 * @param need_erase 是否需要擦除固存（0为不擦除  非0为擦除）
 *
 * @return 返回E_NO_ERR（-1）为正常
 */
int dtb_mem_record(uint8_t mem_num, uint8_t need_erase);

/**
 * 数传机回放下行流程
 *
 * @param mem_num 存储区号 1、2、3、4
 * @param data_rate 下行码速率1Mbps、2Mbps、4Mbps、8Mbps
 * @return 返回E_NO_ERR（-1）为正确
 */
int dtb_mem_back(uint8_t mem_num, uint8_t data_rate);

/**
 * 下行码速率选择
 *
 * @param down_rate 下行码速率选择 1--1Mbps 2--2Mbps 3--4Mbps 4--8Mbps
 * @return 返回E_NO_ERR（-1）为正常
 */
int rf_rate_select( data_rate down_rate );

/**
 * 数传遥测打印
 *
 * @param tm
 */
void dtb_tm_print(dtb_tm_pack *tm);

#endif /* CONTRL_DTB_805_H_ */
