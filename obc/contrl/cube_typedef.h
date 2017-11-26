/*
 * cube_typedef.h
 *
 *  Created on: 2017年11月13日
 *      Author: Ma Wenli
 */

#ifndef CONTRL_CUBE_TYPEDEF_H_
#define CONTRL_CUBE_TYPEDEF_H_

#include <stdint.h>
/*********************************主帧*********************************/
/*星务计算机本地遥测，37 Byte*/
typedef struct __attribute__((packed))
{
    /**卫星号*/
    uint8_t         sat_id;                 //1
    /**软件版本号*/
    uint8_t         soft_id;                //1
    /**重启计数*/
    uint16_t        reboot_count;           //2
    /**上行本地指令计数*/
    uint16_t        rec_cmd_count;          //2
    /**下行遥测帧总计数*/
    uint32_t        hk_down_count;          //4
    /**存储遥测帧总计数*/
    uint32_t        hk_store_count;         //4
    /**i2c驱动错误计数*/
    uint32_t        i2c_error_count;        //4
    /**上次复位时间*/
    uint32_t        last_reset_time;        //4
    /**工作模式*/
    uint8_t         work_mode;              //1
    /**UTC时间*/
    uint32_t        utc_time;               //4
    /**CPU片内温度*/
    uint16_t        tmep_mcu;               //2
    /**obc板上温度*/
    uint16_t        tmep_board;             //2
    /**开关状态*/
    uint32_t        on_off_status;          //4
    /**RAM延时遥测主帧索引*/
    uint8_t         mindex;                 //1
    /**RAM延时遥测辅帧索引*/
    uint8_t         aindex;                 //1
} obc_hk_t;

/*电源分系统遥测，64 Byte*/
typedef struct __attribute__((packed))
{
    /**两路电池板温度*/
    int16_t         temp_batt_board[2];     //4
    /**四路电源控制板温度*/
    int16_t         temp_eps[4];            //8
    /**六路光电流*/
    uint16_t        sun_c[6];               //12
    /**六路光电压*/
    uint16_t        sun_v[6];               //12
    /**输出母线电流*/
    uint16_t        out_BusC;               //2
    /**输出母线电压*/
    uint16_t        out_BusV;               //2
    /**通信板电流*/
    uint16_t        UV_board_C;             //2
    /**六路可控输出的电流遥测*/
    uint16_t        Vol_5_C[6];             //12
    /**五路母线保护输出电流遥测*/
    uint16_t        Bus_c[5];               //10
} eps_hk_t;

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

/*获取发射单元遥测响应结构体*/
typedef  struct __attribute__((packed)) {
    uint16_t ReflectedPower;
    uint16_t ForwardPower;
    uint16_t BusVoltage;
    uint16_t TotalCurrent;
    uint16_t AmplifierTemp;
    uint16_t OscillatorTemp;
} rsp_tx_tm;

/*获取发射单元遥测响应结构体*/
typedef  struct __attribute__((packed)) {
    uint8_t IdleState: 1;    // bit0
    uint8_t BeaconAct: 1;    // bit1
    uint8_t BitRate: 2;      // bit2~bit3
    uint8_t FM_On: 1;        // bit4
    uint8_t padding: 3;
} rsp_transmitter_state;

/*ISISvu通信机遥测，46 Byte*/
typedef struct __attribute__((packed))
{
    /**接收单元自上次复位以来的运行时间*/
    uint32_t        ru_uptime;              //4
    /**接收单元当前所有遥测*/
    rsp_rx_tm       ru_curt;                //12
    /**接收单元收到上行数据时遥测*/
    receiving_tm    ru_last;                //2
    /**发射单元自上次复位以来的运行时间*/
    uint32_t        tu_uptime;              //4
    /**发射单元当前所有遥测*/
    rsp_tx_tm       tu_curt;                //12
    /**发射单元上次下行数据时所有遥测*/
    rsp_tx_tm       tu_last;                //12
    /**发射机工作状态*/
    rsp_transmitter_state tx_state;

} vu_isis_hk_t;

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
        uint8_t MEM1_BACK_CNT; //W13
        /**存储器2区回放数据量计数*/
        uint8_t MEM2_BACK_CNT; //W14
        /**存储器3区回放数据量计数*/
        uint8_t MEM3_BACK_CNT; //W15
        /**存储器4区回放数据量计数*/
        uint8_t MEM4_BACK_CNT; //W16
} dtb_tm_pack;

/*数传机遥测，17 Byte*/
typedef struct __attribute__((packed))
{
    dtb_tm_pack dtb_hk;
} dtb_805_hk_t;


/**传输方式*/
typedef enum __attribute__((__packed__))
{
    TransOff = 0,
    LVDS,
    TTL
} trans_mode;

/**工作模式*/
typedef enum __attribute__((__packed__))
{
    ImageRaw = 0,
    Image1fps,
    Video,
    Backup
} work_mode;

/**曝光模式*/
typedef enum __attribute__((__packed__))
{
    AutoExpoOn = 0,
    AutoExpoOff
} expo_mode;

/** 相机控制模式 */
typedef struct __attribute__((__packed__))
{
    trans_mode tran;
    work_mode mode;
    expo_mode expo;
} cam_ctl_t;

/*遥感相机遥测，14 Byte*/
typedef struct __attribute__((packed))
{
    /**相机采温点1温度*/
    uint16_t        point_1_temp;           //2
    /**相机采温点2温度*/
    uint16_t        point_2_temp;           //2
    /**相机采温点3温度*/
    uint16_t        point_3_temp;           //2
    /**曝光时间*/
    uint32_t        exposure_time;          //4
    /**增益*/
    uint8_t         gain;                   //1
    /**相机模式*/
    cam_ctl_t       work_mode;              //3
//    /**当前最新的备份图像ID*/
//    uint32_t        currt_image_id;         //4
} cam_805_hk_t;

typedef struct __attribute__((packed))
{
    obc_hk_t        obc;
    eps_hk_t        eps;
    vu_isis_hk_t    ttc;
    dtb_805_hk_t    dtb;
    cam_805_hk_t    cam;
} HK_Main_t;

/**********************************辅帧********************************/

typedef struct __attribute__((packed)) {
    uint16_t        rst_cnt;
    uint16_t        rcv_cnt;
    uint16_t        ack_cnt;
    uint32_t        rst_time;
    uint32_t        utc_time;
    uint16_t        cpu_temp;
    uint8_t         adcs_ctrl_mode;
    uint16_t        downAdcsMagDotDmpCnt;
    uint16_t        downAdcsPitFltComCnt;
    uint16_t        downAdcsAttStaCnt;
    uint8_t         error;

}adcs805_hk_workmode_t;

typedef struct __attribute__((packed)) {
    uint16_t        sw_status;
    int16_t         downAdcsMagnetometer[3];
    int16_t         downAdcsGyro_Meas[3];
    uint16_t        downAdcsSun1_Meas[4];
    uint16_t        downAdcsSun2_Meas[4];
    uint16_t        downAdcsSun3_Meas[4];
    uint8_t         downAdcsSunSensorFlag;
    float           downAdcsSun_Meas[3];  //上面是两个字节
    uint16_t        downAdcsWheelSpeed_Meas;
    int16_t         downAdcsMTQOut[3];
    int16_t         downAdcsMagInO[3];

}adcs805_hk_component_t;

typedef struct __attribute__((packed)) {
    int16_t         downAdcsPitAngle;
    int16_t         downAdcsPitFltState[2];
    float           downAdcsPitFltNormP;
    float           downAdcsTwoVector_euler[3];
    uint16_t        downAdcsTwoVectorCnt;
    uint16_t        downAdcsMagSunFltCnt;
    uint16_t        downAdcsMagGyroFltCnt;
    float           downAdcsMagSunFltQ[4];
    float           downAdcsMagSunFltW[3];
    float           downAdcsMagSunFltNormP;
    float           downAdcsMagGyroFltQ[4];
    float           downAdcsMagGyroFltw[3];
    float           downAdcsMagGyroFltNormP;

}adcs805_hk_attitude_t;

typedef struct __attribute__((packed)) {
    float           downAdcsOrbPos[3];
    int16_t         downAdcsOrbVel[3];
    uint8_t         GPS_status;
    uint8_t         GPS_numV;
    uint16_t        GPS_pdop;

}adcs805_hk_orbit_t;

typedef struct __attribute__((packed)) {
    int16_t          adc[10];
}adcs805_hk_temp_t;

typedef struct __attribute__((packed)) {
    adcs805_hk_workmode_t     adcs805_hk_workmode;
    adcs805_hk_component_t    adcs805_hk_component;
    adcs805_hk_attitude_t     adcs805_hk_attitude;
    adcs805_hk_orbit_t        adcs805_hk_orbit;
    adcs805_hk_temp_t         adcs805_hk_temp;
}adcs805_hk_t;

typedef struct __attribute__((packed)) {
    adcs805_hk_t        adcs_hk;
}HK_Append_t;

#endif /* CONTRL_CUBE_TYPEDEF_H_ */
