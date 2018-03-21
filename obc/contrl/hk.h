/*
 * hk.h
 *
 *  Created on: 2016年05月31日
 *      Author: Administrator
 */

#ifndef SRC_HK_H_
#define SRC_HK_H_

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <time.h>

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#include "ff.h"
#include "dtb_805.h"
#include "camera_805.h"
#include "if_trxvu.h"
#include "if_jlgvu.h"

#define HK_SDCARD					0x00
#define HK_SRAM						0x01

#define HK_FIFO_EMPTY 				0x00
#define HK_FIFO_FULL 				0x01
#define HK_FIFO_OK 					0x02

#define HK_FIFO_BUFFER_SIZE 		255U
#define HK_FIFO_BUFFER_CNT	 		20U

#define HK_FRAME_MAIN				0x01
#define HK_FRAME_APPEND				0x02

#define HK_MAIN_LENGTH			    sizeof(HK_Main_t)
#define HK_APPEND_LENGTH			sizeof(HK_Append_t)

#define HK_LIST_NODE_CNT			(1000U+1)
#define HK_STORAGE_INTERVAL         15
#define HK_OFFSET_H_MS				(uint32_t)(HK_STORAGE_INTERVAL * HK_FILE_MAX_COUNT / 2)

/*一个遥测文件最多存储240条遥测信息*/
#define HK_FILE_MAX_COUNT			240

typedef struct hkListNode {
	uint32_t TimeValue;
	struct hkListNode * pxNext;
	struct hkListNode * pxPrevious;
	void * pvContainer;
}hkListNode_t;

typedef struct __attribute__((packed))
{
	uint32_t TimeValue;
	struct hkListNode * pxNext;
	struct hkListNode * pxPrevious;
}hkMiniListnode_t;

typedef struct __attribute__((packed))
{
	uint32_t uxNumberOfItems;
	struct hkListNode * pxIndex;
	hkMiniListnode_t xListEnd;
} hkList_t;

typedef struct __attribute__((packed)) {
	unsigned char 	frame[HK_FIFO_BUFFER_CNT][HK_FIFO_BUFFER_SIZE];
	unsigned int 	bufferCount ;
	unsigned int 	front ;
	unsigned int 	rear ;
} HK_Fifo_t;


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
    /**通信机接收指令计数*/
    uint16_t        vu_rec_count;           //2  /*新增*/
    /**下行遥测帧总计数*/
    uint16_t        hk_down_count;          //2
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

/*ISISvu通信机遥测，49 Byte*/
typedef struct __attribute__((packed))
{
    /**接收单元自上次复位以来的运行时间*/
    uint32_t        ru_uptime;              //4
    /**接收单元当前所有遥测*/
    rsp_rx_tm       ru_curt;                //12
    /**接收单元收到上行数据时遥测*/
    receiving_tm    ru_last;                //4
    /**发射单元自上次复位以来的运行时间*/
    uint32_t        tu_uptime;              //4
    /**发射单元当前所有遥测*/
    rsp_tx_tm       tu_curt;                //12
    /**发射单元上次下行数据时所有遥测*/
    rsp_tx_tm       tu_last;                //12
    /**发射机工作状态*/
    rsp_transmitter_state tx_state;         //1

} vu_isis_hk_t;

/*备份通信机遥测，17 Byte*/
typedef struct __attribute__((packed))
{
    /**备份通信机所有遥测*/
    rsp_vu_tm vu_tm;                        //16
    /**发射机工作状态*/
    rsp_transmitter_state tx_state;         //1
} vu_jlg_hk_t;

/*数传机遥测，17 Byte*/
typedef struct __attribute__((packed))
{
    dtb_tm_pack dtb_hk;
} dtb_805_hk_t;

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

/* 主遥测帧, 181 byte*/
typedef struct __attribute__((packed))
{
    obc_hk_t        obc;
    eps_hk_t        eps;
    vu_isis_hk_t    ttc;
    vu_jlg_hk_t     jlg;
    cam_805_hk_t    cam;
    dtb_805_hk_t    dtb;
} HK_Main_t;

typedef struct __attribute__((packed))
{
    uint16_t        rst_cnt;
    uint16_t        rcv_cnt;
    uint16_t        ack_cnt;
    uint32_t        rst_time;
    uint32_t        utc_time;
    uint16_t        cpu_temp;
    uint8_t         adcs_ctrl_mode;
    uint32_t        downAdcsMagDotDmpCnt;
    uint32_t        downAdcsPitFltComCnt;
    uint32_t        downAdcsAttStaCnt;
    uint8_t         error;
}adcs805_hk_workmode_t;

typedef struct __attribute__((packed))
{
    uint16_t        sw_status;
    int16_t         downAdcsMagnetometer[3];
    int16_t         downAdcsGyro_Meas[3];
    uint16_t        downAdcsSun1_Meas[4];     //add*
    uint16_t        downAdcsSun2_Meas[4];     //add*
    uint16_t        downAdcsSun3_Meas[4];     //add*
    uint8_t         downAdcsSunSensorFlag;    //add*
    float           downAdcsSun_Meas[3];      //add*  //上面是两个字节
    uint16_t        downAdcsWheelSpeed_Meas;
    int16_t         downAdcsMTQOut[3];
    int16_t         downAdcsMagInO[3];

} adcs805_hk_component_t;

typedef struct __attribute__((packed))
{
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
} adcs805_hk_attitude_t;

typedef struct __attribute__((packed))
{
        float           downAdcsOrbPos[3];
        int16_t         downAdcsOrbVel[3];
        uint8_t         GPS_status;
        uint8_t         GPS_numV;
        uint16_t        GPS_pdop;
} adcs805_hk_orbit_t;

typedef struct __attribute__((packed))
{
    int16_t         adc[10];
} adcs805_hk_temp_t;

typedef struct __attribute__((packed))
{
    adcs805_hk_workmode_t     adcs805_hk_workmode;
    adcs805_hk_component_t    adcs805_hk_component;
    adcs805_hk_attitude_t     adcs805_hk_attitude;
    adcs805_hk_orbit_t        adcs805_hk_orbit;
    adcs805_hk_temp_t         adcs805_hk_temp;
} adcs805_hk_t;

typedef struct __attribute__((packed))
{
	adcs805_hk_t		adcs_hk;
} HK_Append_t;


typedef struct __attribute__((packed))
{
	HK_Main_t 	main_frame;
	HK_Append_t append_frame;
} HK_Store_t;

extern HK_Store_t		hk_frame;
extern HK_Store_t		hk_old_frame;

extern uint8_t 			hk_select;
extern uint16_t			hk_sram_index;
extern uint32_t			hk_sd_time;
extern uint8_t 			hk_sd_path[25];
extern FIL 				hkfile;
extern UINT				hkrbytes;
extern uint32_t			hkleek;
//extern uint32_t         i2c_error_count;

extern uint16_t  hk_frame_index;
extern HK_Fifo_t hk_main_fifo;
extern HK_Fifo_t hk_append_fifo;
extern hkList_t  hk_list;

void hk_list_init(hkList_t * pxList);
uint32_t hk_list_insert(hkList_t * pxList, uint32_t xValueOfInsertion);
void * hk_list_find(uint32_t time);
uint32_t hk_list_recover(void);
uint32_t hk_list_remove(hkListNode_t * pxNodeToRemove);
void HK_fifoInit(HK_Fifo_t *Q);
uint8_t HK_fifoIn(HK_Fifo_t *Q, unsigned char *pdata, uint8_t opt);
uint8_t HK_fifoOut(HK_Fifo_t *Q, unsigned char *pdata, uint8_t opt);
uint16_t hk_fifo_find(const HK_Fifo_t *Q, uint32_t timevalue);
void hk_collect_no_store(void);
void hk_collect(void);
int hk_store_init(void);
void hk_out(void);
int hk_store_add(void);
void hk_file_task(void * paragram);
void vTelemetryFileManage(void * paragram);

/**
 * 星务遥测采集任务
 */
void obc_hk_task(void);

/**
 * 星务遥测获取函数
 *
 * @param obc 星务遥测结构体指针
 * @return pdTRUE为正常，pdFALSE不正常
 */
int obc_hk_get_peek(obc_hk_t * obc);

/**
 * 电源系统遥测采集任务，采集数值放入eps_hk_queue队列中
 *
 */
void eps_hk_task(void);

/**
 * 通过eps_hk_queue队列获取EPS遥测值
 *
 * @param tm 接收缓冲区指针
 * @return pdTRUE为正常，pdFALSE不正常
 */
int eps_hk_get_peek(eps_hk_t *eps);

/**
 * TTC遥测采集任务，采集到的数据送入ttc_hk_queue队列
 *
 */
void ttc_hk_task(void);

/**
 * 通过队列获取TTC遥测值
 *
 * @param tm 接收缓冲区指针
 * @return pdTRUE为正常，pdFALSE不正常
 */
int ttc_hk_get_peek(vu_isis_hk_t *ttc);

/**
 * JLG通信机遥测采集任务，采集到的数据送入jlg_hk_queue队列
 *
 */
void jlg_hk_task(void);

/**
 * 通过队列获取JLG通信机遥测值
 *
 * @param tm 接收缓冲区指针
 * @return pdTRUE为正常，pdFALSE不正常
 */
int jlg_hk_get_peek(vu_jlg_hk_t *jlg);

/**
 * 数传机数据采集任务，采集数值放入eps_hk_queue中
 *
 */
void dtb_hk_task(void);

/**
 * 通过队列获取dtb遥测值
 *
 * @param tm 接收缓冲区指针
 * @return pdTRUE为正常，pdFALSE不正常
 */
int dtb_hk_get_peek(dtb_805_hk_t *dtb);

/**
 * 相机遥测采集任务，采集到的遥测放入cam_hk_queue队列中
 *
 * 遥测值获取调用int cam_hk_get_peek(cam_805_hk_t *cam)函数
 */
void cam_hk_task(void);

/**
 * 通过队列获取TTC遥测值
 *
 * @param tm 接收缓冲区指针
 * @return pdTRUE为正常，pdFALSE不正常
 */
int cam_hk_get_peek(cam_805_hk_t *cam);

/**
 * 姿控系统数据采集任务，采集数值放入adcs_hk_queue中
 *
 */
void adcs_hk_task(void);

/**
 * 通过队列获取ADCS遥测值
 *
 * @param tm 接收缓冲区指针
 * @return pdTRUE为正常，pdFALSE不正常
 */
int adcs_hk_get_peek(adcs805_hk_t *adcs);

/**
 * 遥测采集任务初始化 创建6个队列， 需要在初始化函数中调用
 */
void hk_collect_task_init(void);


#endif /* SRC_HK_H_ */
