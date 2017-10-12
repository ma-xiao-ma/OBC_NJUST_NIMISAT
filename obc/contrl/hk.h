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
#include "if_trxvu.h"
#include "dtb_805.h"
#include "camera_805.h"

#define HK_SDCARD					0x00
#define HK_SRAM						0x01

#define HK_FIFO_EMPTY 				0x00
#define HK_FIFO_FULL 				0x01
#define HK_FIFO_OK 					0x02
#define HK_FIFO_BUFFER_SIZE 		200U
#define HK_FIFO_BUFFER_CNT	 		20U

#define HK_FRAME_MAIN				0x01
#define HK_FRAME_APPEND				0x02

#define HK_MAIN_LENGTH				168U
#define HK_APPEND_LENGTH			185U

#define HK_LIST_NODE_CNT			(1000U+1)
#define HK_STORAGE_INTERVAL         15
#define HK_OFFSET_H_MS				(uint32_t)(HK_STORAGE_INTERVAL * HK_FILE_MAX_COUNT / 2)

/*一个遥测文件最多存储200条遥测信息*/
#define HK_FILE_MAX_COUNT			200

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


/*星务计算机本地遥测，23 Byte*/
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
    /**遥测帧总计数（存储+下行）*/
    uint16_t        down_count;             //2
    /**上次复位时间*/
    uint32_t        last_reset_time;        //4
    /**工作模式*/
    uint8_t         work_mode;              //1
    /**UTC时间*/
    uint32_t        utc_time;               //4
    /**CPU片内温度*/
    uint16_t        tmep_hk;                //2
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


/*ISISvu通信机遥测，46 Byte*/
typedef struct __attribute__((packed))
{
    /**接收单元自上次复位以来的运行时间*/
    uint32_t        RU_uptime;              //4
    /**接收单元当前所有遥测*/
    rsp_rx_tm       RU_curt;                //12
    /**接收单元收到上行数据时遥测*/
    receiving_tm    RU_last;                //2
    /**发射单元自上次复位以来的运行时间*/
    uint32_t        TU_uptime;              //4
    /**发射单元当前所有遥测*/
    rsp_tx_tm       TU_curt;                //12
    /**发射单元上次下行数据时所有遥测*/
    rsp_tx_tm       TU_last;                //12
} vu_isis_hk_t;

/*数传机遥测，17 Byte*/
typedef struct __attribute__((packed))
{
    dtb_tm_pack dtb_hk;
} dtb_805_hk_t;

/*遥感相机遥测，16 Byte*/
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
    cam_mode_t      work_mode;              //1
    /**当前最新的备份图像ID*/
    uint32_t        currt_image_id;          //4
} cam_805_hk_t;


			/* total of 94 byte	*/
typedef struct __attribute__((packed)) {
	// header 2
	unsigned char 			header[2];  //0xeb 0x50         //2

	//obc  23
	unsigned char 			sat_id;                         //1
	unsigned char 			soft_id;                        //1
	unsigned short int 		reboot_count;                 	//2
	unsigned short int 		rec_cmd_count;             	    //2
	unsigned short int 		down_count;                    	//2
	unsigned int    		last_reset_time;                //4
	unsigned char 			work_mode;                      //1
//	unsigned char 			status_sensor_on_off;           //1
	unsigned int 			utc_time;                       //4
	unsigned short int 		tmep_hk;                        //2
    unsigned int            on_off_status;                  //4
    unsigned char           mindex;                         //1
    unsigned char           aindex;                         //1
//    unsigned short int      mindex;                         //2
//    unsigned short int      aindex;                         //2


//	//uv    41
//	unsigned short int 		rec_buffer_cnt;                 //2
//	unsigned char   		rec_status[14];                 //14
//	unsigned char   		rec_time[4];                    //4
//
//	unsigned char 			send_status_cur[8];             //8
//	unsigned char 			send_status_pre[8];             //8
//	unsigned char 			send_time[4];                   //4
//	unsigned char 			send_work_status;               //1

	//eps 64
	short int 				temp_batt_board[2];             //4
	short int 				temp_eps[4];                    //8
	unsigned short int 				sun_c[6];               //12
	unsigned short int 				sun_v[6];               //12

	unsigned short int 				out_BusC;               //2
	unsigned short int 				out_BusV;               //2
	unsigned short int 				UV_board_C;             //2
	unsigned short int 				Vol_5_C[6];             //12
	unsigned short int 				Bus_c[5];               //10
	unsigned char                   others[66];
	//gps   28
//	unsigned short int 		gps_week;                       //2
//	unsigned short int 		gps_pdop;                       //2
//	float 					gps_posi[3];                    //12    3*4
//	short int  				gps_vel[3];                     //6     3*2
//	float 					gps_time;                       //4
//	unsigned char 			gps_status;                     //1
//	unsigned char 			gps_star_num;                   //1


	//tail 2
//	unsigned char 			endbit;                         //'*'
//	uint32_t				crc;							//1
}HK_Main_t;

//typedef  struct __attribute__((packed)) {
//	uint8_t 		adcs_ctrl_mode;           //downAdcsmagDotDmpFlg downAdcspitFltComFlg downAdcsattStaFlg
//	uint16_t		downAdcscntDmp;
//	uint16_t        downAdcscntPitcom;
//	uint16_t		downAdcscntAttSta;
//
//	int16_t			downAdcsPitAngle;
//	int16_t			downAdcsPitFltState[2];
//	float			downAdcsPitFltNormP;
//
//	int16_t			downAdcsMagnetometer[3];	//double to int16_t  原始值个位不要 除以10取整
//
//	int16_t			downAdcsMagInO[3];			//double to int16_t  原始值个位不要 除以10取整
//
//	uint16_t		downAdcsWheelSpeed_Meas;	//取整
//
//	int16_t			downAdcsMTQOut[3];			// *1000
//
//	float			downAdcsOrbPos[3]; 			// 位置downAdcsOrb[0-2]  速度downAdcsOrb[3-5]
//	int16_t			downAdcsOrbVel[3];			//取整
//
//}adcs_info_t;
//
//typedef  struct __attribute__((packed)) {
//	//uint8_t			header[2];
//	uint16_t		rst_cnt;
//	uint16_t		rcv_cnt;
//	uint16_t		ack_cnt;
//	uint32_t		rst_time;
//	uint16_t		sw_status;
//	uint32_t		utc_time;
//	uint16_t		cpu_temp;
//	adcs_info_t		adcs_info;
//	int16_t			adc[10];
//	uint8_t 		error;
////	uint8_t			index;
////	uint32_t		crc;
//}adcs_hk_t;

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
        uint8_t		 	header[2];   //0x1a 0x51
	adcs805_hk_t		adcs_hk;
}HK_Append_t;

typedef struct __attribute__((packed)) {
	HK_Main_t 	main_frame;
	HK_Append_t append_frame;
}HK_Store_t;

extern HK_Store_t		hk_frame;
extern HK_Store_t		hk_old_frame;

extern uint8_t 			hk_select;
extern uint16_t			hk_sram_index;
extern uint32_t			hk_sd_time;
extern uint8_t 			hk_sd_path[25];
extern FIL 				hkfile;
extern UINT				hkrbytes;
extern uint32_t			hkleek;

extern uint16_t  hk_frame_index;
extern HK_Fifo_t hk_main_fifo __attribute__((section(".hk")));
extern HK_Fifo_t hk_append_fifo __attribute__((section(".hk")));
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
int hk_collect_no_store(void);
int hk_collect(void);
int hk_store_init(void);
void hk_out(void);
int hk_store_add(void);
void hk_file_task(void * paragram);
void vTelemetryFileManage(void * paragram);


int hk_collect_test(void);


#endif /* SRC_HK_H_ */
