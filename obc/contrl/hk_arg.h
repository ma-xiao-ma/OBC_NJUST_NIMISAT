/*
 * hk.h
 *
 *  Created on: 2016骞�5鏈�13鏃�
 *      Author: Administrator
 */

#ifndef CONTRL_HK_ARG_H_
#define CONTRL_HK_ARG_H_

#include <stdint.h>

#include "bsp_adc.h"
#include "sensor/flash_sd.h"

#define OBC_STORE_ADDR				((uint32_t)(0x080C0000))

extern double mag_get[3];
extern float magtemparature;

extern uint32_t antenna_status;
extern uint32_t battery_status;

extern flash_store_t store_info;

extern uint16_t ad7490_data[16];

extern unsigned int obc_boot_count;
extern uint32_t     obc_reset_time;

extern uint8_t up_hk_down_cmd;

extern uint32_t StorageIntervalCount;
extern uint32_t Stop_Down_Time;		//鍗槦杩涚珯鍚庯紝姣�4s涓嬭涓�娆℃暟鎹紝鎸佺画 8 min 鍒╃敤4绉掑畾鏃讹紝120娆″嵆涓�8鍒嗛挓
extern uint32_t downtimeset;	//鍗曚綅ms
//extern uint32_t saveinterval;

extern uint8_t openpanel_times;
extern uint32_t antenna_status;
extern uint32_t panel_status;

extern flash_store_t store_info;

extern EpsAdcValue_t	EpsHouseKeeping;

extern ObcAdcValue_t 	ObcAdData;
extern uint8_t 			EpsCaliFlag;
extern int16_t 			EpsCaliTable[REG_NUM + UREG_NUM];

extern uint16_t TempAdValue[16][5];              	//鐢垫簮鏉挎俯搴﹂噰鏍风偣
extern uint16_t TempAdValueAver[16];			  	//鐢垫簮鏉挎俯搴﹀潎鍊�
extern uint16_t ObcAdValue[16][5];              	//鐢垫簮鏉跨數鍘嬮噰鏍风偣
extern uint16_t ObcAdValueAver[16];					//鐢垫簮鏉跨數鍘嬪潎鍊�
extern uint16_t EpsAdValue[32][5];   				//鐢垫簮鏉跨數婧愯幏鍙栭噺閲囨牱鐐�
extern uint16_t EpsAdValueAver[32];					//鐢垫簮鏉跨數婧愯幏鍙栭噺鍧囧��

extern uint8_t EpsAdCorrectEnable;
extern EpsAdcValue_t EpsHouseKeeping;
extern uint16_t EpsTranOTCnt;	//鍙戦�佽秴鏃惰鏁�
extern uint16_t EpsRevOTCnt;   //鎺ユ敹瓒呮椂璁℃暟

extern uint8_t updateTimeFlag;
extern uint8_t AdcsGpsUse;

extern uint8_t mode;

#endif /* CONTRL_HK_ARG_H_ */
