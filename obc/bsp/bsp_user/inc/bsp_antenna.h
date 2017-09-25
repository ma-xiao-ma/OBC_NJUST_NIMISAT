////////////////////////////////////////////////////////////////////////////////
//	功能： 通过I2C接口实现ISIS天线展开功能头文件
//
//	版本：V1.0
//  迭代：
//												南京理工大学微纳卫星中心
//												   2015.11.08
////////////////////////////////////////////////////////////////////////////////

#ifndef __ANTENNA_H_
#define	__ANTENNA_H_

#include "stdint.h"

///////////////////////////////ISIS天线I2C地址////////////////////////////////////
#define ISIS_ANTENA_ADDR		0x31

#define ISIS_ANTTENNA_RETRY			5	//展开尝试次数
#define ISIS_BURN_TIME_INIT			5	//展开超时时间 秒
///////////////////////////////ISIS天线指令//////////////////////////////////////
#define 	ANTENA_CMD_RESET		0xAA		//重启
#define 	ANTENA_CMD_ARM			0xAD		//ARM
#define 	ANTENA_CMD_DISARM		0xAC		//去ARM

#define 	ANTENA_CMD_DEPLOY1		0xA1		//天线1展开
#define 	ANTENA_CMD_DEPLOY2		0xA2		//天线2展开
#define 	ANTENA_CMD_DEPLOY3		0xA3		//天线3展开
#define 	ANTENA_CMD_DEPLOY4		0xA4		//天线4展开
#define 	ANTENA_CMD_DEPLOY_SEQ	0xA5
#define 	ANTENA_CMD_DEPLOY1_OW	0xBA
#define 	ANTENA_CMD_DEPLOY2_OW	0xBB
#define 	ANTENA_CMD_DEPLOY3_OW	0xBC
#define 	ANTENA_CMD_DEPLOY4_OW	0xBD
#define 	ANTENA_CMD_DEPLOY1_CNT	0xB0		//天线1
#define 	ANTENA_CMD_DEPLOY2_CNT	0xB1
#define 	ANTENA_CMD_DEPLOY3_CNT	0xB2
#define 	ANTENA_CMD_DEPLOY4_CNT	0xB3

#define 	ANTENA_CMD_DEPLOY1_TIME	0xB4
#define 	ANTENA_CMD_DEPLOY2_TIME	0xB5
#define 	ANTENA_CMD_DEPLOY3_TIME	0xB6
#define 	ANTENA_CMD_DEPLOY4_TIME	0xB7

#define 	ANTENA_CMD_DEPLOY_ESC	0xA9
#define 	ANTENA_CMD_TEMP			0xC0
#define 	ANTENA_CMD_DEPLOY_STAT	0xC3

#define 	ANTENNA_STA_1			12				
#define 	ANTENNA_STA_2			8	
#define 	ANTENNA_STA_3			4	
#define 	ANTENNA_STA_4			0

#define		ANTENNA_STA_DEPLOY		0x08
#define 	ANTENNA_STA_LAST		0x04
#define 	ANTENNA_STA_ACTIVE		0x02

#define		ANNTENNA_ARGS(x,y)		((x) << (y))

#define 	ANNTENNA_STA_ARMED		0x0001


uint8_t bsp_AntennaTemperatureGet(uint16_t* data);
uint8_t bsp_AntennaReset(void);
uint8_t bsp_AntennaArm(uint8_t op);
uint8_t bsp_AntennaDeploy(uint8_t number, uint8_t time);
uint8_t bsp_AntennaDeployOver(uint8_t number, uint8_t time);
uint8_t bsp_AntennaDeployStop(void);
uint8_t bsp_AntennaGetStatus(uint16_t* data);
uint8_t bsp_AntennaGetDeployCnt(uint8_t number, uint8_t* data);
uint8_t bsp_AntennaGetDeployTime(uint8_t number, uint16_t* data);
uint8_t bsp_AntennaDeployAuto(void);

#endif

