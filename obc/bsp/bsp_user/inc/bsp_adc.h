////////////////////////////////////////////////////////////////////////////////
//	功能：电源板电量、温度采集校正头文件
//
//	版本：V1.0
//  迭代：
//												南京理工大学微纳卫星中心
//												   2015.11.08
////////////////////////////////////////////////////////////////////////////////
#ifndef __BSP_ADC_H
#define __BSP_ADC_H

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define CALIBRATION_TIME	20

#define SV_NUM          6
#define REG_NUM         6
#define UREG_NUM 		5
#define EPS_TEMP_NUM	4
#define BAT_TEMP_NUM	2

#define EPS_ADC_NUM		32

#define EPS_FLAG_OK		0x01
#define EPS_FLAG_ENTER	0x02
#define EPS_FLAG_DOING	0x04
#define EPS_FLAG_IDLE	0x08



#define TEMP_CS_GPIO		GPIOF			//OBC板温度采集片选
#define TEMP_CS_PIN			GPIO_Pin_11
#define TEMP_CS_LOW()      	TEMP_CS_GPIO->BSRRH = TEMP_CS_PIN
#define TEMP_CS_HIGH()     	TEMP_CS_GPIO->BSRRL = TEMP_CS_PIN


#define OBC_CS_GPIO		    GPIOB			//OBC板模拟采样片选
#define OBC_CS_PIN			GPIO_Pin_1
#define OBC_CS_LOW()       	OBC_CS_GPIO->BSRRH = OBC_CS_PIN
#define OBC_CS_HIGH()      	OBC_CS_GPIO->BSRRL = OBC_CS_PIN
#define OBC_SPI_BAUD		SPI_BaudRatePrescaler_128


#define EPS_CS1_GPIO		GPIOA			//EPS板模拟采样片选
#define EPS_CS1_PIN			GPIO_Pin_4
#define EPS_CS1_LOW()      	EPS_CS1_GPIO->BSRRH = EPS_CS1_PIN
#define EPS_CS1_HIGH()     	EPS_CS1_GPIO->BSRRL = EPS_CS1_PIN


#define EPS_CS2_GPIO		GPIOA			//EPS板模拟采样片选2
#define EPS_CS2_PIN			GPIO_Pin_15
#define EPS_CS2_LOW()      	EPS_CS2_GPIO->BSRRH = EPS_CS2_PIN
#define EPS_CS2_HIGH()     	EPS_CS2_GPIO->BSRRL = EPS_CS2_PIN


#define EPS_AD_CS1          0x01			//OBC板模拟采样极值
#define EPS_AD_CS2          0x02
#define OBC_AD_CS1          0x03
#define TEMP_AD_CS1			0x04



#define SUN_NUM				6
#define OBC_TEMP_NUM		2
#define UN_USE_NUM			6

#define OBC_ADC_NUM			16

typedef struct
{
	uint16_t	InSunSensorV[SUN_NUM];
	uint16_t 	InObc5V;
	uint16_t 	InObc3V3;
	int16_t 	ObcTemp[OBC_TEMP_NUM];
	uint16_t	InAdcRes[UN_USE_NUM];
}ObcAdcValue_t;

typedef struct CaliFactor_t
{
	float 		k;
	int16_t 	b;
}CaliFactor_t;	


typedef struct EpsAdcValue_t
{
	uint16_t 	In_SunC[SV_NUM];
	uint16_t 	In_SunV[SV_NUM];
	uint16_t 	Out_ComC;
	uint16_t 	Out_BusC;
	uint16_t 	Out_BusV;
	uint16_t 	Out_BranchC[REG_NUM+UREG_NUM];
	int16_t 	EpsTemp[EPS_TEMP_NUM];
	int16_t 	BatTemp[BAT_TEMP_NUM];
} EpsAdcValue_t;


extern EpsAdcValue_t	EpsHouseKeeping;

void bsp_InitSPI1(void);
EpsAdcValue_t* EpsAdToReal(uint16_t* rec, EpsAdcValue_t* tar);
void EpsAdToReal2(uint16_t* rec, EpsAdcValue_t* tar);
void EpsAdCalibration(void);

void eps_start(void);
void eps_hk(void);
void cmd_eps_setup(void);


#endif
