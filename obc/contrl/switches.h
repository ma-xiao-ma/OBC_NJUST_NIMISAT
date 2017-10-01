/*
 * switches.h
 *
 *  Created on: 2016年5月10日
 *      Author: Administrator
 */

#ifndef CONTRL_SWITCHES_H_
#define CONTRL_SWITCHES_H_

#include "hk.h"

//#define M1_POWER_MASK			0x01
//#define M2_POWER_MASK			0x02
//#define M3_POWER_MASK			0x04
//#define M4_POWER_MASK			0x08
//#define OUT_EN_5V				0x10
//#define GR_POWER_MASK			0x20
//#define MAG_POWER_MASK		0x40
//#define GPS_POWER_MASK		0x80
//#define MAGA_EN_MASK			0x01
//#define MAGB_EN_MASK			0x02
//#define GPSA_EN_MASK			0x04
//#define GPSB_EN_MASK			0x08

/*第一个字节*/
#define ANTS1					(0x01<<0)
#define ANTS2					(0x01<<1)
#define ANTS3					(0x01<<2)
#define ANTS4					(0x01<<3)
#define ARM						(0x01<<4)
#define ANTSMSK					(0x1F)
#define PANELA					(0x01<<5)
#define PANELB					(0x01<<6)

/*第二个字节*/
#define ADCS_EN					(0x01<<0)
#define ANTS_EN					(0x01<<1)
#define DIGI_TRAN_5V_EN			(0x01<<2)
#define DIGI_TRAN_12V_EN		(0x01<<3)
#define CAMERA_10W_5V_EN		(0x01<<4)
#define CAMERA_5W_5V_EN		    (0x01<<5)
#define CAMERA_HEAT1_EN			(0x01<<6)
#define CAMERA_HEAT2_EN         (0x01<<7)

/*第三个字节*/
//0x21
#define M1_POWER_MASK			0x01
#define M2_POWER_MASK			0x02
#define M3_POWER_MASK			0x04
#define M4_POWER_MASK			0x08
#define OUT_EN_5V				0x10
#define GR_POWER_MASK			0x20
//0x20
#define MAG_POWER_MASK			0x40
#define GPS_POWER_MASK			0x80
/*第四个字节*/
//0x20
#define MAGA_EN_MASK			0x01
#define MAGB_EN_MASK			0x02
#define GPSA_EN_MASK			0x04
#define GPSB_EN_MASK			0x08
//0x21
#define MAGBAR_EN_MASK			0x10


//#define OUT_GPS_5V				OUT_5_3_3V_1
//#define OUT_HEAT_7V				OUT_7_4V_1
#define OUT_ADCS_7V				OUT_EPS_S0
#define OUT_ANTS_3V				OUT_EPS_S1
//#define OUT_FIPEX_5V			OUT_EPS_S2
//#define OUT_FIPEX_3V			OUT_EPS_S3
//#define OUT_PANEL_7V			OUT_SOLAR_EN
//读取引脚高低电平状态
//#define OUT_GPS_5V_PIN()		SW_5_3_3V_1_PIN()
//#define OUT_HEAT_7V_PIN()		SW_7_4V_1_PIN()
#define OUT_ADCS_7V_PIN()		SW_EPS_S0_PIN()
#define OUT_ANTS_3V_PIN()	    SW_EPS_S3_PIN()
//#define OUT_FIPEX_5V_PIN()		SW_EPS_S1_PIN()
//#define OUT_FIPEX_3V_PIN()		SW_EPS_S3_PIN()
#define OUT_PANEL_7V_PIN()		SW_SOLAR_EN_PIN()

//#define DISABLE_GPS_5V			SW_5_3_3V_1_DISABLE
//#define DISABLE_HEAT_7V			SW_7_4V_1_DISABLE
#define DISABLE_ADCS_7V			SW_EPS_S0_DISABLE
#define DISABLE_ANTS_3V			SW_EPS_S1_DISABLE
//#define DISABLE_FIPEX_5V		SW_EPS_S2_DISABLE
//#define DISABLE_FIPEX_3V		SW_EPS_S3_DISABLE
#define DISABLE_PANEL_7V		SW_SOLAR_EN_DISABLE

//#define ENABLE_GPS_5V			SW_5_3_3V_1_ENABLE
//#define ENABLE_HEAT_7V			SW_7_4V_1_ENABLE
#define ENABLE_ADCS_7V			SW_EPS_S0_ENABLE
#define ENABLE_ANTS_3V			SW_EPS_S1_ENABLE
//#define ENABLE_FIPEX_5V			SW_EPS_S2_ENABLE
//#define ENABLE_FIPEX_3V			SW_EPS_S3_ENABLE
#define ENABLE_PANEL_7V			SW_SOLAR_EN_ENABLE

//#define GPIO_PANELA_PORT	    GPIOB
//#define RCC_PANELA_PORT      	RCC_AHB1Periph_GPIOB
//#define PANELA_CLK_INIT         RCC_APB1PeriphClockCmd
//#define GPIO_PANELA_PIN			GPIO_Pin_12
//#define PANELA_PIN_STATUS()		GPIO_ReadOutputDataBit(GPIO_PANELA_PORT, GPIO_PANELA_PIN)

//#define GPIO_PANELB_PORT	    GPIOB
//#define RCC_PANELB_PORT      	RCC_AHB1Periph_GPIOB
//#define PANELB_CLK_INIT         RCC_APB1PeriphClockCmd
//#define GPIO_PANELB_PIN			GPIO_Pin_13
//#define PANELB_PIN_STATUS()		GPIO_ReadOutputDataBit(GPIO_PANELB_PORT, GPIO_PANELA_PIN)

int get_switch_status(uint8_t * pstatus);

int open_antenna(void);

int enable_antspwr(uint32_t delay, uint32_t data __attribute__((unused)));

int disable_antspwr(uint32_t delay, uint32_t data __attribute__((unused)));

uint8_t get_antenna_status_nopara(void);

uint8_t get_antenna_status(uint8_t * sta);

uint8_t get_panel_status(void);

int enable_panel(uint32_t delay, uint32_t data __attribute__((unused)));

int disable_panel(uint32_t delay, uint32_t data __attribute__((unused)));

int obc_closeall(uint32_t delay, uint32_t data __attribute__((unused)));

#endif /* CONTRL_SWITCHES_H_ */
