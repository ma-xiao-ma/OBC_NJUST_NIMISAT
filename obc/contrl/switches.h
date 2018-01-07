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
//#define MAG_POWER_MASK		    0x40
//#define GPS_POWER_MASK		    0x80
//#define MAGA_EN_MASK			0x01
//#define MAGB_EN_MASK			0x02
//#define GPSA_EN_MASK			0x04
//#define GPSB_EN_MASK			0x08

/*第一个字节*/
#define ANTS1                   (0x01<<0)
#define ANTS2                   (0x01<<1)
#define ANTS3                   (0x01<<2)
#define ANTS4                   (0x01<<3)
#define ARM                     (0x01<<4)
#define ANTSMSK                 (0x1F)
/*帆板展开分离开关*/
#define PANELA                  (0x01<<5)
#define PANELB                  (0x01<<6)
/*电池加热*/
#define BATTERY_HEAT_EN         (0x01<<7) /*新增*/

/*第二个字节*/
#define ADCS_EN                 (0x01<<0)
#define ANTS_EN                 (0x01<<1)
#define DTB_5V_EN               (0x01<<2)
#define DTB_12V_EN              (0x01<<3)
#define CAMERA_10W_5V_EN        (0x01<<4)
#define CAMERA_5W_5V_EN         (0x01<<5)
#define CAMERA_HEAT1_EN         (0x01<<6)
#define CAMERA_HEAT2_EN         (0x01<<7)

/*第三个字节*/
#define JLG_VU_BUS_EN           (0x01<<0) /*新增*/
#define SW_VU_5V_EN             (0x01<<1) /*新增*/
/*可展开帆展开检测*/
#define EXPANDABLE_SAIL         (0x01<<2) /*新增*/




/*OBC IO引脚状态，4 Byte*/
typedef struct __attribute__((packed))
{
    /**ISIS天线1状态*/
    uint32_t     ants_1: 1;              //W0B0 最低字节的最低位
    /**ISIS天线2状态*/
    uint32_t     ants_2: 1;              //W0B1
    /**ISIS天线3状态*/
    uint32_t     ants_3: 1;              //W0B2
    /**ISIS天线4状态*/
    uint32_t     ants_4: 1;              //W0B3
    /**ISIS天线板ARM状态*/
    uint32_t     arm: 1;                 //W0B4
    /**电池阵A展开状态*/
    uint32_t     panel_a: 1;             //W0B5
    /**电池阵B展开状态*/
    uint32_t     panel_b: 1;             //W0B6
    /**电池加热开关*/
    uint32_t     battery_heat_on: 1;     //W0B7


    /**姿控系统电源开关*/
    uint32_t     adcs_pwr: 1;            //W1B0
    /**天线系统电源开关*/
    uint32_t     ants_pwr: 1;            //W1B1
    /**数传机5V电开关*/
    uint32_t     dtb_5v_pwr: 1;          //W1B2
    /**数传机12V电源开关*/
    uint32_t     dtb_12v_pwr: 1;         //W1B3
    /**805相机5V_5W电开关*/
    uint32_t     cam_5w_5v_pwr: 1;       //W1B4
    /**805相机5V_10W电开关*/
    uint32_t     cam_10w_5v_pwr: 1;      //W1B5
    /**805相机加热1开关*/
    uint32_t     cam_heat_1_pwr: 1;      //W1B6
    /**805相机加热2开关*/
    uint32_t     cam_heat_2_pwr: 1;      //W1B7

    /**备份通信机电源开关*/
    uint32_t     jlg_vu_on: 1;              //W3B0
    /**通信机信道切换板电源开关*/
    uint32_t     switch_vu_on: 1;           //W3B1
    /**展开帆 展开状态*/
    uint32_t     sail: 1;                   //W3B2
    /**保留位域1*/
    uint32_t     reserved1: 5;              //W3B3~W3B7


    /**保留位域2*/
    uint32_t     reserved2: 8;              //W4
} obc_switch_t;

/**工作模式*/
typedef enum {
    OFF = 0,
    ON  = 1
} switch_state;

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

/*使能编号重定义*/
#define OUT_ADCS_7V             OUT_EPS_S0
#define OUT_HEAT_5V				OUT_EPS_S1
#define OUT_CAM_5V              OUT_EPS_S2
#define OUT_ANTS_3V				OUT_EPS_S3
#define OUT_SAIL_7V             OUT_7_4V_2
#define OUT_PANEL_7V			OUT_SOLAR_EN
#define OUT_BACKUP_VU           OUT_USB_EN
#define OUT_SWITCH_5V           OUT_5_3_3V_3


//读取引脚高低电平状态
#define OUT_ADCS_7V_PIN()		SW_EPS_S0_PIN()
#define OUT_HEAT_5V_PIN()       SW_EPS_S1_PIN()
#define OUT_CAM_5V_PIN()        SW_EPS_S2_PIN()
#define OUT_ANTS_3V_PIN()	    SW_EPS_S3_PIN()

#define OUT_PANEL_7V_PIN()		SW_SOLAR_EN_PIN()
#define OUT_BACKUP_VU_PIN()     SW_USB_EN_PIN()
#define OUT_SW_VU_5V_PIN()      SW_5_3_3V_3_PIN()
#define OUT_SAIL_7V_PIN()       SW_7_4V_2_PIN()

//#define DISABLE_GPS_5V			SW_5_3_3V_1_DISABLE
//#define DISABLE_HEAT_7V			SW_7_4V_1_DISABLE
#define DISABLE_ADCS_7V			SW_EPS_S0_DISABLE
#define DISABLE_HEAT_5V		    SW_EPS_S1_DISABLE
#define DISABLE_CAM_5V		    SW_EPS_S2_DISABLE
#define DISABLE_ANTS_3V		    SW_EPS_S3_DISABLE
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

/**
 * 可展开电池阵展开系统使能
 *
 * @param delay 展开系统烧线使能时间（单位：毫秒）
 * @return 返回E_NO_ERR(-1)为成功
 */
int enable_panel(uint32_t delay);

/**
 * 配有检测开关的电池阵展开函数
 *
 * @param SafeTime_s 烧线的最长时间 （单位：秒）
 * @return 返回E_NO_ERR（-1）为正常
 */
int Solar_Array_Unfold(uint8_t SafeTime_s);

/**
 * 离轨帆展开
 *
 * @return 返回E_NO_ERR（-1）为正常
 */
int Sail_Unfold(void);

int disable_panel(uint32_t delay, uint32_t data __attribute__((unused)));

int obc_closeall(uint32_t delay, uint32_t data __attribute__((unused)));

#endif /* CONTRL_SWITCHES_H_ */
