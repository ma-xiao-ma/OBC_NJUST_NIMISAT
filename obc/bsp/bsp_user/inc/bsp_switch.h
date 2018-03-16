////////////////////////////////////////////////////////////////////////////////
//	功能：电源板各开关接口控制头文件
//
//	版本：V1.0
//  迭代：
//												南京理工大学微纳卫星中心
//												   2015.11.26
////////////////////////////////////////////////////////////////////////////////
#ifndef __BSP_SWITCH_H_
#define __BSP_SWITCH_H_

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "stm32f4xx.h"

#define EPS_OK			0x00
#define EPS_ERROR		0x01
#define EPS_UNDO		0x02
#define EPS_IDLE		0x04

//电源板上的各传感器开关通道号
#define OUT_5_3_3V_1			0
#define OUT_5_3_3V_2			1
#define OUT_5_3_3V_3			2
#define OUT_7_4V_1				3
#define OUT_7_4V_2				4
#define OUT_7_4V_3				5
#define OUT_USB_EN				6
#define OUT_EPS_S0				7
#define OUT_EPS_S1				8
#define OUT_EPS_S2				9
#define OUT_EPS_S3				10
#define OUT_SOLAR_EN			11

#define OUT_DTB_5V		        12
#define OUT_DTB_12V		        13
#define OUT_CAMERA_10W			14
#define OUT_CAMERA_5W			15
#define OUT_CAMERA_HEAT_1		16
#define OUT_CAMERA_HEAT_2		17

#define OUT_ALL					20

#define GPIO_5_3_3V_1_PORT	    GPIOE
#define RCC_5_3_3V_1_PORT      	RCC_AHB1Periph_GPIOE
#define GPIO_5_3_3V_1_PIN		GPIO_Pin_6

#define GPIO_5_3_3V_2_PORT	    GPIOE
#define RCC_5_3_3V_2_PORT      	RCC_AHB1Periph_GPIOE
#define GPIO_5_3_3V_2_PIN		GPIO_Pin_2

#define GPIO_5_3_3V_3_PORT	    GPIOA
#define RCC_5_3_3V_3_PORT      	RCC_AHB1Periph_GPIOA
#define GPIO_5_3_3V_3_PIN		GPIO_Pin_9

#define GPIO_7_4V_1_PORT	    GPIOA
#define RCC_7_4V_1_PORT      	RCC_AHB1Periph_GPIOA
#define GPIO_7_4V_1_PIN			GPIO_Pin_8

#define GPIO_7_4V_2_PORT	    GPIOB
#define RCC_7_4V_2_PORT      	RCC_AHB1Periph_GPIOB
#define GPIO_7_4V_2_PIN			GPIO_Pin_3

#define GPIO_7_4V_3_PORT	    GPIOB
#define RCC_7_4V_3_PORT      	RCC_AHB1Periph_GPIOB
#define GPIO_7_4V_3_PIN			GPIO_Pin_4

#define GPIO_SOLAR_EN_PORT	    GPIOG
#define RCC_SOLAR_EN_PORT      	RCC_AHB1Periph_GPIOG
#define GPIO_SOLAR_EN_PIN		GPIO_Pin_14

#define GPIO_USB_EN_PORT	    GPIOG
#define RCC_USB_EN_PORT      	RCC_AHB1Periph_GPIOG
#define GPIO_USB_EN_PIN			GPIO_Pin_15

#define GPIO_EPS_S0_PORT	    GPIOD
#define RCC_EPS_S0_PORT      	RCC_AHB1Periph_GPIOD
#define GPIO_EPS_S0_PIN			GPIO_Pin_3

#define GPIO_EPS_S1_PORT	    GPIOG
#define RCC_EPS_S1_PORT      	RCC_AHB1Periph_GPIOG
#define GPIO_EPS_S1_PIN			GPIO_Pin_8

#define GPIO_EPS_S2_PORT	    GPIOG
#define RCC_EPS_S2_PORT      	RCC_AHB1Periph_GPIOG
#define GPIO_EPS_S2_PIN			GPIO_Pin_7

#define GPIO_EPS_S3_PORT	    GPIOG
#define RCC_EPS_S3_PORT      	RCC_AHB1Periph_GPIOG
#define GPIO_EPS_S3_PIN			GPIO_Pin_6


/*   805   */
#define GPIO_DTB_5V_PORT	GPIOF
#define RCC_DTB_5V_PORT   RCC_AHB1Periph_GPIOF
#define GPIO_DTB_5V_PIN	GPIO_Pin_6

#define GPIO_DTB_12V_PORT	GPIOF
#define RCC_DTB_12V_PORT  RCC_AHB1Periph_GPIOF
#define GPIO_DTB_12V_PIN	GPIO_Pin_7

#define GPIO_CAMERA_10W_PORT	GPIOF
#define RCC_CAMERA_10W_PORT     RCC_AHB1Periph_GPIOF
#define GPIO_CAMERA_10W_PIN	    GPIO_Pin_8

#define GPIO_CAMERA_5W_PORT	    GPIOF
#define RCC_CAMERA_5W_PORT      RCC_AHB1Periph_GPIOF
#define GPIO_CAMERA_5W_PIN	    GPIO_Pin_9

#define GPIO_CAMERA_HEAT_1_PORT	GPIOF
#define RCC_CAMERA_HEAT_1_PORT  RCC_AHB1Periph_GPIOF
#define GPIO_CAMERA_HEAT_1_PIN	GPIO_Pin_10

#define GPIO_CAMERA_HEAT_2_PORT	GPIOC
#define RCC_CAMERA_HEAT_2_PORT  RCC_AHB1Periph_GPIOC
#define GPIO_CAMERA_HEAT_2_PIN	GPIO_Pin_2

#define GPIO_PAL_STATUS_1_PORT  GPIOC
#define RCC_PAL_STATUS_1_PORT   RCC_AHB1Periph_GPIOC
#define GPIO_PAL_STATUS_1_PIN	GPIO_Pin_3

#define GPIO_PAL_STATUS_2_PORT  GPIOC
#define RCC_PAL_STATUS_2_PORT   RCC_AHB1Periph_GPIOC
#define GPIO_PAL_STATUS_2_PIN	GPIO_Pin_4

//#define GPIO_SAIL_STATUS_PORT   GPIOC
//#define RCC_SAIL_STATUS_PORT    RCC_AHB1Periph_GPIOC
//#define GPIO_SAIL_STATUS_PIN    GPIO_Pin_2


//读取引脚高低电平状态
#define SW_5_3_3V_1_PIN()           GPIO_ReadOutputDataBit(GPIO_5_3_3V_1_PORT, GPIO_5_3_3V_1_PIN)
#define SW_5_3_3V_2_PIN()           GPIO_ReadOutputDataBit(GPIO_5_3_3V_2_PORT, GPIO_5_3_3V_2_PIN)
#define SW_5_3_3V_3_PIN()           GPIO_ReadOutputDataBit(GPIO_5_3_3V_3_PORT, GPIO_5_3_3V_3_PIN)

#define SW_7_4V_1_PIN()       	    GPIO_ReadOutputDataBit(GPIO_7_4V_1_PORT, GPIO_7_4V_1_PIN)
#define SW_7_4V_2_PIN()       	    GPIO_ReadOutputDataBit(GPIO_7_4V_2_PORT, GPIO_7_4V_2_PIN)
#define SW_7_4V_3_PIN()       	    GPIO_ReadOutputDataBit(GPIO_7_4V_3_PORT, GPIO_7_4V_3_PIN)

#define SW_SOLAR_EN_PIN()           GPIO_ReadOutputDataBit(GPIO_SOLAR_EN_PORT, GPIO_SOLAR_EN_PIN)

#define SW_USB_EN_PIN()       	    GPIO_ReadOutputDataBit(GPIO_USB_EN_PORT, GPIO_USB_EN_PIN)

#define SW_EPS_S0_PIN()       	    GPIO_ReadOutputDataBit(GPIO_EPS_S0_PORT, GPIO_EPS_S0_PIN)
#define SW_EPS_S1_PIN()       	    GPIO_ReadOutputDataBit(GPIO_EPS_S1_PORT, GPIO_EPS_S1_PIN)
#define SW_EPS_S2_PIN()       	    GPIO_ReadOutputDataBit(GPIO_EPS_S2_PORT, GPIO_EPS_S2_PIN)
#define SW_EPS_S3_PIN()       	    GPIO_ReadOutputDataBit(GPIO_EPS_S3_PORT, GPIO_EPS_S3_PIN)

/*805*/
#define OUT_SW_DTB_5V_PIN()         GPIO_ReadOutputDataBit(GPIO_DTB_5V_PORT, GPIO_DTB_5V_PIN)
#define OUT_SW_DTB_12V_PIN()        GPIO_ReadOutputDataBit(GPIO_DTB_12V_PORT, GPIO_DTB_12V_PIN)
#define OUT_SW_CAMERA_10W_PIN()     GPIO_ReadOutputDataBit(GPIO_CAMERA_10W_PORT, GPIO_CAMERA_10W_PIN)
#define OUT_SW_CAMERA_5W_PIN()      GPIO_ReadOutputDataBit(GPIO_CAMERA_5W_PORT, GPIO_CAMERA_5W_PIN)
#define OUT_SW_CAMERA_HEAT_1_PIN()  GPIO_ReadOutputDataBit(GPIO_CAMERA_HEAT_1_PORT, GPIO_CAMERA_HEAT_1_PIN)
#define OUT_SW_CAMERA_HEAT_2_PIN()  GPIO_ReadOutputDataBit(GPIO_CAMERA_HEAT_2_PORT, GPIO_CAMERA_HEAT_2_PIN)
#define IN_SW_PAL_STATUS_1_PIN()    GPIO_ReadInputDataBit(GPIO_PAL_STATUS_1_PORT, GPIO_PAL_STATUS_1_PIN)
#define IN_SW_PAL_STATUS_2_PIN()    GPIO_ReadInputDataBit(GPIO_PAL_STATUS_2_PORT, GPIO_PAL_STATUS_2_PIN)

/*恩来*/
//#define IN_SW_SAIL_STATUS_PIN()     GPIO_ReadInputDataBit(GPIO_SAIL_STATUS_PORT, GPIO_SAIL_STATUS_PIN)

//设定开关量状态
#define SW_5_3_3V_1_ENABLE      GPIO_SetBits(GPIO_5_3_3V_1_PORT, GPIO_5_3_3V_1_PIN)
#define SW_5_3_3V_1_DISABLE     GPIO_ResetBits(GPIO_5_3_3V_1_PORT, GPIO_5_3_3V_1_PIN)

#define SW_5_3_3V_2_ENABLE      GPIO_SetBits(GPIO_5_3_3V_2_PORT, GPIO_5_3_3V_2_PIN)
#define SW_5_3_3V_2_DISABLE     GPIO_ResetBits(GPIO_5_3_3V_2_PORT, GPIO_5_3_3V_2_PIN)

#define SW_5_3_3V_3_ENABLE      GPIO_SetBits(GPIO_5_3_3V_3_PORT, GPIO_5_3_3V_3_PIN)
#define SW_5_3_3V_3_DISABLE     GPIO_ResetBits(GPIO_5_3_3V_3_PORT, GPIO_5_3_3V_3_PIN)

#define SW_7_4V_1_ENABLE      	GPIO_SetBits(GPIO_7_4V_1_PORT, GPIO_7_4V_1_PIN)
#define SW_7_4V_1_DISABLE     	GPIO_ResetBits(GPIO_7_4V_1_PORT, GPIO_7_4V_1_PIN)

#define SW_7_4V_2_ENABLE      	GPIO_SetBits(GPIO_7_4V_2_PORT, GPIO_7_4V_2_PIN)
#define SW_7_4V_2_DISABLE     	GPIO_ResetBits(GPIO_7_4V_2_PORT, GPIO_7_4V_2_PIN)

#define SW_7_4V_3_ENABLE      	GPIO_SetBits(GPIO_7_4V_3_PORT, GPIO_7_4V_3_PIN)
#define SW_7_4V_3_DISABLE     	GPIO_ResetBits(GPIO_7_4V_3_PORT, GPIO_7_4V_3_PIN)

#define SW_SOLAR_EN_ENABLE      GPIO_SetBits(GPIO_SOLAR_EN_PORT, GPIO_SOLAR_EN_PIN)
#define SW_SOLAR_EN_DISABLE     GPIO_ResetBits(GPIO_SOLAR_EN_PORT, GPIO_SOLAR_EN_PIN)

#define SW_USB_EN_ENABLE      	GPIO_SetBits(GPIO_USB_EN_PORT, GPIO_USB_EN_PIN)
#define SW_USB_EN_DISABLE     	GPIO_ResetBits(GPIO_USB_EN_PORT, GPIO_USB_EN_PIN)

#define SW_EPS_S0_ENABLE      	GPIO_SetBits(GPIO_EPS_S0_PORT, GPIO_EPS_S0_PIN)
#define SW_EPS_S0_DISABLE     	GPIO_ResetBits(GPIO_EPS_S0_PORT, GPIO_EPS_S0_PIN)

#define SW_EPS_S1_ENABLE      	GPIO_SetBits(GPIO_EPS_S1_PORT, GPIO_EPS_S1_PIN)
#define SW_EPS_S1_DISABLE     	GPIO_ResetBits(GPIO_EPS_S1_PORT, GPIO_EPS_S1_PIN)

#define SW_EPS_S2_ENABLE      	GPIO_SetBits(GPIO_EPS_S2_PORT, GPIO_EPS_S2_PIN)
#define SW_EPS_S2_DISABLE     	GPIO_ResetBits(GPIO_EPS_S2_PORT, GPIO_EPS_S2_PIN)

#define SW_EPS_S3_ENABLE      	GPIO_SetBits(GPIO_EPS_S3_PORT, GPIO_EPS_S3_PIN)
#define SW_EPS_S3_DISABLE     	GPIO_ResetBits(GPIO_EPS_S3_PORT, GPIO_EPS_S3_PIN)


/*805*/
#define SW_DTB_5V_ENABLE            GPIO_SetBits(GPIO_DTB_5V_PORT,GPIO_DTB_5V_PIN)
#define SW_DTB_5V_DISABLE           GPIO_ResetBits(GPIO_DTB_5V_PORT,GPIO_DTB_5V_PIN)

#define SW_DTB_12V_ENABLE           GPIO_SetBits(GPIO_DTB_12V_PORT,GPIO_DTB_12V_PIN)
#define SW_DTB_12V_DISABLE          GPIO_ResetBits(GPIO_DTB_12V_PORT,GPIO_DTB_12V_PIN)

#define SW_CAMERA_10W_ENABLE        GPIO_SetBits(GPIO_CAMERA_10W_PORT,GPIO_CAMERA_10W_PIN)
#define SW_CAMERA_10W_DISABLE       GPIO_ResetBits(GPIO_CAMERA_10W_PORT,GPIO_CAMERA_10W_PIN)

#define SW_CAMERA_5W_ENABLE         GPIO_SetBits(GPIO_CAMERA_5W_PORT,GPIO_CAMERA_5W_PIN)
#define SW_CAMERA_5W_DISABLE        GPIO_ResetBits(GPIO_CAMERA_5W_PORT,GPIO_CAMERA_5W_PIN)

#define SW_CAMERA_HEAT_1_ENABLE     GPIO_SetBits(GPIO_CAMERA_HEAT_1_PORT,GPIO_CAMERA_HEAT_1_PIN)
#define SW_CAMERA_HEAT_1_DISABLE    GPIO_ResetBits(GPIO_CAMERA_HEAT_1_PORT,GPIO_CAMERA_HEAT_1_PIN)

#define SW_CAMERA_HEAT_2_ENABLE     GPIO_SetBits(GPIO_CAMERA_HEAT_2_PORT,GPIO_CAMERA_HEAT_2_PIN)
#define SW_CAMERA_HEAT_2_DISABLE    GPIO_ResetBits(GPIO_CAMERA_HEAT_2_PORT,GPIO_CAMERA_HEAT_2_PIN)


void bsp_InitSwitch(void);
uint8_t EpsOutSwitch(uint8_t ch, uint8_t status);

#endif

