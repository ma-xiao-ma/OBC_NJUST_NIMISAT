////////////////////////////////////////////////////////////////////////////////
//	功能： 软件I2C初始化、读写控制头文件
//
//	版本：V1.0
//  迭代：
//												南京理工大学微纳卫星中心
//												   2015.11.08
////////////////////////////////////////////////////////////////////////////////
#ifndef __BSP_I2C0_H_
#define __BSP_I2C0_H_

#include "stdint.h"
#include "stm32f4xx.h"

#define I2C0_WR	0		//写控制位
#define I2C0_RD	1		//读控制位

///实现软件I2C的GPIO引脚
#define GPIO_PORT_I2C	GPIOA						//GPIO端口
#define RCC_I2C_PORT 	RCC_AHB1Periph_GPIOA		//GPIO时钟
#define I2C_SCL_PIN		GPIO_Pin_11					//连接到SCL时钟的GPIO引脚
#define I2C_SDA_PIN		GPIO_Pin_12					//连接到SDA时钟的GPIO引脚




void bsp_InitI2C0(void);
static void i2c0_Delay(void);
void i2c0_Start(void);
void i2c0_Stop(void);
void i2c0_SendByte(uint8_t _ucByte);
uint8_t i2c0_ReadByte(void);
uint8_t i2c0_WaitAck(void);
void i2c0_Ack(void);
void i2c0_NAck(void);
uint8_t i2c0_CheckDevice(uint8_t _Address);
#endif
