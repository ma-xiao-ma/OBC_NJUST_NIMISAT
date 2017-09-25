#ifndef __BSP_I2C1_H_
#define __BSP_I2C1_H_

#include "stdint.h"
#include "stm32f4xx.h"

#define I2C1_WR	0		//写控制位
#define I2C1_RD	1		//读控制位

///实现软件I2C的GPIO引脚
#define GPIO_PORT_I2C1		GPIOB					//GPIO端口
#define RCC_I2C1_PORT 		RCC_AHB1Periph_GPIOB	//GPIO时钟
#define I2C1_SCL_PIN		GPIO_Pin_8				//连接到SCL时钟的GPIO引脚
#define I2C1_SDA_PIN		GPIO_Pin_9				//连接到SDA时钟的GPIO引脚



void bsp_InitI2C1(void);
static void i2c1_Delay(void);
void i2c1_Start(void);
void i2c1_Stop(void);
void i2c1_SendByte(uint8_t _ucByte);
uint8_t i2c1_ReadByte(void);
uint8_t i2c1_WaitAck(void);
void i2c1_Ack(void);
void i2c1_NAck(void);
uint8_t i2c1_CheckDevice(uint8_t _Address);
#endif
