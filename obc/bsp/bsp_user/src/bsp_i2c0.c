////////////////////////////////////////////////////////////////////////////////
//	功能： 软件I2C初始化、读写控制头文件
//
//	版本：V1.0
//  迭代：
//												南京理工大学微纳卫星中心
//												   2015.11.08
////////////////////////////////////////////////////////////////////////////////
#include "bsp_i2c0.h"


////读写SCL
//#define I2C_SCL_1()  GPIO_PORT_I2C->BSRRL = I2C_SCL_PIN			// SCL = 1
//#define I2C_SCL_0()  GPIO_PORT_I2C->BSRRH = I2C_SCL_PIN			// SCL = 0
//
////读写SDA
//#define I2C_SDA_1()  GPIO_PORT_I2C->BSRRL = I2C_SDA_PIN			// SDA = 1
//#define I2C_SDA_0()  GPIO_PORT_I2C->BSRRH = I2C_SDA_PIN			// SDA = 0

//读写SCL
#define I2C_SCL_1()  GPIOB->BSRRL = GPIO_Pin_15			// SCL = 1
#define I2C_SCL_0()  GPIOB->BSRRH = GPIO_Pin_15 		// SCL = 0

//读写SDA
#define I2C_SDA_1()  GPIOG->BSRRL = GPIO_Pin_11			// SDA = 1
#define I2C_SDA_0()  GPIOG->BSRRH = GPIO_Pin_11			// SDA = 0

//读SDA状态
#define I2C_SDA_READ()  ((GPIO_PORT_I2C->IDR & I2C_SDA_PIN) != 0)
//读SCL状态
#define I2C_SCL_READ()  ((GPIO_PORT_I2C->IDR & I2C_SCL_PIN) != 0)


////////////////////////////////////////////////////////////////////////////////
//	功能说明:I2C总线初始化
//	形    参:
//	返 回 值: 无
////////////////////////////////////////////////////////////////////////////////
void bsp_InitI2C0(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

//	RCC_AHB1PeriphClockCmd(RCC_I2C_PORT, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;		//输出
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;		//开漏模式
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	//上下拉电阻不使能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//IO口最大速度

//	GPIO_InitStructure.GPIO_Pin = I2C_SCL_PIN | I2C_SDA_PIN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOG, &GPIO_InitStructure);

	//输出停止信号，复位I2C总线上的所有设备到待机模式
	i2c0_Stop();
}

////////////////////////////////////////////////////////////////////////////////
//	功能说明:I2C总线延迟
//	形    参:
//	返 回 值: 无
////////////////////////////////////////////////////////////////////////////////
static void i2c0_Delay(void)
{
	uint8_t i;

	for (i = 0; i < 30; i++);
	{
		__asm volatile( "nop" );
		__asm volatile( "nop" );
	}
}

////////////////////////////////////////////////////////////////////////////////
//	功能说明:I2C发起总线启动信号
//	形    参:
//	返 回 值: 无
////////////////////////////////////////////////////////////////////////////////
void i2c0_Start(void)
{
	//当SCL高电平时,SDA出现一个下跳沿表示I2C总线启动信号
	I2C_SDA_1();
	I2C_SCL_1();
	i2c0_Delay();
	I2C_SDA_0();
	i2c0_Delay();
	I2C_SCL_0();
	i2c0_Delay();
}

////////////////////////////////////////////////////////////////////////////////
//	功能说明:I2C发起总线停止信号
//	形    参:
//	返 回 值: 无
////////////////////////////////////////////////////////////////////////////////
void i2c0_Stop(void)
{
	//当SCL高电平时,SDA出现一个上升沿表示I2C总线启动信号
	I2C_SDA_0();
	I2C_SCL_1();
	i2c0_Delay();
	I2C_SDA_1();
}

////////////////////////////////////////////////////////////////////////////////
//	功能说明:CPU向I2C总线设备发送8bit数据
//	形    参:
//	返 回 值: 无
////////////////////////////////////////////////////////////////////////////////
void i2c0_SendByte(uint8_t _ucByte)
{
	uint8_t i;
	
	//先发送字节的高位bit7
	for (i = 0; i < 8; i++)
	{
		if (_ucByte & 0x80)
		{
			I2C_SDA_1();
		}
		else
		{
			I2C_SDA_0();
		}
		i2c0_Delay();
		I2C_SCL_1();
		i2c0_Delay();
		I2C_SCL_0();
		if (i == 7)
		{
			 I2C_SDA_1(); //释放总线
		}
		_ucByte <<= 1;	//左移一个bit
		i2c0_Delay();
	}
}

////////////////////////////////////////////////////////////////////////////////
//	功能说明:CPU从I2C总线设备读取8bit数据
//	形    参:
//	返 回 值: 读取的数据
////////////////////////////////////////////////////////////////////////////////
uint8_t i2c0_ReadByte(void)
{
	uint8_t i;
	uint8_t value;

	//读到第一个bit位数据的bit7
	value = 0;
	for (i = 0; i < 8; i++)
	{
		value <<= 1;
		I2C_SCL_1();
		i2c0_Delay();
		if (I2C_SDA_READ())
		{
			value++;
		}
		I2C_SCL_0();
		i2c0_Delay();
	}
	return value;
}

////////////////////////////////////////////////////////////////////////////////
//	功能说明:CPU产生一个时钟,并读取器件的ACK应答信号
//	形    参:
//	返 回 值: 0:正确应答;1:无器件响应
////////////////////////////////////////////////////////////////////////////////
uint8_t i2c0_WaitAck(void)
{
	uint8_t re;

	I2C_SDA_1();	//CPU释放SDA总线
	i2c0_Delay();
	I2C_SCL_1();	//CPU驱动SCL=1,此时器件会返回ACK应答
	i2c0_Delay();
	if (I2C_SDA_READ())	//CPU读取SDA线状态
	{
		re = 1;
	}
	else
	{
		re = 0;
	}
	I2C_SCL_0();
	i2c0_Delay();
	return re;
	
}

////////////////////////////////////////////////////////////////////////////////
//	功能说明:CPU产生一个ACK应答信号
//	形    参:
//	返 回 值: 0:正确应答;1:无器件响应
////////////////////////////////////////////////////////////////////////////////
void i2c0_Ack(void)
{
	I2C_SDA_0();	//CPU驱动SDA = 0
	i2c0_Delay();
	I2C_SCL_1();	// CPU产生一个时钟
	i2c0_Delay();
	I2C_SCL_0();
	i2c0_Delay();
	I2C_SDA_1();	//CPU释放SDA总线
}

////////////////////////////////////////////////////////////////////////////////
//	功能说明:CPU产生一个NACK应答信号
//	形    参:
//	返 回 值:
////////////////////////////////////////////////////////////////////////////////
void i2c0_NAck(void)
{
	I2C_SDA_1();	//CPU驱动SDA = 1
	i2c0_Delay();
	I2C_SCL_1();	// CPU产生一个时钟
	i2c0_Delay();
	I2C_SCL_0();
	i2c0_Delay();
}

////////////////////////////////////////////////////////////////////////////////
//	功能说明:检查I2C总线设备,CPU发送设备地址,读取设备应答判断该设备是否存在
//	形    参:_Address:设备的I2C总线地址
//	返 回 值:0:存在设备;1:未检测到设备
////////////////////////////////////////////////////////////////////////////////
uint8_t i2c0_CheckDevice(uint8_t _Address)
{
	uint8_t ucAck;

	if (I2C_SDA_READ() && I2C_SCL_READ())
	{
		i2c0_Start();		//发送启动信号

		//发送设备地址+读写控制bit(0=w,1=r),bit7先传
		i2c0_SendByte(_Address | I2C0_WR);
		ucAck = i2c0_WaitAck();	//检测设备的ACK应答

		i2c0_Stop();			//发送停止信号

		return ucAck;
	}
	return 1;	//I2C总线异常
}
