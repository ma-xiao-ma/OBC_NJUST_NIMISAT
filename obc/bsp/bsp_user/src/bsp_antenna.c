////////////////////////////////////////////////////////////////////////////////
//	功能： 通过I2C接口实现ISIS天线展开功能
//
//	版本：V1.0
//  迭代：
//												南京理工大学微纳卫星中心
//												   2015.11.08
////////////////////////////////////////////////////////////////////////////////
#include "bsp_antenna.h"
#include "bsp_i2c1.h"
#include "FreeRTOS.h"
#include "task.h"



#define ISIS_BURN_TIME_INCREMENT	1

#define I2C_WRITE_FLAG			0x00
#define I2C_READ_FLAG			0x01

#define ISIS_ANTENA_READ_ADDR	(ISIS_ANTENA_ADDR<<1 | I2C_READ_FLAG)
#define ISIS_ANTENA_WRITE_ADDR	(ISIS_ANTENA_ADDR<<1 | I2C_WRITE_FLAG)

////////////////////////////////////////////////////////////////////////////////
//	功能说明:读取天线温度
//	形    参: 待读取的温度数据指针
//	返 回 值:  1:读取失败
//			0:读取成功
////////////////////////////////////////////////////////////////////////////////
uint8_t bsp_AntennaTemperatureGet(uint16_t* data) {
	uint16_t temp = 0;

	i2c1_Start();
	i2c1_SendByte(ISIS_ANTENA_WRITE_ADDR);
	if (i2c1_WaitAck())	//i2c no ack
	{
		i2c1_Stop();
		return 1;
	}
	vTaskDelay(1);//延迟1ms

	i2c1_SendByte(ANTENA_CMD_TEMP);
	if (i2c1_WaitAck())	//i2c no ack
	{
		i2c1_Stop();
		return 1;
	}
	vTaskDelay(1);

	i2c1_Start();
	i2c1_SendByte(ISIS_ANTENA_READ_ADDR);
	if (i2c1_WaitAck())	//i2c no ack
	{
		i2c1_Stop();
		return 1;
	}
	vTaskDelay(1);

	temp |= i2c1_ReadByte();
	i2c1_Ack();
	vTaskDelay(1);

	temp |= (i2c1_ReadByte() << 8);
	i2c1_Ack();
	vTaskDelay(1);

	i2c1_Stop();

	*data = temp;
	return 0;
}

////////////////////////////////////////////////////////////////////////////////
//	功能说明:重置天线状态
//	形    参:
//	返 回 值:  1:失败
//			0:成功
////////////////////////////////////////////////////////////////////////////////
uint8_t bsp_AntennaReset(void) {

	i2c1_Start();
	i2c1_SendByte(ISIS_ANTENA_WRITE_ADDR);
	if (i2c1_WaitAck())	//i2c no ack
	{
		i2c1_Stop();
		return 1;
	}
	vTaskDelay(1);

	i2c1_SendByte(ANTENA_CMD_RESET);
	if (i2c1_WaitAck())	//i2c no ack
	{
		i2c1_Stop();
		return 1;
	}
	vTaskDelay(1);

	i2c1_Stop();
	return 0;
}

////////////////////////////////////////////////////////////////////////////////
//	功能说明:天线ARM指令
//	形    参: op: 1:ARM
//				0:去ARM
//	返 回 值:  1:失败
//			0:成功
////////////////////////////////////////////////////////////////////////////////
uint8_t bsp_AntennaArm(uint8_t op) {

	i2c1_Start();
	i2c1_SendByte(ISIS_ANTENA_WRITE_ADDR);
	if (i2c1_WaitAck())	//i2c no ack
	{
		i2c1_Stop();
		return 1;
	}
	vTaskDelay(1);

	if (op == 0)
		i2c1_SendByte(ANTENA_CMD_DISARM);
	else
		i2c1_SendByte(ANTENA_CMD_ARM);

	if (i2c1_WaitAck())	//i2c no ack
	{
		i2c1_Stop();
		return 1;
	}
	vTaskDelay(1);

	i2c1_Stop();
	return 0;
}

////////////////////////////////////////////////////////////////////////////////
//	功能说明:天线展开
//	形    参: number: 展开天线编号
//			0:顺序全展开;1-4:天线1-4
//		   time:超时时间
//	返 回 值:  1:命令发送失败
//			0:命令发送成功
////////////////////////////////////////////////////////////////////////////////
uint8_t bsp_AntennaDeploy(uint8_t number, uint8_t time) {
	uint8_t antenna_num;

	switch (number) {
	case 0:
		antenna_num = ANTENA_CMD_DEPLOY_SEQ;
		break;
	case 1:
		antenna_num = ANTENA_CMD_DEPLOY1;
		break;
	case 2:
		antenna_num = ANTENA_CMD_DEPLOY2;
		break;
	case 3:
		antenna_num = ANTENA_CMD_DEPLOY3;
		break;
	case 4:
		antenna_num = ANTENA_CMD_DEPLOY4;
		break;
	default:
		return 1;
	}

	i2c1_Start();
	i2c1_SendByte(ISIS_ANTENA_WRITE_ADDR);
	if (i2c1_WaitAck())	//i2c no ack
	{
		i2c1_Stop();
		return 1;
	}
	vTaskDelay(1);


	i2c1_SendByte(antenna_num);
	if (i2c1_WaitAck())	//i2c no ack
	{
		i2c1_Stop();
		return 1;
	}
	vTaskDelay(1);

	i2c1_SendByte(time);
	if (i2c1_WaitAck())	//i2c no ack
	{
		i2c1_Stop();
		return 1;
	}
	vTaskDelay(1);

	i2c1_Stop();
	return 0;
}

////////////////////////////////////////////////////////////////////////////////
//	功能说明:天线展开
//	形    参: number: 天线编号
//					1-4:天线1-4
//		   time:超时时间
//	返 回 值:  1:命令发送失败
//			0:命令发送成功
////////////////////////////////////////////////////////////////////////////////
uint8_t bsp_AntennaDeployOver(uint8_t number, uint8_t time) {
	uint8_t antenna_num;

	switch (number) {
	case 1:
		antenna_num = ANTENA_CMD_DEPLOY1_OW;
		break;
	case 2:
		antenna_num = ANTENA_CMD_DEPLOY2_OW;
		break;
	case 3:
		antenna_num = ANTENA_CMD_DEPLOY3_OW;
		break;
	case 4:
		antenna_num = ANTENA_CMD_DEPLOY4_OW;
		break;
	default:
		return 1;
	}

	i2c1_Start();
	i2c1_SendByte(ISIS_ANTENA_WRITE_ADDR);
	if (i2c1_WaitAck())	//i2c no ack
	{
		i2c1_Stop();
		return 1;
	}
	vTaskDelay(1);

	i2c1_SendByte(antenna_num);
	if (i2c1_WaitAck())	//i2c no ack
	{
		i2c1_Stop();
		return 1;
	}
	vTaskDelay(1);

	i2c1_SendByte(time);
	if (i2c1_WaitAck())	//i2c no ack
	{
		i2c1_Stop();
		return 1;
	}
	vTaskDelay(1);

	i2c1_Stop();
	return 0;
}


////////////////////////////////////////////////////////////////////////////////
//	功能说明:天线展开停止
//	形    参:
//	返 回 值:  1:命令发送失败
//			0:命令发送成功
////////////////////////////////////////////////////////////////////////////////
uint8_t bsp_AntennaDeployStop(void) {

	i2c1_Start();
	i2c1_SendByte(ISIS_ANTENA_WRITE_ADDR);
	if (i2c1_WaitAck())	//i2c no ack
	{
		i2c1_Stop();
		return 1;
	}
	vTaskDelay(1);

	i2c1_SendByte(ANTENA_CMD_DEPLOY_ESC);
	if (i2c1_WaitAck())	//i2c no ack
	{
		i2c1_Stop();
		return 1;
	}
	vTaskDelay(1);

	i2c1_Stop();
	return 0;
}

////////////////////////////////////////////////////////////////////////////////
//	功能说明:获取天线展开状态
//	形    参: 天线展开状态
//	返 回 值:  1:命令发送失败
//			0:命令发送成功
////////////////////////////////////////////////////////////////////////////////
uint8_t bsp_AntennaGetStatus(uint16_t* data) {
	uint16_t temp = 0;

	i2c1_Start();
	i2c1_SendByte(ISIS_ANTENA_WRITE_ADDR);
	if (i2c1_WaitAck())	//i2c no ack
	{
		i2c1_Stop();
		return 1;
	}
	vTaskDelay(1);

	i2c1_SendByte(ANTENA_CMD_DEPLOY_STAT);
	if (i2c1_WaitAck())	//i2c no ack
	{
		i2c1_Stop();
		return 1;
	}
	vTaskDelay(1);

	i2c1_Start();
	i2c1_SendByte(ISIS_ANTENA_READ_ADDR);
	if (i2c1_WaitAck())	//i2c no ack
	{
		i2c1_Stop();
		return 1;
	}
	vTaskDelay(1);

	temp |= i2c1_ReadByte();
	i2c1_Ack();
	vTaskDelay(1);

	temp |= (i2c1_ReadByte() << 8);
	i2c1_Ack();
	vTaskDelay(1);

	i2c1_Stop();

	*data = temp;
	return 0;
}


uint8_t bsp_AntennaGetDeployCnt(uint8_t number, uint8_t* data) {
	uint8_t temp = 0;
	uint8_t antenna_num;

	switch (number) {
	case 1:
		antenna_num = ANTENA_CMD_DEPLOY1_CNT;
		break;
	case 2:
		antenna_num = ANTENA_CMD_DEPLOY2_CNT;
		break;
	case 3:
		antenna_num = ANTENA_CMD_DEPLOY3_CNT;
		break;
	case 4:
		antenna_num = ANTENA_CMD_DEPLOY4_CNT;
		break;
	default:
		return 1;
	}

	i2c1_Start();
	i2c1_SendByte(ISIS_ANTENA_WRITE_ADDR);
	if (i2c1_WaitAck())	//i2c no ack
	{
		i2c1_Stop();
		return 1;
	}
	vTaskDelay(1);

	i2c1_SendByte(antenna_num);
	if (i2c1_WaitAck())	//i2c no ack
	{
		i2c1_Stop();
		return 1;
	}
	vTaskDelay(1);

	i2c1_Start();
	i2c1_SendByte(ISIS_ANTENA_READ_ADDR);
	if (i2c1_WaitAck())	//i2c no ack
	{
		i2c1_Stop();
		return 1;
	}
	vTaskDelay(1);

	temp = i2c1_ReadByte();
	i2c1_Ack();
	vTaskDelay(1);

	i2c1_Stop();

	*data = temp;
	return 0;
}

uint8_t bsp_AntennaGetDeployTime(uint8_t number, uint16_t* data) {
	uint16_t temp = 0;
	uint8_t antenna_num;

	switch (number) {
	case 1:
		antenna_num = ANTENA_CMD_DEPLOY1_TIME;
		break;
	case 2:
		antenna_num = ANTENA_CMD_DEPLOY2_TIME;
		break;
	case 3:
		antenna_num = ANTENA_CMD_DEPLOY3_TIME;
		break;
	case 4:
		antenna_num = ANTENA_CMD_DEPLOY4_TIME;
		break;
	default:
		return 1;
	}

	i2c1_Start();
	i2c1_SendByte(ISIS_ANTENA_WRITE_ADDR);
	if (i2c1_WaitAck())	//i2c no ack
	{
		i2c1_Stop();
		return 1;
	}
	vTaskDelay(1);

	i2c1_SendByte(antenna_num);
	if (i2c1_WaitAck())	//i2c no ack
	{
		i2c1_Stop();
		return 1;
	}
	vTaskDelay(1);

	i2c1_Start();
	i2c1_SendByte(ISIS_ANTENA_READ_ADDR);
	if (i2c1_WaitAck())	//i2c no ack
	{
		i2c1_Stop();
		return 1;
	}
	vTaskDelay(1);

	temp |= i2c1_ReadByte();
	i2c1_Ack();
	vTaskDelay(1);

	temp |= (i2c1_ReadByte() << 8);
	i2c1_Ack();
	vTaskDelay(1);

	i2c1_Stop();

	*data = temp;
	return 0;
}



uint8_t bsp_AntennaDeployAuto(void) {

	uint16_t status = 0;
	uint8_t retry=0, burntime;

//	retry = 0;
	while (bsp_AntennaReset() && (retry++ < ISIS_ANTTENNA_RETRY)) {
		vTaskDelay(1);
	}		//ISIS天线初始化
	if (retry >= ISIS_ANTTENNA_RETRY)	//超时退出
		return 1;

	retry = 0;
	while (bsp_AntennaArm(1) && retry++ < ISIS_ANTTENNA_RETRY) {
		vTaskDelay(1);
	}		//ISIS天线控制器ARM使能
	if (retry >= ISIS_ANTTENNA_RETRY)
		return 1;

	retry = 0;
	while (retry++ < ISIS_ANTTENNA_RETRY)	//获取ARM使能状态
	{
		bsp_AntennaGetStatus(&status);
		if (status & ANNTENNA_STA_ARMED)
			break; //使能则跳出继续
		vTaskDelay(1);
	}

	if (retry >= ISIS_ANTTENNA_RETRY)
		return 1;


	burntime = ISIS_BURN_TIME_INIT;

	//天线自动展开
	retry = 0;
	while (bsp_AntennaDeploy(0, burntime) && retry++ < ISIS_ANTTENNA_RETRY) {
		vTaskDelay(1);
	}
	if (retry >= ISIS_ANTTENNA_RETRY)
		return 1;
	vTaskDelay(burntime);

	while (1) {

		//读取天线状态
		retry = 0;
		while (bsp_AntennaGetStatus(&status) && retry++ < ISIS_ANTTENNA_RETRY)
		{
			vTaskDelay(1);
		}
		if (retry >= ISIS_ANTTENNA_RETRY)
			return 1;

		if (status & 0x8000 || status & 0x0800 || status & 0x0080
				|| status & 0x0008) {
			//printf("Anttenna not all deployed...\r\n");
			bsp_AntennaDeploy(0, burntime);
			vTaskDelay(burntime);
		} else {
			return 0;
		}
	}
}

