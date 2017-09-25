/*
 * bsp_icd.h
 *
 *  Created on: 2016年4月28日
 *      Author: Administrator
 */

#include <stdint.h>
#include <inttypes.h>
#include <stddef.h>

#include "bsp_icd.h"
#include "bsp_pca9665.h"

#include "error.h"
#include "QB50_mem.h"

/**
 * @brief 发送命令  无参数  无返回
 * @para  指令  和 地址
 *
 *@retval  无
 */

int I2C_ICD_send_command( uint8_t command, uint8_t Address )
{
	uint8_t addr = Address;
	size_t txlen = 1;
	uint8_t * txbuf = &command;
	uint8_t * rxbuf = NULL;
	size_t rxlen = 0;

	int result = i2c_master_transaction(ISIS_HANDLE, addr, txbuf,  txlen, rxbuf, rxlen, ISIS_TIMEOUT);

	return result;
}


/**
  * @brief  发送命令 无参数 有返回值
  *
  * @param
  *
  *		@arg command:命令
  *     @arg NumByteToWrite:写入字节数
  * @retval  返回值
  */
int I2C_ICD_read_para(uint8_t* pBuffer, uint8_t command , uint8_t Address ,  uint16_t NumByteToRead )
{

	uint8_t addr = Address;
	size_t txlen = 1;
	uint8_t * txbuf = &command;
	uint8_t * rxbuf = pBuffer;
	size_t rxlen = NumByteToRead;

	int result = i2c_master_transaction(ISIS_HANDLE, addr, txbuf,  txlen, rxbuf, rxlen, ISIS_TIMEOUT);

	return result;
}


/**
  * @brief  发送命令  有参数  无返回
  *
  * @param
  *		@arg pBuffer:写入参数
  *		@arg command:命令
  *     @arg NumByteToWrite:写入字节数
  * @retval  无
  */
int I2C_ICD_send_para(uint8_t* pBuffer, uint8_t command , uint8_t NumByteToWrite, uint8_t Address )
{

	uint8_t addr = Address;
	size_t txlen = NumByteToWrite+1;
	char txbuf[NumByteToWrite+1];
	char * rxbuf = NULL;
	size_t rxlen = 0;
	int i = 1;

	txbuf[0]=command;
	for(i=1;i<NumByteToWrite+1;i++){
		txbuf[i] = *pBuffer;
		pBuffer++;
	}

	int result = i2c_master_transaction(ISIS_HANDLE, addr, txbuf,  txlen, rxbuf, rxlen, ISIS_TIMEOUT);

	return result;
}



/**
  * @brief   从Receiver读取数据
  * @param
  *		@arg pBuffer:接收的数据
  *		@arg command:指令
  *     @arg NumByteToWrite:接收数据个数  （来自于读取数据帧数）
  * @retval  无
  */
//check 5.18 13 .11
int I2C_ICD_read_frame(uint8_t* pBuffer)
{
	uint16_t count = 0;
	size_t txlen = 1;
	uint8_t * rxbuf = pBuffer;
	uint8_t txbuf = Get_frames;
	int result = 0;

//	int result = I2C_ICD_read_para((uint8_t*)&count, Get_frame_number , Receiver_Address ,2);
//
//	if(result != E_NO_ERR)
//		return 0;
//
//	if(count<=0) return 0;

	count = 6;

	result = i2c_master_transaction(ISIS_HANDLE, Receiver_Address, &txbuf, txlen, rxbuf, count, ISIS_DELAY_TO_READ);

	return result;
}

int I2C_ICD_get_frame(uint8_t* pBuffer)
{
	uint16_t count = 6;
	size_t txlen = 1;
	uint8_t * rxbuf = pBuffer;
	uint8_t txbuf = Get_frames;

	int result = i2c_master_transaction(ISIS_HANDLE, Receiver_Address, &txbuf, txlen, rxbuf, count, ISIS_DELAY_TO_READ);

	return result;
}

int I2C_ICD_get_frame_stable(uint8_t* pBuffer)
{
	uint16_t count = 156;
	size_t txlen = 1;
	uint8_t * rxbuf = pBuffer;
	uint8_t txbuf = Get_frames;

	int result = i2c_master_transaction(ISIS_HANDLE, Receiver_Address, &txbuf, txlen, rxbuf, count, ISIS_TIMEOUT);

	return result;
}

/**
  * @brief  发送数据
  *
  * @param
  *		@arg pBuffer:写入参数
  *		@arg command:命令
  *     @arg NumByteToWrite:写入字节数
  * @retval  剩余帧数
  */
int I2C_ICD_send_date(uint8_t* pBuffer, uint8_t NumByteToWrite, uint8_t* rest_frames)
{

	size_t txlen = NumByteToWrite+1;
	uint8_t  txbuf[NumByteToWrite+1];
	uint8_t * rxbuf = rest_frames;
	size_t rxlen = 1;
	int i = 1;

	txbuf[0] = Send_frame;
	for(i=1;i<NumByteToWrite+1;i++){
		txbuf[i] = * pBuffer++;
	}

	int result = i2c_master_transaction(ISIS_HANDLE, Transmitter_Address, txbuf,  txlen, rxbuf, rxlen, ISIS_TIMEOUT);

	return result;
}


//2015.5.10
/**
  * @brief  发送AX2.5数据
  *
  * @param
  *		@arg pBuffer:写入参数
  *		@arg command:命令
  *     @arg NumByteToWrite:写入字节数
  * @retval  剩余帧数
  */
int I2C_ICD_send_Axdate(uint8_t * ogc, uint8_t * dec, uint8_t* pBuffer, uint8_t NumByteToWrite, uint8_t* rest_frames)
{
	Date_with_call * pdata = (Date_with_call *)qb50Malloc(I2C_MTU);

	pdata->command = Send_frame_with_callsigns;

	memcpy(pdata->origcall, ogc, 7);
	memcpy(pdata->descall, dec, 7);
	memcpy(pdata->date, pBuffer, NumByteToWrite);

	size_t txlen = NumByteToWrite+15;
	uint8_t * rxbuf = rest_frames;
	size_t rxlen = 1;

	int result = i2c_master_transaction(ISIS_HANDLE, Transmitter_Address, pdata,  NumByteToWrite+15, rxbuf, rxlen, ISIS_TIMEOUT);

	qb50Free(pdata);

	return result;
}

int ICD_send(uint8_t* pBuffer, uint8_t NumByteToWrite, uint8_t* rest_frames)
{
	Date_with_call * pdata = (Date_with_call *)qb50Malloc(I2C_MTU);

	pdata->command = Send_frame_with_callsigns;

	memcpy(pdata->origcall, SCALL, 7);
	memcpy(pdata->descall, DCALL, 7);
	memcpy(pdata->date, pBuffer, NumByteToWrite);

	size_t txlen = NumByteToWrite+15;
	uint8_t * rxbuf = rest_frames;
	size_t rxlen = 1;

	int result = i2c_master_transaction(ISIS_HANDLE, Transmitter_Address, pdata,  NumByteToWrite+15, rxbuf, rxlen, ISIS_TIMEOUT);

	qb50Free(pdata);

	return result;
}

/**
  * @brief  信标设置
  *
  * @param
  *		@arg pBuffer:写入参数
  *		@arg command:命令
  *     @arg NumByteToWrite:写入字节数
  * @retval  剩余帧数
  */


int I2C_ICD_beacon_set(uint8_t* ben, uint8_t* pBuffer, uint8_t NumByteToWrite)
{
	Beacon_set * bdate = (Beacon_set * )qb50Malloc(I2C_MTU);

	bdate->command = Set_beacon;

	memcpy(bdate->beacon, ben, 2);
	memcpy(bdate->date, pBuffer, NumByteToWrite);

	size_t txlen = NumByteToWrite+3;
	char * rxbuf = NULL;
	size_t rxlen = 0;

	int result = i2c_master_transaction(ISIS_HANDLE, Transmitter_Address,bdate, NumByteToWrite+3, rxbuf, rxlen, ISIS_TIMEOUT);

	qb50Free(bdate);

	return result;
}


/**
  * @brief  AX信标设置
  *
  * @param
  *		@arg pBuffer:写入参数
  *		@arg command:命令
  *     @arg NumByteToWrite:写入字节数
  * @retval  剩余帧数
  */


int I2C_ICD_Axbeacon_set(uint8_t *ben, uint8_t * ogc, uint8_t * dec,uint8_t* pBuffer, uint8_t NumByteToWrite)
{
	AX_Beacon_set * Adata = (AX_Beacon_set *)qb50Malloc(I2C_MTU);

	Adata->command = Set_beacon_with_callsigns;

	memcpy(Adata->beacon, ben, 2);
	memcpy(Adata->descall, ogc, 7);
	memcpy(Adata->origcall, ogc, 7);
	memcpy(Adata->date, pBuffer, NumByteToWrite);

	size_t txlen = NumByteToWrite+17;
	char * rxbuf = NULL;
	size_t rxlen = 0;

	int result = i2c_master_transaction(ISIS_HANDLE, Transmitter_Address,Adata, NumByteToWrite+17, rxbuf, rxlen, ISIS_TIMEOUT);

	qb50Free(Adata);

	return result;
}

/*      2016.5.3        */
/**
 * @brief 读取通信机状态
 * @para  返回一个字节
 *
 *@retval  无
 */
int I2C_ICD_read_workstate(uint8_t* pBuffer)
{
	uint16_t NumByteToRead = 1;
	uint8_t command = Report_transmitter_state;
	uint8_t Address = Transmitter_Address;

	int retval = I2C_ICD_read_para(pBuffer, command , Address , NumByteToRead );
	retval = I2C_ICD_read_para(pBuffer, command , Address , NumByteToRead );

	return retval;
}


/**读取发射机系统时间
 * @para  返回4个字节
 *
 *@retval  无
 */
int  I2C_ICD_read_Transmitter_systime(uint8_t* pBuffer)
{

	uint16_t NumByteToRead = 4;
	uint8_t command = Report_transmitter_uptime;
	uint8_t Address = Transmitter_Address;

	int retval = I2C_ICD_read_para(pBuffer, command , Address ,NumByteToRead );
	retval = I2C_ICD_read_para(pBuffer, command , Address ,NumByteToRead );

	return retval;
}


/**
 * @brief 读取上次发射机状态
 * @para  返回7个字节
 *
 *@retval  无
 */
int  I2C_ICD_read_laststate(uint8_t* pBuffer)
{

	uint16_t NumByteToRead = 8;
	uint8_t command = Store_last_states;
	uint8_t Address = Transmitter_Address;

	int retval = I2C_ICD_read_para(pBuffer, command , Address ,NumByteToRead );
	retval = I2C_ICD_read_para(pBuffer, command , Address ,NumByteToRead );

	return retval;
}


/**
 * @brief 读取发射机状态
 * @para  返回8个字节
 *
 *@retval  无
 */
int  I2C_ICD_read_Transmitter_all_states(uint8_t* pBuffer)
{

	uint16_t NumByteToRead = 8;
	uint8_t command = Measure_all_states;
	uint8_t Address = Transmitter_Address;

	int retval = I2C_ICD_read_para(pBuffer, command , Address ,NumByteToRead );
	retval = I2C_ICD_read_para(pBuffer, command , Address ,NumByteToRead );

	return retval;
}


/**
 * @brief 读取接收机通信机时间
 * @para  返回4个字节
 *
 *@retval  无
 */
int  I2C_ICD_read_Receiver_systime(uint8_t* pBuffer)
{

	uint16_t NumByteToRead = 4;
	uint8_t command = Report_Receiver_uptime;
	uint8_t Address = Receiver_Address;

	int retval = I2C_ICD_read_para(pBuffer, command , Address ,NumByteToRead );

	return retval;
}


/**
 * @brief 读取接收机测量值
 *
 * @para  返回14个字节
 *
 *@retval  无
 */
int  I2C_ICD_read_Receiver_all_states(uint8_t* pBuffer)
{

	uint16_t NumByteToRead = 14;
	uint8_t command = Measure_states;
	uint8_t Address = Receiver_Address;

	int retval = I2C_ICD_read_para(pBuffer, command, Address, NumByteToRead);
	retval = I2C_ICD_read_para(pBuffer, command, Address, NumByteToRead);

	return retval;
}


/**
 * @brief 读取buffter帧数
 *
 * @para  返回2个字节
 *
 *@retval  无
 */
int I2C_ICD_read_countofframe(uint8_t* pBuffer)
{

	uint16_t NumByteToRead = 2;
	uint8_t command = Get_frame_number;
	uint8_t Address = Receiver_Address;

	int retval = I2C_ICD_read_para(pBuffer, command , Address ,NumByteToRead );
	retval = I2C_ICD_read_para(pBuffer, command , Address ,NumByteToRead );

	return retval;
}

/**
 * @brief 接收机buffter清零
 *
 * @para  无
 *
 *@retval  无
 */
int  I2C_ICD_sweep_butter()
{

	uint8_t command = Remove_frames;
	uint8_t Address = Receiver_Address;

	int retval = I2C_ICD_send_command( command, Address );

	return retval;
}


/**
 * @brief 信标清零
 *
 * @para  无
 *
 *@retval  无
 */
int  I2C_ICD_beacon_clear()
{

	uint8_t command = Clear_beacon;
	uint8_t Address = Transmitter_Address;

	int retval = I2C_ICD_send_command( command, Address );

	return retval;
}


/**
 * @brief 设置目的呼号
 *
 * @para  无
 *
 *@retval  无
 */
int I2C_ICD_Set_des_callsigns(uint8_t* pBuffer)
{
	uint8_t NumByteToWrite = 7;
	uint8_t command = Set_des_callsigns;
	uint8_t Address = Transmitter_Address;

	int retval = I2C_ICD_send_para( pBuffer,command ,NumByteToWrite, Address );

	return retval;
}

/**
 * @brief 设置源呼号
 *
 * @para  无
 *
 *@retval  无
 */
int I2C_ICD_Set_orig_callsigns(uint8_t* pBuffer)
{
	uint8_t NumByteToWrite = 7;
	uint8_t command = Set_orig_callsigns;
	uint8_t Address = Transmitter_Address;

	int retval = I2C_ICD_send_para( pBuffer,command ,NumByteToWrite, Address );

	return retval;
}


/**
 * @brief 设置发射机的休眠状态
 *
 * @para  无
 *
 *@retval  无
 */
int I2C_ICD_Set_transmitter_idle_state(uint8_t* pBuffer)
{
	uint8_t NumByteToWrite = 1;
	uint8_t command = Set_transmitter_idle_state;
	uint8_t Address = Transmitter_Address;

	int retval = I2C_ICD_send_para( pBuffer,command ,NumByteToWrite, Address );

	return retval;
}

/**
 * @brief 信标设置   or AX.25信标设置
 *
 * @para  无
 *
 *@retval  无
 */
/*
int I2C_ICD_beacon_Set(uint8_t* pBuffer,uint8_t NumByteToWrite,uint8_t command)
{
	uint8_t Address = Transmitter_Address;
	int retval = I2C_ICD_send_para (pBuffer, command , NumByteToWrite, Address );

	return retval;
}
*/



/**
 * @brief 设置发射码速率
 *
 * @para  无
 *
 *@retval  无
 */
int I2C_ICD_Set_transmission_bitrate(uint8_t* pBuffer)
{
	uint8_t NumByteToWrite = 1;
	uint8_t command = Set_transmission_bitrate;
	uint8_t Address = Transmitter_Address;

	int retval = I2C_ICD_send_para (pBuffer, command , NumByteToWrite, Address );

	return retval;
}

int set_transmission_bitrate_1200(void)
{
	uint8_t rate 			= 1;
	uint8_t NumByteToWrite 	= 1;
	uint8_t command 		= Set_transmission_bitrate;
	uint8_t Address 		= Transmitter_Address;

	int retval = I2C_ICD_send_para (&rate, command , NumByteToWrite, Address );

	return retval;
}

int set_transmission_bitrate_4800(void)
{
	uint8_t rate 			= 4;
	uint8_t NumByteToWrite 	= 1;
	uint8_t command 		= Set_transmission_bitrate;
	uint8_t Address 		= Transmitter_Address;

	int retval = I2C_ICD_send_para (&rate, command , NumByteToWrite, Address );

	return retval;
}

/**
 * @brief 看门狗复位
 *
 * @para  无
 *
 *@retval  无
 */
int I2C_ICD_Watchingdog_reset()
{

	uint8_t command =  Watchingdog_reset;
	uint8_t Address = Transmitter_Address;

	int retval = I2C_ICD_send_command( command, Address );

	return retval;
}

/**
 * @brief 软件复位
 *
 * @para  无
 *
 *@retval  无
 */
int I2C_ICD_software_reset()
{

	uint8_t command = Software_reset;
	uint8_t Address = Transmitter_Address;

	int retval = I2C_ICD_send_command( command, Address );

	return retval;
}


/**
 * @brief  硬件复位
 *
 * @para  无
 *
 *@retval  无
 */
int I2C_ICD_Hardware_system_reset()
{

	uint8_t command = Hardware_system_reset;
	uint8_t Address = Transmitter_Address;

	int retval = I2C_ICD_send_command( command, Address );

	return retval;
}



/*
 * @brief  Date_with_call
 *
 * @para  无
 *
 *@retval  无
 */




