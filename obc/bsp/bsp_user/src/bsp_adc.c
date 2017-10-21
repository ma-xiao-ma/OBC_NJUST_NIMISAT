////////////////////////////////////////////////////////////////////////////////
//	功能： 电源板电量、温度采集校正
//
//	版本：V1.0
//  迭代：
//												南京理工大学微纳卫星中心
//												   2015.11.08
////////////////////////////////////////////////////////////////////////////////
#include "bsp_ad7490.h"
#include "stm32f4xx_rcc.h"
#include "bsp_adc.h"
#include "command.h"
#include "bsp_switch.h"
#include "FreeRTOS.h"
#include "queue.h"

////////////////////////////////////本地函数定义///////////////////////////////////
void InitSPI1(void);
void InitSPI1_GPIO(void);

//void ObcAdStart(void);
void EpsAdStart(void);
//void TempAdStart(void);


//uint8_t ObcAdUpdate(void);
//uint8_t TempAdUpdate(void);

//void ObcDataProcess(uint16_t data);

//uint16_t ObcSendByte(uint16_t _ucValue);
uint16_t EpsSendByte(uint16_t _ucValue, uint8_t _chipNum);


//ObcAdcValue_t* ObcAdToReal(uint16_t* rec, ObcAdcValue_t* tar);


////////////////////////////////////////////////////////////////////////////////

uint16_t TempAdValue[16][5];              		//电源板温度采样点
uint16_t TempAdValueAver[16] = {0};			  	//电源板温度均值
uint16_t ObcAdValue[16][5];              		//电源板电压采样点
uint16_t ObcAdValueAver[16] = {0};				//电源板电压均值
uint16_t EpsAdValue[32][5];   					//电源板电源获取量采样点
uint16_t EpsAdValueAver[32] = {0};				//电源板电源获取量均值

ObcAdcValue_t ObcAdData;
uint8_t EpsCaliFlag = 0;
int16_t EpsCaliTable[REG_NUM + UREG_NUM] = {0};

//const int AD_Channel[16] = { CHANNEL_0, CHANNEL_1, CHANNEL_2, CHANNEL_3,
//CHANNEL_4, CHANNEL_5, CHANNEL_6, CHANNEL_7,
//CHANNEL_8, CHANNEL_9, CHANNEL_10, CHANNEL_11,
//CHANNEL_12, CHANNEL_13, CHANNEL_14, CHANNEL_15 };

//// obc采用值->真实值映射表
//// [1]6通道粗太敏，默认转换方程为Vout = 5.0 - Vin
//// [2]1通道星务板5V采样
//// [3]1通道星务板3V3采样
//// [4]2通道温度传感器采样
//// [5]6通道保留通道采样
//const uint8_t ObcAdMap[] = { 1, 2, 4, 3, 6, 5,  //in_sun_v[6]
//        0,                  	//in_obc_5v
//		9,						//in_obc_3v3
//		7, 8,					//obc_temp[2]
//		10, 11, 12, 13, 14, 15	//obc_rev[6]
//		};
//
//CaliFactor_t AdcCaliMap[] = { { -1.0, 5000 }, { -1.0, 5000 }, { -1.0, 5000 }, {
//		-1.0, 5000 }, { -1.0, 5000 }, { -1.0, 5000 }, { 1.0, 0 }, { 1.0, 0 }, //OBC电压基准
//		{ 1.0, 0 }, { 1.0, 0 }, //OBC温度传感器
//		{ 1.0, 0 }, { 1.0, 0 }, { 1.0, 0 }, { 1.0, 0 }, { 1.0, 0 }, { 1.0, 0 }, //保留通道数
//		};


/**
 * EPS模拟通道映射表
 *
 * 其中0--15对应于电源控制板ADC_1:AD7490_1通道0--15
 *   16--31对应于电源控制板ADC_2:AD7490_2通道0--15
 */
const uint8_t EpsAdcMap[] =
{
    29, 30, 10,  7,  6,  5, //In_SunC[SV_NUM] 光电流
    28, 11,  9,  8,  4,  3,	//In_SunV[SV_NUM] 光电压
    16,                     //Out_ComC 通信板电流
    12,                     //Out_BusC 输出母线电流
    24,		                //Out_BusV 输出母线电压
    17, 18, 19, 27, 15, 14,	//Out_BranchC[REG_NUM] 可控输出的电流遥测
    23, 21, 13,		        //Out_BranchC[+UREG_NUM] 母线保护输出电流遥测
    31,  0, 26, 25,			//EpsTemp[EPS_TEMP_NUM] 电源控制板温度
     1,  2, 20, 22,			//BatTemp[BAT_TEMP_NUM] 电池板温度
};

//EPS采样值ֵ->真实值换算表
//定义adc数据比例，电压量为分压比，典型值ֵ49.9k/10k,转换关系为V_real = V*Vdiv
//电流量为1/(R*G),典型值R=0.051欧，G=25V/V;总线电流采样电阻R=0.051/2
//转换关系为C=V*(1/R*G)
//温度量为V = 0.75+0.01*(T-25);转换关系为T = (V-0.75)/0.01+25

#if 1  //老版本EPS
const float AdcDiv[] =
{
    0.7843, 0.7843, 0.7843, 0.7843, 0.7843, 0.7843,//In_SunC[SV_NUM] 光电流
      5.99,   5.99,   5.99,   5.99,   5.99,   5.99,//In_SunV[SV_NUM] 光电压
    0.7843, //Out_ComC 通信板电流
    1.5686, //Out_BusC 输出母线电流
      6.10, //Out_BusV 输出母线电压
    0.7843, 0.7843, 0.7843, 0.7843, 0.7843, 0.7843,//Out_BranchC[REG_NUM] 可控输出的电流遥测
    0.7843, 0.7843, 0.7843,         //Out_BranchC[+UREG_NUM] 母线保护输出电流遥测
    1.0, 1.0, 1.0, 1.0,             //EpsTemp[EPS_TEMP_NUM] 电源控制板温度
    1.0, 1.0 ,1.0, 1.0,             //BatTemp[BAT_TEMP_NUM] 电池板温度
};
#else
const float AdcDiv[]=
{
    0.7843, 0.7843, 0.7843, 0.7843, 0.7843, 0.7843,
	6.10, 6.10, 6.10, 6.10, 6.10, 6.10,
	0.7843, 1.5686, 6.10,
	0.7843, 0.7843, 0.7843, 0.7843, 0.7843, 0.7843,
	0.7843, 0.7843, 0.7843, 0.7843, 0.7843,
	1.0, 1.0, 1.0, 1.0, 1.0, 1.0
};
#endif

//误差校准矩阵
//CaliFactor_t EpsCaliMap[] = { { 1.0, 0 }, { 1.0, 0 }, { 1.0, 0 }, { 1.0, 0 }, {
//		1.0, 0 }, { 1.0, 0 }, //c_s[6]
//{ 1.0, 0 }, { 1.0, 0 }, { 1.0, 0 }, { 1.0, 0 }, { 1.0, 0 }, { 1.0, 0 }, //v_s[6]
//{ 1.0 / 1.25, -1 }, { 1.0, 0 }, { 1.0, 0 }, //c_com,c_bus,v_bus
//{ 1.0 / 1.06, 0 }, { 1.0 / 1.08, 0 }, { 1.0 / 1.03, 0 }, { 1.0 / 0.96, -5 }, {
//		1.0 / 1.0, -15 }, { 1.0, 0 }, //c_reg[6]
//{ 1.0 / 1.064, -10 }, { 1.0 / 1.07, 0 }, { 1.0 / 1.07, 0 }, { 1.0 / 1.07, 30 },
//		{ 1.0 / 1.022, 0 }, //c_ureg[5]
//};

CaliFactor_t EpsCaliMap[] =
{
    { 1.0, 0 }, { 1.0, 0 }, { 1.0, 0 }, { 1.0, 0 }, { 1.0, 0 }, { 1.0, 0 }, //In_SunC[SV_NUM] 光电流
    { 1.0, 0 }, { 1.0, 0 }, { 1.0, 0 }, { 1.0, 0 }, { 1.0, 0 }, { 1.0, 0 }, //In_SunV[SV_NUM] 光电压
    { 1.0 / 1.25, -1 }, //Out_ComC 通信板电流
    { 1.0, 0 },         //Out_BusC 输出母线电流
    { 1.0, 0 },         //Out_BusV 输出母线电压
    { 1.0 / 1.06, 0 }, { 1.0, 0 }, { 1.0, 0 }, { 1.0, -5 }, {1.0 / 1.0, -15 }, { 1.0, 0 }, //Out_BranchC[REG_NUM] 可控输出的电流遥测
    { 1.0, 0 }, { 1.0, 0 },{ 1.0 / 1.022, 0 },  //Out_BranchC[+UREG_NUM] 母线保护输出电流遥测
};



uint8_t EpsAdCorrectEnable = 1;
EpsAdcValue_t EpsHouseKeeping;
extern QueueHandle_t eps_hk_queue;
uint16_t EpsTranOTCnt = 0;	//发送超时计数
uint16_t EpsRevOTCnt = 0;   //接收超时计数

void bsp_InitSPI1(void)
{
	InitSPI1_GPIO();
	InitSPI1();
}

void InitSPI1(void)
{
	SPI_InitTypeDef SPI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	SPI_Cmd(SPI1, DISABLE);

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; // 数据方向：2线全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;	//工作模式 ：主机模式
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;	//数据位长度 16b

	// SPI_CPOL和SPI_CPHA结合使用决定时钟和数据采样点的相位关系
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;	// 时钟上升沿采样数据
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;	// 时钟的第1个边沿采样数据
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;	// 片选控制方式：软件控制

	// 设置波特率预分频系数 SPI_BaudRatePrescaler_128
	SPI_InitStructure.SPI_BaudRatePrescaler = OBC_SPI_BAUD;	//OBC_SPI_BAUD
	// 数据位传输次序：高位先传
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;	// CRC多项式寄存器，复位后为7
	SPI_Init(SPI1, &SPI_InitStructure);

	SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_ERR, ENABLE);
	SPI_Cmd(SPI1, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = SPI1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void InitSPI1_GPIO(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	// 配置MSK、MISO、MOSI复用功能
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);
	// 配置MSK、MISO、MOSI引脚功能
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	// 配置片选线
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = OBC_CS_PIN;
	GPIO_Init(OBC_CS_GPIO, &GPIO_InitStructure);

//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_InitStructure.GPIO_Pin = TEMP_CS_PIN;
//	GPIO_Init(TEMP_CS_GPIO, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = EPS_CS1_PIN;
	GPIO_Init(EPS_CS1_GPIO, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = EPS_CS2_PIN;
	GPIO_Init(EPS_CS2_GPIO, &GPIO_InitStructure);

//	TEMP_CS_HIGH();
	OBC_CS_HIGH();
	EPS_CS1_HIGH();
	EPS_CS2_HIGH();
}

void SPI1_IRQHandler(void) {

	if (SPI_I2S_GetITStatus(SPI1, SPI_IT_CRCERR) == SET) {
		SPI_I2S_ClearITPendingBit(SPI1, SPI_IT_CRCERR);
	}

	if (SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_OVR) == SET) {
		SPI_I2S_ClearITPendingBit(SPI1, SPI_I2S_IT_OVR);
	}

	if (SPI_I2S_GetITStatus(SPI1, SPI_IT_MODF) == SET) {
		SPI_I2S_ClearITPendingBit(SPI1, SPI_IT_MODF);
	}
}

//uint16_t TempSendByte(uint16_t _ucValue) {
//	uint32_t i = 0;
//	// 等待上次数据发送完毕
//	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) {
//		asm volatile ( "NOP" );
//		asm volatile ( "NOP" );
//		asm volatile ( "NOP" );
//		if (i++ > 200000) {
//			EpsTranOTCnt++;
//			return 1;
//		}
//	}
//	TEMP_CS_LOW();
//	// 通过SPI1发送一个字节
//	SPI_I2S_SendData(SPI1, _ucValue);
//
//	// 等待接接收一个字节完成
//	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET) {
//		asm volatile ( "NOP" );
//		asm volatile ( "NOP" );
//		asm volatile ( "NOP" );
//		if (i++ > 200000) {
//			EpsRevOTCnt++;
//			return 1;
//		}
//	}
//	// 返回SPI接收字节
//	TEMP_CS_HIGH();
//	return SPI_I2S_ReceiveData(SPI1);
//}

//void TempAdStart(void) {
//	uint16_t command;
//	command = WRITE | SEQ_CFG | CHANNEL_0 | POWER_NORMAL | RANGE_DOUBLE
//			| DATA_BIN;
//
//	TempSendByte(command);
//	asm volatile ( "NOP" );
//	asm volatile ( "NOP" );
//	asm volatile ( "NOP" );
//	TempSendByte(ALL_CHANNEL);
//
//}

//void TempDataProcess(uint16_t data) {
//	uint8_t index;
//	uint8_t channel;
//	uint16_t ad_temp;
//
//	channel = (uint8_t) (data >> 12); // 获取转换的通道
//	ad_temp = data & 0x0FFF; //获取该通道的AD转换值
//
//	for (index = 0; index < 4; index++) {
//		TempAdValue[channel][index] = TempAdValue[channel][index + 1];
//	}
//	TempAdValue[channel][4] = ad_temp; //将AD值传入AD矩阵
//}
//
//uint8_t TempAdUpdate(void) {
//	uint16_t data;
//	data = TempSendByte(DATA_UPDATE);
//	TempDataProcess(data);
//	return 0;
//}

//void ObcDataProcess(uint16_t data) {
//	uint8_t index;
//	uint8_t channel;
//	uint16_t ad_temp;
//
//	channel = (uint8_t) (data >> 12); 	// 获取转换的通道
//	ad_temp = data & 0x0FFF; 			// 获取该通道的AD转换值
//
//	for (index = 0; index < 4; index++) {
//		ObcAdValue[channel][index] = ObcAdValue[channel][index + 1];
//	}
//	ObcAdValue[channel][4] = ad_temp; 	//将AD值传入AD矩阵
//}

//void ObcAdStart(void) {
//	uint16_t command;
//	command = WRITE | SEQ_CFG | CHANNEL_0 | POWER_NORMAL | RANGE_DOUBLE
//			| DATA_BIN;
//
//	ObcSendByte(command);
//	asm volatile ( "NOP" );
//	asm volatile ( "NOP" );
//	asm volatile ( "NOP" );
//	ObcSendByte(ALL_CHANNEL);
//
//}
//
//uint8_t ObcAdUpdate(void) {
//	uint16_t data;
//	data = ObcSendByte(DATA_UPDATE);
//	ObcDataProcess(data);
//	return 0;
//}

//uint16_t ObcSendByte(uint16_t _ucValue) {
//	uint32_t i = 0;
//	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) {
//		asm volatile ( "NOP" );
//		asm volatile ( "NOP" );
//		asm volatile ( "NOP" );
//		if (i++ > 200000) {
//			EpsTranOTCnt++;
//			return 1;
//		}
//	}
//	OBC_CS_LOW();
//	SPI_I2S_SendData(SPI1, _ucValue);
//
//	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET) {
//		asm volatile ( "NOP" );
//		asm volatile ( "NOP" );
//		asm volatile ( "NOP" );
//		if (i++ > 200000) {
//			EpsRevOTCnt++;
//			return 1;
//		}
//	}
//	OBC_CS_HIGH();
//	return SPI_I2S_ReceiveData(SPI1);
//}

void EpsAdStart(void) {
	uint16_t command;
	command = WRITE | SEQ_CFG | CHANNEL_0 | POWER_NORMAL | RANGE_NORMAL
			| DATA_BIN;

	EpsSendByte(command, EPS_AD_CS1);
	asm volatile ( "NOP" );
	asm volatile ( "NOP" );
	asm volatile ( "NOP" );
	EpsSendByte(ALL_CHANNEL, EPS_AD_CS1);

	EpsSendByte(command, EPS_AD_CS2);
	asm volatile ( "NOP" );
	asm volatile ( "NOP" );
	asm volatile ( "NOP" );
	EpsSendByte(ALL_CHANNEL, EPS_AD_CS2);
}

void EpsDataProcess(uint16_t data, uint8_t chipNum)
{
	uint8_t index;
	uint8_t channel;
	uint16_t ad_temp;

	channel = (uint8_t) (data >> 12);
	ad_temp = data & 0x0FFF;

	if (chipNum == EPS_AD_CS2) //如果为AD2采样，则通道数加上偏移
	{
		channel += 16;
	}

	for (index = 0; index < 4; index++) {
		EpsAdValue[channel][index] = EpsAdValue[channel][index + 1];
	}
	EpsAdValue[channel][4] = ad_temp; // 将AD值传入AD矩阵
}

uint8_t EpsAdUpdate(uint8_t chip)
{
	uint16_t data;
	data = EpsSendByte(DATA_UPDATE, chip);
	EpsDataProcess(data, chip);
	return 0;
}

uint16_t EpsSendByte(uint16_t _ucValue, uint8_t _chipNum)
{
	uint32_t i = 0;
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) {
		asm volatile ( "NOP" );
		asm volatile ( "NOP" );
		asm volatile ( "NOP" );
		if (i++ > 200000) {
			EpsTranOTCnt++;
			return 1;
		}
	}

	if (_chipNum == EPS_AD_CS1) {
		EPS_CS1_LOW();
	} else if (_chipNum == EPS_AD_CS2) {
		EPS_CS2_LOW();
	}

	SPI_I2S_SendData(SPI1, _ucValue);

	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET) {
		asm volatile ( "NOP" );
		asm volatile ( "NOP" );
		asm volatile ( "NOP" );
		if (i++ > 200000) {
			EpsRevOTCnt++;
			return 1;
		}
	}

	if (_chipNum == EPS_AD_CS1) {
		EPS_CS1_HIGH();
	} else if (_chipNum == EPS_AD_CS2) {
		EPS_CS2_HIGH();
	}

	return SPI_I2S_ReceiveData(SPI1);
}

void AdDataFliter(uint16_t ad_table[][5], uint16_t* ad_aver_table,
		uint8_t channel_num) {
	uint16_t max;
	uint16_t min;
	uint16_t sum;
	uint8_t channel_temp;
	uint8_t index;

	for (channel_temp = 0; channel_temp < channel_num; channel_temp++) {
		max = ad_table[channel_temp][0];
		min = ad_table[channel_temp][0];
		sum = 0;
		for (index = 0; index < 5; index++) {
			max = ad_table[channel_temp][index] > max ?
					ad_table[channel_temp][index] : max;
			min = ad_table[channel_temp][index] < min ?
					ad_table[channel_temp][index] : min;
			sum = sum + ad_table[channel_temp][index];
		}

		ad_aver_table[channel_temp] = (sum - max - min) / 3;
	}
}

//ObcAdcValue_t* ObcAdToReal(uint16_t* rec, ObcAdcValue_t* tar) {
//	uint16_t *p;
//	int16_t *q;
//	uint8_t i;
//	uint16_t adc_temp;
//
//	p = (uint16_t *) (&(tar->InSunSensorV[0]));
//	q = (int16_t *) (&(tar->InSunSensorV[0]));
//	for (i = 0; i < 16; i++) 	//6通道粗太敏转换
//			{
//		if (i == 8 || i == 9) 	//温度通道
//			*(q + i) = (int16_t) (((float) (*(rec + ObcAdMap[i])) * ADC_REF
//					* 2.0f / ADC_FULL_SCALE) * AdcCaliMap[i].k)
//					+ AdcCaliMap[i].b;
//		else				//其他通道
//		{
//
//			adc_temp = (uint16_t) (((float) (*(rec + ObcAdMap[i])) * ADC_REF
//					* 2.0f / ADC_FULL_SCALE) * AdcCaliMap[i].k)
//					+ AdcCaliMap[i].b;
//			*(p + i) = adc_temp > 60000 ? 0 : adc_temp;
//		}
//	}
//	return tar;
//}

//void EpsAdCalibration(void) {
//	uint8_t i;
//	static uint8_t EpsCaliTime = 0;
//	if (!(EpsCaliFlag == EPS_FLAG_IDLE || EpsCaliFlag == EPS_FLAG_OK)) {
//		if (EpsCaliFlag & EPS_FLAG_ENTER) // 第一次进入
//		{
//			EpsOutSwitch(OUT_ALL, DISABLE);
//
//			for (i = 0; i < REG_NUM + UREG_NUM; i++) {
//				EpsCaliTable[i] = EpsHouseKeeping.Out_BranchC[i];
//			}
//			EpsCaliTime = 0;
//			EpsCaliFlag = EPS_FLAG_DOING;
//		} else //第N次进入
//		{
//			for (i = 0; i < REG_NUM + UREG_NUM; i++) {
//				EpsCaliTable[i] = (EpsHouseKeeping.Out_BranchC[i]
//						+ EpsCaliTable[i]) / 2;
//			}
//
//			if (EpsCaliTime++ > CALIBRATION_TIME) {
//				EpsCaliFlag = EPS_FLAG_OK;
//				for (i = 0; i < REG_NUM + UREG_NUM; i++) {
//					EpsCaliMap[i + 15].b = EpsCaliMap[i + 15].b
//							- EpsCaliTable[i];
//				}
//
////				EpsOutSwitch(OUT_MTQ, ENABLE);
////				EpsOutSwitch(OUT_MAG, ENABLE);
////				EpsOutSwitch(OUT_RES, ENABLE);
//			}
//		}
//	} else if (EpsCaliFlag == EPS_FLAG_OK) {
//		EpsCaliFlag = EPS_FLAG_IDLE;
//	}
//}

EpsAdcValue_t* EpsAdCorrect(EpsAdcValue_t* p) //EPS通道相关量修正
{
	//103ͨ通道->4ͨ通道影响修正
	int32_t x_temp;
	int32_t y_temp;
	uint8_t i;
	x_temp = p->Out_BranchC[0] + p->Out_BranchC[1] + p->Out_BranchC[3];

	if (x_temp > 25)
		y_temp = (int32_t) ((float) (0.1637f * x_temp) + 26.87f);
	else
		y_temp = (int32_t) ((float) (1.24f * x_temp) + 1.6f);

	p->Out_BranchC[4] = (uint16_t) (p->Out_BranchC[4] - y_temp);
	p->Out_BranchC[4] = p->Out_BranchC[4] > 60000 ? 0 : p->Out_BranchC[4];


//	 0->1  0.05743   0.5238
//	 1->0
//
//	 c->6  0.03759  -0.1304
//	 c->7,8,9 0.05761  0.2935
//
//
//	 7,8,9->6 0.03759   -0.1304
//	 7,8,9->c 0.04599   -0.1413
//	 6->7,8,9  0.0386   2.225
//
//	 6->c  0.03338   -0.8788

	p->Out_BranchC[0] -= (int32_t) ((float) p->Out_BranchC[1] * 0.05743f
			+ 0.5238f);
	p->Out_BranchC[0] = p->Out_BranchC[0] > 60000 ? 0 : p->Out_BranchC[0];

	p->Out_BranchC[1] -= (int32_t) ((float) p->Out_BranchC[0] * 0.05743f
			+ 0.5238f);
	p->Out_BranchC[1] = p->Out_BranchC[1] > 60000 ? 0 : p->Out_BranchC[1];

	x_temp = p->Out_BranchC[7] + p->Out_BranchC[8] + p->Out_BranchC[9]
			+ p->Out_BranchC[p->Out_ComC];
	p->Out_BranchC[6] -= (int32_t) ((float) x_temp * 0.03759f - 0.1304f);
	p->Out_BranchC[6] = p->Out_BranchC[6] > 60000 ? 0 : p->Out_BranchC[6];

	x_temp = p->Out_BranchC[7] + p->Out_BranchC[8] + p->Out_BranchC[9];
	x_temp = (int32_t) (((float) x_temp * 0.04599f - 0.1413f)
			+ ((float) p->Out_BranchC[6] * 0.03338f - 0.8788f));
	p->Out_ComC -= x_temp;
	p->Out_ComC = p->Out_ComC > 60000 ? 0 : p->Out_ComC;

	for (i = 7; i <= 9; i++)
	{
		x_temp = p->Out_BranchC[6] + p->Out_BranchC[7] + p->Out_BranchC[8]
				+ p->Out_BranchC[9];
		x_temp -= p->Out_BranchC[i];
		x_temp = (int32_t) (((float) x_temp * 0.0386f + 2.225f)
				+ ((float) p->Out_ComC * 0.05761f + 0.2935f));
		p->Out_BranchC[i] -= x_temp;
		p->Out_BranchC[i] = p->Out_BranchC[i] > 60000 ? 0 : p->Out_BranchC[i];
	}

	return p;
}


//AD值转换为实际值
void EpsAdToReal2(uint16_t* rec, EpsAdcValue_t* tar) {

	uint8_t i = 0;

    for (i = 0; i < SV_NUM; i++)
    {
        tar->In_SunC[i] = (uint16_t) (((float) (*(rec + EpsAdcMap[i]))) * AdcDiv[i]
                    * ADC_REF / ADC_FULL_SCALE * EpsCaliMap[i].k) + EpsCaliMap[i].b;

        tar->In_SunV[i] = (uint16_t) (((float) (*(rec + EpsAdcMap[i+6]))) * AdcDiv[i+6]
                    * ADC_REF / ADC_FULL_SCALE * EpsCaliMap[i+6].k) + EpsCaliMap[i+6].b;
    }
    /**
     * 还差通信板电流转换公式
     */
    tar->Out_BusC  = (uint16_t) (((float) (*(rec + EpsAdcMap[13]))) * AdcDiv[13]
                    * ADC_REF / ADC_FULL_SCALE * EpsCaliMap[13].k) + EpsCaliMap[13].b;

    tar->Out_BusV  = (uint16_t) (((float) (*(rec + EpsAdcMap[14]))) * AdcDiv[14]
                    * ADC_REF / ADC_FULL_SCALE * EpsCaliMap[14].k) + EpsCaliMap[14].b;

    for(i =0; i<REG_NUM+UREG_NUM; i++)
        tar->Out_BranchC[i] =(uint16_t) (((float) (*(rec + EpsAdcMap[i+15]))) * AdcDiv[i+15]
                   * ADC_REF / ADC_FULL_SCALE * EpsCaliMap[i+15].k) + EpsCaliMap[i+15].b;

	for (i = 0; i<EPS_TEMP_NUM; i++)
		tar->EpsTemp[i] = (int16_t) (((((float) (*(rec + EpsAdcMap[i + 24]))) * ADC_REF
				/ ADC_FULL_SCALE - 750) / 10) + 25);

    for (i = 0; i<BAT_TEMP_NUM; i++)
        tar->BatTemp[i] = (int16_t) (((((float) (*(rec + EpsAdcMap[i + 28]))) * ADC_REF
                / ADC_FULL_SCALE - 750) / 10) + 25);
}


//AD值转换为实际值
EpsAdcValue_t* EpsAdToReal(uint16_t* rec, EpsAdcValue_t* tar) {
	uint8_t i;
	uint16_t *p;
	int16_t *q;
	uint16_t adc_temp;
	p = (uint16_t*) tar;
	q = (int16_t*) (&(tar->EpsTemp[0]));

	for (i = 0; i < EPS_ADC_NUM - (EPS_TEMP_NUM + BAT_TEMP_NUM); i++) {
		adc_temp = (uint16_t) (((float) (*(rec + EpsAdcMap[i]))) * AdcDiv[i]
				* ADC_REF / ADC_FULL_SCALE * EpsCaliMap[i].k) + EpsCaliMap[i].b;
		*(p + i) = adc_temp > 60000 ? 0 : adc_temp;

//		if(*(p+i) < ZERO_DRIFT)		*(p+i) = 0; //出现零值漂移后直接置零
	}

	for (i = 0; i < (EPS_TEMP_NUM + BAT_TEMP_NUM); i++) {
		*(q + i) = (int16_t) (((((float) (*(rec + EpsAdcMap[i + 24]))) * ADC_REF
				/ ADC_FULL_SCALE - 750) / 10) + 25);
	}

	if (EpsAdCorrectEnable)
		EpsAdCorrect(tar); //转换修正
	return tar;
}

void eps_start(void) {
	EpsAdStart();   // 电源控制板两个AD7490配置
//	ObcAdStart();   // 星务板上一个AD7490配置
//	TempAdStart();  // 电路图无此使能引脚
	EpsCaliFlag = EPS_FLAG_ENTER;
}

//void eps_hk(void)
//{
//	uint8_t kc = 0;
//
//	for (kc = 0; kc < 16; kc++)
//	{
//		EpsAdUpdate(EPS_AD_CS1);
//		EpsAdUpdate(EPS_AD_CS2);
////		ObcAdUpdate();
////		TempAdUpdate();  //没有用到
//	}
//
//	AdDataFliter(EpsAdValue, EpsAdValueAver, 32);
//	EpsAdToReal(EpsAdValueAver, &EpsHouseKeeping);
//	//EpsAdToReal2(EpsAdValueAver, &EpsHouseKeeping);
//
//
//
////	AdDataFliter(ObcAdValue, ObcAdValueAver, 16);
////	ObcAdToReal(ObcAdValueAver, &ObcAdData);
////	AdDataFliter(TempAdValue, TempAdValueAver, 16);
//
////  EpsAdCalibration();
//
//}

/**
 *获取电源系统遥测值，由采集任务调用
 *
 * @param eps_hk 采集接收缓冲区
 */
void eps_get_hk(EpsAdcValue_t *eps_hk)
{
    uint8_t kc = 0;

    for (kc = 0; kc < 16; kc++)
    {
        EpsAdUpdate(EPS_AD_CS1);
        EpsAdUpdate(EPS_AD_CS2);
    }

    AdDataFliter(EpsAdValue, EpsAdValueAver, 32);
    EpsAdToReal(EpsAdValueAver, eps_hk);
//    EpsAdToReal2(EpsAdValueAver, eps_hk);
}


/**
 * 经过5个采样周期后，即 eps_get_hk(EpsAdcValue_t *eps_hk)函数被调用5次后
 * 各参数才有效
 *
 * @param ctx
 * @return
 */
int cmd_eps_read(struct command_context *ctx __attribute__((unused)))
{

	char *temp;

    if (eps_hk_queue == NULL)
        return CMD_ERROR_SYNTAX;

    if (xQueuePeek(eps_hk_queue, &EpsHouseKeeping, 0) != pdTRUE)
        return CMD_ERROR_SYNTAX;

	printf("\r\n************* EPS total infomation ************\r\n");
	printf("*             Bus Voltage: %04d mV            *\r\n",
			EpsHouseKeeping.Out_BusV);
	printf("*             Bus Current: %04d mA            *\r\n",
			EpsHouseKeeping.Out_BusC);
    printf("*             Com Current: %04d mA            *\r\n",
            EpsHouseKeeping.Out_ComC);
	printf("***********************************************\r\n");
	printf("*      Channel      Status      Current       *\r\n");

	if (SW_5_3_3V_1_PIN())
		temp = "on ";
	else
		temp = "off";
	printf("*       %s         %s        %03d mA        *\r\n",
	        "BLS1", temp, EpsHouseKeeping.Out_BranchC[0]);
	if (SW_5_3_3V_2_PIN())
		temp = "on ";
	else
		temp = "off";
	printf("*       %s         %s        %03d mA        *\r\n",
	        "BLS2", temp, EpsHouseKeeping.Out_BranchC[1]);
	if (SW_5_3_3V_3_PIN())
		temp = "on ";
	else
		temp = "off";
	printf("*       %s         %s        %03d mA        *\r\n",
	        "BLS3", temp, EpsHouseKeeping.Out_BranchC[2]);
	if (SW_7_4V_3_PIN())
		temp = "on ";
	else
		temp = "off";
	printf("*       %s         %s        %03d mA        *\r\n",
	        "BLL3", temp, EpsHouseKeeping.Out_BranchC[6]);
	if (SW_USB_EN_PIN())
		temp = "on ";
	else
		temp = "off";
	printf("*       %s         %s        %03d mA        *\r\n",
	        "BLL4", temp, EpsHouseKeeping.Out_BranchC[7]);
	if (SW_EPS_S0_PIN())
		temp = "on ";
	else
		temp = "off";
	printf("*       %s         %s        %03d mA        *\r\n",
	        "ADCS", temp, EpsHouseKeeping.Out_BranchC[8]);
//	if (SW_USB_EN_PIN())
//		temp = "on ";
//	else
//		temp = "off";
//	printf("*       %s           %s        %03dmA       *\r\n",
//	"PG15", temp, EpsHouseKeeping.Out_BranchC[9]);
//	if (SW_EPS_S0_PIN())
//		temp = "on ";
//	else
//		temp = "off";
//	printf("*       %s           %s        %03dmA       *\r\n",
//	"ADCS", temp, EpsHouseKeeping.Out_BranchC[10]);
	if (SW_EPS_S1_PIN())
		temp = "on ";
	else
		temp = "off";
	printf("*       %s         %s        %03d mA        *\r\n",
	        "BLS4", temp, EpsHouseKeeping.Out_BranchC[3]);
	if (SW_EPS_S2_PIN())
		temp = "on ";
	else
		temp = "off";
	printf("*       %s         %s        %03d mA        *\r\n",
	        "BLS5", temp, EpsHouseKeeping.Out_BranchC[4]);
	if (SW_EPS_S3_PIN())
			temp = "on ";
		else
			temp = "off";
    printf("*       %s         %s        %03d mA        *\r\n",
            "ANTS", temp, EpsHouseKeeping.Out_BranchC[5]);

	printf("***********************************************\r\n");
	printf("*              EPS board temperature          *\r\n");
	printf("*    Temperature test point        Value      *\r\n");

	printf("*            Point #1               %03d       *\r\n",
			EpsHouseKeeping.EpsTemp[0]);
	printf("*            Point #2               %03d       *\r\n",
			EpsHouseKeeping.EpsTemp[1]);
	printf("*            Point #3               %03d       *\r\n",
			EpsHouseKeeping.EpsTemp[2]);
	printf("*            Point #4               %03d       *\r\n",
			EpsHouseKeeping.EpsTemp[3]);
	printf("*           Battery #1              %03d       *\r\n",
			EpsHouseKeeping.BatTemp[0]);
	printf("*           Battery #2              %03d       *\r\n",
			EpsHouseKeeping.BatTemp[1]);
	printf("*           Battery #3              %03d       *\r\n",
            EpsHouseKeeping.BatTemp[2]);
    printf("*           Battery #4              %03d       *\r\n",
            EpsHouseKeeping.BatTemp[3]);
	printf("***********************************************\r\n");
	printf("*       %s            %03d mA              *\r\n",
	        "SINC1=", EpsHouseKeeping.In_SunC[0]);
	printf("*       %s            %03d mA              *\r\n",
	        "SINC2=", EpsHouseKeeping.In_SunC[1]);
	printf("*       %s            %03d mA              *\r\n",
	        "SINC3=", EpsHouseKeeping.In_SunC[2]);
	printf("*       %s            %03d mA              *\r\n",
	        "SINC4=", EpsHouseKeeping.In_SunC[3]);
	printf("*       %s            %03d mA              *\r\n",
	        "SINC5=", EpsHouseKeeping.In_SunC[4]);
	printf("*       %s            %03d mA              *\r\n",
	        "SINC6=", EpsHouseKeeping.In_SunC[5]);
	printf("***********************************************\r\n");
	printf("*       %s           %04d mV              *\r\n",
	        "SINV1=", EpsHouseKeeping.In_SunV[0]);
	printf("*       %s           %04d mV              *\r\n",
	        "SINV2=", EpsHouseKeeping.In_SunV[1]);
	printf("*       %s           %04d mV              *\r\n",
	        "SINV3=", EpsHouseKeeping.In_SunV[2]);
	printf("*       %s           %04d mV              *\r\n",
	        "SINV4=", EpsHouseKeeping.In_SunV[3]);
	printf("*       %s           %04d mV              *\r\n",
	        "SINV5=", EpsHouseKeeping.In_SunV[4]);
	printf("*       %s           %04d mV              *\r\n",
	        "SINV6=", EpsHouseKeeping.In_SunV[5]);
	printf("***********************************************\r\n");
	return CMD_ERROR_NONE;

}

struct command eps_subcommands[] = {
	{
		.name = "hk",
		.help = "eps read",
		.handler = cmd_eps_read,
	}
};

struct command __root_command eps_commands_master[] = {
	{
		.name = "eps",
		.help = "eps master commands",
		.chain = INIT_CHAIN(eps_subcommands),
	}
};

void cmd_eps_setup(void) {
	command_register(eps_commands_master);
}
