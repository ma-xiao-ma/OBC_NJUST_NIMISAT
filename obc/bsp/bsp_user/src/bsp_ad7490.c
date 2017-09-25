/*
*******************************************************************************************************
**bsp_ad7490.c
**
**
********************************************************************************************************
*/

#include "bsp_ad7490.h"

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

#include "bsp_spi.h"

#include "stm32f4xx.h"

uint16_t ad7490_data[16];

static spi_chip_t ad7490chip = {
		.Direction = SPI_Direction_2Lines_FullDuplex,
		.Mode = SPI_Mode_Master,
		.BaudRatePrescaler = SPI_BaudRatePrescaler_256,
		.DataSize = SPI_DataSize_16b,
		.CPOL = SPI_CPOL_High,
		.CPHA = SPI_CPHA_1Edge,
		.NSS = SPI_NSS_Soft,
		.FirstBit = SPI_FirstBit_MSB,
		.CRCPolynomial = 7,
		.Cs = 2,
};


void AD7490_Init(void)  //NOT USED
{
	uint32_t i;
	uint16_t ctr_reg;

	/* write to control register */
    /* configure	to shadow mode,normal power mode normal range ,bin coding format */
	ctr_reg = WRITE | SEQ_CFG | CHANNEL_0 | POWER_NORMAL | RANGE_DOUBLE | DATA_BIN;
	AD_SendByte_own(ctr_reg);
	for(i=0;i<20000;i++)
	{}

	/* write shadow register with all channels selected */
	AD_SendByte_own(ALL_CHANNEL);

	/* write to control register with a power off mode */
//	ctr_reg = WRITE | SEQ_CFG | CHANNEL_0 | POWER_FULLDOWN | RANGE_NORMAL | DATA_BIN;
//	AD_SendByte_own(ctr_reg);
}

uint16_t *AD7490_Read_NoIntSeq_own(void)
{
	uint16_t i;
	uint8_t channel;
	uint16_t *prxdata;
	uint16_t rxdata;
	prxdata = ad7490_data;
		
	for(i=0;i<16;i++)
	{
		//write 0x0000 to the ad7490 to read converted value
		//the outputs contain the channeel num and adc value in a 16 bits data
		// the format is 4 bits channel num MSB and the 12bits LSB is adc value
		rxdata = AD_SendByte_own(DATA_UPDATE);
		channel = (uint8_t)(rxdata>>12);
		ad7490_data[channel] = rxdata & 0x0fff;
		printf("ad_data[%2d]	%4x\r\n",channel,ad7490_data[channel]);
	}
	//configure the ad7490 to power down mode 
//	ctr_reg = WRITE | SEQ_CFG | CHANNEL_0 | POWER_FULLDOWN | RANGE_NORMAL| DATA_BIN;
//	AD_SendByte_own(ctr_reg);

	return prxdata;
}

void AD7490_Read(void)
{
	uint16_t i;
	uint8_t channel;
	uint16_t rxdata;

	for(i=0;i<16;i++)
	{
		rxdata = AD_SendByte_own(DATA_UPDATE);
		channel = (uint8_t)(rxdata>>12);
		ad7490_data[channel] = rxdata & 0x0fff;
	}
}

uint16_t AD_SendByte_own(uint16_t _ucValue)
{
	uint16_t data = 0;

	data = BSP_SendHalfWord(&ad7490chip, (uint16_t) _ucValue);

	return data;
}






/****************************************************************************************************/

