/**
 * spi.c
 *
 *  Created on: Aug 19, 2009
 *      Author: karl
 *
 */

#include "bsp_spi.h"

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include <task.h>

#include "stm32f4xx.h"

#include "driver_debug.h"

/**
 * Keep a record of the current, or latest used SPI device
 */
spi_chip_t * current_chip = NULL;

/**
 * Interrupt Service Routine and DMA semaphore
 */
xSemaphoreHandle spi_lock;

void BSP_LowLevel_DeInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /*!< Disable the BSP_SPI  ************************************************/
  SPI_Cmd(BSP_SPI, DISABLE);

  /*!< DeInitializes the BSP_SPI *******************************************/
  SPI_I2S_DeInit(BSP_SPI);

  /*!< BSP_SPI Periph clock disable ****************************************/
  BSP_SPI_CLK_INIT(BSP_SPI_CLK, DISABLE);

  /*!< Configure all pins used by the SPI as input floating *******************/
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

  GPIO_InitStructure.GPIO_Pin = BSP_SPI_SCK_PIN;
  GPIO_Init(BSP_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = BSP_SPI_MISO_PIN;
  GPIO_Init(BSP_SPI_MISO_GPIO_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = BSP_SPI_MOSI_PIN;
  GPIO_Init(BSP_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = BSP_CS_PIN0 | BSP_CS_PIN1;
  GPIO_Init(BSP_CS_GPIOA_PORT, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = BSP_CS_PIN2;
  GPIO_Init(BSP_CS_GPIOB_PORT, &GPIO_InitStructure);
}


/**
 * Wait for SPI hardware to become ready, with simple timeout.
 * @return positive number if ready, zero if not.
 */
static inline int spi_busy_wait_tx(void) {
	volatile unsigned int j = 100000;

	/*!< Loop while DR register in not emplty */
	while (SPI_I2S_GetFlagStatus(BSP_SPI, SPI_I2S_FLAG_TXE) == RESET&& j--)
		continue;
	return j;
}

/**
 * Wait for SPI hardware to become ready, with simple timeout.
 * @return positive number if ready, zero if not.
 */
static inline int spi_busy_wait_rx(void) {
	volatile unsigned int j = 100000;
	while (SPI_I2S_GetFlagStatus(BSP_SPI, SPI_I2S_FLAG_RXNE) == RESET && j--)
		continue;
	return j;
}

void BSP_SPI_IO_init(void) {

	GPIO_InitTypeDef GPIO_InitStructure;

	/*!< Enable the SPI clock */
	BSP_SPI_CLK_INIT(BSP_SPI_CLK, ENABLE);

	/*!< Enable GPIO clocks */
	RCC_AHB1PeriphClockCmd(BSP_SPI_SCK_GPIO_CLK | BSP_SPI_MISO_GPIO_CLK |
						 BSP_SPI_MOSI_GPIO_CLK | BSP_CS_GPIOA_CLK | BSP_CS_GPIOB_CLK, ENABLE);

	/*!< SPI pins configuration *************************************************/

	/*!< Connect SPI pins to AF5 */
	GPIO_PinAFConfig(BSP_SPI_SCK_GPIO_PORT, BSP_SPI_SCK_SOURCE, BSP_SPI_SCK_AF);
	GPIO_PinAFConfig(BSP_SPI_MISO_GPIO_PORT, BSP_SPI_MISO_SOURCE, BSP_SPI_MISO_AF);
	GPIO_PinAFConfig(BSP_SPI_MOSI_GPIO_PORT, BSP_SPI_MOSI_SOURCE, BSP_SPI_MOSI_AF);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;

	/*!< SPI SCK pin configuration */
	GPIO_InitStructure.GPIO_Pin = BSP_SPI_SCK_PIN;
	GPIO_Init(BSP_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);

	/*!< SPI MOSI pin configuration */
	GPIO_InitStructure.GPIO_Pin =  BSP_SPI_MOSI_PIN;
	GPIO_Init(BSP_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);

	/*!< SPI MISO pin configuration */
	GPIO_InitStructure.GPIO_Pin =  BSP_SPI_MISO_PIN;
	GPIO_Init(BSP_SPI_MISO_GPIO_PORT, &GPIO_InitStructure);

	/*!< Configure BSP Card CS pin in output pushpull mode ********************/
	GPIO_InitStructure.GPIO_Pin = BSP_CS_PIN0 | BSP_CS_PIN1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(BSP_CS_GPIOA_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = BSP_CS_PIN2;
	GPIO_Init(BSP_CS_GPIOB_PORT, &GPIO_InitStructure);
}

/**
 * Initialize SPI device
 * @param spi_dev A SPI module hardware configuration struct
 * @param spi_hw_index Hardware number (0 for SPI0, 1 for SPI1)
 */
int spi_init_dev(void) {

	/* Create device lock semaphore */
	spi_lock = NULL;
	spi_lock = xSemaphoreCreateMutex();

	BSP_SPI_IO_init();

	/*!< Deselect the FLASH: Chip Select high */
	BSP_CS_AHIGH();

	return 1;

}

/**
 * @brief After initializing hardware using spi_init_dev, this function must be called to setup the CS line
 * @param spi_chip a chip configuration struct, this must be pre-configured with spi_dev and all options.
 */
int spi_setup_chip(spi_chip_t * spi_chip) {

	/* Calculate the value of the SPI mode
	 * 0: CPOL = 0, CPHA = 0;
	 * 1: CPOL = 0, CPHA = 1;
	 * 2: CPOL = 1, CPHA = 0;
	 * 3: CPOL = 1, CPHA = 1;

	/*!< Enable the BSP_SPI  */
	SPI_Cmd(BSP_SPI, DISABLE);

	SPI_Init(BSP_SPI, (SPI_InitTypeDef*)spi_chip);
	/*!< Enable the BSP_SPI  */
	SPI_Cmd(BSP_SPI, ENABLE);

	current_chip = spi_chip;

	return 1;
}

void chip_select_n(uint32_t cs) {
	switch(cs) {
		case 0:
			BSP_CS_AHIGH();
			BSP_CS_LOW(BSP_CS_PIN0);
			break;
		case 1:
			BSP_CS_AHIGH();
			BSP_CS_LOW(BSP_CS_PIN1);
			break;
		case 2:
			BSP_CS_AHIGH();
			BSP_CS_LOW(BSP_CS_PIN2);
			break;
		default:
			driver_debug(DEBUG_SPI,"Chip Select Error<0~2>\n");
			break;
	}
}

void chip_select_p(uint32_t cs) {
	switch(cs) {
		case 0:
			BSP_CS_HIGH(BSP_CS_PIN0);
			break;
		case 1:
			BSP_CS_HIGH(BSP_CS_PIN1);
			break;
		case 2:
			BSP_CS_HIGH(BSP_CS_PIN2);
			break;
		default:
			driver_debug(DEBUG_SPI,"Chip Select Error<0~2>\n");
			break;
	}
}

int spi_lock_dev(void) {
	if ((!spi_lock))
		return -1;
	if (xSemaphoreTake(spi_lock, SPI_TIMEOUT) == pdFALSE) {
		driver_debug(DEBUG_SPI,"SD_LOCK");
		return -1;
	}
	return 0;
}

void spi_unlock_dev(void) {
	if ((!spi_lock))
		return;
	xSemaphoreGive(spi_lock);
}

/**
 * Send a byte via SPI
 * @param spi_chip The chip to send to
 * @param 16 bits to send
 */
uint16_t BSP_SendHalfWord(spi_chip_t * spi_chip, uint16_t halfword)
{
	volatile unsigned int j = 100000;
	uint16_t data = 0;

	/* Re-initialise new chip in new mode */
	if (current_chip != spi_chip) {
		spi_setup_chip(spi_chip);
		current_chip = spi_chip;
	}

	spi_lock_dev();
	chip_select_n(spi_chip->Cs);

	/*!< Loop while DR register in not emplty */
	while (SPI_I2S_GetFlagStatus(BSP_SPI, SPI_I2S_FLAG_TXE) == RESET && j--);
	if(j == 0) driver_debug(DEBUG_SPI,"OVER TIME\n");
	/*!< Send byte through the SPI1 peripheral */
	SPI_I2S_SendData(BSP_SPI, halfword);

	j = 100000;
	/*!< Wait to receive a byte */
	while (SPI_I2S_GetFlagStatus(BSP_SPI, SPI_I2S_FLAG_RXNE) == RESET && j--);
	if(j == 0) driver_debug(DEBUG_SPI,"OVER TIME\n");

	data = SPI_I2S_ReceiveData(BSP_SPI);
	SPI_I2S_ClearFlag(BSP_SPI, SPI_I2S_FLAG_RXNE);

	chip_select_p(spi_chip->Cs);
	spi_unlock_dev();
	/*!< Return the byte read from the SPI bus */
	return data;
}

void spi_write(spi_chip_t * spi_chip, uint16_t byte) {

	spi_lock_dev();

	spi_busy_wait_tx();

	/* Re-initialise new chip in new mode */
	if (current_chip != spi_chip) {
		spi_setup_chip(spi_chip);
		current_chip = spi_chip;
	}

	BSP_SPI->DR = byte;

	spi_unlock_dev();
}

/**
 * Read a byte from SPI
 * @param spi_chip Chip to read from
 * @return 16 bits received
 */
uint16_t spi_read(spi_chip_t * spi_chip) {
	spi_busy_wait_rx();
	uint16_t data = BSP_SPI->DR;
	return data;
}
