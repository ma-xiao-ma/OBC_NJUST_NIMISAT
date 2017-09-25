

#ifndef SPI_H_
#define SPI_H_

#include <stdint.h>

#include <FreeRTOS.h>
#include <semphr.h>

#include "stm32f4xx.h"

#define SPI_TIMEOUT	(10 * configTICK_RATE_HZ)

#define BSP_SPI                           SPI1
#define BSP_SPI_CLK                       RCC_APB2Periph_SPI1
#define BSP_SPI_CLK_INIT                  RCC_APB2PeriphClockCmd

#define BSP_SPI_HANDLER					  SPI1_IRQHandler

#define BSP_SPI_SCK_PIN                   GPIO_Pin_5
#define BSP_SPI_SCK_GPIO_PORT             GPIOA
#define BSP_SPI_SCK_GPIO_CLK              RCC_AHB1Periph_GPIOA
#define BSP_SPI_SCK_SOURCE                GPIO_PinSource5
#define BSP_SPI_SCK_AF                    GPIO_AF_SPI1

#define BSP_SPI_MISO_PIN                  GPIO_Pin_6
#define BSP_SPI_MISO_GPIO_PORT            GPIOA
#define BSP_SPI_MISO_GPIO_CLK             RCC_AHB1Periph_GPIOA
#define BSP_SPI_MISO_SOURCE               GPIO_PinSource6
#define BSP_SPI_MISO_AF                   GPIO_AF_SPI1

#define BSP_SPI_MOSI_PIN                  GPIO_Pin_7
#define BSP_SPI_MOSI_GPIO_PORT            GPIOA
#define BSP_SPI_MOSI_GPIO_CLK             RCC_AHB1Periph_GPIOA
#define BSP_SPI_MOSI_SOURCE               GPIO_PinSource7
#define BSP_SPI_MOSI_AF                   GPIO_AF_SPI1

#define BSP_CS_PIN0                       GPIO_Pin_4
#define BSP_CS_GPIOA_PORT                 GPIOA
#define BSP_CS_GPIOA_CLK                  RCC_AHB1Periph_GPIOA

#define BSP_CS_PIN1                       GPIO_Pin_15
//#define BSP_CS_GPIOA_PORT                 GPIOA
//#define BSP_CS_GPIOA_CLK                  RCC_AHB1Periph_GPIOA

#define BSP_CS_PIN2                       GPIO_Pin_1
#define BSP_CS_GPIOB_PORT                 GPIOB
#define BSP_CS_GPIOB_CLK                  RCC_AHB1Periph_GPIOB

/* Select BSP_SPI: Chip Select pin low */
#define BSP_CS_LOW(PIN)       if(PIN == 2) GPIO_ResetBits(BSP_CS_GPIOB_PORT, PIN); \
								else GPIO_ResetBits(BSP_CS_GPIOA_PORT, PIN)
/* Deselect BSP_SPI: Chip Select pin high */
#define BSP_CS_HIGH(PIN)      if(PIN == 2) GPIO_SetBits(BSP_CS_GPIOB_PORT, PIN); \
								else GPIO_SetBits(BSP_CS_GPIOA_PORT, PIN)

/* Select BSP_SPI: All chip Select pin low */
#define BSP_CS_ALOW()         {GPIO_ResetBits(BSP_CS_GPIOA_PORT, BSP_CS_PIN0 | BSP_CS_PIN1); \
								GPIO_ResetBits(BSP_CS_GPIOB_PORT, BSP_CS_PIN2);}
/* Deselect BSP_SPI: All chip Select pin high */
#define BSP_CS_AHIGH()        {GPIO_SetBits(BSP_CS_GPIOA_PORT, BSP_CS_PIN0 | BSP_CS_PIN1); \
								GPIO_SetBits(BSP_CS_GPIOB_PORT, BSP_CS_PIN2);}


/**
 * Parameters for a single chip-select line
 **/
typedef struct __attribute__((packed)) {
	uint16_t Direction;
	uint16_t Mode;
	uint16_t DataSize;
	uint16_t CPOL;
	uint16_t CPHA;
	uint16_t NSS;
	uint16_t BaudRatePrescaler;
	uint16_t FirstBit;
	uint16_t CRCPolynomial;

	uint32_t Cs;		  		  /*!< Chip select: value from 0 ~ 2*/
} spi_chip_t;

/**
 * Initialize SPI device
 * @param spi_dev A SPI module hardware configuration struct
 * @param spi_hw_index Hardware number (0 for SPI0, 1 for SPI1)
 */
int spi_init_dev(void);

/**
 * Lock device
 * @param spi_dev
 * @return 0 if OK, -1 if error
 */
int spi_lock_dev(void);

/**
 * Unlock device
 * @param spi_dev
 */
void spi_unlock_dev(void);

/**
 * After initializing hardware using spi_init_dev, this function must be called to setup the CS line
 * @param spi_chip a chip configuration struct, this must be pre-configured with spi_dev and all options.
 */
int spi_setup_chip(spi_chip_t * spi_chip);

/**
 * Send a byte/word via SPI
 * Depending on the transfer-size specified in spi_setup_chip, 8 to 16 bits
 * will be transferred from the 16 bit data-word
 * @param spi_chip The chip to send to
 * @param 8-16 bits to send
 */
void spi_write(spi_chip_t * spi_chip, uint16_t data);
uint16_t BSP_SendHalfWord(spi_chip_t * spi_chip, uint16_t halfword);

/**
 * Read a byte/word from SPI
 * Depending on the transfer-size specified in spi_setup_chip, 8 to 16
 * bits will be received from the SPI device.
 * @param spi_chip Chip to read from
 * @return bits received
 */
uint16_t spi_read(spi_chip_t * spi_chip);

#endif /* SPI_H_ */
