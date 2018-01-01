/*
 * driver_debug.h
 *
 *  Created on: 03/03/2011
 *      Author: johan
 */

#ifndef DRIVER_DEBUG_H_
#define DRIVER_DEBUG_H_

#include "command.h"

typedef enum driver_debug_e {
    DEBUG_ERROR = 0,
    DEBUG_WARN = 1,
    DEBUG_INFO = 2,
    DEBUG_I2C = 3,
	DEBUG_OBC = 4,
	DEBUG_SPI = 5,
	DEBUG_CAMERA = 6,
	DEBUG_ICD = 7,
	DEBUG_HK = 8,
	DEBUG_TTC = 9,
	DEBUG_FLASH = 10,
	DEBUG_ROUTER = 11,
	DEBUG_ENUM_MAX = DEBUG_ROUTER,
} driver_debug_t;

typedef enum driver_debug_value_e {
	DRIVER_DEBUG_OFF = 0,
	DRIVER_DEBUG_ON = 1,
} driver_debug_value_t;

#define driver_debug(driver, format, ...) { if (driver_debug_enabled(driver)) { printf(format, ##__VA_ARGS__);} };
unsigned int driver_debug_enabled(driver_debug_t driver);
void driver_debug_toggle(driver_debug_t driver);
void driver_debug_set(driver_debug_t driver, driver_debug_value_t);
int cmd_driver_debug_toggle(struct command_context * context);

#endif /* DRIVER_DEBUG_H_ */
