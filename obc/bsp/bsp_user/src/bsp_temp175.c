/*
 * bsp_temp175.c
 *
 *  Created on: 2017年10月20日
 *      Author: Ma Wenli
 */
#include <stdint-gcc.h>
#include <stdio.h>

#include "bsp_pca9665.h"
#include "error.h"
#include "csp_endian.h"
#include "command.h"

#include "bsp_temp175.h"

#define CONF_REG 0x01
#define TEMP_REG 0x00

#define RESOLUT_MAX 0x60

#define TEPM175_I2C_HANDLE 1
#define TEPM175_I2C_ADDR 0x48

static uint8_t conf_arry[2] = {CONF_REG, RESOLUT_MAX};

/**
 * 通过I2C配置 temp175-q1芯片 最高采温精度0.0625
 *
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int temp175_init(void)
{
    return i2c_master_transaction(TEPM175_I2C_HANDLE, TEPM175_I2C_ADDR, conf_arry, 2, NULL, 0, 0);
}

/**
 * 通过I2C获取 temp175-q1芯片 温度 最高采温精度0.0625
 *
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int get_obc_temp(uint16_t *temp)
{
    uint16_t big_endian_temp;
    int ret;

    ret = i2c_master_transaction(TEPM175_I2C_HANDLE, TEPM175_I2C_ADDR, TEMP_REG, 1, &big_endian_temp, 2, 100);

    if(ret != E_NO_ERR)
        return ret;

    *temp = csp_betoh16(big_endian_temp);

    return E_NO_ERR;
}

int board_temp_get_cmd(struct command_context *ctx __attribute__((unused)))
{
    temp_reg_t obc_temp = {0};

    if(get_obc_temp((uint16_t *)&obc_temp) == E_NO_ERR)
    {
        if(obc_temp.is_negative)
            printf( "TEMP:-%.4f C\n", ~(obc_temp.temp+1) * 0.0625 );
        else
            printf( "TEMP: %.4f C\n", obc_temp.temp * 0.0625 );
    }

    return CMD_ERROR_NONE;
}

command_t __sub_command __root_command ina_temp_commands_master[] = {
    {
        .name = "temp175",
        .help = "Board temp get",
        .handler = board_temp_get_cmd,
    }
};


void cmd_ina_temp_setup(void)
{
    command_register(ina_temp_commands_master);
}
