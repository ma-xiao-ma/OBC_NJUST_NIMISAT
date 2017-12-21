/*
 * bsp_temp175.h
 *
 *  Created on: 2017年10月20日
 *      Author: Ma Wenli
 */

#ifndef BSP_BSP_USER_INC_BSP_TEMP175_H_
#define BSP_BSP_USER_INC_BSP_TEMP175_H_


typedef struct __attribute__((packed))
{
    uint16_t padding:4;
    uint16_t temp:11;
    uint16_t is_negative:1;
} temp_reg_t;

/**
 * 通过I2C获取 temp175-q1芯片 温度 最高采温精度0.0625
 *
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int get_obc_temp(uint16_t *temp);

/**
 * 通过I2C配置 temp175-q1芯片 最高采温精度0.0625
 *
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int temp175_init(void);

void cmd_ina_temp_setup(void);

#endif /* BSP_BSP_USER_INC_BSP_TEMP175_H_ */
