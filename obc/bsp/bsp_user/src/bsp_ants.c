/**
 * ISIS AntS Driver
 *
 * @author Johan De Claville Christiansen
 * Copyright 2011 GomSpace ApS. All rights reserved.
 */

#include <stdio.h>

#include "console.h"
#include "error.h"
#include "hexdump.h"

#include "bsp_ants.h"
#include "bsp_pca9665.h"
#include "obc_mem.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#define ANTS_I2C_HANDLE			0
#define ISIS_I2C_TIME_OUT       200

extern xSemaphoreHandle i2c_lock;
extern pca9665_device_object_t device[2];

/**
 * ISIS天线I2C接口函数
 *
 * @param addr 天线板MCU I2C地址
 * @param txbuf 发送指针
 * @param txlen 发送字节数
 * @param rxbuf 接收指针
 * @param rxlen 接收字节数
 * @param delay 主发主收之间的延时，毫秒数
 * @return E_NO_ERR为执行正确，其他错误类型参见error.h
 */
static int isis_ants_delay_cmd(uint8_t addr, void * txbuf, size_t txlen, void * rxbuf, size_t rxlen, int delay)
{
    if (ANTS_I2C_HANDLE >= pca9665_device_count)
        return E_NO_DEVICE;

    if (!device[ANTS_I2C_HANDLE].is_initialised)
        return E_NO_DEVICE;

    if ((txlen > I2C_MTU) || (rxlen > I2C_MTU))
        return E_INVALID_BUF_SIZE;

    i2c_frame_t * t_frame = (i2c_frame_t *) ObcMemMalloc(sizeof(i2c_frame_t));
    if (t_frame == NULL)
        return E_NO_BUFFER;

    /* Take the I2C lock */
    xSemaphoreTake(i2c_lock, 10 * configTICK_RATE_HZ);

    /* Temporarily disable the RX callback, because we wish the received message to go into the I2C queue instead */
    void * tmp_callback = device[ANTS_I2C_HANDLE].callback;
    device[ANTS_I2C_HANDLE].callback = NULL;

    t_frame->dest = addr;
    memcpy(&t_frame->data[ANTS_I2C_HANDLE], txbuf, txlen);
    t_frame->len = txlen;
    t_frame->len_rx = 0;

    if (i2c_send(ANTS_I2C_HANDLE, t_frame, 0) != E_NO_ERR) {
        ObcMemFree(t_frame);
        device[ANTS_I2C_HANDLE].callback = tmp_callback;
        xSemaphoreGive(i2c_lock);
        return E_TIMEOUT;
    }

    vTaskDelay(delay); /** 主发主收之间需要一个短暂延时给通信机准备数据的时间 */

    if (rxlen == 0) { /*若i2c_send()函数执行成功则内存应该在底层被正确释放，此处不再释放内存*/
        device[ANTS_I2C_HANDLE].callback = tmp_callback;
        xSemaphoreGive(i2c_lock);
        return E_NO_ERR;
    }

    /**
     * 若有I2C主收需求，则执行下面代码
     */
    t_frame = (i2c_frame_t *) ObcMemMalloc(sizeof(i2c_frame_t));
    if (t_frame == NULL)
        return E_NO_BUFFER;

    t_frame->dest = addr;
    t_frame->len = 0;
    t_frame->len_rx = rxlen;

    if (i2c_send(ANTS_I2C_HANDLE, t_frame, 0) != E_NO_ERR) {
        ObcMemFree(t_frame);
        device[ANTS_I2C_HANDLE].callback = tmp_callback;
        xSemaphoreGive(i2c_lock);
        return E_TIMEOUT;
    }

    i2c_frame_t * r_frame;

    if ( (i2c_receive(ANTS_I2C_HANDLE, &r_frame, ISIS_I2C_TIME_OUT) != E_NO_ERR) ||
            r_frame != t_frame) {
        ObcMemFree(t_frame);
        device[ANTS_I2C_HANDLE].callback = tmp_callback;
        xSemaphoreGive(i2c_lock);
        return E_TIMEOUT;
    }

    memcpy(rxbuf, &r_frame->data[0], rxlen);

    ObcMemFree(r_frame);
    device[ANTS_I2C_HANDLE].callback = tmp_callback;
    xSemaphoreGive(i2c_lock);
    return E_NO_ERR;
}


int ants_status(isis_ants_status_t * status) {

	uint8_t tx[1], rx[2], tx_len, rx_len;

	tx[0] = ISIS_ANTS_CMD_STATUS_DEPLOY;
	tx_len = 1;
	rx_len = 2;

	//i2c_init(ANTS_I2C_HANDLE, I2C_MASTER, 0x08, 100, 5, 5, NULL);

	if (isis_ants_delay_cmd(ISIS_ANTS_DFL_ADDR_A, &tx, tx_len, &rx, rx_len, 10) != E_NO_ERR) {
		return 0;
	}

	status->armed = rx[0] & 0x01;
	status->switch_ignore = rx[1] & 0x01;

	status->ant[0].not_deployed = rx[1] & 0x80;
	status->ant[0].time_limit_reached = rx[1] & 0x40;
	status->ant[0].deployment_active = rx[1] & 0x20;

	status->ant[1].not_deployed = rx[1] & 0x08;
	status->ant[1].time_limit_reached = rx[1] & 0x04;
	status->ant[1].deployment_active = rx[1] & 0x02;

	status->ant[2].not_deployed = rx[0] & 0x80;
	status->ant[2].time_limit_reached = rx[0] & 0x40;
	status->ant[2].deployment_active = rx[0] & 0x20;

	status->ant[3].not_deployed = rx[0] & 0x08;
	status->ant[3].time_limit_reached = rx[0] & 0x04;
	status->ant[3].deployment_active = rx[0] & 0x02;

	return E_NO_ERR;
}

int isis_ants_status(uint8_t i2c_addr, isis_ants_status_t * status) {

	uint8_t tx[1], rx[2], tx_len, rx_len;

	tx[0] = ISIS_ANTS_CMD_STATUS_DEPLOY;
	tx_len = 1;
	rx_len = 2;

	//i2c_init(ANTS_I2C_HANDLE, I2C_MASTER, 0x08, 100, 5, 5, NULL);

	if (isis_ants_delay_cmd(i2c_addr, &tx, tx_len, &rx, rx_len, 10) != E_NO_ERR) {
		printf("I2C transaction error\r\n");
		return 0;
	}
		
	status->armed = rx[0] & 0x01;
	status->switch_ignore = rx[1] & 0x01;

	status->ant[0].not_deployed = rx[1] & 0x80;
	status->ant[0].time_limit_reached = rx[1] & 0x40;
	status->ant[0].deployment_active = rx[1] & 0x20;

	status->ant[1].not_deployed = rx[1] & 0x08;
	status->ant[1].time_limit_reached = rx[1] & 0x04;
	status->ant[1].deployment_active = rx[1] & 0x02;

	status->ant[2].not_deployed = rx[0] & 0x80;
	status->ant[2].time_limit_reached = rx[0] & 0x40;
	status->ant[2].deployment_active = rx[0] & 0x20;

	status->ant[3].not_deployed = rx[0] & 0x08;
	status->ant[3].time_limit_reached = rx[0] & 0x04;
	status->ant[3].deployment_active = rx[0] & 0x02;

	tx[0] = ISIS_ANTS_CMD_COUNT_1;
	tx_len = 1;
	rx_len = 1;

	//i2c_init(ANTS_I2C_HANDLE, I2C_MASTER, 0x08, 100, 5, 5, NULL);

	if (isis_ants_delay_cmd(i2c_addr, &tx, tx_len, &rx, rx_len, 10) != E_NO_ERR) {
		printf("I2C transaction error\r\n");
		return 0;
	}		
	status->ant[0].activation_count = rx[0];

	tx[0] = ISIS_ANTS_CMD_COUNT_2;
	tx_len = 1;
	rx_len = 1;

	//i2c_init(ANTS_I2C_HANDLE, I2C_MASTER, 0x08, 100, 5, 5, NULL);

	if (isis_ants_delay_cmd(i2c_addr, &tx, tx_len, &rx, rx_len, 10)  != E_NO_ERR) {
		printf("I2C transaction error\r\n");
		return 0;
	}		
	status->ant[1].activation_count = rx[0];

	tx[0] = ISIS_ANTS_CMD_COUNT_3;
	tx_len = 1;
	rx_len = 1;

	//i2c_init(ANTS_I2C_HANDLE, I2C_MASTER, 0x08, 100, 5, 5, NULL);

	if (isis_ants_delay_cmd(i2c_addr, &tx, tx_len, &rx, rx_len, 10)  != E_NO_ERR) {
		printf("I2C transaction error\r\n");
		return 0;
	}		
	status->ant[2].activation_count = rx[0];

	tx[0] = ISIS_ANTS_CMD_COUNT_4;
	tx_len = 1;
	rx_len = 1;

	//i2c_init(ANTS_I2C_HANDLE, I2C_MASTER, 0x08, 100, 5, 5, NULL);

	if (isis_ants_delay_cmd(i2c_addr, &tx, tx_len, &rx, rx_len, 10)  != E_NO_ERR) {
		printf("I2C transaction error\r\n");
		return 0;
	}		
	status->ant[3].activation_count = rx[0];

	tx[0] = ISIS_ANTS_CMD_TIME_1;
	tx_len = 1;
	rx_len = 2;

	//i2c_init(ANTS_I2C_HANDLE, I2C_MASTER, 0x08, 100, 5, 5, NULL);

	if (isis_ants_delay_cmd(i2c_addr, &tx, tx_len, &rx, rx_len, 10)  != E_NO_ERR) {
		printf("I2C transaction error\r\n");
		return 0;
	}	
	status->ant[0].activation_time = (rx[1] << 8) | rx[0];

	tx[0] = ISIS_ANTS_CMD_TIME_2;
	tx_len = 1;
	rx_len = 2;

	//i2c_init(ANTS_I2C_HANDLE, I2C_MASTER, 0x08, 100, 5, 5, NULL);

	if (isis_ants_delay_cmd(i2c_addr, &tx, tx_len, &rx, rx_len, 10)  != E_NO_ERR) {
		printf("I2C transaction error\r\n");
		return 0;
	}
	status->ant[1].activation_time = (rx[1] << 8) | rx[0];

	tx[0] = ISIS_ANTS_CMD_TIME_3;
	tx_len = 1;
	rx_len = 2;

	//i2c_init(ANTS_I2C_HANDLE, I2C_MASTER, 0x08, 100, 5, 5, NULL);

	if (isis_ants_delay_cmd(i2c_addr, &tx, tx_len, &rx, rx_len, 10)  != E_NO_ERR) {
		printf("I2C transaction error\r\n");
		return 0;
	}
	status->ant[2].activation_time = (rx[1] << 8) | rx[0];

	tx[0] = ISIS_ANTS_CMD_TIME_4;
	tx_len = 1;
	rx_len = 2;

	//i2c_init(ANTS_I2C_HANDLE, I2C_MASTER, 0x08, 100, 5, 5, NULL);

	if (isis_ants_delay_cmd(i2c_addr, &tx, tx_len, &rx, rx_len, 10)  != E_NO_ERR) {
		printf("I2C transaction error\r\n");
		return 0;
	}
	status->ant[3].activation_time = (rx[1] << 8) | rx[0];

	return E_NO_ERR;

}

int isis_ants_temp(uint8_t i2c_addr, uint16_t * temp) {

	uint8_t tx[1], rx[2], tx_len, rx_len;

	tx[0] = ISIS_ANTS_CMD_TEMP;
	tx_len = 1;
	rx_len = 2;

	//i2c_init(ANTS_I2C_HANDLE, I2C_MASTER, 0x08, 100, 5, 5, NULL);

	if (isis_ants_delay_cmd(i2c_addr, &tx, tx_len, &rx, rx_len, 10)  != E_NO_ERR) {
		printf("I2C transaction error\r\n");
		return 0;
	}

	*temp = (rx[1] << 8) | rx[0];
	return E_NO_ERR;

}

int isis_ants_deploy_single(uint8_t i2c_addr, int isis_ant_nr, uint8_t time_sec, unsigned int override) {

	uint8_t tx[2], tx_len;

	if (override) {
		switch(isis_ant_nr) {
		case 0: tx[0] = ISIS_ANTS_CMD_O_DEPLOY_1; break;
		case 1: tx[0] = ISIS_ANTS_CMD_O_DEPLOY_2; break;
		case 2: tx[0] = ISIS_ANTS_CMD_O_DEPLOY_3; break;
		case 3: tx[0] = ISIS_ANTS_CMD_O_DEPLOY_4; break;
		default: return -1;
		}
	} else {
		switch(isis_ant_nr) {
		case 0: tx[0] = ISIS_ANTS_CMD_DEPLOY_1; break;
		case 1: tx[0] = ISIS_ANTS_CMD_DEPLOY_2; break;
		case 2: tx[0] = ISIS_ANTS_CMD_DEPLOY_3; break;
		case 3: tx[0] = ISIS_ANTS_CMD_DEPLOY_4; break;
		default: return -1;
		}
	}

	tx[1] = time_sec;
	tx_len = 2;
	
	//i2c_init(ANTS_I2C_HANDLE, I2C_MASTER, 0x08, 100, 5, 5, NULL);

	if (isis_ants_delay_cmd(i2c_addr, &tx, tx_len, NULL, 0, 10) != E_NO_ERR) {
		printf("I2C transaction error\r\n");
		return 0;
	}

	return E_NO_ERR;

}

int isis_ants_deploy_auto(uint8_t i2c_addr, uint8_t time_sec) {

	uint8_t tx[2], tx_len;

	tx[0] = ISIS_ANTS_CMD_DEPLOY_AUTO;
	tx[1] = time_sec;
	tx_len = 2;

	//i2c_init(ANTS_I2C_HANDLE, I2C_MASTER, 0x08, 100, 5, 5, NULL);

	if (isis_ants_delay_cmd(i2c_addr, &tx, tx_len, NULL, 0, 10) != E_NO_ERR) {
		printf("I2C transaction error\r\n");
		return 0;
	}
	
	return E_NO_ERR;

}

int ants_deploy_auto(uint8_t time_sec) {

	uint8_t tx[2], tx_len;

	tx[0] = ISIS_ANTS_CMD_DEPLOY_AUTO;
	tx[1] = time_sec;
	tx_len = 2;

	//i2c_init(ANTS_I2C_HANDLE, I2C_MASTER, 0x08, 100, 5, 5, NULL);

	if (isis_ants_delay_cmd(ISIS_ANTS_DFL_ADDR_A, &tx, tx_len, NULL, 0, 10) != E_NO_ERR) {
		return 0;
	}

	return E_NO_ERR;

}

int isis_ants_deploy_cancel(uint8_t i2c_addr) {

	uint8_t tx[1], tx_len;

	tx[0] = ISIS_ANTS_CMD_DEPLOY_CANCEL;
	tx_len = 1;

	//i2c_init(ANTS_I2C_HANDLE, I2C_MASTER, 0x08, 100, 5, 5, NULL);

	if (isis_ants_delay_cmd(i2c_addr, &tx, tx_len, NULL, 0, 10) != E_NO_ERR) {
		printf("I2C transaction error\r\n");
		return 0;
	}
	
	return E_NO_ERR;

}

int isis_ants_disarm(uint8_t i2c_addr) {

	uint8_t tx[1], tx_len;

	tx[0] = ISIS_ANTS_CMD_DISARM;
	tx_len = 1;

	//i2c_init(ANTS_I2C_HANDLE, I2C_MASTER, 0x08, 100, 5, 5, NULL);

	if (isis_ants_delay_cmd(i2c_addr, &tx, tx_len, NULL, 0, 10) != E_NO_ERR) {
		printf("I2C transaction error\r\n");
		return 0;
	}
	
	return E_NO_ERR;

}

int isis_ants_arm(uint8_t i2c_addr) {

	uint8_t tx[1], tx_len;

	tx[0] = ISIS_ANTS_CMD_ARM;
	tx_len = 1;

	//i2c_init(ANTS_I2C_HANDLE, I2C_MASTER, 0x08, 100, 5, 5, NULL);

	if (isis_ants_delay_cmd(i2c_addr, &tx, tx_len, NULL, 0, 10) != E_NO_ERR) {
		printf("I2C transaction error\r\n");
		return 0;
	}
	
	return E_NO_ERR;

}

int ants_arm(void) {

	uint8_t tx[1], tx_len;

	tx[0] = ISIS_ANTS_CMD_ARM;
	tx_len = 1;

	//i2c_init(ANTS_I2C_HANDLE, I2C_MASTER, 0x08, 100, 5, 5, NULL);

	if (isis_ants_delay_cmd(ISIS_ANTS_DFL_ADDR_A, &tx, tx_len, NULL, 0, 10) != E_NO_ERR) {
		return 0;
	}

	return E_NO_ERR;

}

int isis_ants_reset(uint8_t i2c_addr) {

	uint8_t tx[1], tx_len;

	tx[0] = ISIS_ANTS_CMD_RESET;
	tx_len = 1;

	//i2c_init(ANTS_I2C_HANDLE, I2C_MASTER, 0x08, 100, 5, 5, NULL);

	if (isis_ants_delay_cmd(i2c_addr, &tx, tx_len, NULL, 0, 10) != E_NO_ERR) {
		printf("I2C transaction error\r\n");
		return 0;
	}
	
	return E_NO_ERR;

}

int isis_ants_countdown_val(unsigned int val) {
	FILE * fd = fopen("/boot/countdown.txt", "w+");

	if (fd == NULL)
		return 0;
	if (fwrite(&val, sizeof(val), 1, fd) == 0)
		return 0;

	fflush(fd);
	fclose(fd);

	return E_NO_ERR;
}
