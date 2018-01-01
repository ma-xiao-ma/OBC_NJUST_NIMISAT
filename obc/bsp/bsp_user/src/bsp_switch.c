////////////////////////////////////////////////////////////////////////////////
//	功能：实现电源板各开关接口控制
//
//	版本：V1.0
//  迭代：
//												南京理工大学微纳卫星中心
//												   2015.11.26
////////////////////////////////////////////////////////////////////////////////

#include "bsp_switch.h"
#include "stm32f4xx.h"
#include "command.h"

void bsp_InitSwitch(void) {
	GPIO_InitTypeDef GPIO_InitStructure;
//	GPIO_InitTypeDef GPIO_InitStructure_status;

	RCC_AHB1PeriphClockCmd(RCC_PAL_STATUS_1_PORT,ENABLE);
	//使能时钟
	RCC_AHB1PeriphClockCmd(
			RCC_5_3_3V_1_PORT | RCC_5_3_3V_2_PORT | RCC_5_3_3V_3_PORT |
			RCC_7_4V_1_PORT   | RCC_7_4V_2_PORT   | RCC_7_4V_3_PORT   |
			RCC_SOLAR_EN_PORT |
			RCC_USB_EN_PORT   |
			RCC_EPS_S0_PORT   | RCC_EPS_S1_PORT   | RCC_EPS_S2_PORT   | RCC_EPS_S3_PORT|
			RCC_DTB_5V_PORT|RCC_DTB_12V_PORT|RCC_CAMERA_10W_PORT|RCC_CAMERA_5W_PORT|
			RCC_CAMERA_HEAT_1_PORT/*|RCC_CAMERA_HEAT_2_PORT*/
			, ENABLE);

	//配置GPIO
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

	GPIO_InitStructure.GPIO_Pin = GPIO_5_3_3V_1_PIN;
	GPIO_Init(GPIO_5_3_3V_1_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_5_3_3V_2_PIN;
	GPIO_Init(GPIO_5_3_3V_2_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_5_3_3V_3_PIN;
	GPIO_Init(GPIO_5_3_3V_3_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_7_4V_1_PIN;
	GPIO_Init(GPIO_7_4V_1_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_7_4V_2_PIN;
	GPIO_Init(GPIO_7_4V_2_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_7_4V_3_PIN;
	GPIO_Init(GPIO_7_4V_3_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_SOLAR_EN_PIN;
	GPIO_Init(GPIO_SOLAR_EN_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_USB_EN_PIN;
	GPIO_Init(GPIO_USB_EN_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_EPS_S0_PIN;
	GPIO_Init(GPIO_EPS_S0_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_EPS_S1_PIN;
	GPIO_Init(GPIO_EPS_S1_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_EPS_S2_PIN;
	GPIO_Init(GPIO_EPS_S2_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_EPS_S3_PIN;
	GPIO_Init(GPIO_EPS_S3_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_DTB_5V_PIN;
	GPIO_Init(GPIO_DTB_5V_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_DTB_12V_PIN;
	GPIO_Init(GPIO_DTB_12V_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_CAMERA_10W_PIN;
	GPIO_Init(GPIO_CAMERA_10W_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_CAMERA_5W_PIN;
	GPIO_Init(GPIO_CAMERA_5W_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_CAMERA_HEAT_1_PIN;
	GPIO_Init(GPIO_CAMERA_HEAT_1_PORT, &GPIO_InitStructure);

//	GPIO_InitStructure.GPIO_Pin = GPIO_CAMERA_HEAT_2_PIN;
//	GPIO_Init(GPIO_CAMERA_HEAT_2_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

	GPIO_InitStructure.GPIO_Pin = GPIO_PAL_STATUS_1_PIN;
	GPIO_Init(GPIO_PAL_STATUS_1_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_PAL_STATUS_2_PIN;
    GPIO_Init(GPIO_PAL_STATUS_2_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_SAIL_STATUS_PIN;
    GPIO_Init(GPIO_SAIL_STATUS_PORT, &GPIO_InitStructure);

	EpsOutSwitch(OUT_ALL, DISABLE);
}

uint8_t EpsOutSwitch(uint8_t ch, uint8_t status) {
	uint8_t res;
	uint8_t time = 5;
	switch (ch) {
	case OUT_5_3_3V_1://PE6
		if (status ^ SW_5_3_3V_1_PIN()) {
			status == ENABLE ? SW_5_3_3V_1_ENABLE : SW_5_3_3V_1_DISABLE;
			while ((status ^ SW_5_3_3V_1_PIN()) && --time) {
				status == ENABLE ? SW_5_3_3V_1_ENABLE : SW_5_3_3V_1_DISABLE;
			}
			if (time)
				res = EPS_OK;
			else
				res = EPS_ERROR;
		} else {  //无效操作
			res = EPS_UNDO;
		}
		break;

	case OUT_5_3_3V_2://PE2
		if (status ^ SW_5_3_3V_2_PIN()) {
			status == ENABLE ? SW_5_3_3V_2_ENABLE : SW_5_3_3V_2_DISABLE;
			while ((status ^ SW_5_3_3V_2_PIN()) && --time) {
				status == ENABLE ? SW_5_3_3V_2_ENABLE : SW_5_3_3V_2_DISABLE;
			}
			if (time)
				res = EPS_OK;
			else
				res = EPS_ERROR;
		} else {
			res = EPS_UNDO;
		}
		break;

	case OUT_5_3_3V_3://PA9
		if (status ^ SW_5_3_3V_3_PIN()) {
			status == ENABLE ? SW_5_3_3V_3_ENABLE : SW_5_3_3V_3_DISABLE;
			while ((status ^ SW_5_3_3V_3_PIN()) && --time) {
				status == ENABLE ? SW_5_3_3V_3_ENABLE : SW_5_3_3V_3_DISABLE;
			}
			if (time)
				res = EPS_OK;
			else
				res = EPS_ERROR;
		} else {
			res = EPS_UNDO;
		}
		break;

	case OUT_7_4V_1://PA8
		if (status ^ SW_7_4V_1_PIN()) {
			status == ENABLE ? SW_7_4V_1_ENABLE : SW_7_4V_1_DISABLE;
			while ((status ^ SW_7_4V_1_PIN()) && --time) {
				status == ENABLE ? SW_7_4V_1_ENABLE : SW_7_4V_1_DISABLE;
			}
			if (time)
				res = EPS_OK;
			else
				res = EPS_ERROR;
		} else {
			res = EPS_UNDO;
		}
		break;

	case OUT_7_4V_2:
		if (status ^ SW_7_4V_2_PIN()) {
			status == ENABLE ? SW_7_4V_2_ENABLE : SW_7_4V_2_DISABLE;
			while ((status ^ SW_7_4V_2_PIN()) && --time) {
				status == ENABLE ? SW_7_4V_2_ENABLE : SW_7_4V_2_DISABLE;
			}
			if (time)
				res = EPS_OK;
			else
				res = EPS_ERROR;
		} else {
			res = EPS_UNDO;
		}
		break;

	case OUT_7_4V_3:
		if (status ^ SW_7_4V_3_PIN()) {
			status == ENABLE ? SW_7_4V_3_ENABLE : SW_7_4V_3_DISABLE;
			while ((status ^ SW_7_4V_3_PIN()) && --time) {
				status == ENABLE ? SW_7_4V_3_ENABLE : SW_7_4V_3_DISABLE;
			}
			if (time)
				res = EPS_OK;
			else
				res = EPS_ERROR;
		} else {
			res = EPS_UNDO;
		}
		break;

	case OUT_SOLAR_EN:
		if (status ^ SW_SOLAR_EN_PIN()) {
			status == ENABLE ? SW_SOLAR_EN_ENABLE : SW_SOLAR_EN_DISABLE;
			while ((status ^ SW_SOLAR_EN_PIN()) && --time) {
				status == ENABLE ? SW_SOLAR_EN_ENABLE : SW_SOLAR_EN_DISABLE;
			}
			if (time)
				res = EPS_OK;
			else
				res = EPS_ERROR;
		} else {
			res = EPS_UNDO;
		}
		break;

	case OUT_USB_EN:
		if (status ^ SW_USB_EN_PIN()) {
			status == ENABLE ? SW_USB_EN_ENABLE : SW_USB_EN_DISABLE;
			while ((status ^ SW_USB_EN_PIN()) && --time) {
				status == ENABLE ? SW_USB_EN_ENABLE : SW_USB_EN_DISABLE;
			}
			if (time)
				res = EPS_OK;
			else
				res = EPS_ERROR;
		} else {
			res = EPS_OK;
		}
		break;

	case OUT_EPS_S0://PD3
		if (status ^ SW_EPS_S0_PIN()) {
			status == ENABLE ? SW_EPS_S0_ENABLE : SW_EPS_S0_DISABLE;
			while ((status ^ SW_EPS_S0_PIN()) && --time) {
				status == ENABLE ? SW_EPS_S0_ENABLE : SW_EPS_S0_DISABLE;
			}
			if (time)
				res = EPS_OK;
			else
				res = EPS_ERROR;
		} else {
			res = EPS_UNDO;
		}
		break;

	case OUT_EPS_S1://PG8
		if (status ^ SW_EPS_S1_PIN()) {
			status == ENABLE ? SW_EPS_S1_ENABLE : SW_EPS_S1_DISABLE;
			while ((status ^ SW_EPS_S1_PIN()) && --time) {
				status == ENABLE ? SW_EPS_S1_ENABLE : SW_EPS_S1_DISABLE;
			}
			if (time)
				res = EPS_OK;
			else
				res = EPS_ERROR;
		} else {
			res = EPS_UNDO;
		}
		break;

	case OUT_EPS_S2://PG7
		if (status ^ SW_EPS_S2_PIN()) {

			status == ENABLE ? SW_EPS_S2_ENABLE : SW_EPS_S2_DISABLE;

			while ((status ^ SW_EPS_S2_PIN()) && --time) {
				status == ENABLE ? SW_EPS_S2_ENABLE : SW_EPS_S2_DISABLE;
			}
			if (time)
				res = EPS_OK;
			else
				res = EPS_ERROR;
		} else {
			res = EPS_UNDO;
		}
		break;

	case OUT_EPS_S3://PG6
		if (status ^ SW_EPS_S3_PIN()) {
			status == ENABLE ? SW_EPS_S3_ENABLE : SW_EPS_S3_DISABLE;
			while ((status ^ SW_EPS_S3_PIN()) && --time) {
				status == ENABLE ? SW_EPS_S3_ENABLE : SW_EPS_S3_DISABLE;
			}
			if (time)
				res = EPS_OK;
			else
				res = EPS_ERROR;
		} else {
			res = EPS_UNDO;
		}
		break;

	case OUT_DTB_5V://PF6 数传5V电使能
		if (status ^ OUT_SW_DTB_5V_PIN()) {
			status == ENABLE ? SW_DTB_5V_ENABLE : SW_DTB_5V_DISABLE;
			while ((status ^ OUT_SW_DTB_5V_PIN()) && --time) {
				status == ENABLE ? SW_DTB_5V_ENABLE : SW_DTB_5V_DISABLE;
			}
			if (time)
				res = EPS_OK;
			else
				res = EPS_ERROR;
		} else {
			res = EPS_UNDO;
		}
		break;

	case OUT_DTB_12V://PF7 数传12V电使能
		if (status ^ OUT_SW_DTB_12V_PIN()) {
			status == ENABLE ? SW_DTB_12V_ENABLE : SW_DTB_12V_DISABLE;
			while ((status ^ OUT_SW_DTB_12V_PIN()) && --time) {
				status == ENABLE ? SW_DTB_12V_ENABLE : SW_DTB_12V_DISABLE;
			}
			if (time)
				res = EPS_OK;
			else
				res = EPS_ERROR;
		} else {
			res = EPS_UNDO;
		}
		break;

	case OUT_CAMERA_10W://PF8  相机5V电使能
		if (status ^ OUT_SW_CAMERA_10W_PIN()) {
			status == ENABLE ? SW_CAMERA_10W_ENABLE : SW_CAMERA_10W_DISABLE;
			while ((status ^ OUT_SW_CAMERA_10W_PIN()) && --time) {
				status == ENABLE ? SW_CAMERA_10W_ENABLE : SW_CAMERA_10W_DISABLE;
			}
			if (time)
				res = EPS_OK;
			else
				res = EPS_ERROR;
		} else {
			res = EPS_UNDO;
		}
		break;

	case OUT_CAMERA_5W://PF9 相机5V电使能
		if (status ^ OUT_SW_CAMERA_5W_PIN()) {
			status == ENABLE ? SW_CAMERA_5W_ENABLE : SW_CAMERA_5W_DISABLE;
			while ((status ^ OUT_SW_CAMERA_5W_PIN()) && --time) {
				status == ENABLE ? SW_CAMERA_5W_ENABLE : SW_CAMERA_5W_DISABLE;
			}
			if (time)
				res = EPS_OK;
			else
				res = EPS_ERROR;
		} else {
			res = EPS_UNDO;
		}
		break;

	case OUT_CAMERA_HEAT_1://PF10 相机加热开关1使能
		if (status ^ OUT_SW_CAMERA_HEAT_1_PIN()) {
			status == ENABLE ? SW_CAMERA_HEAT_1_ENABLE : SW_CAMERA_HEAT_1_DISABLE;
			while ((status ^ OUT_SW_CAMERA_HEAT_1_PIN()) && --time) {
				status == ENABLE ? SW_CAMERA_HEAT_1_ENABLE : SW_CAMERA_HEAT_1_DISABLE;
			}
			if (time)
				res = EPS_OK;
			else
				res = EPS_ERROR;
		} else {
			res = EPS_UNDO;
		}
		break;

//	case OUT_CAMERA_HEAT_2://PC2 相机加热开关2使能
//		if (status ^ OUT_SW_CAMERA_HEAT_2_PIN()) {
//			status == ENABLE ? SW_CAMERA_HEAT_2_ENABLE : SW_CAMERA_HEAT_2_DISABLE;
//			while ((status ^ OUT_SW_CAMERA_HEAT_2_PIN()) && --time) {
//				status == ENABLE ? SW_CAMERA_HEAT_2_ENABLE : SW_CAMERA_HEAT_2_DISABLE;
//			}
//			if (time)
//				res = EPS_OK;
//			else
//				res = EPS_ERROR;
//		} else {
//			res = EPS_UNDO;
//		}
//		break;



	case OUT_ALL:
		if (status) {
			SW_5_3_3V_1_ENABLE;
			SW_5_3_3V_2_ENABLE;
			SW_5_3_3V_3_ENABLE;
			SW_7_4V_1_ENABLE;
			SW_7_4V_2_ENABLE;
			SW_7_4V_3_ENABLE;
			SW_SOLAR_EN_ENABLE;
			SW_USB_EN_ENABLE;
			SW_EPS_S0_ENABLE;
			SW_EPS_S1_ENABLE;
			SW_EPS_S2_ENABLE;
			SW_EPS_S3_ENABLE;
			SW_DTB_5V_ENABLE;
			SW_DTB_12V_ENABLE;
			SW_CAMERA_10W_ENABLE;
			SW_CAMERA_5W_ENABLE;
			SW_CAMERA_HEAT_1_ENABLE;
//			SW_CAMERA_HEAT_2_ENABLE;

			res = EPS_OK;
		} else {

			SW_5_3_3V_1_DISABLE;
			SW_5_3_3V_2_DISABLE;
			SW_5_3_3V_3_DISABLE;
			SW_7_4V_1_DISABLE;
			SW_7_4V_2_DISABLE;
			SW_7_4V_3_DISABLE;
			SW_SOLAR_EN_DISABLE;
			SW_USB_EN_DISABLE;
			SW_EPS_S0_DISABLE;
			SW_EPS_S1_DISABLE;
			SW_EPS_S2_DISABLE;
			SW_EPS_S3_DISABLE;
			SW_DTB_5V_DISABLE;
			SW_DTB_12V_DISABLE;
			SW_CAMERA_10W_DISABLE;
			SW_CAMERA_5W_DISABLE;
			SW_CAMERA_HEAT_1_DISABLE;
//			SW_CAMERA_HEAT_2_DISABLE;

			res = EPS_OK;
		}
	}

	return res;
}

int hub_5_3_3V_1(struct command_context *ctx) {

	char * args = command_args(ctx);
	uint32_t opt = 3;

	if (sscanf(args, " %u", &opt) != 1)
		printf("No such option\r\n");

	if (opt == 0) {
		EpsOutSwitch(OUT_5_3_3V_1, DISABLE);
		printf("5_3_3V_1 close\r\n");
	}

	if (opt == 1) {
		EpsOutSwitch(OUT_5_3_3V_1, ENABLE);
		printf("5_3_3V_1 open\r\n");
	}

	if (opt != 1 && opt != 0)
		printf("No such option\r\n");

	return CMD_ERROR_NONE;
}

int hub_5_3_3V_2(struct command_context *ctx) {

	char * args = command_args(ctx);
	uint32_t opt = 3;

	if (sscanf(args, " %u", &opt) != 1)
		printf("No such option\r\n");

	if (opt == 0) {
		EpsOutSwitch(OUT_5_3_3V_2, DISABLE);
		printf("5_3_3V_2 close\r\n");
	}

	if (opt == 1) {
		EpsOutSwitch(OUT_5_3_3V_2, ENABLE);
		printf("5_3_3V_2 open\r\n");
	}

	if (opt != 1 && opt != 0)
		printf("No such option\r\n");

	return CMD_ERROR_NONE;
}

int hub_5_3_3V_3(struct command_context *ctx) {

	char * args = command_args(ctx);
	uint32_t opt = 3;

	if (sscanf(args, " %u", &opt) != 1)
		printf("No such option\r\n");

	if (opt == 0) {
		EpsOutSwitch(OUT_5_3_3V_3, DISABLE);
		printf("5_3_3V_3 close\r\n");
	}

	if (opt == 1) {
		EpsOutSwitch(OUT_5_3_3V_3, ENABLE);
		printf("5_3_3V_3 open\r\n");
	}

	if (opt != 1 && opt != 0)
		printf("No such option\r\n");

	return CMD_ERROR_NONE;
}

int hub_7_4V_1(struct command_context *ctx) {

	char * args = command_args(ctx);
	uint32_t opt = 3;

	if (sscanf(args, " %u", &opt) != 1)
		printf("No such option\r\n");

	if (opt == 0) {
		EpsOutSwitch(OUT_7_4V_1, DISABLE);
		printf("7_4V_1 close\r\n");
	}

	if (opt == 1) {
		EpsOutSwitch(OUT_7_4V_1, ENABLE);
		printf("5_7_4V_1 open\r\n");
	}

	if (opt != 1 && opt != 0)
		printf("No such option\r\n");

	return CMD_ERROR_NONE;
}

int hub_7_4V_2(struct command_context *ctx) {

	char * args = command_args(ctx);
	uint32_t opt = 3;

	if (sscanf(args, " %u", &opt) != 1)
		printf("No such option\r\n");

	if (opt == 0) {
		EpsOutSwitch(OUT_7_4V_2, DISABLE);
		printf("7_4V_2 close\r\n");
	}

	if (opt == 1) {
		EpsOutSwitch(OUT_7_4V_2, ENABLE);
		printf("5_7_4V_2 open\r\n");
	}

	if (opt != 1 && opt != 0)
		printf("No such option\r\n");

	return CMD_ERROR_NONE;
}

int hub_7_4V_3(struct command_context *ctx) {

	char * args = command_args(ctx);
	uint32_t opt = 3;

	if (sscanf(args, " %u", &opt) != 1)
		printf("No such option\r\n");

	if (opt == 0) {
		EpsOutSwitch(OUT_7_4V_3, DISABLE);
		printf("7_4V_3 close\r\n");
	}

	if (opt == 1) {
		EpsOutSwitch(OUT_7_4V_3, ENABLE);
		printf("5_7_4V_3 open\r\n");
	}

	if (opt != 1 && opt != 0)
		printf("No such option\r\n");

	return CMD_ERROR_NONE;
}

int hub_solar_en(struct command_context *ctx) {

	char * args = command_args(ctx);
	uint32_t opt = 3;

	if (sscanf(args, " %u", &opt) != 1)
		printf("No such option\r\n");

	if (opt == 0) {
		EpsOutSwitch(OUT_SOLAR_EN, DISABLE);
		printf("SOLAR_EN close\r\n");
	}

	if (opt == 1) {
		EpsOutSwitch(OUT_SOLAR_EN, ENABLE);
		printf("SOLAR_EN open\r\n");
	}

	if (opt != 1 && opt != 0)
		printf("No such option\r\n");

	return CMD_ERROR_NONE;
}

int hub_usb_en(struct command_context *ctx) {

	char * args = command_args(ctx);
	uint32_t opt = 3;

	if (sscanf(args, " %u", &opt) != 1)
		printf("No such option\r\n");

	if (opt == 0) {
		EpsOutSwitch(OUT_USB_EN, DISABLE);
		printf("USB_EN close\r\n");
	}

	if (opt == 1) {
		EpsOutSwitch(OUT_USB_EN, ENABLE);
		printf("USB_EN open\r\n");
	}

	if (opt != 1 && opt != 0)
		printf("No such option\r\n");

	return CMD_ERROR_NONE;
}

int hub_eps_s_0(struct command_context *ctx) {

	char * args = command_args(ctx);
	uint32_t opt = 3;

	if (sscanf(args, " %u", &opt) != 1)
		printf("No such option\r\n");

	if (opt == 0) {
		EpsOutSwitch(OUT_EPS_S0, DISABLE);
		printf("EPS_S0 close\r\n");
	}

	if (opt == 1) {
		EpsOutSwitch(OUT_EPS_S0, ENABLE);
		printf("EPS_S0 open\r\n");
	}

	if (opt != 1 && opt != 0)
		printf("No such option\r\n");

	return CMD_ERROR_NONE;
}

int hub_eps_s_1(struct command_context *ctx) {

	char * args = command_args(ctx);
	uint32_t opt = 3;

	if (sscanf(args, " %u", &opt) != 1)
		printf("No such option\r\n");

	if (opt == 0) {
		EpsOutSwitch(OUT_EPS_S1, DISABLE);
		printf("EPS_S1 close\r\n");
	}

	if (opt == 1) {
		EpsOutSwitch(OUT_EPS_S1, ENABLE);
		printf("EPS_S1 open\r\n");
	}

	if (opt != 1 && opt != 0)
		printf("No such option\r\n");

	return CMD_ERROR_NONE;
}

int hub_eps_s_2(struct command_context *ctx) {

	char * args = command_args(ctx);
	uint32_t opt = 3;

	if (sscanf(args, " %u", &opt) != 1)
		printf("No such option\r\n");

	if (opt == 0) {
		EpsOutSwitch(OUT_EPS_S2, DISABLE);
		printf("EPS_S2 close\r\n");
	}

	if (opt == 1) {
		EpsOutSwitch(OUT_EPS_S2, ENABLE);
		printf("EPS_S2 open\r\n");
	}

	if (opt != 1 && opt != 0)
		printf("No such option\r\n");

	return CMD_ERROR_NONE;
}

int hub_eps_s_3(struct command_context *ctx) {

	char * args = command_args(ctx);
	uint32_t opt = 3;

	if (sscanf(args, " %u", &opt) != 1)
		printf("No such option\r\n");

	if (opt == 0) {
		EpsOutSwitch(OUT_EPS_S3, DISABLE);
		printf("EPS_S3 close\r\n");
	}

	if (opt == 1) {
		EpsOutSwitch(OUT_EPS_S3, ENABLE);
		printf("EPS_S3 open\r\n");
	}

	if (opt != 1 && opt != 0)
		printf("No such option\r\n");

	return CMD_ERROR_NONE;
}

struct command vol_5_3_3_subcommands[] = {
	{
		.name = "ch1",
		.help = "channel 1",
		.usage = "<opt>0:off 1:on",
		.handler = hub_5_3_3V_1,
	},{
		.name = "ch2",
		.help = "channel 2",
		.usage = "<opt>0:off 1:on",
		.handler = hub_5_3_3V_2,
	},{
		.name = "ch3",
		.help = "channel 3",
		.usage = "<opt>0:off 1:on",
		.handler = hub_5_3_3V_3,
	},
};

struct command vol_7_4_subcommands[] = {
	{
		.name = "ch1",
		.help = "channel 1",
		.usage = "<opt>0:off 1:on",
		.handler = hub_7_4V_1,
	},{
		.name = "ch2",
		.help = "channel 2",
		.usage = "<opt>0:off 1:on",
		.handler = hub_7_4V_2,
	},{
		.name = "ch3",
		.help = "channel 3",
		.usage = "<opt>0:off 1:on",
		.handler = hub_7_4V_3,
	},
};

struct command eps_sx_subcommands[] = {
	{
		.name = "s0",
		.help = "channel 0",
		.usage = "<opt>0:off 1:on",
		.handler = hub_eps_s_0,
	},{
		.name = "s1",
		.help = "channel 1",
		.usage = "<opt>0:off 1:on",
		.handler = hub_eps_s_1,
	},{
		.name = "s2",
		.help = "channel 2",
		.usage = "<opt>0:off 1:on",
		.handler = hub_eps_s_2,
	},{
		.name = "s3",
		.help = "channel 3",
		.usage = "<opt>0:off 1:on",
		.handler = hub_eps_s_3,
	},
};

struct command pwr_subcommands[] = {
	{
		.name = "5/3.3V",
		.help = "5/3.3V",
		.chain = INIT_CHAIN(vol_5_3_3_subcommands),
	},{
		.name = "7.4V",
		.help = "7.4V",
		.chain = INIT_CHAIN(vol_7_4_subcommands),
	},{
		.name = "eps",
		.help = "eps sx<0~3>",
		.chain = INIT_CHAIN(eps_sx_subcommands),
	},{
		.name = "solar",
		.help = "solar",
		.usage = "<opt>0:off 1:on",
		.handler = hub_solar_en,
	},{
		.name = "usb",
		.help = "usb",
		.usage = "<opt>0:off 1:on",
		.handler = hub_usb_en,
	},
};

struct command __root_command pwr_commands_master[] = {
	{
		.name = "pwr",
		.help = "sw master commands",
		.chain = INIT_CHAIN(pwr_subcommands),
	},
};

void cmd_pwr_setup(void) {
	command_register(pwr_commands_master);
}
