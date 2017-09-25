#include <stdio.h>

#include "cube_com.h"

#include "sensor/flash_sd.h"

#include "bsp_usart.h"
#include "bsp_reset.h"
#include "bsp_delay.h"
#include "bsp_ds1302.h"
#include "bsp_switch.h"
#include "dtb_805.h"

#include "FreeRTOS.h"
#include "task.h"

#include "camera_805.h"
#include "contrl.h"
#include "ctrl_cmd_types.h"
#include "cubesat.h"
#include "switches.h"
#include "command.h"
#include "hk_arg.h"
#include "hk.h"
#include "driver_debug.h"
#include "bsp_cis.h"
#include "downlink_interface.h"

#define ORGCALL			"BI4ST0"
#define DETCALL			"BI4ST1"

extern uint8_t adcs_pwr_sta;
extern uint8_t up_cmd_adcs_pwr;

unsigned char cube_buf[80];

unsigned char func =0;
uint32_t rec_cmd_cnt = 0; //obc接收本地指令计数

struct USART_TypeDefStruct GCS_Usart;
#if USE_SERIAL_PORT_DOWNLINK_INTERFACE
    void obc_cmd_ack(void *ack, uint32_t length)
    {
        vSerialACK(ack, length);
    }

#else
    void obc_cmd_ack(void *ack, uint32_t length) {
        uint8_t rec_len = 0;

        set_transmission_bitrate_4800();
        vTaskDelay(10);
        I2C_ICD_send_Axdate((uint8_t *)ORGCALL, (uint8_t *)DETCALL,
                (uint8_t *)ack, length, &rec_len);
    }
#endif

void CubeUnPacket(const unsigned char *str)
{
	ctrl_nopara_t * cmd = (ctrl_nopara_t *)str;

	if(cmd->id == 1)
	{
	    /*obc接收本地指令计数*/
	    rec_cmd_cnt++;

		if(cmd->delay > 0)
		{
			vTaskDelay(cmd->delay);
		}

		switch ((cmd->cmd) & 0xF0) {
			case 0x00:
				up_group_zero_Cmd_pro(cmd->cmd,str);
				break;
			case 0x10:
				up_group_one_Cmd_pro(cmd->cmd,str);
				break;
			case 0x20:
				up_group_two_Cmd_pro(cmd->cmd,str);
				break;
			case 0x30:
				up_group_three_Cmd_pro(cmd->cmd,str);
				break;
			case 0x40:
				up_group_four_Cmd_pro(cmd->cmd,str);
				break;
			case 0x50:
				up_group_five_Cmd_pro(cmd->cmd,str);
				break;
			case 0x60:
				up_group_six_Cmd_pro(cmd->cmd,str);
				break;
			case 0x70:
				up_group_seven_Cmd_pro(cmd->cmd,str);
				break;
			default:
				break;
		}
	}
}

void up_group_zero_Cmd_pro(unsigned char cmd_id, const unsigned char *cube_buf) {
	int result = -1;
	/* 上注时间结构体 */
	ctrl_syntime_t * ptime 	= (ctrl_syntime_t *)cube_buf;
	/* 延时遥测指令结构体 */
	ctrl_delayhk_t * phk	= (ctrl_delayhk_t *)cube_buf;
	/* 普通无参命令结构体 */
	ctrl_nopara_t * pdata   = (ctrl_nopara_t *)cube_buf;
	/* 远程遥控指令结构体 */
    rsh_command_t * rsh_cmd = (rsh_command_t *)cube_buf;

	uint8_t	* pcmds			= (uint8_t *)cube_buf;
	uint8_t cmd_len			= *(pcmds - 1) - 10;

	cmd_ack_t obc_ack = { .head[0] = 0x1A,
	                      .head[1] = 0x53,
	                      .id = 1,
	                      .cmd = cmd_id,
	                      .result = 0   };

	switch (cmd_id)
	{
        case INS_HK_GET:

            IsRealTelemetry = 0;
            hk_select 	= (uint8_t)phk->select;

            if(phk->select == HK_SDCARD)
            {

                hkListNode_t * hk_node = NULL;

                hkleek = 0;

                /*如果成功找到对应的延时遥测*/
                if((hk_node = hk_list_find(phk->secs)) != NULL)
                {
                    sprintf(hk_sd_path, "hk/%u.txt", hk_node->TimeValue);

                    int result = f_open(&hkfile, hk_sd_path, FA_READ | FA_OPEN_EXISTING);
                    /*如果文件打开失败*/
                    if(result != FR_OK)
                    {
                        IsRealTelemetry 	= 1;
                        obc_ack.result 	= 0;
                        driver_debug(DEBUG_HK,"the filename is not existing\r\n");
                        driver_debug(DEBUG_HK,"open file error ,result is :%u\r\n",result);
                    }
                    obc_ack.result = 1;
                }
                else
                {
                    IsRealTelemetry 	= 1;
                    obc_ack.result 	= 0;
                }
            }else if(phk->select == HK_SRAM)
            {

                f_close(&hkfile);

                hk_sram_index = (hk_main_fifo.rear - phk->index)>0?
                                (hk_main_fifo.rear - phk->index):
                                (HK_FIFO_BUFFER_CNT - phk->index + hk_main_fifo.rear);
                obc_ack.result = 1;
            }
            obc_cmd_ack(&obc_ack, sizeof(cmd_ack_t));
            break;
        case INS_OBC_STR_DOWN:

            up_hk_down_cmd = 1;
            obc_ack.result = 1;
            obc_cmd_ack(&obc_ack, sizeof(cmd_ack_t));
            break;
        case INS_OBC_STO_DOWN:

            vContrlStopDownload();
            obc_ack.result = 1;
            obc_cmd_ack(&obc_ack, sizeof(cmd_ack_t));
            break;
        case INS_OBC_RST:

            obc_ack.result = 1;
            obc_cmd_ack(&obc_ack, sizeof(cmd_ack_t));
            delay_ms(10);
            cpu_reset();
            break;
        case INS_TIME_TO_OBC:

            ptime->msecs += 2;
            result = timesync(ptime->msecs);
            if(result == -1) {
                obc_ack.result = 1;
            }else {
                obc_ack.result = 0;
            }
            obc_cmd_ack(&obc_ack, sizeof(cmd_ack_t));
            break;
        case INS_ADCS_ON:

            adcs_pwr_sta	= 1;
            up_cmd_adcs_pwr	= 1;
            obc_ack.result 	= 1;
            EpsOutSwitch(OUT_EPS_S0, ENABLE);
            EpsOutSwitch(OUT_EPS_S0, ENABLE);
            obc_cmd_ack(&obc_ack, sizeof(cmd_ack_t));
            break;
        case INS_ADCS_OFF:

            EpsOutSwitch(OUT_EPS_S0, DISABLE);
            EpsOutSwitch(OUT_EPS_S0, DISABLE);
            up_cmd_adcs_pwr = 0;
            adcs_pwr_sta	= 0;
            obc_ack.result 	= 1;
            obc_cmd_ack(&obc_ack, sizeof(cmd_ack_t));
            break;
        case INS_PAL_ON:

            result = enable_panel(pdata->delay,0);
            if(result == 1) {
                obc_ack.result = 1;
            }else {
                obc_ack.result = 0;
            }
            obc_cmd_ack(&obc_ack, sizeof(cmd_ack_t));
            break;
        case INS_PAL_OFF:

            result = disable_panel(pdata->delay,0);
            if(result == 1) {
                obc_ack.result = 1;
            }else {
                obc_ack.result = 0;
            }
            obc_cmd_ack(&obc_ack, sizeof(cmd_ack_t));
            break;
        case INS_ANTS_ON:

            result = open_antenna();
            if(result == -1) {
                obc_ack.result = 1;
            }else {
                obc_ack.result = 0;
            }
            obc_cmd_ack(&obc_ack, sizeof(cmd_ack_t));
            break;
        case INS_ANTS_PWR_ON:

            result = enable_antspwr(pdata->delay, 0);
            if(result == 1) {
                obc_ack.result 	= 1;
            }else {
                obc_ack.result = 0;
            }
            obc_cmd_ack(&obc_ack, sizeof(cmd_ack_t));
            break;
        case INS_ANTS_PWR_OFF:

            result = disable_antspwr(pdata->delay, 0);
            if(result == 1) {
                obc_ack.result 	= 1;
            }else {
                obc_ack.result = 0;
            }
            obc_cmd_ack(&obc_ack, sizeof(cmd_ack_t));
            break;
        case INS_RSH_CMD:

            if (command_run(rsh_cmd->command) == CMD_ERROR_NONE)
            {
                obc_ack.result = 1;
            }
            else
            {
                obc_ack.result = 0;
            }
            obc_cmd_ack(&obc_ack, sizeof(cmd_ack_t));
            break;
        case INS_BATCH_CMD:

            obc_ack.result = 1;
            obc_cmd_ack(&obc_ack, sizeof(cmd_ack_t));
            pcmds += 7;
            while(cmd_len > 0)
            {
                CubeUnPacket(pcmds);
                while(*pcmds++ != '\0' && --cmd_len);
            }
            break;
        default:
            break;
	}
}

void up_group_one_Cmd_pro(unsigned char cmd_id, const unsigned char *cube_buf) {
	int result = 0;
	ctrl_nopara_t * pdata = (ctrl_nopara_t *)cube_buf;
	cmd_ack_t obc_ack = { .head[0] = 0x1A, .head[1] = 0x53, .id = 1, .cmd = cmd_id};

	switch (cmd_id)
	{

        case TR_BOOT:

            if(xDTBTeleControlSend(0x01, 1000) != 0)
                obc_ack.result = 0;
            else
                obc_ack.result = 1;

                obc_cmd_ack(&obc_ack, sizeof(cmd_ack_t));
            break;
        case TR_SHUT_DOWN:

            if(xDTBTeleControlSend(0x02, 1000) != 0)
                obc_ack.result = 0;
            else
                obc_ack.result = 1;

                obc_cmd_ack(&obc_ack, sizeof(cmd_ack_t));
            break;
        case TR_MEM_RESET:

            if(xDTBTeleControlSend(0x03, 1000) != 0)
                obc_ack.result = 0;
            else
                obc_ack.result = 1;

                obc_cmd_ack(&obc_ack, sizeof(cmd_ack_t));
            break;
        case TR_MEM1_RECORD:

            if(xDTBTeleControlSend(0x04, 1000) != 0)
                obc_ack.result = 0;
            else
                obc_ack.result = 1;

                obc_cmd_ack(&obc_ack, sizeof(cmd_ack_t));
                break;
        case TR_MEM2_RECORD:

            if(xDTBTeleControlSend(0x05, 1000) != 0)
                obc_ack.result = 0;
            else
                obc_ack.result = 1;

                obc_cmd_ack(&obc_ack, sizeof(cmd_ack_t));
            break;
        case TR_MEM3_RECORD:

            if(xDTBTeleControlSend(0x06, 1000) != 0)
                obc_ack.result = 0;
            else
                obc_ack.result = 1;

                obc_cmd_ack(&obc_ack, sizeof(cmd_ack_t));
            break;
        case TR_MEM4_RECORD:

            if(xDTBTeleControlSend(0x07, 1000) != 0)
                obc_ack.result = 0;
            else
                obc_ack.result = 1;

                obc_cmd_ack(&obc_ack, sizeof(cmd_ack_t));
            break;
        case TR_MEM_STOP:

            if(xDTBTeleControlSend(0x08, 1000) != 0)
                obc_ack.result = 0;
            else
                obc_ack.result = 1;

                obc_cmd_ack(&obc_ack, sizeof(cmd_ack_t));
            break;
        case TR_MEM1_BACK:

            if(xDTBTeleControlSend(0x09, 1000) != 0)
                obc_ack.result = 0;
            else
                obc_ack.result = 1;

                obc_cmd_ack(&obc_ack, sizeof(cmd_ack_t));
            break;
        case TR_MEM2_BACK:

            if(xDTBTeleControlSend(0x0A, 1000) != 0)
                obc_ack.result = 0;
            else
                obc_ack.result = 1;

                obc_cmd_ack(&obc_ack, sizeof(cmd_ack_t));
            break;
        case TR_MEM3_BACK:

            if(xDTBTeleControlSend(0x0B, 1000) != 0)
                obc_ack.result = 0;
            else
                obc_ack.result = 1;

                obc_cmd_ack(&obc_ack, sizeof(cmd_ack_t));
            break;
        case TR_MEM4_BACK:

            if(xDTBTeleControlSend(0x0C, 1000) != 0)
                obc_ack.result = 0;
            else
                obc_ack.result = 1;

                obc_cmd_ack(&obc_ack, sizeof(cmd_ack_t));
            break;
        case TR_MEM1_ERA:

            if(xDTBTeleControlSend(0x0D, 1000) != 0)
                obc_ack.result = 0;
            else
                obc_ack.result = 1;

                obc_cmd_ack(&obc_ack, sizeof(cmd_ack_t));
            break;
        case TR_MEM2_ERA:

            if(xDTBTeleControlSend(0x0E, 1000) != 0)
                obc_ack.result = 0;
            else
                obc_ack.result = 1;

                obc_cmd_ack(&obc_ack, sizeof(cmd_ack_t));
            break;
        case TR_MEM3_ERA:

            if(xDTBTeleControlSend(0x0F, 1000) != 0)
                obc_ack.result = 0;
            else
                obc_ack.result = 1;

                obc_cmd_ack(&obc_ack, sizeof(cmd_ack_t));
            break;
        case TR_MEM4_ERA:

            if(xDTBTeleControlSend(0x10, 1000) != 0)
                obc_ack.result = 0;
            else
                obc_ack.result = 1;

                obc_cmd_ack(&obc_ack, sizeof(cmd_ack_t));
            break;
        default:
            break;
	}
}

void up_group_two_Cmd_pro(unsigned char cmd_id, const unsigned char *cube_buf) {
	int result = 0;
	ctrl_nopara_t * pdata = (ctrl_nopara_t *)cube_buf;
	cmd_ack_t obc_ack = { .head[0] = 0x1A, .head[1] = 0x53, .id = 1, .cmd = cmd_id };

	switch (cmd_id) {

        case TR_PC_ON:

            if(xDTBTeleControlSend(0x11, 1000) != 0)
                obc_ack.result = 0;
            else
                obc_ack.result = 1;

                obc_cmd_ack(&obc_ack, sizeof(cmd_ack_t));
            break;
        case TR_PC_OFF:

            if(xDTBTeleControlSend(0x12, 1000) != 0)
                obc_ack.result = 0;
            else
                obc_ack.result = 1;

                obc_cmd_ack(&obc_ack, sizeof(cmd_ack_t));
            break;
        case TR_1M_RATE:

            if(xDTBTeleControlSend(0x13, 1000) != 0)
                obc_ack.result = 0;
            else
                obc_ack.result = 1;

                obc_cmd_ack(&obc_ack, sizeof(cmd_ack_t));
            break;
        case TR_2M_RATE:

            if(xDTBTeleControlSend(0x14, 1000) != 0)
                obc_ack.result = 0;
            else
                obc_ack.result = 1;

                obc_cmd_ack(&obc_ack, sizeof(cmd_ack_t));
            break;
        case TR_4M_RATE:

            if(xDTBTeleControlSend(0x15, 1000) != 0)
                obc_ack.result = 0;
            else
                obc_ack.result = 1;

                obc_cmd_ack(&obc_ack, sizeof(cmd_ack_t));
            break;
        case TR_5V_ON:

            if(EpsOutSwitch(OUT_DIGI_TRAN_5V, ENABLE) == EPS_OK)
            {
                obc_ack.result = 1;
            }
            else
            {
                obc_ack.result = 0;
            }

            obc_cmd_ack(&obc_ack, sizeof(cmd_ack_t));
            break;

        case TR_5V_OFF:

            if(EpsOutSwitch(OUT_DIGI_TRAN_5V, DISABLE) == EPS_OK)
            {
                obc_ack.result = 1;
            }
            else
            {
                obc_ack.result = 0;
            }

            obc_cmd_ack(&obc_ack, sizeof(cmd_ack_t));
            break;

        case TR_12V_ON:

            if(EpsOutSwitch(OUT_DIGI_TRAN_12V, ENABLE) == EPS_OK)
            {
                obc_ack.result = 1;
            }
            else
            {
                obc_ack.result = 0;
            }

            obc_cmd_ack(&obc_ack, sizeof(cmd_ack_t));
            break;

        case TR_12V_OFF:

            if(EpsOutSwitch(OUT_DIGI_TRAN_12V, DISABLE) == EPS_OK)
            {
                obc_ack.result = 1;
            }
            else
            {
                obc_ack.result = 0;
            }

            obc_cmd_ack(&obc_ack, sizeof(cmd_ack_t));
            break;

        default:
            break;
	}
}

void up_group_three_Cmd_pro(unsigned char cmd_id, const unsigned char *cube_buf)
{
	int result = -1;
	ctrl_nopara_t * pdata   = (ctrl_nopara_t *)cube_buf;
	rsh_command_t * rsh_cmd = (rsh_command_t *)cube_buf;
	cam_cmd_t * pcmd_cam    = (cam_cmd_t *)cube_buf;
	cam_opt opt = {0};
	cmd_ack_t obc_ack = { .head[0] = 0x1A,
	                      .head[1] = 0x53,
	                      .id = 0x01,
	                      .cmd = cmd_id};

	switch (cmd_id)
	{

        case CAM_SOFTWARE_RESET:

            if(Camera_805_reset() == 0x7e)
                obc_ack.result = 1;
            else
                obc_ack.result = 0;

            obc_cmd_ack(&obc_ack, sizeof(cmd_ack_t));
            break;

        case CAM_EXPOSURE_TIME_SET:

            if(Camera_Exposure_Time_Set(pcmd_cam->CamPara) == pcmd_cam->CamPara)
                obc_ack.result = 1;
            else
                obc_ack.result = 0;

            obc_cmd_ack(&obc_ack, sizeof(cmd_ack_t));
            break;

        case CAM_GAIN_SET:

            if(Camera_Gain_Set(pcmd_cam->CamPara) == pcmd_cam->CamPara)
                obc_ack.result = 1;
            else
                obc_ack.result = 0;

            obc_cmd_ack(&obc_ack, sizeof(cmd_ack_t));
            break;

        case CAM_WORK_MODE_SET:

            if(Camera_Work_Mode_Set(pcmd_cam->TransMode, pcmd_cam->WorkMode,
                    pcmd_cam->AutoExpo, pcmd_cam->ImageId))
            {
                obc_ack.result = 1;
            }
            else
            {
                obc_ack.result = 0;
            }

            obc_cmd_ack(&obc_ack, sizeof(cmd_ack_t));
            break;
        case DOWN_IMAGE_INFO_FALSH:

            if(xImageInfoDownload(1, 0) == 0)
            {
                obc_ack.result = 1;
            }
            else
            {
                obc_ack.result = 0;
            }

            obc_cmd_ack(&obc_ack, sizeof(cmd_ack_t));
            break;
        case DOWN_IMAGE_INFO_TF:

            if(xImageInfoDownload(0, pcmd_cam->ImageId) == 0)
            {
                obc_ack.result = 1;
            }
            else
            {
                obc_ack.result = 0;
            }

            obc_cmd_ack(&obc_ack, sizeof(cmd_ack_t));
            break;
        case DOWN_IMAGE_FLASH_WHOLE:

            opt.is_sd = 0;
            opt.is_single = 0;

            if(xImageDownload(opt, 0, 0) != 0)
                obc_ack.result = 0;
            else
                obc_ack.result = 1;

            obc_cmd_ack(&obc_ack, sizeof(cmd_ack_t));
            break;
        case DOWN_IMAGE_FALSH_SINGLE:

            opt.is_sd = 0;
            opt.is_single = 1;

            if(xImageDownload(opt, 0, pcmd_cam->PacketId) != 0)
                obc_ack.result = 0;
            else
                obc_ack.result = 1;

            obc_cmd_ack(&obc_ack, sizeof(cmd_ack_t));
            break;
        case DOWN_IMAGE_TF_WHOLE:

            opt.is_sd = 1;
            opt.is_single = 0;

            if(xImageDownload(opt, pcmd_cam->ImageId, 0) != 0)
                obc_ack.result = 0;
            else
                obc_ack.result = 1;

            obc_cmd_ack(&obc_ack, sizeof(cmd_ack_t));
            break;
        case DOWN_IMAGE_TF_SINGLE:

            opt.is_sd = 1;
            opt.is_single = 1;

            if(xImageDownload(opt, pcmd_cam->ImageId, pcmd_cam->PacketId) != 0)
                obc_ack.result = 0;
            else
                obc_ack.result = 1;

            obc_cmd_ack(&obc_ack, sizeof(cmd_ack_t));
            break;
        case CAM_POWER_ON:

            if(EpsOutSwitch(OUT_CAMERA_10W, ENABLE) == EPS_OK
                    && EpsOutSwitch(OUT_CAMERA_5W, ENABLE) == EPS_OK)
            {
                obc_ack.result = 1;
            }
            else
            {
                obc_ack.result = 0;
            }

            obc_cmd_ack(&obc_ack, sizeof(cmd_ack_t));
            break;

        case CAM_POWER_OFF:

            if(EpsOutSwitch(OUT_CAMERA_10W, DISABLE) == EPS_OK
                    && EpsOutSwitch(OUT_CAMERA_5W, DISABLE) == EPS_OK)
            {
                obc_ack.result = 1;
            }
            else
            {
                obc_ack.result = 0;
            }

            obc_cmd_ack(&obc_ack, sizeof(cmd_ack_t));
            break;
        default:
            break;
	}
}

void up_group_four_Cmd_pro(unsigned char cmd_id, const unsigned char *cube_buf) {
	ctrl_nopara_t 	* pdata 	= (ctrl_nopara_t *)cube_buf;
	ctrl_downtime_t * pperiod	= (ctrl_downtime_t *)cube_buf;
	cmd_ack_t obc_ack = { .head[0] = 0x1A, .head[1] = 0x53, .id = 1, .cmd = cmd_id };
	int result = -1;

	switch (cmd_id) {
	case INS_DOWN_PERIOD:
		if(pperiod->msecs > 0) {
			downtimeset = pperiod->msecs;
			StorageIntervalCount = (HK_STORAGE_INTERVAL*1000)/downtimeset;;
			Stop_Down_Time = 600000/downtimeset;
		}
		break;
	default:
		break;
	}
}

void up_group_five_Cmd_pro(unsigned char cmd_id, const unsigned char *cube_buf) {
	int result = -1;
	ctrl_syntime_t * ptime 		= (ctrl_syntime_t *)cube_buf;
	cmd_ack_t obc_ack 			= { .head[0] = 0x1A, .head[1] = 0x53, .id = 1, .cmd = cmd_id };


	switch (cmd_id) {
	default:
	case INS_TIME_SYN:
		result = timesync(ptime->msecs);
		if(result == -1) {
			obc_ack.result = 1;
		}else {
			obc_ack.result = 0;
		}
		obc_cmd_ack(&obc_ack, sizeof(cmd_ack_t));
		break;
	}
}

void up_group_six_Cmd_pro(unsigned char cmd_id, const unsigned char *cube_buf) {
	ctrl_nopara_t * pdata = (ctrl_nopara_t *)cube_buf;
	int result = -1;
	cmd_ack_t obc_ack 			= { .head[0] = 0x1A, .head[1] = 0x53, .id = 1, .cmd = cmd_id };


	switch (cmd_id) {
	default:
		break;
	}
}

void up_group_seven_Cmd_pro(unsigned char cmd_id, const unsigned char *cube_buf){

	uint32_t camera_delay= 0;
	ctrl_downtime_t * pperiod	= (ctrl_downtime_t *)cube_buf;
	int result = -1;
	cmd_ack_t obc_ack 			= { .head[0] = 0x1A, .head[1] = 0x53, .id = 1, .cmd = cmd_id };

	camera_delay = pperiod->msecs;
	switch (cmd_id) {
		case JPG_DELAY_TASK:
			obc_ack.result = 1;
			obc_cmd_ack(&obc_ack, sizeof(cmd_ack_t));
			vTaskDelay(camera_delay);
			get_jpg();
			break;
		case DOWNLOAD_SAVED_AUDIOFILES:
			obc_ack.result = 1;
			obc_cmd_ack(&obc_ack, sizeof(cmd_ack_t));
			vTaskDelay(camera_delay);
			download_jpg();
			break;
		case UP_NEW_AUDIOFILES:
			break;
		case DOWNLOAD_NEW_AUDIOFILES:
			break;

		default:
			break;
		}


};

