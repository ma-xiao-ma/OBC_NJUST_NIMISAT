#include <if_downlink_serial.h>
#include <stdio.h>

#include "cube_com.h"

#include "sensor/flash_sd.h"

#include "bsp_usart.h"
#include "bsp_reset.h"
#include "bsp_delay.h"
#include "bsp_ds1302.h"
#include "bsp_switch.h"
#include "dtb_805.h"
#include "QB50_mem.h"

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
#include "route.h"
#include "router_io.h"

#define ORGCALL			"BI4ST0"
#define DETCALL			"BI4ST1"

extern uint8_t adcs_pwr_sta;
extern uint8_t up_cmd_adcs_pwr;

unsigned char cube_buf[80];

unsigned char func =0;
uint32_t rec_cmd_cnt = 0; //obc接收本地指令计数

struct USART_TypeDefStruct GCS_Usart;
#if USE_SERIAL_PORT_DOWNLINK_INTERFACE
    void obc_cmd_ack(uint8_t type, uint8_t result)
    {
        route_packet_t *packet = (route_packet_t *)qb50Malloc((size_t)(sizeof(route_packet_t) + 1));

        packet->len = 1;
        packet->dst = GND_ROUTE_ADDR;
        packet->src = MY_ROUTE_ADDR;
        packet->typ = type;
        packet->dat[0] = result;

        route_queue_wirte(packet, NULL);
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

void CubeUnPacket(const void *str)
{
    route_packet_t * packet = (route_packet_t *)str;


    /*obc接收本地指令计数*/
    rec_cmd_cnt++;

    switch ((packet->typ) & 0xF0) {
        case 0x00:
            up_group_zero_Cmd_pro(packet->typ, packet->dat);
            break;
        case 0x10:
            up_group_one_Cmd_pro(packet->typ, packet->dat);
            break;
        case 0x20:
            up_group_two_Cmd_pro(packet->typ, packet->dat);
            break;
        case 0x30:
            up_group_three_Cmd_pro(packet->typ, packet->dat);
            break;
        case 0x40:
            up_group_four_Cmd_pro(packet->typ, packet->dat);
            break;
        case 0x50:
            up_group_five_Cmd_pro(packet->typ, packet->dat);
            break;
        case 0x60:
            up_group_six_Cmd_pro(packet->typ, packet->dat);
            break;
        case 0x70:
            up_group_seven_Cmd_pro(packet->typ, packet->dat);
            break;
        default:
            break;
	}
}

void up_group_zero_Cmd_pro(unsigned char cmd_id, const unsigned char *cube_buf) {
	int result = 0;
//	/* 上注时间结构体 */
//	ctrl_syntime_t * ptime 	= (ctrl_syntime_t *)cube_buf;
	/* 延时遥测指令结构体 */
	ctrl_delayhk_t * phk	= (ctrl_delayhk_t *)cube_buf;
	/* 普通无参命令结构体 */
	ctrl_nopara_t * pdata   = (ctrl_nopara_t *)cube_buf;
	/* 远程遥控指令结构体 */
    rsh_command_t * rsh_cmd = (rsh_command_t *)cube_buf;

	uint8_t	* pcmds			= (uint8_t *)cube_buf;
	uint8_t cmd_len			= *(pcmds - 1) - 10;


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

                    FRESULT f_result = f_open(&hkfile, hk_sd_path, FA_READ | FA_OPEN_EXISTING);
                    /*如果文件打开失败*/
                    if(f_result != FR_OK)
                    {
                        IsRealTelemetry 	= 1;
                        result 	= 0;
                        driver_debug(DEBUG_HK,"the filename is not existing\r\n");
                        driver_debug(DEBUG_HK,"open file error ,result is :%u\r\n",f_result);
                    }
                    result = 1;
                }
                else
                {
                    IsRealTelemetry 	= 1;
                    result 	= 0;
                }
            }else if(phk->select == HK_SRAM)
            {

                f_close(&hkfile);

                hk_sram_index = (hk_main_fifo.rear - phk->index)>0?
                                (hk_main_fifo.rear - phk->index):
                                (HK_FIFO_BUFFER_CNT - phk->index + hk_main_fifo.rear);
                result = 1;
            }
            obc_cmd_ack(cmd_id, result);
            break;
        case INS_OBC_STR_DOWN:

            up_hk_down_cmd = 1;
            result = 1;
            obc_cmd_ack(cmd_id, result);
            break;
        case INS_OBC_STO_DOWN:

            vContrlStopDownload();
            result = 1;
            obc_cmd_ack(cmd_id, result);
            break;
        case INS_OBC_RST:

            result = 1;
            obc_cmd_ack(cmd_id, result);
            delay_ms(10);
            cpu_reset();
            break;
        case INS_TIME_TO_OBC:

//            ptime->msecs += 2;
//            result = timesync(ptime->msecs);
//            if(result == -1) {
//                result = 1;
//            }else {
//                result = 0;
//            }
//            obc_cmd_ack(cmd_id, result);
            break;
        case INS_ADCS_ON:

            adcs_pwr_sta	= 1;
            up_cmd_adcs_pwr	= 1;
            result 	= 1;
            EpsOutSwitch(OUT_EPS_S0, ENABLE);
            EpsOutSwitch(OUT_EPS_S0, ENABLE);
            obc_cmd_ack(cmd_id, result);
            break;
        case INS_ADCS_OFF:

            EpsOutSwitch(OUT_EPS_S0, DISABLE);
            EpsOutSwitch(OUT_EPS_S0, DISABLE);
            up_cmd_adcs_pwr = 0;
            adcs_pwr_sta	= 0;
            result 	= 1;
            obc_cmd_ack(cmd_id, result);
            break;
        case INS_PAL_ON:

            if(enable_panel(pdata->delay,0) == 1)
                result = 1;
            else
                result = 0;

            obc_cmd_ack(cmd_id, result);
            break;
        case INS_PAL_OFF:

            if(disable_panel(pdata->delay,0) == 1)
                result = 1;
            else
                result = 0;

            obc_cmd_ack(cmd_id, result);
            break;
        case INS_ANTS_ON:

            if (open_antenna() == -1)
                result = 1;
            else
                result = 0;

            obc_cmd_ack(cmd_id, result);
            break;

        case INS_ANTS_PWR_ON:

            if(enable_antspwr(pdata->delay,0) == 1)
                result = 1;
            else
                result = 0;

            obc_cmd_ack(cmd_id, result);
            break;
        case INS_ANTS_PWR_OFF:

            if(disable_antspwr(pdata->delay,0) == 1)
                result = 1;
            else
                result = 0;

            obc_cmd_ack(cmd_id, result);
            break;
        case INS_RSH_CMD:

            if (command_run(rsh_cmd->command) == CMD_ERROR_NONE)
                result = 1;
            else
                result = 0;

            obc_cmd_ack(cmd_id, result);
            break;
        case INS_BATCH_CMD:

//            result = 1;
//            obc_cmd_ack(cmd_id, result);
//            pcmds += 7;
//            while(cmd_len > 0)
//            {
//                CubeUnPacket(pcmds);
//                while(*pcmds++ != '\0' && --cmd_len);
//            }
            break;
        default:
            break;
	}
}

void up_group_one_Cmd_pro(unsigned char cmd_id, const unsigned char *cube_buf) {
	int result = 0;

	switch (cmd_id)
	{

        case TR_BOOT:

            if(xDTBTeleControlSend(Boot, 1000) != 0)
                result = 0;
            else
                result = 1;

                obc_cmd_ack(cmd_id, result);
            break;
        case TR_SHUT_DOWN:

            if(xDTBTeleControlSend(ShutDown, 1000) != 0)
                result = 0;
            else
                result = 1;

                obc_cmd_ack(cmd_id, result);
            break;
        case TR_MEM_RESET:

            if(xDTBTeleControlSend(MemReset, 1000) != 0)
                result = 0;
            else
                result = 1;

                obc_cmd_ack(cmd_id, result);
            break;
        case TR_MEM1_RECORD:

            if(xDTBTeleControlSend(Mem1Record, 1000) != 0)
                result = 0;
            else
                result = 1;

                obc_cmd_ack(cmd_id, result);
                break;
        case TR_MEM2_RECORD:

            if(xDTBTeleControlSend(Mem2Record, 1000) != 0)
                result = 0;
            else
                result = 1;

                obc_cmd_ack(cmd_id, result);
            break;
        case TR_MEM3_RECORD:

            if(xDTBTeleControlSend(Mem3Record, 1000) != 0)
                result = 0;
            else
                result = 1;

                obc_cmd_ack(cmd_id, result);
            break;
        case TR_MEM4_RECORD:

            if(xDTBTeleControlSend(Mem4Record, 1000) != 0)
                result = 0;
            else
                result = 1;

                obc_cmd_ack(cmd_id, result);
            break;
        case TR_MEM_STOP:

            if(xDTBTeleControlSend(MemStop, 1000) != 0)
                result = 0;
            else
                result = 1;

                obc_cmd_ack(cmd_id, result);
            break;
        case TR_MEM1_BACK:

            if(xDTBTeleControlSend(Mem1Back, 1000) != 0)
                result = 0;
            else
                result = 1;

                obc_cmd_ack(cmd_id, result);
            break;
        case TR_MEM2_BACK:

            if(xDTBTeleControlSend(Mem2Back, 1000) != 0)
                result = 0;
            else
                result = 1;

                obc_cmd_ack(cmd_id, result);
            break;
        case TR_MEM3_BACK:

            if(xDTBTeleControlSend(Mem3Back, 1000) != 0)
                result = 0;
            else
                result = 1;

                obc_cmd_ack(cmd_id, result);
            break;
        case TR_MEM4_BACK:

            if(xDTBTeleControlSend(Mem4Back, 1000) != 0)
                result = 0;
            else
                result = 1;

                obc_cmd_ack(cmd_id, result);
            break;
        case TR_MEM1_ERA:

            if(xDTBTeleControlSend(Mem1Erase, 1000) != 0)
                result = 0;
            else
                result = 1;

                obc_cmd_ack(cmd_id, result);
            break;
        case TR_MEM2_ERA:

            if(xDTBTeleControlSend(Mem2Erase, 1000) != 0)
                result = 0;
            else
                result = 1;

                obc_cmd_ack(cmd_id, result);
            break;
        case TR_MEM3_ERA:

            if(xDTBTeleControlSend(Mem3Erase, 1000) != 0)
                result = 0;
            else
                result = 1;

                obc_cmd_ack(cmd_id, result);
            break;
        case TR_MEM4_ERA:

            if(xDTBTeleControlSend(Mem4Erase, 1000) != 0)
                result = 0;
            else
                result = 1;

                obc_cmd_ack(cmd_id, result);
            break;
        default:
            break;
	}
}

void up_group_two_Cmd_pro(unsigned char cmd_id, const unsigned char *cube_buf)
{
	int result = 0;

	switch (cmd_id) {

        case TR_PC_ON:

            if(xDTBTeleControlSend(PseudoOn, 1000) != 0)
                result = 0;
            else
                result = 1;

                obc_cmd_ack(cmd_id, result);
            break;
        case TR_PC_OFF:

            if(xDTBTeleControlSend(PseudoOff, 1000) != 0)
                result = 0;
            else
                result = 1;

                obc_cmd_ack(cmd_id, result);
            break;
        case TR_1M_RATE:

            if(xDTBTeleControlSend(Rate1Mbps, 1000) != 0)
                result = 0;
            else
                result = 1;

                obc_cmd_ack(cmd_id, result);
            break;
        case TR_2M_RATE:

            if(xDTBTeleControlSend(Rate2Mbps, 1000) != 0)
                result = 0;
            else
                result = 1;

                obc_cmd_ack(cmd_id, result);
            break;
        case TR_4M_RATE:

            if(xDTBTeleControlSend(Rate4Mbps, 1000) != 0)
                result = 0;
            else
                result = 1;

                obc_cmd_ack(cmd_id, result);
            break;
        case TR_5V_ON:

            if(EpsOutSwitch(OUT_DTB_5V, ENABLE) == EPS_OK)
                result = 1;
            else
                result = 0;

            obc_cmd_ack(cmd_id, result);
            break;

        case TR_5V_OFF:

            if(EpsOutSwitch(OUT_DTB_5V, DISABLE) == EPS_OK)
                result = 1;
            else
                result = 0;

            obc_cmd_ack(cmd_id, result);
            break;

        case TR_12V_ON:

            if(EpsOutSwitch(OUT_DTB_12V, ENABLE) == EPS_OK)
                result = 1;
            else
                result = 0;

            obc_cmd_ack(cmd_id, result);
            break;

        case TR_12V_OFF:

            if(EpsOutSwitch(OUT_DTB_12V, DISABLE) == EPS_OK)
                result = 1;
            else
                result = 0;

            obc_cmd_ack(cmd_id, result);
            break;

        default:
            break;
	}
}

void up_group_three_Cmd_pro(unsigned char cmd_id, const unsigned char *cube_buf)
{
	int result = 0;

	cam_cmd_t * cam_cmd    = (cam_cmd_t *)cube_buf;

	switch (cmd_id)
	{

        case CAM_SOFTWARE_RESET:

            if(Camera_805_reset() == E_NO_ERR)
                result = 1;
            else
                result = 0;

            obc_cmd_ack(cmd_id, result);
            break;

        case CAM_EXPOSURE_TIME_SET:

            if(Camera_Exposure_Time_Set( cam_cmd->exp_time ) == E_NO_ERR)
                result = 1;
            else
                result = 0;

            obc_cmd_ack(cmd_id, result);
            break;

        case CAM_GAIN_SET:

            if(Camera_Gain_Set( cam_cmd->gain ) == E_NO_ERR)
                result = 1;
            else
                result = 0;

            obc_cmd_ack(cmd_id, result);
            break;

        case CAM_WORK_MODE_SET:

            if(Camera_Work_Mode_Set( cam_cmd->cam_ctl_mode ) == E_NO_ERR)
                result = 1;
            else
                result = 0;

            obc_cmd_ack(cmd_id, result);
            break;
        case DOWN_NEWEST_IMAGE_INFO:

            if(cam_newest_img_info_down() == E_NO_ERR)
                result = 1;
            else
                result = 0;

            obc_cmd_ack(cmd_id, result);
            break;
        case DOWN_IMAGE_INFO:

            if(cam_img_info_down( cam_cmd->img_down.img_id ) == E_NO_ERR)
                result = 1;
            else
                result = 0;

            obc_cmd_ack(cmd_id, result);
            break;
        case DOWN_IMAGE_DATA_WHOLE:

            if(cam_img_data_down( cam_cmd->img_down.img_id ) != E_NO_ERR)
                result = 0;
            else
                result = 1;

            obc_cmd_ack(cmd_id, result);
            break;
        case DOWN_IMAGE_DATA_SINGLE:

            if(cam_img_packet_down( cam_cmd->img_down.img_id, cam_cmd->img_down.packet_id ) != E_NO_ERR)
                result = 0;
            else
                result = 1;

            obc_cmd_ack(cmd_id, result);
            break;
        case DOWN_IMAGE_DATA_PART:

            if(cam_img_data_packet_down( cam_cmd->img_down.img_id, cam_cmd->img_down.packet_id ) != E_NO_ERR)
                result = 0;
            else
                result = 1;

            obc_cmd_ack(cmd_id, result);
            break;
        case CAM_POWER_ON:

            if(EpsOutSwitch(OUT_CAMERA_10W, ENABLE) == EPS_OK
                    && EpsOutSwitch(OUT_CAMERA_5W, ENABLE) == EPS_OK)
            {
                result = 1;
            }
            else
            {
                result = 0;
            }

            obc_cmd_ack(cmd_id, result);
            break;

        case CAM_POWER_OFF:

            if(EpsOutSwitch(OUT_CAMERA_10W, DISABLE) == EPS_OK
                    && EpsOutSwitch(OUT_CAMERA_5W, DISABLE) == EPS_OK)
            {
                result = 1;
            }
            else
            {
                result = 0;
            }

            obc_cmd_ack(cmd_id, result);
            break;

        case CAM_HEAT2_ON:

            if(EpsOutSwitch(OUT_CAMERA_HEAT_2, ENABLE) == EPS_OK)
            {
                result = 1;
            }
            else
            {
                result = 0;
            }

            obc_cmd_ack(cmd_id, result);
            break;

        case CAM_HEAT2_OFF:

            if(EpsOutSwitch(OUT_CAMERA_HEAT_2, DISABLE) == EPS_OK)
            {
                result = 1;
            }
            else
            {
                result = 0;
            }

            obc_cmd_ack(cmd_id, result);
            break;
        default:
            break;
	}
}

void up_group_four_Cmd_pro(unsigned char cmd_id, const unsigned char *cube_buf) {
	ctrl_nopara_t 	* pdata 	= (ctrl_nopara_t *)cube_buf;
	ctrl_downtime_t * pperiod	= (ctrl_downtime_t *)cube_buf;
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
			result = 1;
		}else {
			result = 0;
		}
		obc_cmd_ack(cmd_id, result);
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
			result = 1;
			obc_cmd_ack(cmd_id, result);
			vTaskDelay(camera_delay);
			get_jpg();
			break;
		case DOWNLOAD_SAVED_AUDIOFILES:
			result = 1;
			obc_cmd_ack(cmd_id, result);
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

