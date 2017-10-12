#include <if_downlink_serial.h>
#include "contrl.h"

#include "hk_arg.h"

#include "cube_com.h"

#include "sensor/flash_sd.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "portmacro.h"

#include "ctrl_cmd_types.h"
#include "error.h"
#include "driver_debug.h"
#include "switches.h"
#include "if_adcs.h"
#include "QB50_mem.h"
#include "hk.h"
#include "crc.h"
#include "command.h"

#include "bsp_ds1302.h"
#include "bsp_switch.h"
#include "bsp_pca9665.h"
#include "bsp_cis.h"
#include "bsp_cpu_flash.h"
#include "router_io.h"

#include "csp.h"

#include <stdint.h>
#include <time.h>
#include <string.h>


#define OpenAntenna_Time 		(10*60)   //
#define OpenBattery_Time 		(15*60)   //

#define Sleep_Battery 			7.2
#define Normal_Battery   		7.5		//
#define Abnormal_Battery   		1.0		//


uint8_t adcs_pwr_sta 	= 0;
uint8_t up_cmd_adcs_pwr	= 1;

///////////////////////////////////////local function////////////////////////////

//static uint8_t downloadopt = 0;
//static uint8_t cnt_to_save = 0;
uint8_t IsRealTelemetry = 1;	//1为实时遥测   0为延时遥测
///work mode variable
uint8_t mode = SLEEP_MODE;
static uint16_t down_cnt = 0; //遥测下行时间控制变量
static uint8_t down_cmd_enable = 0;
/////////////////////////////////////////////////////////////////////////////////


void taskdelaytime(uint32_t *time) {
	downtimeset = *time;
}

void downdelaytimes(uint32_t *times) {
    StorageIntervalCount = *times;
}

void chcontinuetimes(uint32_t *times) {
	Stop_Down_Time = *times;
}

#if USE_SERIAL_PORT_DOWNLINK_INTERFACE

    void SendDownCmd(void *pData, uint32_t Length)
    {
        vSerialSend(pData, Length);
    }

#else

    #define MaxDataToSend   220

    /* ISIS.TrxVU.ICD
     * Transmission Bitrate:        4800 Bit/s
     * MaxDataByte each I2C packet: 220  Byte
    */
    /*******************************************************************************
    函数说明:  ISIS通信板下行数据接口函数
    入口参数:
            pData     : 待下行数据指针
            Datalen   : 待下行数据长度
    返回值:   无
    *******************************************************************************/
    void SendDownCmd(void *pData, uint32_t Length) {
        uint8_t rest_frames     = 0;
        uint8_t DataLen       = 0;
        uint8_t* DataToSend     = (uint8_t*)pData;
        uint8_t ErrorCount      = 0;
        if(set_transmission_bitrate_4800() != -1)
        {
            driver_debug(DEBUG_ICD, "isis set rate error\n");
        }
        vTaskDelay(10);
        do{
            DataLen = (Length>MaxDataToSend) ? MaxDataToSend: Length;
            if(ICD_send(DataToSend, DataLen, &rest_frames)!= -1)
            {
                ErrorCount ++;
                driver_debug(DEBUG_ICD, "isis send transaction error\n");
            }
            else
            {
                DataToSend += DataLen;
                Length -= DataLen;
            }
            if(Length != 0)
                vTaskDelay(400);
        }while(Length != 0 && ErrorCount < 3);
    }

#endif



void NormalWorkMode(void)
{

	if (mode == SLEEP_MODE)
	{
		mode = NORMAL_MODE;
	}

	adcs_send_mode(mode);
}

void SleepWorkMode(void)
{

	if (mode == NORMAL_MODE)
	{
		mode = SLEEP_MODE;
	}

	adcs_send_mode(mode);
}

//void ControlTask(void * pvParameters __attribute__((unused))) {
//
//	portTickType xLastWakeTime = xTaskGetTickCount(); //for the 10s timer task
//
//	vTaskDelayUntil(&xLastWakeTime, (10000 / portTICK_RATE_MS));
//
//	while (1)
//	{
//
//		switch (Battery_Task())
//		{
//            case 0:
//                SleepWorkMode();
//                break;
//            case 1:
//                NormalWorkMode();
//                break;
//            case 2:
//                break;
//            default:
//                break;
//		}
//
//		vTaskDelayUntil(&xLastWakeTime, (5000 / portTICK_RATE_MS));
//	}
//}

void eps_task(void *pvParameters __attribute__((unused)))
{
    static uint8_t ctime;

    eps_start();

    while (1)
    {
        eps_hk();

        vTaskDelay(1000);

//        if (ctime++ > 10)
//        {
//
//            ctime = 0;

//            switch (Battery_Task())
//            {
//                case 0:
//                    SleepWorkMode();
//                    break;
//                case 1:
//                    NormalWorkMode();
//                    break;
//                case 2:
//                    break;
//                default:
//                    break;
//            }
//        }
    }
}

void OpenAntenna_Task(void* param __attribute__((unused))) {

	portTickType CurTime = xTaskGetTickCount();

	if(obc_boot_count <= 5){
		vTaskDelayUntil(&CurTime, OpenAntenna_Time * (1000 / portTICK_RATE_MS));
	}else{
		vTaskDelayUntil(&CurTime, 120 * (1000 / portTICK_RATE_MS));
	}

//	else if (antenna_status == 2)
//	{
//		vTaskDelete(NULL);
//	}

	enable_antspwr(0,0);

	vTaskDelay(1000 / portTICK_RATE_MS);

	while (1)
	{
	    /*若天线全部展开*/
		if (get_antenna_status_nopara() == 2)
		{
		    /*置展开标志为2*/
			antenna_status = 2;
			disable_antspwr(0,0);
			vTaskDelete(NULL);
		}

		if (open_antenna() != 0)
		{
			vTaskDelay(20000 / portTICK_RATE_MS);

			if (get_antenna_status_nopara() == 1)
			{
				antenna_status = 1;
			}
			else if (get_antenna_status_nopara() == 2)
			{
				antenna_status = 2;
				disable_antspwr(0,0);
				vTaskDelete(NULL);
			}
			else
			{
				vTaskDelay(2000 / portTICK_RATE_MS);
			}
		}
		else
		{
			vTaskDelay(2000 / portTICK_RATE_MS);
		}
	}
}

/////////////////////////////////////////////
void OpenPanel_Task(void* param __attribute__((unused))) {

	if (obc_boot_count > 5) {
		vTaskDelete(NULL);
	}

	portTickType CurTime = xTaskGetTickCount();

	if(obc_boot_count <= 3){
		vTaskDelayUntil(&CurTime, OpenBattery_Time * (1000 / portTICK_RATE_MS));
	}else{
		vTaskDelayUntil(&CurTime, 180 * (1000 / portTICK_RATE_MS));
	}

	while (1) {
		enable_panel(0,0);
		openpanel_times++;
		if (openpanel_times >= 2) {
			disable_panel(0,0);
			vTaskDelete(NULL);
		}
		vTaskDelay(10000 / portTICK_RATE_MS);
	}
}

int Battery_Task(void) {

	float BatteryVoltage = 0.0;

	BatteryVoltage = EpsHouseKeeping.Out_BusV*0.001;

	if (BatteryVoltage > Normal_Battery || BatteryVoltage < Abnormal_Battery)
		return 1;
	else if (BatteryVoltage < Sleep_Battery)
		return 0;
	else
		return 2;
}

void vContrlStopDownload(void)
{
    up_hk_down_cmd = 0;
    vTaskDelay(100);
    down_cmd_enable = 0;
    down_cnt = 0;
}

void down_save_task(void * pvParameters __attribute__((unused))) {

//	portTickType xLastWakeTime_2 = xTaskGetTickCount(); //for the 2s timer task

    /*初始化主帧FIFO和辅帧FIFO*/
	HK_fifoInit(&hk_main_fifo);
	HK_fifoInit(&hk_append_fifo);

	hk_store_init();

	/*存储时间间隔15秒*/
	StorageIntervalCount = (HK_STORAGE_INTERVAL*1000)/downtimeset;
	/*下行时间间隔downtimeset毫秒， 一共下行10分钟，也就是600000毫秒*/
	Stop_Down_Time = 600000/downtimeset;

	while (1)
	{

		//if ((up_hk_down_cmd == 1 || PassFlag == 1) && down_cnt <= 60) {
		if ((up_hk_down_cmd == 1 ) && down_cnt <= 60)
		{
			up_hk_down_cmd = 0;
			down_cmd_enable = 1;
//			PassFlag = 0;
		}
		/* 如果开始下行标志置1，则下行遥测 */
		if (down_cmd_enable == 1)
		{
			hk_down_proc_task();

			if (++down_cnt >= Stop_Down_Time)  //下行次数 = 600000/downtimeset 默认下载200次
			{
				down_cnt = 0;
				down_cmd_enable = 0;
				IsRealTelemetry = 1;
			}
		}
		/* 否则保存遥测 */
		else
		{

			if (++down_cnt >= StorageIntervalCount)  //存储间隔15秒
			{
				down_cnt = 0;
				up_hk_down_cmd = 0;
//				PassFlag = 0;

				hk_data_save_task();
			}
		}

		vTaskDelay((downtimeset / portTICK_RATE_MS));
	}
}

void hk_down_proc_task(void)
{
    /*如果是实时遥测数据下行*/
	if (IsRealTelemetry == 1)
	{
        hk_collect_no_store();
        SendDownCmd(&hk_frame.main_frame, sizeof(HK_Main_t));
        vTaskDelay(10 / portTICK_RATE_MS);
        SendDownCmd(&hk_frame.append_frame, sizeof(HK_Append_t));
	}
	/*如果是延时遥测下行*/
	else
	{
	    /*若选择TF卡延时遥测下行*/
        if(hk_select == HK_SDCARD)
        {
//              int result = f_open(&hkfile, hk_sd_path, FA_READ | FA_OPEN_EXISTING);
//              if(result != FR_OK){
//                  driver_debug(DEBUG_HK,"the filename is not existing\r\n");
//                  driver_debug(DEBUG_HK,"open file error ,result is :%u\r\n",result);
//              }
            vPortEnterCritical();
            int result = f_lseek(&hkfile, hkleek);
            if(result != FR_OK)
            {
                f_close(&hkfile);
                hkleek      = 0;
                IsRealTelemetry = 1;
                vPortExitCritical();
                return;
            }

            result = f_read(&hkfile, &hk_old_frame, sizeof(HK_Store_t), &hkrbytes);
            if(result == FR_OK)
            {
                if(hkrbytes != sizeof(HK_Store_t))
                {
                    f_close(&hkfile);
                    hkleek      = 0;
                    IsRealTelemetry = 1;
                    vPortExitCritical();
                    return;
                }
                hkleek += sizeof(HK_Store_t);
            }
            else
            {
                f_close(&hkfile);
                hkleek      = 0;
                IsRealTelemetry = 1;
            }
            vPortExitCritical();

            SendDownCmd(&hk_old_frame.main_frame, sizeof(HK_Main_t));
            vTaskDelay(10 / portTICK_RATE_MS);
            SendDownCmd(&hk_old_frame.append_frame, sizeof(HK_Append_t));
        }

        /* 若选择SRAM延时遥测下行 */
        if(hk_select == HK_SRAM)
        {
            memcpy(&hk_old_frame.main_frame, hk_main_fifo.frame[hk_sram_index], sizeof(HK_Main_t));
            SendDownCmd(&hk_old_frame.main_frame, sizeof(HK_Main_t));
            memcpy(&hk_old_frame.append_frame, hk_append_fifo.frame[hk_sram_index], sizeof(HK_Append_t));
            SendDownCmd(&hk_old_frame.append_frame, sizeof(HK_Append_t));
            if((hk_sram_index++)%HK_FIFO_BUFFER_CNT == hk_main_fifo.rear)
            {
                IsRealTelemetry   = 1;
                hk_sram_index = 0;
            }
        }
	}
}

void hk_down_store_task(void) {

	if (antenna_status != 0) {

		hk_collect();
		SendDownCmd(&hk_frame.main_frame, sizeof(HK_Main_t));
		vTaskDelay(10 / portTICK_RATE_MS);
		SendDownCmd(&hk_frame.append_frame, sizeof(HK_Append_t));
	}
}

void hk_data_save_task(void) {

	hk_collect();
	hk_store_add();
}


void adcs_pwr_task(void *pvParameters __attribute__((unused))) {

	EpsOutSwitch(OUT_EPS_S0, ENABLE);  //enable ADCS power

	while (1) {
		EpsOutSwitch(OUT_EPS_S0, ENABLE);  //enable ADCS power
		if(SW_EPS_S0_PIN()) {
			vTaskDelete(NULL);
		}
		vTaskDelay(200);
		EpsOutSwitch(OUT_EPS_S0, ENABLE);  //enable ADCS power
		if(SW_EPS_S0_PIN()) {
			vTaskDelete(NULL);
		}
		vTaskDelay(200);
	}

}

void DownloadSavedAudioFiles (void *para)
{
	uint8_t cmd_length = 0;
	cup_flash_content_t *pdata 	= NULL;
	cup_flash_cmd_t *cmd = (cup_flash_cmd_t *)para;


	portTickType xLastWakeTime = xTaskGetTickCount();

	uint32_t FlashAddr = ADDR_FLASH_SECTOR_8 + (uint32_t)(cmd->index * FlanshBlockSize);
	pdata = (cup_flash_content_t *)(FlashAddr + (uint32_t)(cmd->id * sizeof(cup_flash_content_t)));

	cmd_length = cmd->len;

	while(cmd_length--)
	{
		uint8_t cis_len = 0;

		(pdata->len > 91) ? (cis_len = 91) : (cis_len = pdata->len);

		for (int i = 0; i < cis_len + 3; i++)
		{
			driver_debug(DEBUG_FLASH, "0x%x ", ((uint8_t *)(pdata))[i]);
		}
		driver_debug(DEBUG_FLASH, "\r\n\r\n");
		FlanshAudioFilesToCis(pdata, cis_len + 3, CIS_DELAY);
		pdata ++;
		vTaskDelayUntil(&xLastWakeTime, ( 600 / portTICK_RATE_MS));
	}
}


uint16_t hton16(uint16_t h16) {
	return (((h16 & 0xff00) >> 8) |
			((h16 & 0x00ff) << 8));
}


void SaveNewAudioFiles (void *para)
{
	cup_flash_content_t *pdata = (cup_flash_content_t *)para;

	if(pdata->index < 0 || pdata->index > 3)
		return;
	if(pdata->id < 0 || pdata->id > 345)
		return;
	uint8_t cis_len = 0;
	(pdata->len > 91) ? (cis_len = 91) : (cis_len = pdata->len);


	uint32_t FlashAddr = ADDR_FLASH_SECTOR_9 + (uint32_t)(pdata->index * FlanshBlockSize) +
			(uint32_t)(pdata->id * sizeof(cup_flash_content_t));

	if(pdata->index == 0 && pdata->id == 0){
		bsp_EraseCpuFlash(FlashAddr);
		bsp_WriteCpuFlash(FlashAddr, (uint8_t *)pdata, (uint32_t)cis_len + 3);
	}
	else{
		bsp_WriteCpuFlash(FlashAddr, (uint8_t *)pdata, (uint32_t)cis_len + 3);
	}
}

void SavePermanentAudioFiles (void *para)
{
	cup_flash_content_t *pdata = (cup_flash_content_t *)para;

	if(pdata->index < 0 || pdata->index > 3)
		return;
	if(pdata->id < 0 || pdata->id > 345)
		return;
	uint8_t cis_len = 0;
	(pdata->len > 91) ? (cis_len = 91) : (cis_len = pdata->len);


	uint32_t FlashAddr = ADDR_FLASH_SECTOR_8 + (uint32_t)(pdata->index * FlanshBlockSize) +
			(uint32_t)(pdata->id * sizeof(cup_flash_content_t));

	if(pdata->index == 0 && pdata->id == 0){
		bsp_EraseCpuFlash(FlashAddr);
		bsp_WriteCpuFlash(FlashAddr, (uint8_t *)pdata, (uint32_t)cis_len + 3);
	}
	else{
		bsp_WriteCpuFlash(FlashAddr, (uint8_t *)pdata, (uint32_t)cis_len + 3);
	}
}


void DownloadNewAudioFiles (void *para)
{
	uint8_t cmd_length = 0;
	cup_flash_content_t *pdata 	= NULL;
	cup_flash_cmd_t *cmd = (cup_flash_cmd_t *)para;
	uint32_t FlashAddr = ADDR_FLASH_SECTOR_9 + (uint32_t)(cmd->index * FlanshBlockSize);
	pdata = (cup_flash_content_t *)(FlashAddr + (uint32_t)(cmd->id * sizeof(cup_flash_content_t)));

	cmd_length = cmd->len;

	portTickType xLastWakeTime = xTaskGetTickCount();

	while(cmd_length--)
	{
		uint8_t cis_len = 0;

		(pdata->len > 91) ? (cis_len = 91) : (cis_len = pdata->len);

		for (int i = 0; i < cis_len + 3; i++)
		{
			driver_debug(DEBUG_FLASH, "0x%x ", ((uint8_t *)(pdata))[i]);
		}
		driver_debug(DEBUG_FLASH, "\r\n\r\n");
		FlanshAudioFilesToCis(pdata, cis_len + 3, CIS_DELAY); //cis_ret = 0 indicate that success to send data to cis
		pdata ++;
		vTaskDelayUntil(&xLastWakeTime, ( 600 / portTICK_RATE_MS));
	}
}


///* 哈工大通信机 */
//void cmd_task(void * para) {
//
//	i2c_frame_t * frame = (i2c_frame_t *)para;
//
//	CubeUnPacket(&frame->data[4]);
//
//	vTaskDelete(NULL);
//}

/* ISIS通信机 */
void ObcUnpacketTask(void *pvPara)
{
    uplink_content_t *pdata = (uplink_content_t *)pvPara;

    CubeUnPacket(&pdata->Packet.Id);
    /* 释放传入任务的内存块 */
    qb50Free(pdata);
    /* 删除自身任务 */
    vTaskDelete(NULL);
}


//void i2c_slave_task_zero(void *param __attribute__((unused))) {
//
//	int 			handle 			= 0;
//	uint32_t 		length			= 0;
//	uint8_t         pname[] 		= "CMD0";
//
//	i2c_frame_t 	* frame 		= NULL;
//	cup_flash_cmd_t * pcmd  		= NULL;
//	csp_id_t		* id			= NULL;
//	ctrl_nopara_t   * cmd			= NULL;
//	while(1)
//	{
//
//		if(i2c_receive(handle, &frame, portMAX_DELAY) == E_NO_ERR){
//
//			length = frame->len;
//
//			driver_debug(DEBUG_I2C, "I2C receive handle %d, length: %u\n\r", handle, length);
//
//			id = (csp_id_t *)frame->data;
//			if((id->src == 1) && (id->dst == 26))
//			{
//
//				cmd = (ctrl_nopara_t *)&frame->data[4];
//
//				if(cmd->id == 2) //id为2的指令是地面发给ADCS的
//					i2c_master_transaction(OBC_TO_ADCS_HANDLE, ADCS_I2C_ADDR, &frame->data[4], frame->len-4, NULL, 0, 1000);
//				else //id为1的指令是地面发给OBC的
//				{
//					pname[3]++;
//					xTaskCreate(cmd_task, (const signed char*) pname, configMINIMAL_STACK_SIZE * 2,frame, tskIDLE_PRIORITY + 4, NULL);
//				}
//			}
//			qb50Free(frame);
//		}
//	}
//}


//void route_server_task(void *param __attribute__((unused))) {
//
//    route_packet_t *packet = NULL;
//	uint16_t len	= 0;
//
//	while(1)
//	{
//		if((adcs_pwr_sta == 0) && (up_cmd_adcs_pwr == 1)) {
//			EpsOutSwitch(OUT_EPS_S0, ENABLE);  //enable ADCS power
//		}
//
//		if(xI2CServerReceive(packet, portMAX_DELAY) == E_NO_ERR)
//		{
//			if(packet == NULL)
//		    {
//            /*driver_debug(DEBUG_I2C,*/printf( "I2C server task error! %u\n\r");
//		        continue;
//			}
//
//            len = packet->len;
//            /*driver_debug(DEBUG_I2C,*/printf( "I2C rx length: %u\n\r", len+3);
//
//
//            /*姿控命令应答*/
//            if(packet->dat[0] == 0xEB && packet->dat[1] == 0x53)
//            {
//                adcs_pwr_sta = 1;
//                packet->dat[0] = 0x1A;
//                obc_cmd_ack(&packet->dat[0], sizeof(cmd_ack_t));
//            }
//
//            /*遥测辅帧*/
//            if(packet->dat[0] == 0x1A && packet->dat[1] == 0x54)
//            {
//                adcs_pwr_sta = 1;
//                memcpy((uint8_t*)&(hk_frame.append_frame.adcs_hk), &packet->dat[2], sizeof(adcs805_hk_t));
//            }
//
//            qb50Free(packet);
//		}
//	}
//}


void isis_read_task(void *para __attribute__((unused))) {

    uint16_t frames_num             = 0;
    uint8_t * pCRC                  = NULL;
    uplink_content_t *pdata         = NULL;
    uint32_t cmd_crc                = 0;
    uint8_t data_len                = 0;
    uint8_t pname[]                 = "CMD0";

    while(1) {
        /* 获取ICD接收指令计数 */
        I2C_ICD_read_countofframe((uint8_t*)&frames_num);
        vTaskDelay(1000);

        if(frames_num == 0)
        {
            continue;
        }
        /* 清空ICD接收指令计数 */
        frames_num = 0;

        /* 为每包数据申请内存 */
        pdata = (uplink_content_t *)qb50Malloc(I2C_MTU);

        /* 从ICD收地面发来的数据包 */
        I2C_ICD_get_frame_stable((uint8_t*)pdata);
        vTaskDelay(1000);
        /* 若读数据失败 */
        if(I2C_ICD_get_frame_stable((uint8_t*)pdata) != E_NO_ERR)
        {
            qb50Free(pdata);
            continue;
        }
        /* 清空ICD接收缓冲区，为接收下一包数据做准备 */
        I2C_ICD_sweep_butter();

        pCRC = &pdata->Packet;
        data_len = pdata->Packet.DataLength - 5; //除了4字节的CRC本身和1字节的Tail,其他数据都参与

        /*CRC校验*/
        cmd_crc = crc32_memory((uint8_t *)pCRC, data_len);
        if(cmd_crc != *((uint32_t *)&pCRC[data_len]))
        {
            I2C_ICD_sweep_butter();
            qb50Free(pdata);
            continue;
        }
        /*收到正确地面正确指令，设开始下行标志为1，启动遥测数据下行*/
//        up_hk_down_cmd = 1;
        switch(pdata->Packet.Id)
        {
            case 1:
                pname[3]++;
                /* 如果任务创建失败，则跳出switch语句，释放内存 */
                if(xTaskCreate(ObcUnpacketTask, (const signed char*)pname, configMINIMAL_STACK_SIZE * 2,
                        pdata, tskIDLE_PRIORITY + 4, NULL) != pdPASS)
                {
                    break;
                }
                /* 如果任务创建成功则在任务中释放内存，进行下一轮循环 */
                continue;
            case 2:
                i2c_master_transaction(OBC_TO_ADCS_HANDLE, ADCS_I2C1_ADDR, &pdata->Packet.Id,
                        pdata->Packet.DataLength, NULL, 0, 1000);
                break;
            default:
                break;
        }

        I2C_ICD_sweep_butter();

        qb50Free(pdata);
    }
}


