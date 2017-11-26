/*
 * cmd_trxvu.c
 *
 *  Created on: 2017年10月02日
 *      Author: Ma Wenli
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <time.h>

#include "FreeRTOS.h"
#include "task.h"

#include "driver_debug.h"
#include "command.h"
#include "console.h"
#include "bsp_pca9665.h"
#include "obc_mem.h"
#include "error.h"
#include "csp_endian.h"
#include "hk.h"

#include "if_trxvu.h"

/*测试发射速率*/
int isis_send_handler(struct command_context * context)
{

    static uint32_t num_of_frame = 0;
    static uint32_t interval_ms = 0;
    static uint8_t frame_len;
    int ret, errors = 0, times = 0;
    uint32_t j = 1;

    char * args = command_args(context);

    if (sscanf(args, "%u %u %u", &frame_len, &num_of_frame, &interval_ms) != 3)
        return CMD_ERROR_SYNTAX;

    if (frame_len < 8 || frame_len > ISIS_MTU)
        return CMD_ERROR_SYNTAX;

	uint8_t *frame = ObcMemMalloc(frame_len/*ISIS_MTU*/);
	if(frame == NULL)
	    return CMD_ERROR_SYNTAX;

	for (int i=0; i<frame_len/*ISIS_MTU*/; i++)
	    frame[i] = 0x55;

	uint8_t rest_of_frame = 0;

	printf("Framelen %u byte, Send %u frame, Interval %u ms.\r\n", frame_len, num_of_frame, interval_ms);
	printf("...\n");

	*(uint32_t *)(frame + 4) = csp_htobe32((uint32_t)0x12345678);
	do {
	    *(uint32_t *)frame = csp_htobe32(j);

	    ret = vu_transmitter_send_frame(frame, frame_len/*ISIS_MTU*/, &rest_of_frame);

        if (ret != E_NO_ERR || rest_of_frame == 0xFF)
        {
            errors++;
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
        else
        {
            j++;
            num_of_frame--;
            /**若发射机缓冲区已满，则等待5000毫秒*/
            if(rest_of_frame == 0)
            {
                times++;
                vTaskDelay(5000 / portTICK_PERIOD_MS);
            }
        }

	    if (interval_ms)
	        vTaskDelay(interval_ms);

	} while((num_of_frame > 0) && (errors < 5));

	printf("Slots %u frame, Remain %u frame, Error %u, Delay %u\r\n", rest_of_frame, num_of_frame, errors, times);

	ObcMemFree(frame);
	return CMD_ERROR_NONE;
}

int isis_transmitter_status_handler(struct command_context * context __attribute__((unused))) {

    rsp_transmitter_state state = {0};

    if (vu_transmitter_get_state(&state) == E_NO_ERR)
    {
        printf("Transmitter idle state: %u\r\n", state.IdleState);
        printf("Beacon active:          %u\r\n", state.BeaconAct);
        printf("Transmitter bit rate:   %u\r\n", state.BitRate);
    }

	return CMD_ERROR_NONE;
}

int isis_read_count_handler(struct command_context * context __attribute__((unused))) {

	uint16_t rest_num = 0;

	if (vu_receiver_get_frame_num(&rest_num) == E_NO_ERR)
	    printf("Number of frame in receiver buffer: %u\n", rest_num);

	return CMD_ERROR_NONE;
}

int isis_tx_uptime_handler(struct command_context * context __attribute__((unused)))
{
	static uint32_t TX_time = 0;

	if (vu_transmitter_get_uptime(&TX_time) == E_NO_ERR)
		printf("Transmitter MCU has been active: %u s\n\n", TX_time);

	return CMD_ERROR_NONE;
}

int isis_rx_uptime_handler(struct command_context * context __attribute__((unused)))
{
    uint32_t RX_time = 0;

	if (vu_receiver_get_uptime(&RX_time) == E_NO_ERR)
	    printf("Receiver MCU has been active: %u s\n\n", RX_time);

	return CMD_ERROR_NONE;
}


int isis_set_rate_handler(struct command_context * context __attribute__((unused)))
{

	uint32_t rate = 0;
	par_transmission_bitrate para = {0};

	char * args = command_args(context);
	if (sscanf(args, "%u" , &rate) != 1)
		return CMD_ERROR_SYNTAX;

	if (rate == 0)
	    para = bps1200;
	else if (rate == 1)
	    para = bps2400;
	else if (rate == 2)
        para = bps4800;
	else
        para = bps9600;

	if (vu_transmitter_set_bitrate(para) == E_NO_ERR)
	    printf("Set OK!!!\r\n");

	return CMD_ERROR_NONE;
}

int isis_sleep_status_handler(struct command_context * context __attribute__((unused))) {

	uint32_t sleep_status = 0;
	par_idle_state IdleState = {0};

	char * args = command_args(context);
	if (sscanf(args, "%u" , &sleep_status) != 1)
		return CMD_ERROR_SYNTAX;

	if (sleep_status == 0)
	{
	    IdleState = TurnOff;
	    printf("Transmitter is turned off when idle!\r\n");
	}
	else
	{
	    IdleState = RemainOn;
	    printf("Transmitter remains on when idle!\r\n");
	}

    if (vu_transmitter_set_idle_state(IdleState) == E_NO_ERR)
        printf("Set OK!!!\r\n");

	return CMD_ERROR_NONE;
}

int isis_measure_tx_status_handler(struct command_context * context __attribute__((unused))) {

    rsp_tx_tm status = {0};

    printf("TX Measured current TM:\r\n\r\n");
	if(vu_transmitter_measure_tm(&status) == E_NO_ERR)
	{
	    printf("TM Item\t\t\tRaw Value\tActual value:\r\n");
	    printf("*******************************************************\r\n");
	    printf("ReflectedPower:\t\t%u\t\t%.4f dBm(%.4f mW)\n"
	            "ForwardPower:\t\t%u\t\t%.4f dBm(%.4f mW)\n"
	            "BusVoltage:\t\t%u\t\t%.4f V\n"
	            "TotalCurrent:\t\t%u\t\t%.4f mA\n"
	            "AmplifierTemp:\t\t%u\t\t%.4f C\n"
	            "OscillatorTemp:\t\t%u\t\t%.4f C\n",
	            status.ReflectedPower,VU_RRP_dBm(status.ReflectedPower),VU_RRP_mW(status.ReflectedPower),
	            status.ForwardPower,VU_RFP_dBm(status.ForwardPower),VU_RFP_mW(status.ForwardPower),
	            status.BusVoltage,VU_PBV_V(status.BusVoltage),
	            status.TotalCurrent,VU_TCC_mA(status.TotalCurrent),
	            status.AmplifierTemp,VU_PAT_C(status.AmplifierTemp),
	            status.OscillatorTemp,VU_LOT_C(status.OscillatorTemp)
               );
	}

	return CMD_ERROR_NONE;
}

int isis_last_status_handler(struct command_context * context __attribute__((unused))) {

    rsp_tx_tm status = {0};

    printf("TX Stored last TM:\r\n\r\n");
    if(vu_transmitter_get_last_tm(&status) == E_NO_ERR)
    {
        printf("TM Item\t\t\tRaw Value\tActual value:\r\n");
        printf("********************************************************\r\n");
        printf("ReflectedPower:\t\t%u\t\t%.4f dBm(%.4f mW)\n"
                "ForwardPower:\t\t%u\t\t%.4f dBm(%.4f mW)\n"
                "BusVoltage:\t\t%u\t\t%.4f V\n"
                "TotalCurrent:\t\t%u\t\t%.4f mA\n"
                "AmplifierTemp:\t\t%u\t\t%.4f C\n"
                "OscillatorTemp:\t\t%u\t\t%.4f C\n",
                status.ReflectedPower,VU_RRP_dBm(status.ReflectedPower),VU_RRP_mW(status.ReflectedPower),
                status.ForwardPower,VU_RFP_dBm(status.ForwardPower),VU_RFP_mW(status.ForwardPower),
                status.BusVoltage,VU_PBV_V(status.BusVoltage),
                status.TotalCurrent,VU_TCC_mA(status.TotalCurrent),
                status.AmplifierTemp,VU_PAT_C(status.AmplifierTemp),
                status.OscillatorTemp,VU_LOT_C(status.OscillatorTemp)
               );
    }
	return CMD_ERROR_NONE;
}


int isis_set_default_callsigns_handler(struct command_context * context __attribute__((unused)))
{

    par_default_call_set callsign = {0};

    /*将字符串的值拷贝到数组*/
    char *str1 = "BI4ST-0";
    for(int i=0; *(str1+i) != '\0'; i++)
        *(callsign.DstCall + i) = *(str1 + i);

    /*将字符串的值拷贝到数组*/
    char *str2 = "NJUST-4";
    for(int i=0; *(str2+i) != '\0'; i++)
        *(callsign.SrcCall + i) = *(str2 + i);

	if (vu_transmitter_set_callsigns(&callsign) == E_NO_ERR)
	    printf("Set OK!!!\r\n");

	return CMD_ERROR_NONE;
}


int isis_clearben_handler(struct command_context * context __attribute__((unused)))
{

	if (vu_transmitter_clear_beacon() == E_NO_ERR)
	    printf("Clear beacon success!!!\r\n");

	return CMD_ERROR_NONE;
}

int isis_set_call_beacon_handler(struct command_context * context __attribute__((unused)))
{

    uint16_t Interval;

    char * args = command_args(context);
    if (sscanf(args, "%u" , &Interval) != 1)
        return CMD_ERROR_SYNTAX;

    par_beacon_new_call_set *frame = ObcMemMalloc(sizeof(par_beacon_new_call_set)+ISIS_MTU);

    frame->RepeatInterval = Interval;

    char *str1 = "BI4ST-0";
    for(int i=0; *(str1+i) != '\0'; i++)
        *(frame->DstCall + i) = *(str1 + i);

    char *str2 = "NJUST-4";
    for(int i=0; *(str2+i) != '\0'; i++)
        *(frame->SrcCall + i) = *(str2 + i);

    for(int i=0; i<ISIS_MTU; i++)
        frame->BeaconContent[i] = i+1;

	if(vu_transmitter_beacon_new_call_set(frame, ISIS_MTU) == E_NO_ERR)
	    printf("Set beacon with new callsigns OK!!!\r\n");

	ObcMemFree(frame);
	return CMD_ERROR_NONE;
}

int isis_set_beacon_handler(struct command_context * context __attribute__((unused)))
{

    uint16_t Interval;

    char * args = command_args(context);
    if (sscanf(args, "%u" , &Interval) != 1)
        return CMD_ERROR_SYNTAX;

    par_beacon_set *beacon = ObcMemMalloc(sizeof(par_beacon_set)+ISIS_MTU);

    beacon->RepeatInterval = Interval;

    for(int i=0; i<ISIS_MTU; i++)
        beacon->BeaconContent[i] = i+1;

	if (vu_transmitter_beacon_set(beacon, ISIS_MTU) == E_NO_ERR)
	    printf("Set beacon with default callsigns OK\r\n");

	ObcMemFree(beacon);
	return CMD_ERROR_NONE;
}

int isis_sendAXdate_handler(struct command_context * context __attribute__((unused)))
{

	par_frame_new_call *frame = ObcMemMalloc(sizeof(par_frame_new_call)+ISIS_MTU);

    char *str1 = "BI4ST-0";
    for(int i=0; *(str1+i) != '\0'; i++)
            *(frame->DstCall + i) = *(str1 + i);

    char *str2 = "NJUST-4";
    for(int i=0; *(str2+i) != '\0'; i++)
            *(frame->SrcCall + i) = *(str2 + i);

    for(int i=0; i<ISIS_MTU; i++)
        frame->FrameContent[i] = i+1;

    uint8_t slot;
	if (vu_transmitter_send_frame_new_callsigns(frame, ISIS_MTU, &slot) == E_NO_ERR)
	{
	    printf("New call frame send success!!\r\n");
	    printf("Slot = %u\r\n", slot);
	}

	ObcMemFree(frame);
	return CMD_ERROR_NONE;
}

int isis_clearbutter_handler(struct command_context * context __attribute__((unused)))
{

	if(vu_receiver_remove_frame() == E_NO_ERR)
	    printf("Receiver buffer remove frame success!!\r\n");

	return CMD_ERROR_NONE;
}

int isis_read_frame_handler(struct command_context * context __attribute__((unused)))
{

    rsp_frame *frame = ObcMemMalloc(sizeof(rsp_frame)+ISIS_RX_MTU);

	if( vu_receiver_get_frame(frame, ISIS_RX_MTU) == E_NO_ERR)
	{
        printf("TM Item\t\t\tRaw Value\tActual value:\r\n");
        printf("********************************************************\r\n");
	    printf("DopplerOffset:\t\t%u\t\t%.4f Hz\n"
	           "RSSI:\t\t\t%u\t\t%.4f dBm\n",
	            frame->DopplerOffset, VU_SDO_Hz(frame->DopplerOffset),
	            frame->RSSI, VU_RSS_dBm(frame->RSSI)
              );

	    for(uint32_t i=0; i<frame->DateSize; i++)
	        printf("0x%02x ", frame->Data[i]);
	    printf("\r\n");
	}

	ObcMemFree(frame);

	return CMD_ERROR_NONE;
}


int isis_watchdog_resetdog_handler(struct command_context * context)
{
    uint32_t opt;

    char * args = command_args(context);
    if (sscanf(args, "%u" , &opt) != 1)
        return CMD_ERROR_SYNTAX;

    if(opt == 0)
    {
        if(vu_receiver_watchdog_reset() == E_NO_ERR)
            printf("Receiver watch dog reset!!!\r\n");
    }
    else
    {
        if(vu_transmitter_watchdog_reset() == E_NO_ERR)
            printf("Transmitter watch dog reset!!!\r\n");
    }

	return CMD_ERROR_NONE;
}

int isis_software_reset_handler(struct command_context * context)
{
    uint32_t opt;

    char * args = command_args(context);
    if (sscanf(args, "%u" , &opt) != 1)
        return CMD_ERROR_SYNTAX;

    if(opt == 0)
    {
        if(vu_receiver_software_reset() == E_NO_ERR)
            printf("Receiver software reset!!!\r\n");
    }
    else
    {
        if(vu_transmitter_software_reset() == E_NO_ERR)
            printf("Transmitter software reset!!!\r\n");
    }

    return CMD_ERROR_NONE;
}

int isis_hardware_reset_handler(struct command_context * context)
{
    uint32_t opt;

    char * args = command_args(context);
    if (sscanf(args, "%u" , &opt) != 1)
        return CMD_ERROR_SYNTAX;

    if(opt == 0)
    {
        if(vu_receiver_hardware_reset() == E_NO_ERR)
            printf("Receiver hardware reset!!!\r\n");
    }
    else
    {
        if(vu_transmitter_hardware_reset() == E_NO_ERR)
            printf("Transmitter hardware reset!!!\r\n");
    }

    return CMD_ERROR_NONE;
}

int isis_measure_of_reciever_handler(struct command_context * context __attribute__((unused)))
{

    rsp_rx_tm rx_tm = {0};

    printf("RX Measured current TM:\r\n\r\n");
    if(vu_receiver_measure_tm(&rx_tm) == E_NO_ERR)
    {
        printf("TM Item\t\t\tRaw Value\tActual value:\r\n");
        printf("*********************************************************\r\n");
        printf("DopplerOffset:\t\t%u\t\t%.4f Hz\n"
               "RSSI:\t\t\t%u\t\t%.4f dBm\n"
               "BusVoltage:\t\t%u\t\t%.4f V\n"
               "TotalCurrent:\t\t%u\t\t%.4f mA\n"
               "AmplifierTemp:\t\t%u\t\t%.4f C\n"
               "OscillatorTemp:\t\t%u\t\t%.4f C\n",
               rx_tm.DopplerOffset, VU_SDO_Hz(rx_tm.DopplerOffset),
               rx_tm.RSSI, VU_RSS_dBm(rx_tm.RSSI),
               rx_tm.BusVoltage, VU_PBV_V(rx_tm.BusVoltage),
               rx_tm.TotalCurrent, VU_TCC_mA(rx_tm.TotalCurrent),
               rx_tm.AmplifierTemp, VU_PAT_C(rx_tm.AmplifierTemp),
               rx_tm.OscillatorTemp, VU_LOT_C(rx_tm.OscillatorTemp)
              );
    }

	return CMD_ERROR_NONE;
}

int isis_all_tm_handler(struct command_context * context __attribute__((unused)))
{

    vu_isis_hk_t *hk = (vu_isis_hk_t *)ObcMemMalloc(sizeof(vu_isis_hk_t));
    if (hk == NULL)
        return CMD_ERROR_SYNTAX;

    if(ttc_hk_get_peek(hk) != pdTRUE)
        return CMD_ERROR_SYNTAX;

    printf("Receiver MCU has been active: %u s\n\n", hk->ru_uptime);
    printf("\n\n");
    printf("RX Measured current TM:\r\n\r\n");

    printf("TM Item\t\t\tRaw Value\tActual value:\r\n");
    printf("*********************************************************\r\n");
    printf("DopplerOffset:\t\t%u\t\t%.4f Hz\n"
           "RSSI:\t\t\t%u\t\t%.4f dBm\n"
           "BusVoltage:\t\t%u\t\t%.4f V\n"
           "TotalCurrent:\t\t%u\t\t%.4f mA\n"
           "AmplifierTemp:\t\t%u\t\t%.4f C\n"
           "OscillatorTemp:\t\t%u\t\t%.4f C\n",
           hk->ru_curt.DopplerOffset, VU_SDO_Hz(hk->ru_curt.DopplerOffset),
           hk->ru_curt.RSSI, VU_RSS_dBm(hk->ru_curt.RSSI),
           hk->ru_curt.BusVoltage, VU_PBV_V(hk->ru_curt.BusVoltage),
           hk->ru_curt.TotalCurrent, VU_TCC_mA(hk->ru_curt.TotalCurrent),
           hk->ru_curt.AmplifierTemp, VU_PAT_C(hk->ru_curt.AmplifierTemp),
           hk->ru_curt.OscillatorTemp, VU_LOT_C(hk->ru_curt.OscillatorTemp)
              );
    printf("\n\n");
    printf("TM Item\t\t\tRaw Value\tActual value:\r\n");
    printf("********************************************************\r\n");
    printf("DopplerOffset:\t\t%u\t\t%.4f Hz\n"
           "RSSI:\t\t\t%u\t\t%.4f dBm\n",
           hk->ru_last.DopplerOffset, VU_SDO_Hz(hk->ru_last.DopplerOffset),
           hk->ru_last.RSSI, VU_RSS_dBm(hk->ru_last.RSSI)
          );
    printf("\n\n");
    printf("\n\n");
    printf("Transmitter MCU has been active: %u s\n\n",  hk->tu_uptime);
    printf("\n\n");
    printf("TX Measured current TM:\r\n");
    printf("TM Item\t\t\tRaw Value\tActual value:\r\n");
    printf("*******************************************************\r\n");
    printf("ReflectedPower:\t\t%u\t\t%.4f dBm(%.4f mW)\n"
            "ForwardPower:\t\t%u\t\t%.4f dBm(%.4f mW)\n"
            "BusVoltage:\t\t%u\t\t%.4f V\n"
            "TotalCurrent:\t\t%u\t\t%.4f mA\n"
            "AmplifierTemp:\t\t%u\t\t%.4f C\n"
            "OscillatorTemp:\t\t%u\t\t%.4f C\n",
            hk->tu_curt.ReflectedPower,VU_RRP_dBm(hk->tu_curt.ReflectedPower),VU_RRP_mW(hk->tu_curt.ReflectedPower),
            hk->tu_curt.ForwardPower,VU_RFP_dBm(hk->tu_curt.ForwardPower),VU_RFP_mW(hk->tu_curt.ForwardPower),
            hk->tu_curt.BusVoltage,VU_PBV_V(hk->tu_curt.BusVoltage),
            hk->tu_curt.TotalCurrent,VU_TCC_mA(hk->tu_curt.TotalCurrent),
            hk->tu_curt.AmplifierTemp,VU_PAT_C(hk->tu_curt.AmplifierTemp),
            hk->tu_curt.OscillatorTemp,VU_LOT_C(hk->tu_curt.OscillatorTemp)
           );

    printf("\n\n");
    printf("TX Stored last TM:\r\n\r\n");
    printf("TM Item\t\t\tRaw Value\tActual value:\r\n");
    printf("********************************************************\r\n");
    printf("ReflectedPower:\t\t%u\t\t%.4f dBm(%.4f mW)\n"
            "ForwardPower:\t\t%u\t\t%.4f dBm(%.4f mW)\n"
            "BusVoltage:\t\t%u\t\t%.4f V\n"
            "TotalCurrent:\t\t%u\t\t%.4f mA\n"
            "AmplifierTemp:\t\t%u\t\t%.4f C\n"
            "OscillatorTemp:\t\t%u\t\t%.4f C\n",
            hk->tu_last.ReflectedPower,VU_RRP_dBm( hk->tu_last.ReflectedPower),VU_RRP_mW( hk->tu_last.ReflectedPower),
            hk->tu_last.ForwardPower,VU_RFP_dBm( hk->tu_last.ForwardPower),VU_RFP_mW( hk->tu_last.ForwardPower),
            hk->tu_last.BusVoltage,VU_PBV_V( hk->tu_last.BusVoltage),
            hk->tu_last.TotalCurrent,VU_TCC_mA( hk->tu_last.TotalCurrent),
            hk->tu_last.AmplifierTemp,VU_PAT_C( hk->tu_last.AmplifierTemp),
            hk->tu_last.OscillatorTemp,VU_LOT_C( hk->tu_last.OscillatorTemp)
           );
    printf("\n\n");
    printf("********************************************************\r\n");
    printf("Transmitter idle state: %u\r\n", hk->tx_state.IdleState);
    printf("Beacon active:          %u\r\n", hk->tx_state.BeaconAct);
    printf("Transmitter bit rate:   %u\r\n", hk->tx_state.BitRate);

    return CMD_ERROR_NONE;
}

struct command cmd_isis_sub[] = {
	{
		.name = "send",
		.help = "Send frame with default AX2.5 callsigns",
		.usage = "<frame_len><num_of_frame><interval_ms>",
		.handler = isis_send_handler,
	},{
		.name = "state",
		.help = "Transmitter state",
		.handler = isis_transmitter_status_handler,
	},{
		.name = "t_uptime",
		.help = "Transmitter MCU has been active since the last reset",
		.handler = isis_tx_uptime_handler,
	},{
		.name = "rate",
		.help = "Set rate of transmitter:0--1200 1--2400 2--4800 3--9600",
		.usage = "<rate>",
		.handler = isis_set_rate_handler,
	},{
		.name = "t_tm",
		.help = "Show measured current TM of transmitter",
		.handler = isis_measure_tx_status_handler,
	},{
		.name = "last",
		.help = "Show stored last TM of transmitter",
		.handler = isis_last_status_handler,
	 },{
		.name = "idle",
		.help = "Set idle state of transmitter:0-turn off 1-remain on",
		.usage = "<opt:0-turn off 1-remain on>",
		.handler = isis_sleep_status_handler,
	},{
		.name = "signs",
		.help = "Set default callsigns of transmitter",
		.handler = isis_set_default_callsigns_handler,
	},{
		.name = "clear",
		.help = "Clear beacon of transmitter",
		.handler = isis_clearben_handler,
	},{
		.name = "c_beacon",
		.help = "Set beacon with new callsigns",
		.usage = "<beacon interval>",
		.handler = isis_set_call_beacon_handler,
	},{
		.name = "beacon",
		.help = "Set beacon with default callsigns",
		.usage = "<beacon interval>",
		.handler = isis_set_beacon_handler,
	},{
		.name = "c_frame",
		.help = "Send frame with new callsigns",
		.handler = isis_sendAXdate_handler,
	},{
		.name = "r_uptime",
		.help = "Receiver MCU has been active since the last reset",
		.handler = isis_rx_uptime_handler,
	},{
		.name = "remove",
		.help = "Remove the oldest frame from receiver buffer",
		.handler = isis_clearbutter_handler,
	},{
		.name = "count",
		.help = "Get receiver buffer frame number",
		.handler = isis_read_count_handler,
	},{
		.name = "receive",
		.help = "Get frame from receiver buffer",
		.handler = isis_read_frame_handler,
	},{
		.name = "watchdog",
		.help = "Watch dog reset: 0-Receiver !0-Transmitter",
		.usage = "<0-Receiver !0-Transmitter>",
		.handler = isis_watchdog_resetdog_handler,
	},{
		.name = "software",
		.help = "Software reset: 0-Receiver !0-Transmitter",
        .usage = "<0-Receiver !0-Transmitter>",
		.handler = isis_software_reset_handler,
	},{
		.name = "hardware",
		.help = "Hardware reset: 0-Receiver !0-Transmitter",
        .usage = "<0-Receiver !0-Transmitter>",
		.handler = isis_hardware_reset_handler,
	},{
		.name = "r_tm",
		.help = "Show measured telemetry of receiver",
		.handler = isis_measure_of_reciever_handler,
	},{
        .name = "hk",
        .help = "Show all hk of trxvu",
        .handler = isis_all_tm_handler,
    }
};

command_t __root_command cmd_isis_master[] = {
	{
		.name = "isis",
		.help = "isis test",
		.chain = INIT_CHAIN(cmd_isis_sub),
	}
};

void cmd_isis_setup(void) {
	command_register(cmd_isis_master);
}

