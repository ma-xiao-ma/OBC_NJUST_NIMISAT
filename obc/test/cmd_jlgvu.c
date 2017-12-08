/*
 * cmd_vu.c
 *
 *  Created on: 2017年10月12日
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
#include "if_trxvu.h"
#include "bsp_pca9665.h"
#include "obc_mem.h"
#include "error.h"
#include "hexdump.h"
#include "csp_endian.h"

#include "if_jlgvu.h"


/*测试发射速率*/
int vu_send_handler(struct command_context * context)
{

    static uint32_t num_of_frame = 0;
    static uint16_t interval_ms = 0;
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

    for(int i=0; i<frame_len/*ISIS_MTU*/; i++)
        frame[i] = 0x55;

    uint16_t rest_of_buffer_byte;

    printf("VU Framelen %u byte, Send %u frame, Interval %u ms.\r\n", frame_len, num_of_frame, interval_ms);
    printf("...\n");

    *(uint32_t *)frame = csp_htobe32((uint32_t)0x12345678);
    do {

        *(uint32_t *)(frame + 4)= csp_htobe32(j);

        ret = vu_send_frame(frame, frame_len/*ISIS_MTU*/, &rest_of_buffer_byte);

        if (ret != E_NO_ERR /*|| rest_of_buffer_byte == 0xFF*/)
            errors++;
        else
        {
            j++;
            num_of_frame--;
            /**若发射机缓冲区已满，则等待50毫秒*/
            if(/*rest_of_buffer_byte == 0*/512 > rest_of_buffer_byte)
            {
                times++;
                vTaskDelay(7000 / portTICK_PERIOD_MS);
            }
        }

        if(interval_ms)
            vTaskDelay(interval_ms);

    } while((num_of_frame > 0) && (errors < 5));

    printf("Slots %u byte, Remain %u frame, Error %u, Delay %u\r\n", rest_of_buffer_byte, num_of_frame, errors, times);

    ObcMemFree(frame);
    return CMD_ERROR_NONE;
}

int vu_transmitter_status_handler(struct command_context * context __attribute__((unused)))
{

    rsp_transmitter_state state = {0};

    if (vu_get_state(&state) == E_NO_ERR)
    {
        printf("FM forwarding on:       %u\r\n", state.FM_On);
        printf("Transmitter bit rate:   %u\r\n", state.BitRate);
        printf("Beacon active:          %u\r\n", state.BeaconAct);
        printf("Transmitter idle state: %u\r\n", state.IdleState);
    }

    return CMD_ERROR_NONE;
}

int vu_read_count_handler(struct command_context * context __attribute__((unused)))
{
    uint16_t rest_num = 0;

    if(vu_get_frame_num(&rest_num) == E_NO_ERR)
        printf("Number of frame in receiver buffer: %u\n", rest_num);

    return CMD_ERROR_NONE;
}

int vu_uptime_handler(struct command_context * context __attribute__((unused)))
{


    uint32_t TX_time = 0;

    if(vu_get_uptime(&TX_time) == E_NO_ERR)
        printf("MCU has been active: %u s\n\n", TX_time);

    return CMD_ERROR_NONE;
}

int vu_rate_handler(struct command_context * context __attribute__((unused)))
{

    uint32_t rate = 0;
    par_transmission_bitrate para;

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

    if(vu_set_bitrate(para) == E_NO_ERR)
        printf("Set OK!!!\r\n");

    return CMD_ERROR_NONE;
}

int vu_sleep_status_handler(struct command_context * context __attribute__((unused))) {

    uint32_t sleep_status = 0;
    par_idle_state IdleState;

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
    if(vu_set_idle_state(IdleState) == E_NO_ERR)
        printf("Set OK!!!\r\n");

    return CMD_ERROR_NONE;
}

int vu_measure_all_status_handler(struct command_context * context __attribute__((unused))) {

    rsp_vu_tm status = {0};

    if(vu_measure_all_tm(&status) == E_NO_ERR)
    {
        printf("TM Item\t\t\tRaw Value\tActual value:\r\n");
        printf("*******************************************************\r\n");
        printf("ReflectedPower:\t\t%u\t\t%.4f dBm(%.4f mW)\n"
                "ForwardPower:\t\t%u\t\t%.4f dBm(%.4f mW)\n"
                "DopplerOffset:\t\t%u\t\t%.4f Hz\n"
                "RSSI:\t\t\t%u\t\t%.4f dBm\n"
                "BusVoltage:\t\t%u\t\t%.4f V\n"
                "TotalCurrent:\t\t%u\t\t%.4f mA\n"
                "AmplifierTemp:\t\t%u\t\t%.4f C\n"
                "OscillatorTemp:\t\t%u\t\t%.4f C\n",
                status.ReflectedPower, VU_RRP_dBm(status.ReflectedPower), VU_RRP_mW(status.ReflectedPower),
                status.ForwardPower, VU_RFP_dBm(status.ForwardPower), VU_RFP_mW(status.ForwardPower),
                status.DopplerOffset, VU_SDO_Hz(status.DopplerOffset),
                status.RSSI, VU_RSS_dBm(status.RSSI),
                status.BusVoltage, VU_PBV_V(status.BusVoltage),
                status.TotalCurrent, VU_TCC_mA(status.TotalCurrent),
                status.AmplifierTemp, VU_PAT_C(status.AmplifierTemp),
                status.OscillatorTemp, VU_LOT_C(status.OscillatorTemp)
               );
    }

    return CMD_ERROR_NONE;
}


int vu_set_default_callsigns_handler(struct command_context * context __attribute__((unused))) {

    par_default_call_set callsign = {0};

    char *str1 = "BI4ST-0";
    for(int i=0; *(str1+i) != '\0'; i++)
        *(callsign.DstCall + i) = *(str1 + i);

    char *str2 = "NJUST-4";
    for(int i=0; *(str2+i) != '\0'; i++)
        *(callsign.SrcCall + i) = *(str2 + i);

    if (vu_set_callsigns(&callsign) == E_NO_ERR)
        printf("Set OK!!!\r\n");

    return CMD_ERROR_NONE;
}


int vu_clear_bencon_handler(struct command_context * context __attribute__((unused))) {

    if (vu_clear_beacon() == E_NO_ERR)
        printf("Set OK!!!\r\n");

    return CMD_ERROR_NONE;
}

int vu_set_call_bencon_handler(struct command_context * context __attribute__((unused))) {

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

    if(vu_beacon_new_call_set(frame, ISIS_MTU) == E_NO_ERR)
        printf("Set OK!!!\r\n");

    ObcMemFree(frame);
    return CMD_ERROR_NONE;
}

int vu_set_beacon_handler(struct command_context * context __attribute__((unused))) {

    uint16_t Interval;

    char * args = command_args(context);
    if (sscanf(args, "%u" , &Interval) != 1)
        return CMD_ERROR_SYNTAX;

    par_beacon_set *beacon = ObcMemMalloc(sizeof(par_beacon_set)+ISIS_MTU);

    beacon->RepeatInterval = Interval;

    for(int i=0; i<ISIS_MTU; i++)
        beacon->BeaconContent[i] = i+1;

    if (vu_beacon_set(beacon, ISIS_MTU) == E_NO_ERR)
        printf("Set OK!!!\r\n");

    ObcMemFree(beacon);
    return CMD_ERROR_NONE;
}

int vu_send_new_call_date_handler(struct command_context * context __attribute__((unused))) {

    par_frame_new_call *frame = ObcMemMalloc(sizeof(par_frame_new_call)+ISIS_MTU);

    char *str1 = "BI4ST-0";
    for(int i=0; *(str1+i) != '\0'; i++)
            *(frame->DstCall + i) = *(str1 + i);

    char *str2 = "NJUST-4";
    for(int i=0; *(str2+i) != '\0'; i++)
            *(frame->SrcCall + i) = *(str2 + i);

    for(int i=0; i<ISIS_MTU; i++)
        frame->FrameContent[i] = i+1;

    uint8_t rsp;
    if (vu_send_frame_new_callsigns(frame, ISIS_MTU, &rsp) == E_NO_ERR)
    {
        printf("Frame send success!\r\n");
        printf("rsp = %u\r\n", rsp);
    }

    ObcMemFree(frame);
    return CMD_ERROR_NONE;
}

int vu_remove_frame_handler(struct command_context * context __attribute__((unused))) {

    if(vu_remove_frame() == E_NO_ERR)
        printf("Set OK!!!\r\n");

    return CMD_ERROR_NONE;
}

int vu_read_frame_handler(struct command_context * context __attribute__((unused))) {

    rsp_frame *frame = ObcMemMalloc(sizeof(rsp_frame)+/*ISIS_RX_MTU*/200);

    if( vu_get_frame(frame, /*ISIS_RX_MTU*/200) == E_NO_ERR)
    {
        printf("TM Item\t\t\tRaw Value\tActual value:\r\n");
        printf("********************************************************\r\n");
        printf("DopplerOffset:\t\t%u\t\t%.4f Hz\n"
               "RSSI:\t\t\t%u\t\t%.4f dBm\n",
                frame->DopplerOffset, VU_SDO_Hz(frame->DopplerOffset),
                frame->RSSI, VU_RSS_dBm(frame->RSSI)
              );
        printf("\n\n");

        printf("Len: %u bytes.", frame->DateSize);
        printf("\n\n");

        if (frame->DateSize /*!= ISIS_RX_MTU*/ > 200 || frame->DateSize == 0)
        {
            printf("ERROR: Rx length!!!");
            return CMD_ERROR_FAIL;
        }
        else
        {
            hex_dump( frame->Data, frame->DateSize );
//        for(uint32_t i=0; i<frame->DateSize; i++)
//            printf("0x%02x ", frame->Data[i]);
//        printf("\r\n");
        }
    }

    ObcMemFree(frame);

    return CMD_ERROR_NONE;
}


int vu_resetdog_handler(struct command_context * context __attribute__((unused))) {

    if(vu_watchdog_reset() == E_NO_ERR)
        printf("Watch dog reset!!!\r\n");

    return CMD_ERROR_NONE;
}

int vu_softwarereset_handler(struct command_context * context __attribute__((unused))) {

    if(vu_software_reset() == E_NO_ERR)
        printf("Software reset!!!\r\n");

    return CMD_ERROR_NONE;
}

int vu_fm_on_handler(struct command_context * context __attribute__((unused))) {

    if(vu_fm_on() == E_NO_ERR)
        printf("fm on!!!\r\n");

    return CMD_ERROR_NONE;
}

int vu_fm_off_handler(struct command_context * context __attribute__((unused))) {

    if(vu_fm_off() == E_NO_ERR)
        printf("fm off!!!\r\n");

    return CMD_ERROR_NONE;
}

int vu_power_on_handler(struct command_context * context __attribute__((unused))) {

    if(vu_power_on() == E_NO_ERR)
        printf("power on!!!\r\n");

    return CMD_ERROR_NONE;
}

int vu_power_off_handler(struct command_context * context __attribute__((unused))) {

    if(vu_power_off() == E_NO_ERR)
        printf("power off!!!\r\n");

    return CMD_ERROR_NONE;
}

struct command cmd_vu_sub[] = {
    {
        .name = "send",
        .help = "Send frame with default AX2.5 callsigns",
        .usage = "<data_len><num_of_frame><interval_ms>",
        .handler = vu_send_handler,
    },{
        .name = "status",
        .help = "The state of tansmmiter",
        .handler = vu_transmitter_status_handler,
    },{
        .name = "uptime",
        .help = "MCU has been active since the last reset",
        .handler = vu_uptime_handler,
    },{
        .name = "rate",
        .help = "Set rate of transmitter:0--1200 1--2400 2--4800 3--9600",
        .usage = "<rate>",
        .handler = vu_rate_handler,
    },{
        .name = "tm",
        .help = "Show all measure telemetry",
        .handler = vu_measure_all_status_handler,
    },{
        .name = "idle",
        .help = "Set idle state of transmitter:date 0-turn of 1-remain on",
        .usage = "<date 0-turn of 1-remain on>",
        .handler = vu_sleep_status_handler,
    },{
        .name = "callsigns",
        .help = "Set default callsigns of transmitter",
        .handler = vu_set_default_callsigns_handler,
    },{
        .name = "clear",
        .help = "Clear active beacon",
        .handler = vu_clear_bencon_handler,
    },{
        .name = "callbeacon",
        .help = "Set beacon with new callsigns",
        .usage = "<beacon interval>",
        .handler = vu_set_call_bencon_handler,
    },{
        .name = "beacon",
        .help = "Set beacon with default callsigns",
        .usage = "<beacon interval>",
        .handler = vu_set_beacon_handler,
    },{
        .name = "sendAX",
        .help = "Send frame with new callsigns",
        .handler = vu_send_new_call_date_handler,
    },{
        .name = "remove",
        .help = "Remove the oldest frame",
        .handler = vu_remove_frame_handler,
    },{
        .name = "count",
        .help = "Get number of frame in receiver buffer",
        .handler = vu_read_count_handler,
    },{
        .name = "frame",
        .help = "Get receiver buffer frame",
        .handler = vu_read_frame_handler,
    },{
        .name = "watchdog",
        .help = "Watch dog reset",
        .handler = vu_resetdog_handler,
    },{
        .name = "software",
        .help = "Software reset",
        .handler = vu_softwarereset_handler,
    },{
        .name = "fm_on",
        .help = "fm on",
        .handler = vu_fm_on_handler,
    },{
        .name = "fm_off",
        .help = "fm off",
        .handler = vu_fm_off_handler,
    },{
        .name = "power_on",
        .help = "receiver transmitter on",
        .handler = vu_power_on_handler,
    },{
        .name = "power_off",
        .help = "receiver transmitter off",
        .handler = vu_power_off_handler,
    }
};

command_t __root_command cmd_vu_master[] = {
    {
        .name = "vu",
        .help = "JLG VU test",
        .chain = INIT_CHAIN(cmd_vu_sub),
    }
};

void cmd_vu_setup(void)
{
    command_register(cmd_vu_master);
}
