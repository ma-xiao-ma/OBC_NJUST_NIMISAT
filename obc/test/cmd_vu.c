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

#include "if_jlgvu.h"
#include "if_trxvu.h"
#include "bsp_pca9665.h"
#include "QB50_mem.h"
#include "error.h"



/*测试发射速率*/
int vu_send_handler(struct command_context * context) {

    static uint32_t num_of_frame = 0;
    static uint16_t interval_ms = 0;
    int ret, Error;

    char * args = command_args(context);

    if (sscanf(args, "%u %u", &num_of_frame, &interval_ms) != 2)
        return CMD_ERROR_SYNTAX;


    uint8_t *frame = qb50Malloc(ISIS_MTU);
    if(frame == NULL)
        return CMD_ERROR_SYNTAX;

    for(int i=0; i<ISIS_MTU; i++)
        frame[i] = i+1;

    uint8_t rest_of_frame;

    printf("TRXVU send %u frame, interval %u ms.\r\n", num_of_frame, interval_ms);

    do {

        ret = vu_send_frame(frame, ISIS_MTU, &rest_of_frame);

        if (ret != E_NO_ERR || rest_of_frame == 0xFF)
            Error++;
        else
        {
            num_of_frame--;
            /**若发射机缓冲区已满，则等待5秒钟*/
            if(rest_of_frame == 0)
                vTaskDelay(50 / portTICK_PERIOD_MS);
        }

        if(interval_ms)
            vTaskDelay(interval_ms);

    } while((num_of_frame > 0) && (Error < 5));

    printf("Slots %u, Remain %u, Error %u\r\n", rest_of_frame, num_of_frame, Error);

    qb50Free(frame);
    return CMD_ERROR_NONE;
}

int vu_transmitter_status_handler(struct command_context * context __attribute__((unused))) {

    rsp_transmitter_state state = {0};

    if (vu_get_state(&state) == E_NO_ERR)
    {
        printf("Transmitter bit rate:   %u\r\n", state.FM_On);
        printf("Transmitter bit rate:   %u\r\n", state.BitRate);
        printf("Beacon active:          %u\r\n", state.BeaconAct);
        printf("Transmitter idle state: %u\r\n", state.IdleState);
    }

    return CMD_ERROR_NONE;
}

int vu_read_count_handler(struct command_context * context __attribute__((unused))) {

    uint16_t rest_num = 0;

    if(vu_get_frame_num(&rest_num) == E_NO_ERR)
        printf("Number of frame in receive buffer: %u\n", rest_num);

    return CMD_ERROR_NONE;
}

int vu_tx_uptime_handler(struct command_context * context __attribute__((unused))) {


    uint32_t TX_time = 0;

    if(vu_get_uptime(&TX_time) == E_NO_ERR)
        printf("Transmitter MCU has been active: %u s\n\n", TX_time);

    return CMD_ERROR_NONE;
}

int vu_recetime_handler(struct command_context * context __attribute__((unused))) {

    uint32_t RX_time = 0;

    if(vu_get_uptime(&RX_time) == E_NO_ERR)
        printf("Receiver MCU has been active: %u s\n\n", RX_time);

    return CMD_ERROR_NONE;
}


int vu_rate_handler(struct command_context * context __attribute__((unused))) {

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
        printf("set ok \r\n");

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
        printf("set ok \r\n");

    return CMD_ERROR_NONE;
}

int vu_measure_tx_status_handler(struct command_context * context __attribute__((unused))) {

    rsp_vu_tm status = {0};

    if(vu_measure_all_tm(&status) == E_NO_ERR)
    {
        printf("Measure all the telemetry channels:\r\n");
        printf("ReflectedPower: %u, ForwardPower: %u, DopplerOffset: %u, RSSI: %u, BusVoltage: %u\r\n"
                "TotalCurrent: %u, AmplifierTemp: %u, OscillatorTemp: %u\r\n",
                status.ReflectedPower, status.ForwardPower, status.DopplerOffset, status.RSSI, status.BusVoltage, status.TotalCurrent,
                status.AmplifierTemp, status.OscillatorTemp);
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

    if (vu_transmitter_set_callsigns(&callsign) == E_NO_ERR)
        printf("set ok \r\n");

    return CMD_ERROR_NONE;
}


int vu_clearben_handler(struct command_context * context __attribute__((unused))) {

    if (vu_transmitter_clear_beacon() == E_NO_ERR)
        printf("set ok \r\n");

    return CMD_ERROR_NONE;
}

int vu_setAX_handler(struct command_context * context __attribute__((unused))) {

    uint16_t Interval;

    char * args = command_args(context);
    if (sscanf(args, "%u" , &Interval) != 1)
        return CMD_ERROR_SYNTAX;

    par_beacon_new_call_set *frame = qb50Malloc(sizeof(par_beacon_new_call_set)+ISIS_MTU);

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
        printf("set ok \r\n");

    qb50Free(frame);
    return CMD_ERROR_NONE;
}

int vu_setbeacon_handler(struct command_context * context __attribute__((unused))) {

    uint16_t Interval;

    char * args = command_args(context);
    if (sscanf(args, "%u" , &Interval) != 1)
        return CMD_ERROR_SYNTAX;

    par_beacon_set *beacon = qb50Malloc(sizeof(par_beacon_set)+ISIS_MTU);

    beacon->RepeatInterval = Interval;

    for(int i=0; i<ISIS_MTU; i++)
        beacon->BeaconContent[i] = i+1;

    if (vu_transmitter_beacon_set(beacon, ISIS_MTU) == E_NO_ERR)
        printf("set ok \r\n");

    qb50Free(beacon);
    return CMD_ERROR_NONE;
}

int vu_sendAXdate_handler(struct command_context * context __attribute__((unused))) {

    par_frame_new_call *frame = qb50Malloc(sizeof(par_frame_new_call)+ISIS_MTU);

    char *str1 = "BI4ST-0";
    for(int i=0; *(str1+i) != '\0'; i++)
            *(frame->DstCall + i) = *(str1 + i);

    char *str2 = "NJUST-4";
    for(int i=0; *(str2+i) != '\0'; i++)
            *(frame->SrcCall + i) = *(str2 + i);

    for(int i=0; i<ISIS_MTU; i++)
        frame->FrameContent[i] = i+1;

    uint8_t rsp;
    if (vu_transmitter_send_frame_new_callsigns(frame, ISIS_MTU, &rsp) == E_NO_ERR)
    {
        printf("frame send success!\r\n");
        printf("rsp = %u\r\n", rsp);
    }

    qb50Free(frame);
    return CMD_ERROR_NONE;
}

int vu_clearbutter_handler(struct command_context * context __attribute__((unused))) {

    if(vu_receiver_remove_frame() == E_NO_ERR)
        printf("set ok \r\n");

    return CMD_ERROR_NONE;
}

int vu_read_frame_handler(struct command_context * context __attribute__((unused))) {

    rsp_frame *frame = qb50Malloc(sizeof(rsp_frame)+ISIS_RX_MTU);

    if( vu_receiver_get_frame(frame, ISIS_RX_MTU) == E_NO_ERR)
    {
        printf("DopplerOffset: %u, RSSI: %u\r\n", frame->DopplerOffset, frame->RSSI);
        for(uint16_t i=0; i<frame->DateSize; i++)
            printf("0x%02x ", frame->Data[i]);
        printf("\r\n");
    }

    qb50Free(frame);

    return CMD_ERROR_NONE;
}


int vu_resetdog_handler(struct command_context * context __attribute__((unused))) {

    if(vu_receiver_watchdog_reset() == E_NO_ERR)
        printf("Receiver watch dog reset!!!\r\n");

    return CMD_ERROR_NONE;
}

int vu_softwarereset_handler(struct command_context * context __attribute__((unused))) {

    if(vu_receiver_software_reset() == E_NO_ERR)
        printf("Receiver software reset!!!\r\n");

    return CMD_ERROR_NONE;
}

int vu_hardwarereset_handler(struct command_context * context __attribute__((unused))) {

    if(vu_receiver_hardware_reset() == E_NO_ERR)
        printf("Receiver hardware reset!!!\r\n");

    return CMD_ERROR_NONE;
}

int vu_measureofreciever_handler(struct command_context * context __attribute__((unused))) {

    rsp_rx_tm rx_tm = {0};

    if(vu_receiver_measure_tm(&rx_tm) == E_NO_ERR)
    {
        printf("Receiver measure all the telemetry channels:\r\n");
        printf("DopplerOffset: %u, RSSI: %u, BusVoltage: %u, TotalCurrent: %u,"
                "  AmplifierTemp: %u, OscillatorTemp: %u\r\n",
                rx_tm.DopplerOffset, rx_tm.RSSI, rx_tm.BusVoltage, rx_tm.TotalCurrent,
                rx_tm.AmplifierTemp, rx_tm.OscillatorTemp);
    }
    return CMD_ERROR_NONE;
}

struct command cmd_vu_sub[] = {
    {
        .name = "send",
        .help = "Send frame with default AX2.5 callsigns",
        .usage = "<num_of_frame><interval_ms>",
        .handler = vu_send_handler,
    },{
        .name = "status",
        .help = "The state of tansmmiter",
        .handler = vu_transmitter_status_handler,
    },{
        .name = "uptime_tx",
        .help = "Transmitter MCU has been active since the last reset",
        .handler = vu_tx_uptime_handler,
    },{
        .name = "rate",
        .help = "Set rate of transmitter:0--1200 1--2400 2--4800 3--9600",
        .usage = "<rate>",
        .handler = vu_rate_handler,
    },{
        .name = "telemetry_tx",
        .help = "Show measure telemetry of transmitter",
        .handler = vu_measure_tx_status_handler,
    },{
        .name = "idle_state",
        .help = "Set idle state of transmitter",
        .usage = "<date 0-turn of 1-remain on>",
        .handler = vu_sleep_status_handler,
    },{
        .name = "callsigns",
        .help = "Set default callsigns of transmitter",
        .handler = vu_set_default_callsigns_handler,
    },{
        .name = "clearbea",
        .help = "Clear beacon of transmitter",
        .handler = vu_clearben_handler,
    },{
        .name = "beacon_with_call",
        .help = "Set beacon with new callsigns",
        .usage = "<beacon interval>",
        .handler = vu_setAX_handler,
    },{
        .name = "setbeacon",
        .help = "Set beacon with default callsigns",
        .usage = "<beacon interval>",
        .handler = vu_setbeacon_handler,
    },{
        .name = "sendAX",
        .help = "Send frame with new callsigns",
        .handler = vu_sendAXdate_handler,
    },{
        .name = "uptime_rx",
        .help = "Receiver MCU has been active since the last reset",
        .handler = vu_recetime_handler,
    },{
        .name = "remove_frame",
        .help = "Remove the oldest frame",
        .handler = vu_clearbutter_handler,
    },{
        .name = "rcount",
        .help = "vu_read_countof rece frame",
        .handler = vu_read_count_handler,
    },{
        .name = "rframe",
        .help = "Get receiver buffer frame number",
        .handler = vu_read_frame_handler,
    },{
        .name = "watchdogreset",
        .help = "Receiver watch dog reset",
        .handler = vu_resetdog_handler,
    },{
        .name = "softwarereset",
        .help = "Receiver software reset",
        .handler = vu_softwarereset_handler,
    },{
        .name = "hardreset",
        .help = "Receiver software reset",
        .handler = vu_hardwarereset_handler,
    },{
        .name = "telemetry_rx",
        .help = "Show measure telemetry of transmitter",
        .handler = vu_measureofreciever_handler,
    }
};

command_t __root_command cmd_vu_master[] = {
    {
        .name = "vu",
        .help = "JLG VU test",
        .chain = INIT_CHAIN(cmd_vu_sub),
    }
};

void cmd_vu_setup(void) {
    command_register(cmd_vu_master);
}
