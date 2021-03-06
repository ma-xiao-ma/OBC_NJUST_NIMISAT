/*
 * if_downlink_vu.c
 *
 *  Created on: 2017年10月5日
 *      Author: Ma Wenli
 */
#include <string.h>

#include "bsp_pca9665.h"
#include "router_io.h"
#include "error.h"
#include "crc.h"
#include "semphr.h"
#include "cube_com.h"
#include "obc_mem.h"
#include "if_jlgvu.h"
#include "hexdump.h"
#include "bsp_ds1302.h"
#include "driver_debug.h"
#include "ff.h"
#include "bsp_switch.h"
#include "task_monitor.h"
#include "switches.h"

#include "if_downlink_vu.h"

typedef struct __attribute__((__packed__))
{
	uint32_t file_size;		 //文件总字节数
    uint16_t total_packet;   //文件包总数
    uint8_t packet_size;     //数据包大小
    uint32_t time;
	uint8_t filename[20];
} file_info_down_t;

/*接收单元上次接收到上行数据时遥测存储队列*/
QueueHandle_t rx_tm_queue;

uint16_t vu_isis_rx_count; //ISISvu 通信机接收上行遥控帧计数
uint16_t vu_jlg_rx_count;  //JLGvu 通信机接收上行遥控帧计数

extern uint8_t IsJLGvuWorking;
extern bool PassFlag;

/**
 * 通用路由协议下行数据接口
 *
 * @param dst 目的地址
 * @param src 源地址
 * @param type 消息类型
 * @param pdata 待发送数据指针
 * @param len 待发送数据长度
 */
int ProtocolSendDownCmd( uint8_t dst, uint8_t src, uint8_t type, void *pdata, uint32_t len )
{

#if USE_SERIAL_PORT_DOWNLINK_INTERFACE
    return ProtocolSerialSend( dst, src, type, pdata, len );
#else
    if (IsJLGvuWorking) /*主备通信机选择*/
        return vu_jlg_send( dst, src, type, pdata, len );
    else
        return vu_isis_send( dst, src, type, pdata, len );
#endif

}

/**
 * obc地面指令响应函数
 *
 * @param type 响应指令类型
 * @param result 指令执行结果
 */
void obc_cmd_ack(uint8_t type, uint8_t result)
{

#if USE_SERIAL_PORT_DOWNLINK_INTERFACE
    ProtocolSerialSend( GND_ROUTE_ADDR, OBC_ROUTE_ADDR, type, &result, 1 );
#else
    if(IsJLGvuWorking)
    {
        vu_jlg_send( GND_ROUTE_ADDR, OBC_ROUTE_ADDR, type, &result, 1 );
        vu_jlg_send( GND_ROUTE_ADDR, OBC_ROUTE_ADDR, type, &result, 1 );
        vu_jlg_send( GND_ROUTE_ADDR, OBC_ROUTE_ADDR, type, &result, 1 );
    }
    else
    {
        vu_isis_send( GND_ROUTE_ADDR, OBC_ROUTE_ADDR, type, &result, 1 );
        vu_isis_send( GND_ROUTE_ADDR, OBC_ROUTE_ADDR, type, &result, 1 );
        vu_isis_send( GND_ROUTE_ADDR, OBC_ROUTE_ADDR, type, &result, 1 );
    }
#endif

}

/**
 * ISIS通信机下行接口函数，由OBC本地调用
 *
 * @param type 下行消息类型
 * @param pdata 下行数据指针
 * @param len 下行数据字节数
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_obc_downlink(uint8_t type, void *pdata, uint32_t len)
{
    int ret;
    uint8_t ErrorCounter = 0, FrameDataSize;
    uint32_t RemainSize = len;

    pdata = (uint8_t *)pdata;

    route_frame_t *downlink = ObcMemMalloc(I2C_MTU);
    if(downlink == NULL)
        return E_MALLOC_FAIL;

    downlink->dst = GND_ROUTE_ADDR;
    downlink->src = router_get_my_address();
    downlink->typ = type;

    do
    {
        FrameDataSize = (RemainSize < DOWNLINK_MTU) ? RemainSize : DOWNLINK_MTU;
        memcpy(downlink->dat, pdata, FrameDataSize);

//        *(uint32_t *)(&downlink->dat[FrameDataSize]) =
//                crc32_memory((uint8_t *)downlink, ROUTE_HEAD_SIZE+FrameDataSize);

        uint8_t TxBuf_Slot_Remain = 40; /* ISIS通信机发送缓冲区剩余空间  0--40 Slot */
        uint16_t TxBuf_Byte_Remain = 32768; /* 解理工通信机发送缓冲区剩余空间  0--32768 Byte */

        if (IsJLGvuWorking)
            ret = vu_send_frame(downlink, FrameDataSize + DOWNLINK_OVERHEAD, &TxBuf_Byte_Remain);
        else
            ret = vu_transmitter_send_frame(downlink, FrameDataSize + DOWNLINK_OVERHEAD, &TxBuf_Slot_Remain);

        /**如果传输成功，且通信机成功将消息加入发送缓冲区，发送指针后移 */
        if (ret == E_NO_ERR && TxBuf_Slot_Remain != 0xFF)
        {
            RemainSize -= FrameDataSize;
            pdata += FrameDataSize;

            /**若发射机缓冲区已满，则等待5秒钟*/
            if(TxBuf_Byte_Remain < 512 || TxBuf_Slot_Remain < 2)
                vTaskDelay(MS_WAIT_TRANS_FREE_BUFF);
        }
        else
            ErrorCounter++;

        if (RemainSize != 0)
            vTaskDelay(PACK_DOWN_INTERVAL);

     /**若发送完成或者错误次数超过5次，则跳出循环 */
    }while ((RemainSize > 0) && (ErrorCounter < 5));

    ObcMemFree(downlink);

    if (RemainSize == 0)
        return E_NO_ERR;
    else
    {   /**如果错误超过5次，则复位发射机*/
        if (IsJLGvuWorking)
            vu_software_reset();
        else
            vu_transmitter_software_reset();
        return E_TRANSMIT_ERROR;
    }
}

/**
 * 通过vu无线接口发送数据
 *
 * @param dst 目的地址
 * @param src 源地址
 * @param type 消息类型
 * @param pdata 待发送数据指针
 * @param len 待发送数据长度
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_isis_send( uint8_t dst, uint8_t src, uint8_t type, void *pdata, uint32_t len )
{
    int ret;
    uint8_t ErrorCounter = 0, FrameDataSize = 0;
    uint32_t RemainSize = len;

    pdata = (uint8_t *)pdata;

    route_frame_t *downlink = ObcMemMalloc(I2C_MTU);
    if(downlink == NULL)
        return E_MALLOC_FAIL;

    downlink->dst = dst;
    downlink->src = src;
    downlink->typ = type;

    do
    {
        FrameDataSize = (RemainSize < DOWNLINK_MTU) ? RemainSize : DOWNLINK_MTU;
        memcpy(downlink->dat, pdata, FrameDataSize);

        /** 添加CRC校验*/
//        *(uint32_t *)(&downlink->dat[FrameDataSize]) =
//                crc32_memory((uint8_t *)downlink, ROUTE_HEAD_SIZE+FrameDataSize);

        uint8_t TxBuf_Slot_Remain = 40; /* ISIS通信机发送缓冲区剩余空间  0--40 Slot */

        ret = vu_transmitter_send_frame(downlink, FrameDataSize + DOWNLINK_OVERHEAD, &TxBuf_Slot_Remain);

        /**如果传输成功，且通信机成功将消息加入发送缓冲区，发送指针后移 */
        if (ret == E_NO_ERR && TxBuf_Slot_Remain != 0xFF)
        {
            RemainSize -= FrameDataSize;
            pdata += FrameDataSize;

            /**若发射机缓冲区已满，则等待5秒钟*/
            if(TxBuf_Slot_Remain < 2)
                vTaskDelay(MS_WAIT_TRANS_FREE_BUFF);
        }
        else
            ErrorCounter++;


        vTaskDelay(PACK_DOWN_INTERVAL);

     /**若发送完成或者错误次数超过5次，则跳出循环 */
    }while (RemainSize > 0 && ErrorCounter < 5);

    ObcMemFree(downlink);

    if (RemainSize == 0)
        return E_NO_ERR;
    else
    {   /**如果错误超过5次，则复位发射机*/
        vu_transmitter_software_reset();
        return E_TRANSMIT_ERROR;
    }
}

/**
 * 文件下行，由OBC本地调用
 *
 * @param file 文件句柄指针
 * @param file_name 文件名指针
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_isis_file_download(FIL *file, char *file_name)
{

	file_info_down_t *file_info = (file_info_down_t *)ObcMemMalloc(sizeof(file_info_down_t));

	if( file_info == NULL )
	{
		printf("ERROR: Malloc fail!!\r\n");
		return E_MALLOC_FAIL;
	}

	file_info->file_size = f_size( file );

	file_info->packet_size = IMAGE_PACK_MAX_SIZE;

	file_info->total_packet = (file_info->file_size % file_info->packet_size) ?
			file_info->file_size/file_info->packet_size + 1 : file_info->file_size/file_info->packet_size;

	file_info->time = clock_get_time_nopara();

	memcpy( file_info->filename, file_name, strlen(file_name));

	ProtocolSendDownCmd(GND_ROUTE_ADDR, OBC_ROUTE_ADDR, FILE_INFO, file_info, sizeof(file_info_down_t));

    int ret;
    uint8_t ErrorCounter = 0;
    uint32_t RemainSize = file_info->file_size;

    UINT byte_read;

    route_frame_t *downlink = ObcMemMalloc(I2C_MTU);
    if(downlink == NULL)
    {
        ObcMemFree(file_info);
		printf("ERROR: Malloc fail!!\r\n");
		return E_MALLOC_FAIL;
    }

    /**组路由协议包*/
    downlink->dst = GND_ROUTE_ADDR;
    downlink->src = OBC_ROUTE_ADDR;
    downlink->typ = FILE_DATA;

    ImagePacket_t * packet = (ImagePacket_t *)downlink->dat;

    packet->PacketID = 0;
    do
    {
        /**组图像数据包*/
        packet->PacketSize = (RemainSize < file_info->packet_size) ? RemainSize : file_info->packet_size;

        f_lseek( file, packet->PacketID * file_info->packet_size);

        f_read( file, packet->ImageData, (UINT)packet->PacketSize, &byte_read);

        if( byte_read != packet->PacketSize)
        {
        	printf("ERROR: File read error!!\r\n");
			return E_INVALID_PARAM;
        }

        uint8_t TxBuf_Slot_Remain = 40; /* ISIS通信机发送缓冲区剩余空间  0--40 Slot */
        uint16_t TxBuf_Byte_Remain = 32768; /* 解理工通信机发送缓冲区剩余空间  0--32768 Byte */

        if (IsJLGvuWorking)
            ret = vu_send_frame(downlink, packet->PacketSize + IMAGE_DOWNLINK_OVERHEAD, &TxBuf_Byte_Remain);
        else
            ret = vu_transmitter_send_frame(downlink, packet->PacketSize + IMAGE_DOWNLINK_OVERHEAD, &TxBuf_Slot_Remain);

        /**如果传输成功，且通信机成功将消息加入发送缓冲区，发送指针后移 */
        if (ret == E_NO_ERR && TxBuf_Slot_Remain != 0xFF)
        {
            RemainSize -= packet->PacketSize;

            packet->PacketID += 1;

            /**若发射机缓冲区已满，则等待5秒钟*/
            if(TxBuf_Byte_Remain < 512 || TxBuf_Slot_Remain < 2)
                vTaskDelay(MS_WAIT_TRANS_FREE_BUFF);
        }
        else
            ErrorCounter++;

        if (RemainSize != 0)
            vTaskDelay(PACK_DOWN_INTERVAL);

     /**若发送完成或者错误次数超过五次，则跳出循环 */
    }while (RemainSize > 0 && ErrorCounter < 5);

    ObcMemFree(file_info);
    ObcMemFree(downlink);

    if (RemainSize == 0)
        return E_NO_ERR;
    else
    {   /**如果错误超过五次，则复位发射机*/
        vu_transmitter_software_reset();
        return E_TRANSMIT_ERROR;
    }
}

/**
 * 图像下行接口，由OBC本地调用
 *
 * @param image_data 图像数据指针
 * @param image_len 图像长度
 * @param start_pack 图像起始包号
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
static int vu_image_downlink(const void * image_data, uint32_t image_len, uint16_t start_pack)
{
    int ret;

    uint8_t ErrorCounter = 0; /*I2C传输错误计数 */
    uint32_t RemainSize = image_len; /*还剩多少字节需要传输 */
    image_data = (uint8_t *)image_data;

    route_frame_t *downlink = ObcMemMalloc(I2C_MTU);
    if(downlink == NULL)
        return E_MALLOC_FAIL;

    /**组路由协议包*/
    downlink->dst = GND_ROUTE_ADDR;
    downlink->src = CAM_ROUTE_ADDR;
    downlink->typ = CAM_IMAGE;

    ImagePacket_t * packet = (ImagePacket_t *)downlink->dat;

    packet->PacketID = start_pack;

    do
    {
        /** 组图像数据包*/
        packet->PacketSize = (RemainSize < IMAGE_PACK_MAX_SIZE) ? RemainSize : IMAGE_PACK_MAX_SIZE;
        memcpy(packet->ImageData, image_data, IMAGE_PACK_MAX_SIZE);

        /** 添加CRC校验*/
//        *(uint32_t *)(&downlink->dat[IMAGE_PACK_HEAD_SIZE + packet->PacketSize]) =
//                crc32_memory((uint8_t *)downlink, ROUTE_HEAD_SIZE + IMAGE_PACK_HEAD_SIZE + packet->PacketSize);

        uint8_t TxBuf_Slot_Remain = 40; /* ISIS通信机发送缓冲区剩余空间  0--40 Slot */
        uint16_t TxBuf_Byte_Remain = 32768; /* 解理工通信机发送缓冲区剩余空间  0--32768 Byte */

        if (IsJLGvuWorking)
            ret = vu_send_frame(downlink, packet->PacketSize + IMAGE_DOWNLINK_OVERHEAD, &TxBuf_Byte_Remain);
        else
            ret = vu_transmitter_send_frame(downlink, packet->PacketSize + IMAGE_DOWNLINK_OVERHEAD, &TxBuf_Slot_Remain);

        /**如果传输成功，且通信机成功将消息加入发送缓冲区，发送指针后移 */
        if (ret == E_NO_ERR && TxBuf_Slot_Remain != 0xFF)
        {
            RemainSize -= packet->PacketSize;
            image_data += packet->PacketSize;

            packet->PacketID += 1;

            /**若发射机缓冲区已满，则等待5秒钟*/
            if(TxBuf_Byte_Remain < 512 || TxBuf_Slot_Remain < 2)
                vTaskDelay(MS_WAIT_TRANS_FREE_BUFF);
        }
        else
            ErrorCounter++;

        if (RemainSize != 0)
            vTaskDelay(PACK_DOWN_INTERVAL);

     /**若发送完成或者错误次数超过五次，则跳出循环 */
    }while ((RemainSize > 0) && (ErrorCounter < 5));

    ObcMemFree(downlink);

    if (RemainSize == 0)
        return E_NO_ERR;
    else
    {   /**如果错误超过五次，则复位发射机*/
        vu_transmitter_software_reset();
        return E_TRANSMIT_ERROR;
    }
}


/**
 * ISIS通信机上行轮询任务
 *
 * @param para 任务参数，没有用到
 */
void vu_isis_uplink_task(void *para __attribute__((unused)))
{
    static uint16_t frame_num;

    if(rx_tm_queue == NULL)
            rx_tm_queue = xQueueCreate(1, sizeof( receiving_tm ));

    while(1)
    {
        task_report_alive(Isis);
        vTaskDelay(1234);

        /**获取接收机缓冲区帧计数*/
        if (vu_receiver_get_frame_num(&frame_num) != E_NO_ERR)
            continue;

        if (frame_num == 0)
            continue;

        rsp_frame *recv_frame = (rsp_frame *)ObcMemMalloc(sizeof(route_packet_t) + MAX_UPLINK_CONTENT_SIZE);
        if (recv_frame == NULL)
        {
            driver_debug(DEBUG_TTC, "ERROR: ISIS task malloc fail!!\r\n");
            continue;
        }

        if (vu_receiver_get_frame(recv_frame, MAX_UPLINK_CONTENT_SIZE) != E_NO_ERR)
        {
            driver_debug(DEBUG_TTC, "ERROR: ISIS task get frame fail!!\r\n");
            ObcMemFree(recv_frame);
            continue;
        }

        /**通信机接收上行消息计数加1*/
        vu_isis_rx_count++;

        /* 给多普勒和信号强度遥测变量赋值 */
        if(rx_tm_queue != NULL)
        {
            receiving_tm rx_tm =
            {
                rx_tm.DopplerOffset = recv_frame->DopplerOffset,
                rx_tm.RSSI = recv_frame->RSSI
            };

            xQueueOverwrite(rx_tm_queue, &rx_tm);
        }

        /* 若收到的帧数据长度字段不匹配，则视为错帧 */
        if (recv_frame->DateSize < ROUTE_HEAD_SIZE || recv_frame->DateSize > MAX_UPLINK_CONTENT_SIZE)
        {
            driver_debug(DEBUG_TTC, "WARNING: ISIS vu has received a incorrect frame!!\r\n");

            ObcMemFree(recv_frame);
            /* 移除错误帧 */
            vu_receiver_remove_frame();
            continue;
        }

        PassFlag = true; /*置过境标志 */

        driver_debug(DEBUG_TTC, "INFO: ISIS rLen: %u bytes.\r\n", recv_frame->DateSize);

        frame_num = recv_frame->DateSize;

        memmove(&((route_packet_t *)recv_frame)->dst, &recv_frame->Data, frame_num);

        /**成功接收后移除此帧*/
        vu_receiver_remove_frame();
        vTaskDelay(100);
        vu_receiver_remove_frame();

        /*若指令为关闭备份通信机指令，为了防止备份通信机接收出问题，在ISIS上行处理任务中直接关闭*/
        if ( ((route_packet_t *)recv_frame)->typ == VU_INS_BACKUP_OFF )
            vu_backup_switch_off();

//        if (!IsJLGvuWorking)/* 若关闭备份通信机 */
//        {
            /* 去掉路由头长度 */
            ((route_packet_t *)recv_frame)->len = frame_num - ROUTE_HEAD_SIZE;

            /* 送入路由队列 */
            route_queue_wirte((route_packet_t *)recv_frame, NULL);

            /*开启连续发射*/
//            vu_transmitter_set_idle_state(RemainOn);
//        }
//        else/* 若开启备份通信机 */
//        {
//            /* 释放申请的内存 */
//            ObcMemFree(recv_frame);
//
//            /*关闭ISIS连续发射*/
//            vu_transmitter_set_idle_state(TurnOff);
//        }
    }
}

/**
 * 大数据下行任务
 *
 * @param para 任务传入参数
 */
static void vu_isis_downlink_task(void *para)
{
    downlink_request * request = (downlink_request *)para;

    if(request->tpye == FILE_DATA)
    {
    	vu_isis_file_download( request->file, request->file_name );

    	f_close(request->file);
    	ObcMemFree(request->file);
    	ObcMemFree(request->file_name);
    }
    /**
     * 访问相机接收缓冲区，需要加锁
     */
    else if(request->tpye == CAM_IMAGE)
        vu_image_downlink(request->pdata, request->data_len, request->start_pack);
    /**
     * 下行完毕，解锁
     */

    vTaskDelete(NULL);
}

/**
 * 整个文件下行接口函数
 *
 * @param file 文件句柄指针
 * @param file_name 文件名指针
 * @return E_NO_ERR 正常，任务创建成功
 */
int file_whole_download(FIL *file, char *file_name)
{
	static downlink_request request;

	request.tpye = FILE_DATA;
	request.file = file;
	request.file_name = file_name;

    int ret = xTaskCreate(vu_isis_downlink_task, "FILE", configMINIMAL_STACK_SIZE * 4, &request, tskIDLE_PRIORITY + 3, NULL);

    if (ret != pdPASS)
        return E_OUT_OF_MEM;

    return E_NO_ERR;

}

/**
 * 恩来星文件下行接口
 *
 * @param file_id 图谱按文件编号
 * @return E_NO_ERR 正常，任务创建成功
 */
int enlai_file_down(uint32_t file_id)
{
    FIL *myfile = (FIL *)ObcMemMalloc( sizeof(FIL) );
    if (myfile == NULL)
        return E_MALLOC_FAIL;

    char *file_name = (char *)ObcMemMalloc(20);
    if(file_name == NULL)
    {
        ObcMemFree(myfile);
        return E_MALLOC_FAIL;
    }

    memset(file_name, 0, 20);
    sprintf(file_name, "0:enlai/%u.jpg", file_id);

    if (f_open( myfile, file_name, FA_READ | FA_OPEN_EXISTING ) != FR_OK)
    {
        ObcMemFree(myfile);
        ObcMemFree(file_name);
        return E_NO_SS;
    }

    memset(file_name, 0, 20);
    sprintf(file_name, "enlai-%u.jpg", file_id);

    if( file_whole_download( myfile, file_name ) != E_NO_ERR)
    {
        ObcMemFree(myfile);
        ObcMemFree(file_name);
        return E_NO_SS;
    }

    return E_NO_ERR;
}

/**
 * 下行整幅图片接口函数
 *
 * @param pdata 图像数据指针
 * @param len 图像数据字节数
 * @param start_pack 图像起始包号
 * @return E_NO_ERR 正常
 */
int image_whole_download(void *pdata, uint32_t len, uint16_t start_pack)
{
    static downlink_request request;

    request.tpye = CAM_IMAGE;
    request.pdata = pdata;
    request.data_len = len;
    request.start_pack = start_pack;

    int ret = xTaskCreate(vu_isis_downlink_task, "IMG", configMINIMAL_STACK_SIZE * 4, &request, tskIDLE_PRIORITY + 3, NULL);

    if (ret != pdPASS)
        return E_OUT_OF_MEM;

    return E_NO_ERR;
}

/**
 * 获取接收单元接收上一帧时的遥测
 *
 * @param tm 接收缓冲区指针
 * @return pdTRUE为正常，pdFALSE不正常
 */
int vu_isis_get_receiving_tm(receiving_tm *tm)
{
    if (rx_tm_queue == NULL)
        return pdFALSE;
    return xQueuePeek(rx_tm_queue, tm, 0);
}

/**
 * 路由器中下行接口调用，接受一个路由包
 *
 * @param packet 送到路由器的待下行的数据包
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_router_downlink(route_packet_t *packet)
{
    int ret;

    if (packet == NULL)
        return E_NO_BUFFER;

    if (!device[ISIS_I2C_HANDLE].is_initialised)
    {
        ObcMemFree(packet);
        return E_NO_DEVICE;
    }

    if (packet->len + ROUTE_HEAD_SIZE > ISIS_MTU)
    {
        ObcMemFree(packet);
        return E_INVALID_BUF_SIZE;
    }

    uint8_t TxBuf_Slot_Remain = 40; /* ISIS通信机发送缓冲区剩余空间  0--40 Slot */
    uint16_t TxBuf_Byte_Remain = 32768; /* 解理工通信机发送缓冲区剩余空间  0--32768 Byte */

    if (IsJLGvuWorking)
        ret = vu_send_frame(&packet->dst, packet->len + ROUTE_HEAD_SIZE, &TxBuf_Byte_Remain);
    else
        ret = vu_transmitter_send_frame(&packet->dst, packet->len + ROUTE_HEAD_SIZE, &TxBuf_Slot_Remain);

    ObcMemFree(packet);

    /**若发射机缓冲区已满，则等待5秒钟*/
    if(TxBuf_Byte_Remain < 512 || TxBuf_Slot_Remain < 2)
        vTaskDelay(MS_WAIT_TRANS_FREE_BUFF);

    return ret;
}

/**
 *解理工通信机下行加协议接口
 *
 * @param dst 目的地址
 * @param src 源地址
 * @param type 消息类型
 * @param pdata 待发送数据指针
 * @param len 待发送数据长度
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_jlg_send( uint8_t dst, uint8_t src, uint8_t type, void *pdata, uint32_t len )
{
    int ret;
    uint8_t Error = 0, FrameDataSize;
    uint16_t TxRemainBufByte = 0;

    uint32_t RemainSize = len;

    pdata = (uint8_t *)pdata;

    route_frame_t *downlink = ObcMemMalloc(I2C_MTU);
    if(downlink == NULL)
        return E_MALLOC_FAIL;

    downlink->dst = dst;
    downlink->src = src;
    downlink->typ = type;

    do
    {
        FrameDataSize = (RemainSize < DOWNLINK_MTU) ? RemainSize : DOWNLINK_MTU;
        memcpy(downlink->dat, pdata, FrameDataSize);

//        *(uint32_t *)(&downlink->dat[FrameDataSize]) =
//                crc32_memory((uint8_t *)downlink, ROUTE_HEAD_SIZE+FrameDataSize);

        ret = vu_send_frame(downlink, (uint8_t)(FrameDataSize + DOWNLINK_OVERHEAD), &TxRemainBufByte);


        /**如果传输成功，且通信机成功将消息加入发送缓冲区，发送指针后移 */
        if (ret == E_NO_ERR)
        {
            RemainSize -= FrameDataSize;
            pdata += FrameDataSize;

            /**若发射机缓冲区已满，则等待5秒钟*/
            if(TxRemainBufByte < 512)
                vTaskDelay(MS_WAIT_TRANS_FREE_BUFF);
        }
        else
            Error++;

        if (RemainSize != 0)
            vTaskDelay(PACK_DOWN_INTERVAL);

     /**若发送完成或者错误次数超过10次，则跳出循环 */
    }while ((RemainSize > 0) && (Error < 10));

    ObcMemFree(downlink);

    if (RemainSize == 0)
        return E_NO_ERR;
    else
    {   /**如果错误超过10次，则复位发射机*/
        vu_software_reset();
        return E_TRANSMIT_ERROR;
    }
}



/**
 * 解理工通信机上行接收任务
 *
 * @param para 没用
 */
void vu_jlg_uplink_task(void *para __attribute__((unused)))
{
    static uint16_t frame_num;

    while(1)
    {
        task_report_alive(Jlg);

        vTaskDelay(1568);

        /**如果解理工备份通信机没有开启*/
//        if (!IsJLGvuWorking)
//            continue;

        /**获取接收机缓冲区帧计数*/
        if (vu_get_frame_num(&frame_num) != E_NO_ERR)
            continue;

        if (frame_num == 0)
            continue;

        rsp_frame *recv_frame = (rsp_frame *)ObcMemMalloc(sizeof(route_packet_t) + MAX_UPLINK_CONTENT_SIZE);
        if (recv_frame == NULL)
            continue;

        if (vu_get_frame(recv_frame, MAX_UPLINK_CONTENT_SIZE) != E_NO_ERR)
        {
            ObcMemFree(recv_frame);
            continue;
        }

        vTaskDelay(100);

        /**成功接收后移除此帧*/
        vu_remove_frame();

        /**通信机接收上行消息计数加1*/
        vu_jlg_rx_count++;

        /* 若收到的帧数据长度字段不匹配，则视为错帧 */
        if (recv_frame->DateSize < ROUTE_HEAD_SIZE || recv_frame->DateSize > MAX_UPLINK_CONTENT_SIZE)
        {
            driver_debug(DEBUG_TTC, "WARNING: JLG vu has received a incorrect frame!!\r\n");
            ObcMemFree(recv_frame);
            continue;
        }

        PassFlag = true; /*置过境标志 */

        driver_debug(DEBUG_TTC, "INFO: JLG rLen: %u bytes.\r\n", recv_frame->DateSize);

//        /*显示解理工通信板上行消息， 上天前需要屏蔽掉*/
//        hex_dump( recv_frame->Data, recv_frame->DateSize );
//        printf("\n\n");

        frame_num = recv_frame->DateSize;

        memmove(&((route_packet_t *)recv_frame)->dst, &recv_frame->Data, frame_num);

        /* 去掉路由头长度 */
        ((route_packet_t *)recv_frame)->len = frame_num - ROUTE_HEAD_SIZE;

        /* 送入路由队列 */
        route_queue_wirte((route_packet_t *)recv_frame, NULL);

//        /*开启连续发射*/
//        vu_set_idle_state(RemainOn);

    }
}

