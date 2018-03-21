/*
 * hk.c
 *
 *  Created on: 2016年5月30日
 *      Author: Administrator
 */

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#include "ff.h"

#include "obc_mem.h"
#include "driver_debug.h"

#include "bsp_temp175.h"
#include "bsp_intadc.h"
#include "bsp_ds1302.h"
#include "error.h"
#include "hk_arg.h"
#include "cube_com.h"
#include "contrl.h"
#include "switches.h"
#include "camera_805.h"
#include "dtb_805.h"
#include "if_adcs.h"
#include "if_downlink_vu.h"

#include "hk.h"

extern unsigned int i2c_error_count;
FIL fd = {0};
static unsigned int fd_timestamp = 0;
static int fs_ok = 0;
static int fd_count = 0;

QueueHandle_t obc_hk_queue;
QueueHandle_t eps_hk_queue;
QueueHandle_t ttc_hk_queue;
QueueHandle_t jlg_hk_queue;
QueueHandle_t dtb_hk_queue;
QueueHandle_t cam_hk_queue;
QueueHandle_t adcs_hk_queue;

uint8_t 		hk_select = 0;
uint16_t		hk_sram_index = 0;
uint32_t		hk_sd_time = 0;
uint8_t 		hk_sd_path[25];
FIL 			hkfile;
UINT			hkrbytes = 0;
uint32_t		hkleek = 0;

HK_Store_t		hk_frame;
HK_Store_t		hk_old_frame;

uint16_t  hk_frame_index = 0;      //used for up down index from here to start

HK_Fifo_t hk_main_fifo 		__attribute__((section(".bss.hk")));
HK_Fifo_t hk_append_fifo 	__attribute__((section(".bss.hk")));
hkList_t  hk_list = {0};

uint32_t hk_down_cnt;
uint32_t hk_store_cnt;

static HK_Store_t hk_store 	__attribute__((section(".bss.hk")));

/*列表项是从大到小排列的，尾项End是最小项*/
void hk_list_init(hkList_t * pxList)
{
	pxList->pxIndex = ( hkListNode_t * ) &( pxList->xListEnd );

	/*遥测列表初始化时把尾项初始化成0*/
	pxList->xListEnd.TimeValue = 0x00UL;

	pxList->xListEnd.pxNext = ( hkListNode_t * ) &( pxList->xListEnd );
	pxList->xListEnd.pxPrevious = ( hkListNode_t * ) &( pxList->xListEnd );

	pxList->uxNumberOfItems = ( uint32_t ) 0U;
}

uint32_t hk_list_insert(hkList_t * pxList, uint32_t xValueOfInsertion) {
	hkListNode_t *pxIterator = NULL;

	/*为列表项申请内存*/
	hkListNode_t *pxNewListNode = (hkListNode_t *)ObcMemMalloc(sizeof(hkListNode_t));

	if(pxNewListNode == NULL)
	{
		driver_debug(DEBUG_HK, " ^^ malloc error ^^\r\n");
		return 0;
	}

	pxNewListNode->TimeValue = xValueOfInsertion;

	/*如果遥测列表的列表项数目大于HK_LIST_NODE_CNT，则移除列表项值最小的一项*/
	if( ++(pxList->uxNumberOfItems) > HK_LIST_NODE_CNT )
	{
//		pxList->uxNumberOfItems = HK_LIST_NODE_CNT;
		pxIterator = ( hkListNode_t * ) ( pxList->xListEnd.pxPrevious );
		if(pxIterator != ( hkListNode_t * ) &( pxList->xListEnd ))
		{
			hk_list_remove(pxIterator);
		}
	}

	for( pxIterator = ( hkListNode_t * ) &( pxList->xListEnd ); xValueOfInsertion < pxIterator->pxNext->TimeValue; pxIterator = pxIterator->pxNext )
	{
		if(pxIterator->pxNext == ( hkListNode_t * ) &( pxList->xListEnd ))
		{
			(pxList->uxNumberOfItems)--;
			return 0;
		}
	}

	pxNewListNode->pxNext = pxIterator->pxNext;
	pxNewListNode->pxNext->pxPrevious = pxNewListNode;
	pxNewListNode->pxPrevious = pxIterator;
	pxIterator->pxNext = pxNewListNode;

	pxNewListNode->pvContainer = ( void * ) pxList;
	pxList->pxIndex = pxNewListNode;

	return pxList->uxNumberOfItems;
}

void * hk_list_find(uint32_t time)
{
    /*从最旧的也就是时间最小的遥测开始找起*/
	hkListNode_t *pxIterator = hk_list.xListEnd.pxPrevious;
	const uint32_t xValueOfFinding = time;

	/*如果要找的时间点小于最旧遥测的时间，则返回最旧的时间点*/
	if(xValueOfFinding < pxIterator->TimeValue)
	    return pxIterator;

	while(1)
	{
		if( xValueOfFinding > pxIterator->TimeValue && xValueOfFinding < pxIterator->pxPrevious->TimeValue)
		{
			break;
		}
		pxIterator = pxIterator->pxPrevious;
		if(pxIterator == ( hkListNode_t * ) &( hk_list.xListEnd )) {
			return NULL;
		}
	}

	return pxIterator;
}

/* 读取文件夹中的每个文件的文件名，来恢复遥测列表 */
uint32_t hk_list_recover(void) {

	FILINFO fno;
	DIR dir;
	int i = 0;

	char fname[20];
	char lfname[25];
	uint32_t lfname_length = 25, timevalue = 0;

	/* 尝试打开根目录中的hk文件夹 */
	int result = f_opendir(&dir, "0:hk");

	/* 若成功打开 */
	if(result == FR_OK)
	{
		fno.lfname = lfname;
		fno.lfsize = lfname_length;
		for(;;)
		{

			result = f_readdir(&dir,&fno);
			if(result != FR_OK)
			{
				/*printf(*/driver_debug(DEBUG_HK, "read directory failed\r\n");
				/*printf(*/driver_debug(DEBUG_HK, "list error ,result is :%u\r\n",result);

				return CMD_ERROR_FAIL;
			}

			if(fno.fname[0] == 0) break;
			if(fno.fname[0] == '.') continue;

			/*printf(*/driver_debug(DEBUG_HK, "%s\n",fno.lfname);

			for(;i<25;i++)
			{
				if(lfname[i] != '.')
				{
					fname[i] = lfname[i];
				}
				else
				{
					fname[i] = '\0';
					i = 0;
					break;
				}
			}
			i = 0;
			fname[19] = '\0';

			timevalue = atol(fname);

			if(!hk_list_insert(&hk_list, timevalue))
			{
				/*printf(*/driver_debug(DEBUG_HK, "Failed to insert\r\n");
			}

			/*printf(*/driver_debug(DEBUG_HK, "Telemetry recover success,file create time: \r\n%s\r\n", ctime(&timevalue));
		}
	}
	else
	{
	    /*printf(*/driver_debug(DEBUG_HK, " The directory has not yet been created '0:hk/'\r\n");
	}

	return 0;
}


uint32_t hk_list_remove(hkListNode_t * pxNodeToRemove) {

    /* 从列表中移除列表项 */
	hkList_t * const pxList = ( hkList_t * ) pxNodeToRemove->pvContainer;

	pxNodeToRemove->pxNext->pxPrevious = pxNodeToRemove->pxPrevious;
	pxNodeToRemove->pxPrevious->pxNext = pxNodeToRemove->pxNext;

	/* 若列表的index指向的是将要移除的列表项，则将index指向此列表项的前一项 */
	if( pxList->pxIndex == pxNodeToRemove )
	{
		pxList->pxIndex = pxNodeToRemove->pxPrevious;
	}

	pxNodeToRemove->pvContainer = NULL;
	( pxList->uxNumberOfItems )--;

	char hkpath[25];
	FIL file_handle;

	/* 移除列表项内容所对应的遥测文件 */
	sprintf(hkpath, "0:hk/%u.txt", pxNodeToRemove->TimeValue);

	/* 尝试打开文件，如果打开失败，则说明文件不存在 */
	if( f_open(&file_handle, hkpath, FA_READ) == FR_OK ) {
        f_close(&file_handle);
        /* 删除文件 */
        f_unlink(hkpath);
        driver_debug(DEBUG_HK, " ^^ DELETE file: %s ^^\r\n", hkpath);
    }

	ObcMemFree(pxNodeToRemove);

	return pxList->uxNumberOfItems;
}

void HK_fifoInit(HK_Fifo_t *Q) {
	Q->front = 0 ;
	Q->rear = 0 ;
	Q->bufferCount = 0 ;
}

uint8_t HK_fifoIn(HK_Fifo_t *Q, unsigned char *pdata, uint8_t opt) {
	uint32_t length = 0;

	if(opt == HK_FRAME_MAIN) {
		length = HK_MAIN_LENGTH;
	}
	if(opt == HK_FRAME_APPEND) {
		length = HK_APPEND_LENGTH;
	}

	if(((Q->rear+1) % HK_FIFO_BUFFER_CNT == Q->front) || (Q->bufferCount == (HK_FIFO_BUFFER_CNT-1) ))
		return HK_FIFO_FULL ;
	Q->rear = (Q->rear + 1) % HK_FIFO_BUFFER_CNT;
	memcpy(Q->frame[Q->rear],pdata,length);
	Q->bufferCount++ ;

	hk_frame_index = (hk_frame_index + 1) % HK_FIFO_BUFFER_CNT;

	return(HK_FIFO_OK) ;
}

uint8_t HK_fifoOut(HK_Fifo_t *Q, unsigned char *pdata, uint8_t opt) {
	uint32_t length = 0;

	if(opt == HK_FRAME_MAIN) {
		length = HK_MAIN_LENGTH;
	}
	if(opt == HK_FRAME_APPEND) {
		length = HK_APPEND_LENGTH;
	}

 	if((Q->front == Q->rear) && (Q->bufferCount == 0))
		return(HK_FIFO_EMPTY) ;
	else {
		Q->front = (Q->front + 1) % HK_FIFO_BUFFER_CNT;
		memcpy(pdata, Q->frame[Q->front],length);
		Q->bufferCount--;

		return(HK_FIFO_OK) ;
	}
}

uint16_t hk_fifo_find(const HK_Fifo_t *Q, uint32_t timevalue)
{

	uint16_t index = 0;
	uint16_t low = 0, high = hk_frame_index, mid;

	mid = (low + high) / 2;

	while(low <= high)
	{
		if(timevalue < ((HK_Main_t *)&(Q->frame[mid]))->obc.utc_time)
		{
			high = mid - 1;
		}
		else if(timevalue > ((HK_Main_t *)&(Q->frame[mid]))->obc.utc_time)
		{
			low = mid + 1;
		}
		else
		{
			index = mid;
			break;
		}
		mid = (low + high) / 2;
	}

	return index;
}

void hk_collect_no_store(void)
{

    hk_down_cnt ++;

	    /*星务计算机本地遥测*/
    obc_hk_get_peek(&hk_frame.main_frame.obc);

        /*电源系统遥测获取*/
	eps_hk_get_peek(&hk_frame.main_frame.eps);

        /*测控分系统遥测获取*/
	ttc_hk_get_peek(&hk_frame.main_frame.ttc);

        /*数传机遥测获取*/
	dtb_hk_get_peek(&hk_frame.main_frame.dtb);
//	jlg_hk_get_peek(&hk_frame.main_frame.jlg);

        /*相机遥测获取*/
    cam_hk_get_peek(&hk_frame.main_frame.cam);

        /*姿控分系统遥测获取*/
    adcs_hk_get_peek(&hk_frame.append_frame.adcs_hk);
}

void hk_collect(void)
{

    hk_store_cnt ++;

        /*星务计算机本地遥测*/
    obc_hk_get_peek(&hk_frame.main_frame.obc);

        /*电源系统遥测获取*/
    eps_hk_get_peek(&hk_frame.main_frame.eps);

        /*测控分系统遥测获取*/
    ttc_hk_get_peek(&hk_frame.main_frame.ttc);

        /*数传机遥测获取*/
    dtb_hk_get_peek(&hk_frame.main_frame.dtb);
//    jlg_hk_get_peek(&hk_frame.main_frame.jlg);

        /*相机遥测获取*/
    cam_hk_get_peek(&hk_frame.main_frame.cam);

        /*姿控分系统遥测获取*/
    adcs_hk_get_peek(&hk_frame.append_frame.adcs_hk);

    HK_fifoIn(&hk_main_fifo, (unsigned char *)&hk_frame.main_frame, (uint8_t)HK_FRAME_MAIN);
    HK_fifoIn(&hk_append_fifo, (unsigned char *)&hk_frame.append_frame, (uint8_t)HK_FRAME_APPEND);
}


/*每调用一次创建一个文件*/
int hk_store_init(void) {

	static uint32_t hk_file_init = 0;

	/*若初始化标志位0，则创建hk文件夹*/
	if(hk_file_init == 0)
	{
		f_mkdir("0:hk");
		hk_file_init++;
	}

	/* 获取当前UNIX时间的时间戳*/
	timestamp_t time_now;
	clock_get_time(&time_now);
	fd_timestamp = time_now.tv_sec;

	driver_debug(DEBUG_HK,"fd_timestamp is %d\r\n", fd_timestamp);

	char path[50];
	sprintf(path, "0:hk/%u.txt", fd_timestamp);
	driver_debug(DEBUG_HK, "Create and Open file %s\r\n", path);

	/*在/hk目录下创建以时间戳命名的.txt文件*/
	int result = (f_open(&fd, path, FA_READ | FA_WRITE | FA_CREATE_ALWAYS));
	if ( result != FR_OK) {
		driver_debug(DEBUG_HK,"hk_store_init fail to Creates:%s  Result:%d \r\n", path, result);
		fs_ok = 0;
		return -1;
	}

	f_sync(&fd);

	/*将文件名加入遥测列表*/
	if(!hk_list_insert(&hk_list, fd_timestamp)) {
		driver_debug(DEBUG_HK, "Failed to insert\r\n");
		fs_ok = 0;
		return -1;
	}

	fs_ok = 1;

	return 0;
}

void hk_out(void) {
	HK_fifoOut(&hk_main_fifo, (unsigned char *)&(hk_store.main_frame), (uint8_t)HK_FRAME_MAIN);
	HK_fifoOut(&hk_append_fifo, (unsigned char *)&(hk_store.append_frame), (uint8_t)HK_FRAME_APPEND);
}

int hk_store_add(void)
{

	/* 从遥测主帧辅帧FIFO中读出遥测，填入hk_store结构体 */
	HK_fifoOut(&hk_main_fifo, (unsigned char *)&(hk_store.main_frame), (uint8_t)HK_FRAME_MAIN);
	HK_fifoOut(&hk_append_fifo, (unsigned char *)&(hk_store.append_frame), (uint8_t)HK_FRAME_APPEND);

	/* 将从FIFO读出的遥测存入TF卡 */
	if (fs_ok)
	{
		unsigned int written;

		/*尝试写入TF卡*/
		int result = f_write(&fd, &(hk_store), sizeof(HK_Store_t), &written);

		/*若写入失败，则关闭文件，置初始化成功标志为0*/
		if (result != FR_OK || written == 0)
		{
			driver_debug(DEBUG_HK, "Failed to write HK to SD-Card\r\n");
			f_close(&fd);
			fs_ok = 0;
			return -1;
		}
		/*刷新打开的文件，跟f_close()执行相同的操作，
		 * 只不过不关闭文件。
		 */
		f_sync(&fd);
	}
	/* 若没有文件存储初始化 */
	else
	{
		hk_store_init();
		return 0;
	}

	/* 一个文件中最多存储HK_FILE_MAX_COUNT条遥测信息 */
	if (fd_count++ > HK_FILE_MAX_COUNT) {
		f_close(&fd);
		fd_count = 0;
		hk_store_init();
	}

	return 0;
}

void hk_file_task(void * paragram) {

	char hk_path[50];
	FILINFO fno = {0};

	hkListNode_t *pxIterator = NULL;
	hkList_t * pxList = (hkList_t *)paragram;

	/* 把遥测列表的首列表项赋给pxIterator */
	pxIterator = ( hkListNode_t * ) (&( pxList->xListEnd ))->pxNext;

	while(1)
	{

	    /* 如果首列表项与尾列表项相同 */
		if(pxIterator == ( hkListNode_t * ) (&( pxList->xListEnd ))) {
		    /* 清空路径 */
			memset(hk_path,'\0',sizeof(hk_path));
			/* 指向下一个列表项 */
			pxIterator = pxIterator->pxNext;
			continue;
		}

		/* 为啥要检查.db文件 */
		sprintf(hk_path, "0:hk/%u.db", pxIterator->TimeValue);
		driver_debug(DEBUG_HK, "Check if the file exists:%s\r\n", hk_path);

		/* 检查文件是否存在，若不存在则将文件名从列表中移除 */
		if ((f_stat(hk_path, &fno)) != FR_OK)
		{
			driver_debug(DEBUG_HK,"File does not exist: %s\r\n", hk_path);
			hk_list_remove(pxIterator);
			memset(hk_path,'\0',sizeof(hk_path));
			/*指针指向列表中下一个列表项*/
			pxIterator = pxIterator->pxNext;
			continue;
		}

		/* 若文件存在，文件小于HK_Store_t结构体大小的文件从列表中移除 */
		if(fno.fsize < sizeof(HK_Store_t))
		{
		    driver_debug(DEBUG_HK,"File size error: %s\r\n", hk_path);
			hk_list_remove(pxIterator);
		}

		/*指针指向列表中下一个列表项*/
		pxIterator = pxIterator->pxNext;
		{
			if(pxIterator == ( hkListNode_t * ) &( pxList->xListEnd ))
			{
				vTaskDelay(200);
			}
		}

		memset(hk_path,'\0',sizeof(hk_path));
	}
}

void vTelemetryFileManage(void * paragram)
{

    char hk_path[50];
    FILINFO fno = {0};

    hkListNode_t *pxIterator = NULL;
    hkList_t * pxList = (hkList_t *)paragram;

    /* 把遥测列表的首列表项赋给pxIterator */
    pxIterator = ( hkListNode_t * )&pxList->xListEnd;

    while( (pxIterator = pxIterator->pxNext) != ( hkListNode_t * )&pxList->xListEnd )
    {

        /*清空路径*/
        memset(hk_path,'\0',sizeof(hk_path));

        sprintf(hk_path, "0:hk/%u.txt", pxIterator->TimeValue);
        driver_debug(DEBUG_HK, "Check if the file exists:%s\r\n", hk_path);

        /* 检查文件是否存在，若不存在则将文件名从列表中移除 */
        if ((f_stat(hk_path, &fno)) != FR_OK)
        {
            driver_debug(DEBUG_HK,"File does not exist: %s\r\n", hk_path);
            hk_list_remove(pxIterator);
            continue;
        }

        /* 若文件存在，文件小于HK_Store_t结构体大小的文件从列表中移除 */
        if(fno.fsize < 4 * sizeof(HK_Store_t))
        {
            driver_debug(DEBUG_HK,"File size error: %s\r\n", hk_path);
            hk_list_remove(pxIterator);
            continue;
        }
    }
}

/**
 * 由星务遥测采集任务内部调用
 * @param obc 遥测结构体指针
 */
static void obc_get_hk(obc_hk_t * obc)
{
    obc->sat_id = 0x06;

    obc->soft_id = 0x06;

    obc->reboot_count = obc_boot_count;

    obc->rec_cmd_count = rec_cmd_cnt;

    obc->vu_rec_count = vu_isis_rx_count;

    obc->hk_down_count = hk_down_cnt;

    obc->hk_store_count = hk_store_cnt;

    obc->i2c_error_count = i2c_error_count;

    obc->last_reset_time = obc_reset_time;

    obc->work_mode = mode;

    obc->utc_time = clock_get_time_nopara();

    obc->tmep_mcu = get_mcu_temp();

    get_obc_temp(&obc->tmep_board);

    get_switch_status((uint8_t *)&obc->on_off_status);

    obc->mindex = hk_main_fifo.rear;

    obc->aindex = hk_append_fifo.rear;
}

/**
 * 星务遥测采集任务
 */
void obc_hk_task(void)
{
    static obc_hk_t obc_hk;

    if (obc_hk_queue == NULL)
        return;

    obc_get_hk(&obc_hk);

    xQueueOverwrite(obc_hk_queue, &obc_hk);
}

/**
 * 星务遥测获取函数
 *
 * @param obc 星务遥测结构体指针
 * @return pdTRUE为正常，pdFALSE不正常
 */
int obc_hk_get_peek(obc_hk_t * obc)
{
    if (obc_hk_queue == NULL)
        return pdFALSE;
    return xQueuePeek(obc_hk_queue, obc, 0);
}



/**
 * 电源系统遥测采集任务，采集数值放入eps_hk_queue队列中
 *
 */
void eps_hk_task(void)
{
    static EpsAdcValue_t eps_hk;

    if (eps_hk_queue == NULL)
        return;

    eps_get_hk(&eps_hk);

    xQueueOverwrite(eps_hk_queue, &eps_hk);

}

/**
 * 通过eps_hk_queue队列获取EPS遥测值
 *
 * @param tm 接收缓冲区指针
 * @return pdTRUE为正常，pdFALSE不正常
 */
int eps_hk_get_peek(eps_hk_t *eps)
{
    EpsAdcValue_t eps_hk;
    if (eps_hk_queue == NULL)
        return pdFALSE;

    if (xQueuePeek(eps_hk_queue, &eps_hk, 0) != pdTRUE)
        return pdFALSE;

    /*电源系统遥测*/
    eps->temp_batt_board[0] = eps_hk.BatTemp[0];
    eps->temp_batt_board[1] = eps_hk.BatTemp[1];
    for(int i=0; i < 4; i++)
        eps->temp_eps[i] = eps_hk.EpsTemp[i];
    for(int i=0; i < 6; i++)
    {
        eps->sun_c[i] = eps_hk.In_SunC[i];
        eps->sun_v[i] = eps_hk.In_SunV[i];
    }
    eps->out_BusC = eps_hk.Out_BusC;
    eps->out_BusV = eps_hk.Out_BusV;
    eps->UV_board_C = eps_hk.Out_ComC;

    for(int i=0; i < 6; i++)
        eps->Vol_5_C[i] = eps_hk.Out_BranchC[i];
    for(int i=0; i < 5; i++)
        eps->Bus_c[i] = eps_hk.Out_BranchC[i+6];

    return pdTRUE;
}

/**
 * TTC遥测采集函数，直接采集数据到接收缓冲区指针
 *
 * @param ttc 接收缓冲区指针
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
static int ttc_get_hk(vu_isis_hk_t *ttc)
{
    int ret;

    ret = vu_receiver_get_uptime(&ttc->ru_uptime);
    if (ret != E_NO_ERR)
        return ret;

    ret = vu_receiver_measure_tm(&ttc->ru_curt);
    if (ret != E_NO_ERR)
         return ret;

    vu_isis_get_receiving_tm(&ttc->ru_last);

    ret = vu_transmitter_get_uptime(&ttc->tu_uptime);
    if (ret != E_NO_ERR)
        return ret;

    ret = vu_transmitter_measure_tm(&ttc->tu_curt);
    if (ret != E_NO_ERR)
        return ret;

    ret = vu_transmitter_get_last_tm(&ttc->tu_last);
    if (ret != E_NO_ERR)
        return ret;

    ret = vu_transmitter_get_state(&ttc->tx_state);
    if (ret != E_NO_ERR)
        return ret;

    return E_NO_ERR;
}

/**
 * TTC遥测采集任务，采集到的数据送入ttc_hk_queue队列
 *
 */
void ttc_hk_task(void)
{
    static vu_isis_hk_t ttc_hk;

    if (ttc_hk_queue == NULL)
        return;

    ttc_get_hk(&ttc_hk);

    xQueueOverwrite(ttc_hk_queue, &ttc_hk);
}

/**
 * 通过队列获取TTC遥测值
 *
 * @param tm 接收缓冲区指针
 * @return pdTRUE为正常，pdFALSE不正常
 */
int ttc_hk_get_peek(vu_isis_hk_t *ttc)
{
    if (ttc_hk_queue == NULL)
        return pdFALSE;
    return xQueuePeek(ttc_hk_queue, ttc, 0);
}

/**
 * JLG通信机遥测采集函数，直接采集数据到接收缓冲区指针
 *
 * @param ttc 接收缓冲区指针
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
static int jlg_get_hk(vu_jlg_hk_t *jlg)
{
    int ret;

    ret = vu_measure_all_tm(&jlg->vu_tm);
    if (ret != E_NO_ERR)
         return ret;

    jlg->vu_tm.RecCount = vu_jlg_rx_count;

    ret = vu_get_uptime(&jlg->vu_tm.Uptime);
    if (ret != E_NO_ERR)
        return ret;

    ret = vu_get_state(&jlg->tx_state);
    if (ret != E_NO_ERR)
        return ret;

    return E_NO_ERR;
}

/**
 * JLG通信机遥测采集任务，采集到的数据送入jlg_hk_queue队列
 *
 */
void jlg_hk_task(void)
{
    static vu_jlg_hk_t jlg_hk;

    if (jlg_hk_queue == NULL)
        return;

    jlg_get_hk(&jlg_hk);

    xQueueOverwrite(jlg_hk_queue, &jlg_hk);
}

/**
 * 通过队列获取JLG通信机遥测值
 *
 * @param tm 接收缓冲区指针
 * @return pdTRUE为正常，pdFALSE不正常
 */
int jlg_hk_get_peek(vu_jlg_hk_t *jlg)
{
    if (jlg_hk_queue == NULL)
        return pdFALSE;
    return xQueuePeek(jlg_hk_queue, jlg, 0);
}

/**
 * 数传机数据采集任务，采集数值放入eps_hk_queue中
 *
 */
void dtb_hk_task(void)
{
    static dtb_805_hk_t dtb_hk;

    if (dtb_hk_queue == NULL)
        return;

    xDTBTelemetryGet((uint8_t *)&dtb_hk, 100);

    xQueueOverwrite(dtb_hk_queue, &dtb_hk);
}

/**
 * 通过队列获取dtb遥测值
 *
 * @param tm 接收缓冲区指针
 * @return pdTRUE为正常，pdFALSE不正常
 */
int dtb_hk_get_peek(dtb_805_hk_t *dtb)
{
    if (dtb_hk_queue == NULL)
        return pdFALSE;
    return xQueuePeek(dtb_hk_queue, dtb, 0);
}

/**
 * 组相机遥测帧
 *
 * @param cam_hk 接收缓冲区
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
static int cam_get_hk(cam_805_hk_t *cam_hk)
{
    /**
     * 需要完成相机驱动后添加对应的
     * 遥测获取指令
     */
    Camera_Temp_Get(0x01, &cam_hk->point_1_temp);

    Camera_Temp_Get(0x02, &cam_hk->point_2_temp);

    Camera_Temp_Get(0x03, &cam_hk->point_3_temp);

    Camera_Exposure_Time_Read(&cam_hk->exposure_time);

    Camera_Gain_Get(&cam_hk->gain);

    Camera_Work_Mode_Get(&cam_hk->work_mode);

    return E_NO_ERR;
}

/**
 * 相机遥测采集任务，采集到的遥测放入cam_hk_queue队列中
 *
 * 遥测值获取调用int cam_hk_get_peek(cam_805_hk_t *cam)函数
 */
void cam_hk_task(void)
{
    static cam_805_hk_t cam_hk;

    if (cam_hk_queue == NULL)
        return;

    cam_get_hk(&cam_hk);

    xQueueOverwrite(cam_hk_queue, &cam_hk);
}

/**
 * 通过队列获取TTC遥测值
 *
 * @param tm 接收缓冲区指针
 * @return pdTRUE为正常，pdFALSE不正常
 */
int cam_hk_get_peek(cam_805_hk_t *cam)
{
    if (cam_hk_queue == NULL)
        return pdFALSE;
    return xQueuePeek(cam_hk_queue, cam, 0);
}

/**
 * 姿控系统数据采集任务，采集数值放入adcs_hk_queue中
 *
 */
void adcs_hk_task(void)
{
    static adcs805_hk_t adcs_hk;

    if (adcs_hk_queue == NULL)
        return;

    int ret;

    if ( (ret = adcs_get_hk(&adcs_hk, 100)) != E_NO_ERR )
        printf("WARN: Get ADCS hk fail! Result: %d\r\n", ret);

    xQueueOverwrite(adcs_hk_queue, &adcs_hk);
}

/**
 * 通过队列获取ADCS遥测值
 *
 * @param tm 接收缓冲区指针
 * @return pdTRUE为正常，pdFALSE不正常
 */
int adcs_hk_get_peek(adcs805_hk_t *adcs)
{
    if (adcs_hk_queue == NULL)
        return pdFALSE;
    return xQueuePeek(adcs_hk_queue, adcs, 0);
}

/**
 * 遥测采集任务初始化 创建6个队列， 需要在初始化函数中调用
 */
void hk_collect_task_init(void)
{
    if (obc_hk_queue == NULL)
        obc_hk_queue = xQueueCreate(1, sizeof( obc_hk_t ));

    if (eps_hk_queue == NULL)
        eps_hk_queue = xQueueCreate(1, sizeof( EpsAdcValue_t ));

    if (ttc_hk_queue == NULL)
        ttc_hk_queue = xQueueCreate(1, sizeof( vu_isis_hk_t ));

    if (dtb_hk_queue == NULL)
        dtb_hk_queue = xQueueCreate(1, sizeof( dtb_805_hk_t ));

    if (jlg_hk_queue == NULL)
        jlg_hk_queue = xQueueCreate(1, sizeof( vu_jlg_hk_t ));

    if (cam_hk_queue == NULL)
        cam_hk_queue = xQueueCreate(1, sizeof( cam_805_hk_t ));

    if (adcs_hk_queue == NULL)
        adcs_hk_queue = xQueueCreate(1, sizeof( adcs805_hk_t ));
}


