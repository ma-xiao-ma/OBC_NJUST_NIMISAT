
////////////////////////////////////////////////////////////////////////////
#ifndef __CUBE_COM_H__
#define __CUBE_COM_H__

#include "stdint.h"

#include "hk.h"

/*GROUP0*/
#define INS_HK_GET				0x01
#define INS_OBC_STR_DOWN		0x02
#define INS_OBC_STO_DOWN		0x03
#define INS_OBC_RST				0x04
#define INS_TIME_TO_OBC			0x05
#define INS_ADCS_ON				0x06
#define INS_ADCS_OFF			0x07
#define INS_PAL_ON				0x08
#define INS_PAL_OFF				0x09
#define INS_ANTS_ON				0x0A
#define INS_ANTS_PWR_ON			0x0B
#define INS_ANTS_PWR_OFF		0x0C
#define INS_RSH_CMD             0x0D
#define INS_DELAY_CMD           0x0E
#define INS_SD_MOUNT            0x0F

/*GROUP1*/
#define TR_BOOT			        0x10
#define TR_SHUT_DOWN			0x11
#define TR_MEM_RESET			0x12
#define TR_MEM1_RECORD			0x13
#define TR_MEM2_RECORD			0x14
#define TR_MEM3_RECORD			0x15
#define TR_MEM4_RECORD			0x16
#define TR_MEM_STOP				0x17
#define TR_MEM1_BACK			0x18
#define TR_MEM2_BACK			0x19
#define TR_MEM3_BACK			0x1A
#define TR_MEM4_BACK			0x1B
#define TR_MEM1_ERA				0x1C
#define TR_MEM2_ERA				0x1D
#define TR_MEM3_ERA				0x1E
#define TR_MEM4_ERA				0x1F

/*GROUP2*/
#define TR_PC_ON				0x20
#define TR_PC_OFF				0x21
#define TR_1M_RATE				0x22
#define TR_2M_RATE				0x23
#define TR_4M_RATE				0x24
#define TR_5V_ON                0x25
#define TR_5V_OFF               0x26
#define TR_12V_ON               0x27
#define TR_12V_OFF              0x28

/*vu通信机指令*/
#define VU_INS_HARDWARE_RESET   0x2A/* 新增 */
#define VU_INS_SOFEWARE_RESET   0x2B/* 新增 */
#define VU_INS_IDLE_STATE_SET   0x2C/* 新增 */
#define VU_INS_BACKUP_ON        0x2D/* 新增 */
#define VU_INS_BACKUP_OFF       0x2E/* 新增 */
#define VU_INS_BACKUP_FM        0x2F/* 新增 */
/*GROUP3*/
/*********************************************/
/*805 Camera*/
#define CAM_SOFTWARE_RESET      0x30
#define CAM_EXPOSURE_TIME_SET   0x31
#define CAM_GAIN_SET            0x32
#define CAM_WORK_MODE_SET       0x33
/**********************************************/

/**********************************************/
/*照片下行，通用*/
#define DOWN_NEWEST_IMAGE_INFO  0x34
#define DOWN_IMAGE_INFO         0x35
#define DOWN_IMAGE_DATA_WHOLE   0x36
#define DOWN_IMAGE_DATA_SINGLE  0x37
#define DOWN_IMAGE_DATA_PART    0x38
/**********************************************/

/**********************************************/
/*805 相机供电*/
#define CAM_POWER_ON            0x39
#define CAM_POWER_OFF           0x3A
/**********************************************/

/*********************************************/
/*805 Camera heat*/
#define CAM_HEAT2_ON            0x3B
#define CAM_HEAT2_OFF           0x3C
/**********************************************/

/*********************************************/
/*恩来相机*/
#define CAM_GET_PICTURE         0x3D/* 新增 */
#define CAM_GET_NUMBER          0x3F/* 新增 */


/*恩来文件*/
#define ENLAI_FILE_DOWN         0x40/* 新增 */
#define SAIL_
/*********************************************/



/*GROUP4*/
#define INS_DOWN_PERIOD			0x45

/*GROUP5*/
#define INS_TIME_SYN			0x5E

/*GROUP6*/

/*GROUP7*/
#define JPG_DELAY_TASK			    0x70
#define DOWNLOAD_SAVED_AUDIOFILES	0x71
#define UP_NEW_AUDIOFILES		    0x72
#define DOWNLOAD_NEW_AUDIOFILES		0x73

/*姿控上行指令*/
#define INS_MTQ_ON             0x80
#define INS_MTQ_OFF            0x81
#define INS_GPS_ON             0x82
#define INS_GPS_OFF            0x83
#define INS_MWA_ON             0x84
#define INS_MWA_OFF            0x85
#define INS_MWB_ON             0x86
#define INS_MWB_OFF            0x87
#define INS_MAGA_ON            0x88
#define INS_MAGA_OFF           0x89
#define INS_MAGB_ON            0x8A
#define INS_MAGB_OFF           0x8B
#define INS_SUN_ON             0x8C
#define INS_SUN_OFF            0x8D
#define INS_GYR_ON             0x8E
#define INS_GYR_OFF            0x8F

#define INS_MagSun_FIL_ON      0x90
#define INS_MagSun_FIL_OFF     0x91
#define INS_MagGyr_FIL_ON      0x92
#define INS_MagGyr_FIL_OFF     0x93

//#define INS_LowPower_Mode_ON   0x94
//#define INS_LowPower_Mode_OFF  0x95

#define INS_Control_Mode       0x96
#define INS_DAMP_COUNT         0x97
#define INS_MEAR_COUNT         0x98

#define INS_DET                0x99  //重新阻尼
#define INS_STA                0x9A

#define INS_Pit_FIL_QR_PARA    0x9B
#define INS_MagSun_FIL_QR_PARA 0x9C
#define INS_MagGyr_FIL_QR_PARA 0x9D
#define INS_CTL_K_PRA          0x9E
#define INS_ADCS_TIME_IN       0x9F
#define INS_CTL_P_PRA          0xA0
#define INS_CTL_D_PRA          0xA1
#define INS_CTL_Z_PRA          0xA2
#define INS_ORB_TLE_FLAG       0xA3   //轨道上注
#define INS_MW_Speed_Set       0xA4
#define INS_Max_MagTorque_Set  0xA5

/*星务和姿控之间的消息类型*/
#define INS_OBC_GET_ADCS_HK    0xC0
#define INS_GET_CROSSING_FLAG  0xC1
#define INS_GET_SAT_TIME       0xC2
#define INS_GET_SAT_POSITION   0xC3   //获取GPS位置
#define INS_SAT_TIME_SYSN      0xC4   //星务同步姿控时间
#define INS_LowPower_Mode_ON   0xC5
#define INS_LowPower_Mode_OFF  0xC6

/*下行消息类型*/
#define OBC_TELEMETRY          0xE1
#define ADCS_TELEMETRY         0xE2
#define CAM_IMAGE_INFO         0xE3
#define CAM_IMAGE              0xE4
#define FILE_INFO			   0xE5
#define FILE_DATA			   0xE6
#define GET_SD_IMAGE_CNT       0xE7

typedef struct __attribute__((packed))
{
    uint8_t DataLength; //Id字段和Data字段的总长
    uint8_t Id;  //0x01为上行的星务计算机的指令，0x02为上行的姿控计算机的指令
    uint8_t Data[]; //数据字段末尾有四字节CRC校验和一字节的Tail
} uplink_data_t;

typedef struct __attribute__((packed))
{
    uint16_t FrameSize;  //帧内容字节数.共两字节，低字节在前
    uint16_t DopplerFrequency;
    uint16_t RSSI;
    uplink_data_t Packet;
} uplink_content_t;

extern uint32_t rec_cmd_cnt;

void obc_cmd_ack(uint8_t type, uint8_t result);

void CubeUnPacket(const void *str);

/**
 * 延时任务解包函数
 *
 * @param str 接收指针
 */
void DelayTask_UnPacket(const void *str);

#endif
