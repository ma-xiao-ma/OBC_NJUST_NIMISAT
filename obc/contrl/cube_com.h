
////////////////////////////////////////////////////////////////////////////
#ifndef __CUBE_COM_H__
#define __CUBE_COM_H__

#include "stdint.h"

#include "hk.h"

#define ADCS_FRAME_LENGTH 		180
#define OBC_FRAME_LENGTH  		162

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
#define INS_BATCH_CMD           0x0E
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

/*GROUP3*/
#define CAM_SOFTWARE_RESET      0x30
#define CAM_EXPOSURE_TIME_SET   0x31
#define CAM_GAIN_SET            0x32
#define CAM_WORK_MODE_SET       0x33
#define DOWN_IMAGE_INFO_FALSH   0x34
#define DOWN_IMAGE_INFO_TF      0x35
#define DOWN_IMAGE_FLASH_WHOLE  0x36
#define DOWN_IMAGE_FALSH_SINGLE 0x37
#define DOWN_IMAGE_TF_WHOLE     0x38
#define DOWN_IMAGE_TF_SINGLE    0x39
#define CAM_POWER_ON            0x3A
#define CAM_POWER_OFF           0x3B



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

void obc_cmd_ack(void *ack, uint32_t length);

void CubeUnPacket(const unsigned char *str);
void up_group_zero_Cmd_pro(unsigned char cmd_id, const unsigned char *cube_buf);
void up_group_one_Cmd_pro(unsigned char cmd_id, const unsigned char *cube_buf);
void up_group_two_Cmd_pro(unsigned char cmd_id, const unsigned char *cube_buf);
void up_group_three_Cmd_pro(unsigned char cmd_id, const unsigned char *cube_buf);
void up_group_four_Cmd_pro(unsigned char cmd_id, const unsigned char *cube_buf);
void up_group_five_Cmd_pro(unsigned char cmd_id, const unsigned char *cube_buf);
void up_group_six_Cmd_pro(unsigned char cmd_id, const unsigned char *cube_buf);
void up_group_seven_Cmd_pro(unsigned char cmd_id, const unsigned char *cube_buf);

#endif
