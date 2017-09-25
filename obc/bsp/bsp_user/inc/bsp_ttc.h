/*
 * bsp_ttc.h
 *
 *  Created on: 2017年4月28日
 *      Author: 84474
 */

#ifndef BSP_BSP_USER_INC_BSP_TTC_H_
#define BSP_BSP_USER_INC_BSP_TTC_H_

typedef struct __attribute__((packed)){
	uint8_t SLH;
	uint8_t SLL;
	uint8_t CMD;
}ttc_cmd_t;

typedef struct __attribute__((packed)){
	uint8_t DSLH;
	uint8_t DSLL;
	uint8_t DATA[];
}ttc_data_t;

/*
 * SL比DSL多3，CMD为0x20
 */
typedef struct __attribute__((packed)){
	uint8_t 	SLH;
	uint8_t 	SLL;
	uint8_t 	CMD;
	ttc_data_t 	PARA;
}ttc_send_t;

typedef struct __attribute__((packed)){
	uint8_t STATUS;
	uint8_t SLH;
	uint8_t SLL;
	uint8_t DATA[];
}ttc_resp_t;


#define TTC_I2C_Addr 			0x2A
#define TTC_I2C_Handle 			0
#define TTC_I2C_Timeout			1000
#define TTC_Data_Valid			0x01

#define TTC_Software_Reset		0x23
/* Receiver */
#define GetFrameNumber_t		0x0B
#define GetFrames_t				0x0C

/* transmitter */
#define SendFrame_t				0x20


int TTC_Cmd(uint8_t CmdNumber, uint8_t* pResp,uint16_t RespLen);
int TTC_Cmd_No_Reply(uint8_t CmdNoRep);
int TTC_Send_Date(uint8_t* pBuffer, uint8_t NumByteToSend);
uint16_t AX25_Encode(void* pData, uint32_t DataLen, uint8_t* pResult);
int AX25_Decode(void* pData, uint32_t DataLen, uint8_t* pResult, uint16_t* pResultLen);
int AX25_Encode_test(void);
int AX25_Decode_test(void);

#endif /* BSP_BSP_USER_INC_BSP_TTC_H_ */
