#include "bsp_pca9665.h"
#include "stm32f4xx.h"
//#include "bsp_ina_temp.h"
#include "FreeRTOS.h"
#include "bsp_usart.h"
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include "command.h"

#define CONFING 0X01
int read_TMP(struct command_context *ctx __attribute__((unused))){
	uint8_t ADDR_TMP=0X48;//0x75电源板上地址，0x48星务板上地址
	uint8_t  i;
	uint8_t TEMPER[]={0x00};
	uint8_t conf_TMP[]={CONFING,0x10};
	uint8_t READ_TMP[2]={0};
	uint16_t TMP0,TMP1,TMP;
	i2c_master_transaction(0,ADDR_TMP,conf_TMP,2,NULL,0,1000);

	for(i=0;i<10;i++)
	{
        i2c_master_transaction(0,ADDR_TMP,TEMPER,1,READ_TMP,2,1000);
        TMP0=READ_TMP[0];
        TMP1=READ_TMP[1];
        TMP0<<=8;
        TMP=TMP0|TMP1;
        printf("TMP=%#X\r\n",TMP);
	 }
	return 0;
}

//#define Configuration_INA  0X00
//#define  CH_1_INA 0X01
//#define  CH_2_INA 0X03
//#define  CH_3_INA 0X05
int read_INA(struct command_context *ctx __attribute__((unused))){
	uint8_t ADDR_INA=0X40;  //0X43电源板上地址，0x41星务板地址
	uint8_t CH_1_INA[]={0x01};
	uint8_t CH_2_INA[]={0x03};
	uint8_t CH_3_INA[]={0x05};
	uint8_t INA_ID[]={0XFE};
	uint8_t  i;
	uint8_t  MANG_ID[2];
	uint16_t MANG_ID_0,MANG_ID_1,ID;

	uint8_t conf_INA[]={0x00,0x71,0x25};
	uint8_t VOL_1[2],VOL_2[2],VOL_3[2];
	uint16_t INA_VOL1_0,INA_VOL1_1,INA_VOL1;
	uint16_t INA_VOL2_0,INA_VOL2_1,INA_VOL2;
	uint16_t INA_VOL3_0,INA_VOL3_1,INA_VOL3;

	i2c_master_transaction(0,ADDR_INA,conf_INA,3,NULL,0,1000);
	delay_ms(10);
	i2c_master_transaction(0,ADDR_INA,INA_ID,1,MANG_ID,2,1000);
	MANG_ID_0=MANG_ID[0];
	MANG_ID_1=MANG_ID[1];
	MANG_ID_0<<=8;
	ID=MANG_ID_0|MANG_ID_1;
	printf("ID=%#X\r\n",ID);
	for(i=0;i<10;i++){
        i2c_master_transaction(0,ADDR_INA,CH_1_INA,1,VOL_1,2,1000);
        INA_VOL1_0=VOL_1[0];
        INA_VOL1_1=VOL_1[1];
        INA_VOL1_0<<=8;
        INA_VOL1=INA_VOL1_0|INA_VOL1_1;
        printf("INA_VOL1=%#X\r\n",INA_VOL1);
        i2c_master_transaction(0,ADDR_INA,CH_2_INA,1,VOL_2,2,1000);
        INA_VOL2_0=VOL_2[0];
        INA_VOL2_1=VOL_2[1];
        INA_VOL2_0<<=8;
        INA_VOL2=INA_VOL2_0|INA_VOL2_1;
        printf("INA_VOL2=%#X\r\n",INA_VOL2);
        i2c_master_transaction(0,ADDR_INA,CH_3_INA,1,VOL_3,2,1000);
        INA_VOL3_0=VOL_3[0];
        INA_VOL3_1=VOL_3[1];
        INA_VOL3_0<<=8;
        INA_VOL3=INA_VOL3_0|INA_VOL3_1;
        printf("INA_VOL3=%#X\r\n",INA_VOL3);
	}
return 0;
}

command_t __sub_command senor_ina_subcommands[] = {
	{
		.name = "read",
		.help = "Read INA",
		.handler = read_INA,
	}
};

command_t __sub_command senor_temp_subcommands[] = {
	{
		.name = "read",
		.help = "Read TEMP",
		.handler = read_TMP,
	}
};

command_t __sub_command senor_subcommands[] = {
	{
		.name = "ina",
		.help = "ina subcommands",
		.chain = INIT_CHAIN(senor_ina_subcommands),
	},{
		.name = "temp",
		.help = "temp subcommands",
		.chain = INIT_CHAIN(senor_temp_subcommands),
	}
};


command_t __sub_command __root_command ina_temp_commands_master[] = {
	{
		.name = "sensor",
		.help = "TMP ADN INA RTC commands",
		.chain = INIT_CHAIN(senor_subcommands),
	},
};


void cmd_ina_temp_setup(void)
{
	command_register(ina_temp_commands_master);
}


