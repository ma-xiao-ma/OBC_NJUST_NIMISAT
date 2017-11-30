#include "bsp_nor_flash.h"

#include "stm32f4xx.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#include <stdint.h>
#include <string.h>


/* 判忙时的执行语句循环次数  */
#define BlockErase_Timeout    	((uint32_t)0x00A00000)
#define ChipErase_Timeout     	((uint32_t)0x00200000)
#define Program_Timeout       	((uint32_t)0x00001400)

/* PD6是NOR FLASH输出到STM32的忙信号，通过GPIO查询方式判断  */
#define NOR_IS_BUSY()			(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_6) == RESET)

static void NOR_QuitToReadStatus(void);
static uint8_t NOR_GetStatus(uint32_t Timeout);


void FSMC_NOR_ReadID(void);
NOR_STATUS FSMC_NOR_GetStatus(u32 Timeout);
/*
****************************************************************************
*	函数名称: bsp_InitNorFlash
*	功能说明: 配置外接NOR FLASH
*	形        参: 无
*	返  回  值: 无
****************************************************************************
*/
void bsp_InitNorFlash(void)
{
	FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
	FSMC_NORSRAMTimingInitTypeDef  p;
	GPIO_InitTypeDef GPIO_InitStructure;
	uint32_t ChipID;	
	

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE |
			RCC_AHB1Periph_GPIOF |RCC_AHB1Periph_GPIOG, ENABLE);

	RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC, ENABLE);

	//	PG9/FSMC_NE2

	/* GPIOD configuration */
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource4, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource10, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource11, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0  | GPIO_Pin_1  | GPIO_Pin_4  | GPIO_Pin_5  |
	                              GPIO_Pin_8  | GPIO_Pin_9  | GPIO_Pin_10 | GPIO_Pin_11 |
	                              GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* GPIOE configuration */
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource3 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource4 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource5 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource7 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource8 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource9 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource10 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource11 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource12 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource13 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource14 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource15 , GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4  | GPIO_Pin_5   | GPIO_Pin_7 |
	                              GPIO_Pin_8  | GPIO_Pin_9  | GPIO_Pin_10 | GPIO_Pin_11|
	                              GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;

	GPIO_Init(GPIOE, &GPIO_InitStructure);


	/* GPIOF configuration */
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource0 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource1 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource2 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource3 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource4 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource5 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource12 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource13 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource14 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource15 , GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0  | GPIO_Pin_1  | GPIO_Pin_2  | GPIO_Pin_3  |
	                              GPIO_Pin_4  | GPIO_Pin_5  | GPIO_Pin_12 | GPIO_Pin_13 |
	                              GPIO_Pin_14 | GPIO_Pin_15;

	GPIO_Init(GPIOF, &GPIO_InitStructure);


	/* GPIOG configuration */
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource0 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource1 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource2 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource3 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource4 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource5 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource9 , GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0  | GPIO_Pin_1  | GPIO_Pin_2  | GPIO_Pin_3 |
	                              GPIO_Pin_4  | GPIO_Pin_5  | GPIO_Pin_9;

	GPIO_Init(GPIOG, &GPIO_InitStructure);

	/* PD6配置 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;		/* 输入模式 */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/*-- FSMC Configuration ------------------------------------------------------*/
//	p.FSMC_AddressSetupTime = 0x06;			/* 0x05正常  0x04异常  */
//	p.FSMC_AddressHoldTime = 0x01;
//	p.FSMC_DataSetupTime = 0x0C;			/* 0x0B正常  0x0A异常  */
//	p.FSMC_BusTurnAroundDuration = 0x00;
//	p.FSMC_CLKDivision = 0x00;
//	p.FSMC_DataLatency = 0x00;
//	p.FSMC_AccessMode = FSMC_AccessMode_B;

//	p.FSMC_AddressSetupTime = 0x05;			/* 0x05正常  0x04异常  */
//	p.FSMC_AddressHoldTime = 0x00;
//	p.FSMC_DataSetupTime = 0x07;			/* 0x0B正常  0x0A异常  */
//	p.FSMC_BusTurnAroundDuration = 0x00;
//	p.FSMC_CLKDivision = 0x00;
//	p.FSMC_DataLatency = 0x00;
//	p.FSMC_AccessMode = FSMC_AccessMode_B;

	/*2017/5/8
	 * lijianfeng
	 */
	p.FSMC_AddressSetupTime = 0x06;
	p.FSMC_AddressHoldTime = 0x01;
	p.FSMC_DataSetupTime = 0x0C;
	p.FSMC_BusTurnAroundDuration = 0x00;
	p.FSMC_CLKDivision = 0x00;
	p.FSMC_DataLatency = 0x00;
	p.FSMC_AccessMode = FSMC_AccessMode_B;

	FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM2;
	FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
	FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_NOR;

//	FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_8b;
	FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;


	FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
	FSMC_NORSRAMInitStructure.FSMC_AsynchronousWait = FSMC_AsynchronousWait_Disable;
	FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
	FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;
	FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
	FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
	FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
	FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;
	FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;

	FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &p;
	FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &p;

	FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure);

	/*!< Enable FSMC Bank1_SRAM2 Bank */
	FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM2, ENABLE);
	
//	NOR_EraseChip();

//	ChipID = NOR_ReadID();
//	FSMC_NOR_ReadID();
//	if(ChipID == 0x017E0A00)
//	{

//	}
}


void FSMC_NOR_ReadID(void)
{
	uint16_t ka,kb,kc,kd;
  NOR_WRITE(WORD_ADDR(0x0555), 0x00AA);
  NOR_WRITE(WORD_ADDR(0x02AA), 0x0055);
  NOR_WRITE(WORD_ADDR(0x0555), 0x0090);

  ka = *(vu16 *) WORD_ADDR(0x0000);
  kb = *(vu16 *) WORD_ADDR(0x0001);
  kc = *(vu16 *) WORD_ADDR(0x000E);
  kd = *(vu16 *) WORD_ADDR(0x000F);

  printf("NorFlash ID = %x, %x, %x, %x\r\n",ka,kb,kc,kd);
}


NOR_STATUS FSMC_NOR_EraseBlock(u32 BlockAddr)
{
  NOR_WRITE(WORD_ADDR(0x05555), 0x00AA);
  NOR_WRITE(WORD_ADDR(0x02AAA), 0x0055);
  NOR_WRITE(WORD_ADDR(0x05555), 0x0080);
  NOR_WRITE(WORD_ADDR(0x05555), 0x00AA);
  NOR_WRITE(WORD_ADDR(0x02AAA), 0x0055);
  NOR_WRITE(WORD_ADDR(BlockAddr), 0x0030);

  return (FSMC_NOR_GetStatus(BlockErase_Timeout));
}

/*******************************************************************************
* Function Name  : FSMC_NOR_EraseChip
* Description    : Erases the entire chip.
* Input          : None
* Output         : None
* Return         : NOR_Status:The returned value can be: NOR_SUCCESS, NOR_ERROR
*                  or NOR_TIMEOUT
*******************************************************************************/
NOR_STATUS FSMC_NOR_EraseChip(void)
{
  NOR_WRITE(WORD_ADDR(0x05555), 0x00AA);
  NOR_WRITE(WORD_ADDR(0x02AAA), 0x0055);
  NOR_WRITE(WORD_ADDR(0x05555), 0x0080);
  NOR_WRITE(WORD_ADDR(0x05555), 0x00AA);
  NOR_WRITE(WORD_ADDR(0x02AAA), 0x0055);
  NOR_WRITE(WORD_ADDR(0x05555), 0x0010);

  return (FSMC_NOR_GetStatus(ChipErase_Timeout));
}

/******************************************************************************
* Function Name  : FSMC_NOR_WriteHalfWord
* Description    : Writes a half-word to the NOR memory.
* Input          : - WriteAddr : NOR memory internal address to write to.
*                  - Data : Data to write.
* Output         : None
* Return         : NOR_Status:The returned value can be: NOR_SUCCESS, NOR_ERROR
*                  or NOR_TIMEOUT
*******************************************************************************/
NOR_STATUS FSMC_NOR_WriteHalfWord(u32 WriteAddr, u16 Data)
{
  NOR_WRITE(WORD_ADDR(0x05555), 0x00AA);
  NOR_WRITE(WORD_ADDR(0x02AAA), 0x0055);
  NOR_WRITE(WORD_ADDR(0x05555), 0x00A0);

  NOR_WRITE(WORD_ADDR(WriteAddr), Data);

  return (FSMC_NOR_GetStatus(Program_Timeout));
}

/*******************************************************************************
* Function Name  : FSMC_NOR_WriteBuffer
* Description    : Writes a half-word buffer to the FSMC NOR memory.
* Input          : - pBuffer : pointer to buffer.
*                  - WriteAddr : NOR memory internal address from which the data
*                    will be written.
*                  - NumHalfwordToWrite : number of Half words to write.
* Output         : None
* Return         : NOR_Status:The returned value can be: NOR_SUCCESS, NOR_ERROR
*                  or NOR_TIMEOUT
*******************************************************************************/
NOR_STATUS FSMC_NOR_WriteBuffer(u16* pBuffer, u32 WriteAddr, u32 NumHalfwordToWrite)
{
	NOR_STATUS status = NOR_ONGOING;

  do
  {
    /* Transfer data to the memory */
    status = FSMC_NOR_WriteHalfWord(WriteAddr, *pBuffer++);
    WriteAddr = WriteAddr + 1;
    NumHalfwordToWrite--;
  }
  while((status == NOR_SUCCESS) && (NumHalfwordToWrite != 0));

  return (status);
}

/*******************************************************************************
* Function Name  : FSMC_NOR_ProgramBuffer
* Description    : Writes a half-word buffer to the FSMC NOR memory. This function
*                  must be used only with S29GL128P NOR memory.
* Input          : - pBuffer : pointer to buffer.
*                  - WriteAddr: NOR memory internal address from which the data
*                    will be written.
*                  - NumHalfwordToWrite: number of Half words to write.
*                    The maximum allowed value is 32 Half words (64 bytes).
* Output         : None
* Return         : NOR_Status:The returned value can be: NOR_SUCCESS, NOR_ERROR
*                  or NOR_TIMEOUT
*******************************************************************************/
NOR_STATUS FSMC_NOR_ProgramBuffer(u16* pBuffer, u32 WriteAddr, u32 NumHalfwordToWrite)
{
  u32 lastloadedaddress = 0x00;
  u32 currentaddress = 0x00;
  u32 endaddress = 0x00;

  /* Initialize variables */
  currentaddress = WriteAddr;
  endaddress = WriteAddr + NumHalfwordToWrite - 1;
  lastloadedaddress = WriteAddr;

  /* Issue unlock command sequence */
  NOR_WRITE(WORD_ADDR(0x005555), 0x00AA);

  NOR_WRITE(WORD_ADDR(0x02AAA), 0x0055);

  /* Write Write Buffer Load Command */
  NOR_WRITE(WORD_ADDR(WriteAddr), 0x0025);
  NOR_WRITE(WORD_ADDR(WriteAddr), (NumHalfwordToWrite - 1));

  /* Load Data into NOR Buffer */
  while(currentaddress <= endaddress)
  {
    /* Store last loaded address & data value (for polling) */
    lastloadedaddress = currentaddress;

    NOR_WRITE(WORD_ADDR(currentaddress), *pBuffer++);
    currentaddress += 1;
  }

  NOR_WRITE(WORD_ADDR(lastloadedaddress), 0x29);

  return(FSMC_NOR_GetStatus(Program_Timeout));
}

/******************************************************************************
* Function Name  : FSMC_NOR_ReadHalfWord
* Description    : Reads a half-word from the NOR memory.
* Input          : - ReadAddr : NOR memory internal address to read from.
* Output         : None
* Return         : Half-word read from the NOR memory
*******************************************************************************/
u16 FSMC_NOR_ReadHalfWord(u32 ReadAddr)
{
    /* 复位NOR_FLASH */
    NOR_WRITE((NOR_FLASH_ADDR + ReadAddr), 0x00F0);
    return (*(vu16 *)(WORD_ADDR(ReadAddr)));
}

/*******************************************************************************
* Function Name  : FSMC_NOR_ReadBuffer
* Description    : Reads a block of data from the FSMC NOR memory.
* Input          : - pBuffer : pointer to the buffer that receives the data read
*                    from the NOR memory.
*                  - ReadAddr : NOR memory internal address to read from.
*                  - NumHalfwordToRead : number of Half word to read.
* Output         : None
* Return         : None
*******************************************************************************/
void FSMC_NOR_ReadBuffer(u16* pBuffer, u32 ReadAddr, u32 NumHalfwordToRead)
{

  NOR_WRITE((NOR_FLASH_ADDR + ReadAddr), 0x00F0);

  for(; NumHalfwordToRead != 0x00; NumHalfwordToRead--) /* while there is data to read */
  {
    /* Read a Halfword from the NOR */
    *pBuffer++ = *(vu16 *)(WORD_ADDR(ReadAddr));
    ReadAddr = ReadAddr + 1;
  }
}

/******************************************************************************
* Function Name  : FSMC_NOR_ReturnToReadMode
* Description    : Returns the NOR memory to Read mode.
* Input          : None
* Output         : None
* Return         : NOR_SUCCESS
*******************************************************************************/
NOR_STATUS FSMC_NOR_ReturnToReadMode(void)
{
  NOR_WRITE(NOR_FLASH_ADDR, 0xF0);

  return (NOR_SUCCESS);
}

/******************************************************************************
* Function Name  : FSMC_NOR_Reset
* Description    : Returns the NOR memory to Read mode and resets the errors in
*                  the NOR memory Status Register.
* Input          : None
* Output         : None
* Return         : NOR_SUCCESS
*******************************************************************************/
NOR_STATUS FSMC_NOR_Reset(void)
{
  NOR_WRITE(WORD_ADDR(0x005555), 0x00AA);
  NOR_WRITE(WORD_ADDR(0x002AAA), 0x0055);
  NOR_WRITE(NOR_FLASH_ADDR, 0x00F0);

  return (NOR_SUCCESS);
}

/******************************************************************************
* Function Name  : FSMC_NOR_GetStatus
* Description    : Returns the NOR operation status.
* Input          : - Timeout: NOR progamming Timeout
* Output         : None
* Return         : NOR_Status:The returned value can be: NOR_SUCCESS, NOR_ERROR
*                  or NOR_TIMEOUT
*******************************************************************************/
NOR_STATUS FSMC_NOR_GetStatus(u32 Timeout)
{
  u16 val1 = 0x00, val2 = 0x00;
  NOR_STATUS status = NOR_ONGOING;
  u32 timeout = Timeout;

  /* Poll on NOR memory Ready/Busy signal ------------------------------------*/
  while((GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_6) != RESET) && (timeout > 0))
  {
    timeout--;
  }

  timeout = Timeout;

  while((GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_6) == RESET) && (timeout > 0))
  {
    timeout--;
  }

  /* Get the NOR memory operation status -------------------------------------*/
  while((Timeout != 0x00) && (status != NOR_SUCCESS))
  {
    Timeout--;

   /* Read DQ6 and DQ5 */
    val1 = *(vu16 *)(NOR_FLASH_ADDR);
    val2 = *(vu16 *)(NOR_FLASH_ADDR);

    /* If DQ6 did not toggle between the two reads then return NOR_Success */
    if((val1 & 0x0040) == (val2 & 0x0040))
    {
      return NOR_SUCCESS;
    }

    if((val1 & 0x0020) != 0x0020)
    {
      status = NOR_ONGOING;
    }

    val1 = *(vu16 *)(NOR_FLASH_ADDR);
    val2 = *(vu16 *)(NOR_FLASH_ADDR);

    if((val1 & 0x0040) == (val2 & 0x0040))
    {
      return NOR_SUCCESS;
    }
    else if((val1 & 0x0020) == 0x0020)
    {
      return NOR_ERROR;
    }
  }

  if(Timeout == 0x00)
  {
    status = NOR_TIMEOUT;
  }

  /* Return the operation status */
  return (status);
}

/******************************************************************************
* 函数名        : USER_NOR_SectorErase
* 描述            : 片外NorFlash扇区擦除，扇区详细描述详见芯片手册S29JL032L第19页 SA0-SA7 每个扇区8K Bytes
*           SA8-SA70每个扇区64K Bytes
* 输入            : - SectorNum: 需要擦除的扇区编号 0-70
* 输出            : None
* 返回            : NOR_Status:擦除状态，返回值可能为: NOR_SUCCESS(0), NOR_ERROR(2)
*                  或NOR_TIMEOUT(3)
*******************************************************************************/
NOR_STATUS USER_NOR_SectorErase(u8 SectorNum)
{
  NOR_WRITE(WORD_ADDR(0x05555), 0x00AA);
  NOR_WRITE(WORD_ADDR(0x02AAA), 0x0055);
  NOR_WRITE(WORD_ADDR(0x05555), 0x0080);
  NOR_WRITE(WORD_ADDR(0x05555), 0x00AA);
  NOR_WRITE(WORD_ADDR(0x02AAA), 0x0055);
  if(SectorNum < 8)
      NOR_WRITE(WORD_ADDR(SectorNum * 4096), 0x0030);
  else
      NOR_WRITE(WORD_ADDR((SectorNum-7) * 32768), 0x0030);

  return (FSMC_NOR_GetStatus(BlockErase_Timeout));
}


uint32_t NOR_ReadID(void)
{
	uint32_t uiID;
	uint8_t id1, id2, id3, id4;

	NOR_BWRITE(BYTE_ADDR(0x0555), 0xAA);
	NOR_BWRITE(BYTE_ADDR(0x02AA), 0x55);
	NOR_BWRITE(BYTE_ADDR(0x0555), 0x90);
	NOR_BWRITE(BYTE_ADDR(0x00), 0x01);

	id1 = *(__IO uint8_t *) BYTE_ADDR(0x00);
	id2 = *(__IO uint8_t *) BYTE_ADDR(0x01);
	id3 = *(__IO uint8_t *) BYTE_ADDR(0x0E);
	id4 = *(__IO uint8_t *) BYTE_ADDR(0x0F);

	uiID = ((uint32_t)id1 << 24) | ((uint32_t)id2 << 16)  | ((uint32_t)id3 << 8) | id4;

//	NOR_BWRITE(NOR_FLASH_ADDR, 0xF0);		/* 退出ID模式 */

	return uiID;
}

static void NOR_QuitToReadStatus(void)
{
	NOR_BWRITE(BYTE_ADDR(0x0555), 0xAA);
	NOR_BWRITE(BYTE_ADDR(0x02AA), 0x55);
	NOR_BWRITE(NOR_FLASH_ADDR, 0xF0);
}

static uint8_t NOR_GetStatus(uint32_t Timeout)
{
	uint8_t val1 = 0x00;
	uint8_t val2 = 0x00;
	uint8_t status = NOR_ONGOING;
	uint32_t timeout = Timeout;

	while ((!NOR_IS_BUSY()) && (timeout > 0))
	{
		timeout--;
	}

	/* 等待NOR输出忙信号，高电平时等待，避免NOR的忙信号还未反映过来导致CPU提前认为不忙了  */
	timeout = Timeout;
	while(NOR_IS_BUSY() && (timeout > 0))
	{
		timeout--;
	}

	while ((Timeout != 0x00) && (status != NOR_SUCCESS))
	{
		Timeout--;

		/* Read DQ6 */
		val1 = *(__IO uint8_t *)(NOR_FLASH_ADDR);
		val2 = *(__IO uint8_t *)(NOR_FLASH_ADDR);

		/* If DQ6 did not toggle between the two reads then return NOR_Success */
		if ((val1 & 0x40) == (val2 & 0x40))
		{
			return NOR_SUCCESS;
		}

		/* Read DQ2 */
		if((val1 & 0x20) != 0x20)
		{
			status = NOR_ONGOING;
		}

		val1 = *(__IO uint8_t *)(NOR_FLASH_ADDR);
		val2 = *(__IO uint8_t *)(NOR_FLASH_ADDR);

		if((val1 & 0x40) == (val2 & 0x40))
		{
			return NOR_SUCCESS;
		}
		else if ((val1 & 0x20) == 0x20)
		{
			status = NOR_ERROR;
			NOR_QuitToReadStatus();
		}
	}

	if (Timeout == 0x00)
	{
		status = NOR_TIMEOUT;
		NOR_QuitToReadStatus();
	}

	return (status);
}

uint8_t NOR_EraseChip(void)
{
	NOR_BWRITE(BYTE_ADDR(0x0555), 0xAA);
	NOR_BWRITE(BYTE_ADDR(0x02AA), 0x55);
	NOR_BWRITE(BYTE_ADDR(0x0555), 0x80);
	NOR_BWRITE(BYTE_ADDR(0x0555), 0xAA);
	NOR_BWRITE(BYTE_ADDR(0x02AA), 0x55);
	NOR_BWRITE(BYTE_ADDR(0x0555), 0x10);

	return NOR_GetStatus(ChipErase_Timeout);
}

uint8_t NOR_StartEraseChip(void)
{
	NOR_BWRITE(BYTE_ADDR(0x0555), 0xAA);
	NOR_BWRITE(BYTE_ADDR(0x02AA), 0x55);
	NOR_BWRITE(BYTE_ADDR(0x0555), 0x80);
	NOR_BWRITE(BYTE_ADDR(0x0555), 0xAA);
	NOR_BWRITE(BYTE_ADDR(0x02AA), 0x55);
	NOR_BWRITE(BYTE_ADDR(0x0555), 0x10);
	
	return NOR_GetStatus(1000);
}

uint8_t NOR_EraseSector(uint32_t _uiBlockAddr)
{
	NOR_BWRITE(BYTE_ADDR(0x0555), 0xAA);
	NOR_BWRITE(BYTE_ADDR(0x02AA), 0x55);
	NOR_BWRITE(BYTE_ADDR(0x0555), 0x80);
	NOR_BWRITE(BYTE_ADDR(0x0555), 0xAA);
	NOR_BWRITE(BYTE_ADDR(0x02AA), 0x55);
	NOR_BWRITE((NOR_FLASH_ADDR + _uiBlockAddr), 0x30);

	return (NOR_GetStatus(BlockErase_Timeout));
}

//uint8_t NOR_EraseSector(uint32_t _uiBlockAddr)
//{
//	NOR_BWRITE(BYTE_ADDR(0xAAA), 0xAA);
//	NOR_BWRITE(BYTE_ADDR(0x555), 0x55);
//	NOR_BWRITE(BYTE_ADDR(0xAAA), 0x80);
//	NOR_BWRITE(BYTE_ADDR(0xAAA), 0xAA);
//	NOR_BWRITE(BYTE_ADDR(0x555), 0x55);
//	NOR_BWRITE((NOR_FLASH_ADDR + _uiBlockAddr), 0x30);
//
//	return (NOR_GetStatus(BlockErase_Timeout));
//}

uint8_t NOR_CheckStatus(void)
{
	uint8_t val1 = 0x00;
	uint8_t val2 = 0x00;
	uint8_t status = NOR_ONGOING;
	uint32_t timeout = 10;
	
	while ((timeout != 0x00) && (status != NOR_SUCCESS))
	{
		timeout--;

		/* Read DQ6 */
		val1 = *(__IO uint8_t *)(NOR_FLASH_ADDR);
		val2 = *(__IO uint8_t *)(NOR_FLASH_ADDR);

		/* If DQ6 did not toggle between the two reads then return NOR_Success */
		if ((val1 & 0x40) == (val2 & 0x40))
		{
			return NOR_SUCCESS;
		}

		/* Read DQ2 */
		if((val1 & 0x20) != 0x20)
		{
			status = NOR_ONGOING;
		}

		val1 = *(__IO uint8_t *)(NOR_FLASH_ADDR);
		val2 = *(__IO uint8_t *)(NOR_FLASH_ADDR);

		if((val1 & 0x40) == (val2 & 0x40))
		{
			return NOR_SUCCESS;
		}
		else if ((val1 & 0x20) == 0x20)
		{
			status = NOR_ERROR;
			NOR_QuitToReadStatus();
		}
	}

	if (timeout == 0x00)
	{
		status = NOR_TIMEOUT;
		//NOR_QuitToReadStatus();
	}

	return (status);
}

uint8_t NOR_ReadByte(uint32_t _uiWriteAddr)
{
	uint8_t data;
	NOR_WRITE((NOR_FLASH_ADDR + _uiWriteAddr), 0x00F0 );

	return data = *(uint8_t *)(NOR_FLASH_ADDR + _uiWriteAddr);
}


void NOR_ReadBuffer(uint32_t _uiWriteAddr, uint8_t *_pBuf, uint32_t _uiBytes)
{
	uint8_t *pNor8;
	uint32_t i;

	pNor8 = (uint8_t *)(_uiWriteAddr + NOR_FLASH_ADDR);
	for (i = 0; i < _uiBytes; i++)
	{
		*_pBuf++ = *pNor8++;
	}
}

//uint8_t NOR_WriteByte(uint32_t _uiWriteAddr, uint8_t _usData)
//{
//	NOR_BWRITE(BYTE_ADDR(0x0555), 0xAA);
//	NOR_BWRITE(BYTE_ADDR(0x02AA), 0x55);
//	NOR_BWRITE(BYTE_ADDR(0x0555), 0xA0);
//	NOR_BWRITE(NOR_FLASH_ADDR + _uiWriteAddr, _usData);
//
//	return (NOR_GetStatus(Program_Timeout));
//}

uint8_t NOR_WriteByte(uint32_t _uiWriteAddr, uint8_t _usData)
{
	NOR_BWRITE(BYTE_ADDR(0xAAA), 0xAA);
	NOR_BWRITE(BYTE_ADDR(0x555), 0x55);
	NOR_BWRITE(BYTE_ADDR(0xAAA), 0xA0);
	NOR_BWRITE(NOR_FLASH_ADDR + _uiWriteAddr, _usData);

	return (NOR_GetStatus(Program_Timeout));
}


uint8_t NOR_WriteBuffer(uint32_t _uiWriteAddr, uint8_t *_pBuf, uint32_t _uiBytes)
{
	uint8_t usByte;
	uint32_t i;
	uint8_t ucStatus;

	for (i = 0; i < _uiBytes; i++)
	{
		usByte = *_pBuf++;
		ucStatus = NOR_WriteByte(_uiWriteAddr, usByte);
		if (ucStatus != NOR_SUCCESS)
		{
			return 	ucStatus;
		}

		_uiWriteAddr ++;
	}

	return 	ucStatus;
}

uint8_t flash_program(uint32_t addr, uint8_t * data, uint32_t len, uint8_t erase) {
	if(erase) {
		if(NOR_EraseSector(addr) != NOR_SUCCESS) {
			return NOR_ERROR;
		}

		if(NOR_WriteBuffer(addr, data, len) != NOR_SUCCESS) {
			return NOR_ERROR;
		}
	}else {
		if(NOR_WriteBuffer(addr, data, len) != NOR_SUCCESS) {
			return NOR_ERROR;
		}
	}

	return NOR_SUCCESS;
}
