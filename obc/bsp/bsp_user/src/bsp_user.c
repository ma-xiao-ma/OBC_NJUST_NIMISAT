#include <bsp_user.h>

enum TEST_STATUS GT_TestState=TEST_IDLE;  //地面测试状态初始化
unsigned char GT_TestIndex;

char GT_RcvEndFlag=0; //地面测试结束标志
char GT_RcvErr=0;

//产生CRC校验码
uint32_t CalCrc(char *c, int len)
{
	register unsigned int crc;
	char     *e = c + len;

  crc = 0xFFFFFFFF;
  while (c < e) 
	{
		crc = ((crc >> 8) & 0x00FFFFFF) ^ crcTable[ (crc^ *c) & 0xFF ];
		++c;
	}
	return( crc^0xFFFFFFFF );
}


char Dec2Ascii(char val)
{
	if(val <= 0x09)
		return (0x30+val);
	else if((val >= 0x0A)&(val <= 0x0F))
		return (0x37+val);
	else
		return 0;
}


uint8_t StrCvt(char *InsBuf, char *StrBuf, uint16_t BufSize)
{
	uint16_t i;
	
	for(i=0; i<BufSize; i++)
	{
		*StrBuf++ = Dec2Ascii((*InsBuf>>4) & 0x0F);
		*StrBuf++ = Dec2Ascii((*InsBuf) & 0x0F);
		InsBuf++;
	}
	return 0;
}

uint32_t GetCrc(char *data, int BufSize)
{
	char *StrBuf;
	uint32_t Temp1;
	
	StrBuf=malloc(2*BufSize+2);
	memset(StrBuf,0,2*BufSize);
	StrCvt(data, StrBuf, BufSize);
	Temp1 = CalCrc(StrBuf, 2*BufSize);
	free(StrBuf);
	
	return Temp1;
}


uint8_t StrCopy(char *src, char *des, uint16_t size)
{
	uint16_t i;
	if(size == 0)
	{
		return 1;
	}	
	else
	{
		for(i=0; i<size; i++)
		{
			*des++ = *src++;
		}
		return 0;
	}
}



uint16_t GetCheckSum(uint16_t *Ptr, uint8_t BufSize)
{
	uint32_t CheckSumTemp;
	uint16_t index;
	
	CheckSumTemp = 0x00000000;
	
	for(index=0; index<BufSize; index++)
		CheckSumTemp+=*(Ptr+index);
	
	return (uint16_t)CheckSumTemp;
}


