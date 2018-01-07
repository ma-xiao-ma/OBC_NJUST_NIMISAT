/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2007        */
/*-----------------------------------------------------------------------*/
/* This is a stub disk I/O module that acts as front end of the existing */
/* disk I/O modules and attach it to FatFs module with common interface. */
/*-----------------------------------------------------------------------*/
#include "string.h"


#include "diskio.h"
#include "stm32f4xx.h"
#include "bsp_sdio_sd.h"
#include "bsp_ds1302.h"

#define BLOCK_SIZE            512 /* Block Size in Bytes */



/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */

DSTATUS disk_initialize (
    BYTE drv                /* Physical drive nmuber (0..) */
)
{
    SD_Error  Status;
    /* Supports only single drive */
    if (drv)
    {
        return STA_NOINIT;
    }
/*-------------------------- SD Init ----------------------------- */
    /** SD卡相关中断配置*/
    SD_NVIC_Configuration();

    SD_DeInit();

    Status = SD_Init();

    if (Status!= SD_OK )
    {
        return STA_NOINIT;
    }
    else
    {
        return RES_OK;
    }

}



/*-----------------------------------------------------------------------*/
/* Return Disk Status                                                    */

DSTATUS disk_status (
    BYTE drv        /* Physical drive nmuber (0..) */
)
{
    return RES_OK;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */

DRESULT disk_read (
    BYTE drv,       /* Physical drive nmuber (0..) */
    BYTE *buff,     /* Data buffer to store read data */
    DWORD sector,   /* Sector address (LBA) */
    BYTE count      /* Number of sectors to read (1..255) */
)
{
//    SD_Error Status = SD_OK;
//
//    if ((DWORD)buff & 3) {
//        DRESULT res = RES_OK;
//        DWORD scratch[BLOCK_SIZE / 4] __attribute__(( aligned(4) ));
//
//        while (count--) {
//            res = disk_read(0, (void *)scratch, sector++, 1);
//
//            if (res != RES_OK) {
//                break;
//            }
//
//            memcpy(buff, scratch, BLOCK_SIZE);
//
//            buff += BLOCK_SIZE;
//        }
//
//        return res;
//    }
//
//    Status = SD_ReadMultiBlocks(buff, sector << 9, BLOCK_SIZE, count);
//
//    if (Status == SD_OK) {
//        SDTransferState State;
//
//        Status = SD_WaitReadOperation();
//
//        while ((State = SD_GetStatus()) == SD_TRANSFER_BUSY);
//
//        if ((State == SD_TRANSFER_ERROR) || (Status != SD_OK)) {
//            return RES_ERROR;
//        } else {
//            return RES_OK;
//        }
//    } else {
//        return RES_ERROR;
//    }

  unsigned int num = 20000;

  if (count > 1)
  {
      SD_ReadMultiBlocks(buff, sector*BLOCK_SIZE, BLOCK_SIZE, count);

            /* Check if the Transfer is finished */
      SD_WaitReadOperation();  //ѭ����ѯdma�����Ƿ����

      /* Wait until end of DMA transfer */
      while(SD_GetStatus() != SD_TRANSFER_OK && num != 0) {
          num--;
      }
      if(num == 0){
          return SD_TRANSFER_ERROR;
      }

  }
  else
  {

      SD_ReadBlock(buff, sector*BLOCK_SIZE, BLOCK_SIZE);

            /* Check if the Transfer is finished */
      SD_WaitReadOperation();  //ѭ����ѯdma�����Ƿ����

      /* Wait until end of DMA transfer */
//        while(SD_GetStatus() != SD_TRANSFER_OK);
      while(SD_GetStatus() != SD_TRANSFER_OK && num != 0) {
          num--;
      }
      if(num == 0){
          return SD_TRANSFER_ERROR;
      }

  }
  return RES_OK;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */

#if _READONLY == 0
DRESULT disk_write (
    BYTE drv,           /* Physical drive nmuber (0..) */
    const BYTE *buff,   /* Data to be written */
    DWORD sector,       /* Sector address (LBA) */
    BYTE count          /* Number of sectors to write (1..255) */
)
{
//    SD_Error Status = SD_OK;
//
//    if ((DWORD)buff & 3) {
//        DRESULT res = RES_OK;
//        DWORD scratch[BLOCK_SIZE / 4] __attribute__(( aligned(4) ));
//
//        while (count--) {
//            memcpy(scratch, buff, BLOCK_SIZE);
//            res = disk_write(0, (void *)scratch, sector++, 1);
//
//            if (res != RES_OK) {
//                break;
//            }
//
//            buff += BLOCK_SIZE;
//        }
//
//        return(res);
//    }
//
//    Status = SD_WriteMultiBlocks((uint8_t *)buff, sector << 9, BLOCK_SIZE, count); // 4GB Compliant
//
//    if (Status == SD_OK) {
//        SDTransferState State;
//
//        Status = SD_WaitWriteOperation(); // Check if the Transfer is finished
//
//        while ((State = SD_GetStatus()) == SD_TRANSFER_BUSY); // BUSY, OK (DONE), ERROR (FAIL)
//
//        if ((State == SD_TRANSFER_ERROR) || (Status != SD_OK)) {
//            return RES_ERROR;
//        } else {
//            return RES_OK;
//        }
//    } else {
//        return RES_ERROR;
//    }

  unsigned int num = 20000;

  if (count > 1)
  {
      SD_WriteMultiBlocks((uint8_t *)buff, sector*BLOCK_SIZE, BLOCK_SIZE, count);

        /* Check if the Transfer is finished */
      SD_WaitWriteOperation();       //�ȴ�dma�������
//        while(SD_GetStatus() != SD_TRANSFER_OK); //�ȴ�sdio��sd���������
      while(SD_GetStatus() != SD_TRANSFER_OK && num != 0) {
          num--;
      }
      if(num == 0){
          return SD_TRANSFER_ERROR;
      }
  }
  else
  {
      SD_WriteBlock((uint8_t *)buff,sector*BLOCK_SIZE, BLOCK_SIZE);

        /* Check if the Transfer is finished */
        SD_WaitWriteOperation();     //�ȴ�dma�������
//        while(SD_GetStatus() != SD_TRANSFER_OK); //�ȴ�sdio��sd���������
      while(SD_GetStatus() != SD_TRANSFER_OK && num != 0) {
          num--;
      }
      if(num == 0){
          return SD_TRANSFER_ERROR;
      }
  }
  return RES_OK;
}
#endif /* _READONLY */




/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */

DRESULT disk_ioctl (
                        BYTE drv,       /* Physical drive nmuber (0..) */
                        BYTE ctrl,      /* Control code */
                        void *buff )    /* Buffer to send/receive control data */
{
    return RES_OK;
}

/*-----------------------------------------------------------------------*/
/* Get current time                                                      */
/*-----------------------------------------------------------------------*/
DWORD get_fattime(void)
{
    /* Returns current time packed into a DWORD variable */
    return    ((DWORD)(2017 - 1980) << 25)  /* Year 2017 */
            | ((DWORD)12 << 21)              /* Month 12 */
            | ((DWORD)11 << 16)             /* Mday 11 */
            | ((DWORD)22 << 11)              /* Hour 22 */
            | ((DWORD)2 << 5)               /* Min 2 */
            | ((DWORD)0 >> 1);              /* Sec 0 */

//  return clock_get_time_nopara();
}
