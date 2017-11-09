/*
 * file_opt.c
 *
 *  Created on: 2017年10月25日
 *      Author: Ma Wenli
 */
#include "ff.h"
#include "obc_mem.h"

#include "file_opt.h"

/**
 * 用户写文件函数
 *
 * @param path 待写文件路径
 * @param buff 待写入数据起始指针
 * @param w_num 待写入数据长度
 * @return 返回FR_OK（0）表示成功，其他数值为错误状态，详细参见ff.h
 */
FRESULT file_write( const TCHAR *wpath, const void *wbuff, UINT w_num )
{
    FRESULT result;
    UINT nbyte_written;

    FIL *file_handle = ( FIL *)ObcMemMalloc( sizeof(FIL) );
    if ( file_handle == NULL )
        return FR_INT_ERR;

    /** 打开或者创建文件，会覆盖已存在的同名文件 */
    result = f_open( file_handle, wpath, FA_WRITE|FA_CREATE_ALWAYS );

    if (result != FR_OK)
    {
        ObcMemFree(file_handle);
        return result;
    }

    /** 给一打开的文件写入数据 */
    result = f_write( file_handle, wbuff, w_num, &nbyte_written );

    if ( nbyte_written != w_num )
    {
        f_close( file_handle );
        f_unlink( wpath );
        ObcMemFree(file_handle);
        return result;
    }

    f_close( file_handle );
    ObcMemFree(file_handle);
    return FR_OK;
}

/**
 * 用户读文件函数
 *
 * @param path 待读文件路径
 * @param buff 读缓冲区起始指针
 * @param r_num 待读数据长度
 * @param ofs 读指针偏移
 * @return 返回FR_OK（0）表示成功，其他数值为错误状态，详细参见ff.h
 */
FRESULT file_read( const TCHAR *rpath, void *rbuff, UINT r_num, DWORD ofs )
{
    FRESULT result;
    UINT nbyte_read;

    FIL *file_handle = (FIL *)ObcMemMalloc( sizeof(FIL) );
    if(file_handle == NULL)
        return FR_INT_ERR;

    result = f_open( file_handle, rpath, FA_READ );

    if ( result != FR_OK )
    {
        ObcMemFree(file_handle);
        return result;
    }

    if ( ofs > f_size( file_handle ) )
    {
        f_close( file_handle );
        ObcMemFree(file_handle);
        return FR_INVALID_PARAMETER;
    }

    result = f_lseek ( file_handle, ofs );

    if ( result != FR_OK )
    {
        f_close( file_handle );
        ObcMemFree(file_handle);
        return result;
    }

    /* 读出数据 */
    result = f_read( file_handle, rbuff, r_num, &nbyte_read );

    if (nbyte_read != r_num)
    {
        f_close( file_handle );
        ObcMemFree(file_handle);
        return result;
    }

    f_close( file_handle );
    ObcMemFree(file_handle);

    return FR_OK;
}


