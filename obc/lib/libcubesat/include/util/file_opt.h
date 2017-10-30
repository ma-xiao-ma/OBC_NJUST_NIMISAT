/*
 * file_opt.h
 *
 *  Created on: 2017年10月25日
 *      Author: Ma Wenli
 */

#ifndef LIB_LIBCUBESAT_INCLUDE_UTIL_FILE_OPT_H_
#define LIB_LIBCUBESAT_INCLUDE_UTIL_FILE_OPT_H_


/**
 * 用户写文件函数
 *
 * @param path 待写文件路径
 * @param buff 待写入数据起始指针
 * @param w_num 待写入数据长度
 * @return 返回FR_OK（0）表示成功，其他数值为错误状态，详细参见ff.h
 */
FRESULT file_write( const TCHAR *wpath, const void *wbuff, UINT w_num );


/**
 * 用户读文件函数
 *
 * @param path 待读文件路径
 * @param buff 读缓冲区起始指针
 * @param r_num 待读数据长度
 * @param ofs 读指针偏移
 * @return 返回FR_OK（0）表示成功，其他数值为错误状态，详细参见ff.h
 */
FRESULT file_read( const TCHAR *rpath, void *rbuff, UINT r_num, DWORD ofs );


#endif /* LIB_LIBCUBESAT_INCLUDE_UTIL_FILE_OPT_H_ */
