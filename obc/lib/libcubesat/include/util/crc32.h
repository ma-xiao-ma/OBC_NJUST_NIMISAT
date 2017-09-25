////////////////////////////////////////////////////////////////////////////////
//	功能： CRC校验头文件
//
//	版本：V1.0
//  迭代：
//												南京理工大学微纳卫星中心
//												   2016.01.06
////////////////////////////////////////////////////////////////////////////////

#ifndef __CRC32_H
#define __CRC32_H

/**
 * Calculate single step of crc32
 */
uint32_t chksum_crc32_step(uint32_t crc, uint8_t byte);

/**
 * Caluclate crc32 of a block
 */
uint32_t chksum_crc32(uint8_t *block, unsigned int length);

#endif /* __CRC32_H */
