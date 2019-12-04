/*
 * MT25QL128ABA.h
 *
 *  Created on: 5 Oct 2019
 *      Author: Arion
 */

#ifndef MT25QL128ABA_H_
#define MT25QL128ABA_H_


#define SECTOR_SIZE 1 << 16
#define PAGE_SIZE 256
#define READ_MAX_BUFFER 2048

// State commands
#define READ_STATUS_REGISTER 0x05
#define READ_FLAG_STATUS_REGISTER 0x70


// Write latch commands
#define WRITE_ENABLE_LATCH 0x06
#define WRITE_DISABLE_LATCH 0x04


// Read commands
#define READ_SINGLE 0x03
#define FREAD_SINGLE 0x0B
#define FREAD_DUAL 0xBB


// Write commands
#define WRITE_SINGLE 0x02
#define FWRITE_DUAL 0xA2
#define FWRITE_DUAL_EXT 0xD2


// Erase commands
#define ERASE_SUBSECTOR 0x20 // 4KB subsector
#define ERASE_SECTOR 0xD8
#define ERASE_ALL 0x60



#endif /* MT25QL128ABA_H_ */
