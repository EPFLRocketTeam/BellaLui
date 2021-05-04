/*
 * block_management.h
 *
 *  Created on: 17 Oct 2019
 *      Author: Arion
 */

#ifndef INC_BLOCK_MANAGEMENT_H_
#define INC_BLOCK_MANAGEMENT_H_

#include <stdint.h>
#include <stdbool.h>

#include "filesystem.h"


typedef enum AccessType { READ, WRITE } AccessType;

void rfs_init_block_management(FileSystem* fs);

uint16_t rfs_block_alloc(FileSystem* fs, FileType type);
void rfs_block_free(FileSystem* fs, uint16_t block_id);
void rfs_block_write_header(FileSystem* fs, uint16_t block_id, uint16_t file_id, uint16_t predecessor);

int32_t rfs_access_memory(FileSystem* fs, uint32_t* address, uint32_t length, AccessType access_type);

uint16_t rfs_load_file_meta(FileSystem* fs, File* file);
void rfs_set_file_root(FileSystem* fs, uint16_t block_id);
uint32_t rfs_compute_block_length(FileSystem* fs, uint16_t block_id);
uint32_t rfs_get_block_base_address(FileSystem* fs, uint16_t block_id);

#endif /* INC_BLOCK_MANAGEMENT_H_ */
