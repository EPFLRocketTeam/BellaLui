/*
 * filesystem.h
 *
 *  Created on: 14 Oct 2019
 *      Author: Arion
 */

#ifndef INC_FILESYSTEM_H_
#define INC_FILESYSTEM_H_

#include <stdbool.h>
#include <stdint.h>

#include "file.h"


/*
 * FS-specific defines
 */
#define NUM_BLOCKS 4080
#define NUM_FILES 64
#define PROTECTED_BLOCKS 8



typedef struct Stream {
	void (*close)();

	void     (*read)(uint8_t* buffer, uint32_t length);
	uint8_t  (*read8)();
	uint16_t (*read16)();
	uint32_t (*read32)();
	uint64_t (*read64)();

	void (*write)(uint8_t* buffer, uint32_t length);
	void (*write8)(uint8_t data);
	void (*write16)(uint16_t data);
	void (*write32)(uint32_t data);
	void (*write64)(uint64_t data);
} Stream;



/*
 * Since stm32f446 only has 128KB memory, we cannot afford more than 16-bytes data-blocks...
 */
typedef struct DataBlock {
	uint16_t successor;
} DataBlock;


typedef struct FileSystem {
	bool device_configured;
	bool io_bound;
	bool mounted;
	bool debug;

	const char *id;
	uint32_t addressable_space;
	uint32_t block_size;

	uint32_t total_used_blocks;
	uint8_t partition_table[NUM_BLOCKS];
	bool partition_table_modified;
	DataBlock data_blocks[NUM_BLOCKS];
	File files[NUM_FILES];

	void (*read)(uint32_t address, uint8_t* buffer, uint32_t length);
	void (*write)(uint32_t address, uint8_t* buffer, uint32_t length);
	void (*erase_block)(uint32_t address);
	void (*erase_sector)(uint32_t address);

	void (*log)(const char*);
} FileSystem;

typedef enum StreamMode { OVERWRITE, APPEND } StreamMode;


void rocket_fs_debug(FileSystem* fs, void (*logger)(const char*));
void rocket_fs_device(FileSystem* fs, const char *id, uint32_t capacity, uint32_t block_size);

void rocket_fs_bind(
	FileSystem* fs,
	void (*read)(uint32_t, uint8_t*, uint32_t),
	void (*write)(uint32_t, uint8_t*, uint32_t),
	void (*erase_block)(uint32_t)
);

void rocket_fs_mount(FileSystem* fs);
void rocket_fs_unmount(FileSystem* fs);
void rocket_fs_format(FileSystem* fs);
void rocket_fs_flush(FileSystem* fs); // Flushes the partition table
File* rocket_fs_newfile(FileSystem* fs, const char* name, FileType type);
void rocket_fs_delfile(FileSystem* fs, File* file);
File* rocket_fs_getfile(FileSystem* fs, const char* name);
bool rocket_fs_stream(Stream* stream, FileSystem* fs, File* file, StreamMode mode);



#endif /* INC_FILESYSTEM_H_ */
