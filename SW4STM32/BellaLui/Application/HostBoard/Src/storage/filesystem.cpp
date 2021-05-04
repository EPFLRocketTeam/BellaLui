/*
 * filesystem.c
 *
 *  Created on: 14 Oct 2019
 *      Author: Arion
 */

#include "filesystem.h"

#include "block_management.h"
#include "file.h"
#include "stream.h"

/*
 * FileSystem structure
 *
 * Each 4KB subsector is called a 'block'.
 *
 * Block 0: Core block
 * 		2KB: RocketFS heuristic magic number
 * 		2KB: Metadata
 * Block 1: Master partition (bit 0...3: FileType, 4...7: relative initialisation time)
 * Block 2: Recovery partition
 * Block 3: Backup slot 1
 * Block 4: Backup slot 2
 * Block 5: Backup slot 3
 * Block 6: Backup slot 4
 * Block 7: Journal
 *
 * Block 8: Data
 * ...
 * Block 4091: Data
 *
 * Block 4092: Reserved
 * Block 4093: Reserved
 * Block 4094: Reserved (INVASIVE_TEST for flash memory)
 * Block 4095: Reserved (GENTLE_TEST for flash memory)
 */

#define CORRUPTION_THRESHOLD 4
#define MAGIC_PERIOD 7
#define BACKUP_MAGIC 0xC0FFEE

/*
 * Utility functions
 */
static void fs_check_mounted(FileSystem *fs);

static uint8_t __clamp(uint8_t input, uint8_t start, uint8_t end);
static uint64_t __signed_shift(int64_t input, int8_t amount);
static uint64_t __generate_periodic(uint8_t period);
static bool __periodic_magic_match(uint8_t period, uint64_t testable_magic);
static void __no_log(const char* _);

/*
 * FileSystem functions
 */
void rocket_fs_debug(FileSystem* fs, void (*logger)(const char*)) {
	fs->log = logger;
	fs->debug = true;

	fs->log("FileSystem log initialised.");
}

void rocket_fs_bind(
	FileSystem* fs,
	void (*read)(uint32_t, uint8_t*, uint32_t),
	void (*write)(uint32_t, uint8_t*, uint32_t),
	void (*erase_block)(uint32_t)
) {
	fs->read = read;
	fs->write = write;
	fs->erase_block = erase_block;
	fs->io_bound = true;

   if(!fs->debug) {
      fs->log = &__no_log;
   }
}

void rocket_fs_device(FileSystem* fs, const char *id, uint32_t capacity, uint32_t block_size) {
	if(block_size < NUM_BLOCKS) {
		fs->log("Fatal: Device's sub-sector granularity is too high. Consider using using a device with higher block_size.");
	} else if(block_size * NUM_BLOCKS > capacity) {
		fs->log("Fatal: Device's sub-sector granularity is too low. Consider using using a device with lower block_size.");
	} else {
		fs->id = id;
		fs->addressable_space = capacity;
		fs->block_size = block_size;
		fs->device_configured = true;
		fs->partition_table_modified = false;
	}
}

void rocket_fs_mount(FileSystem* fs) {
	fs->log("Mounting filesystem...");

	if(fs->mounted) {
		fs->log("Error: FileSystem already mounted.");
		return;
	}

	uint32_t core_base = rfs_get_block_base_address(fs, 0);
	uint32_t master_base = rfs_get_block_base_address(fs, 1);

	Stream stream;
	init_stream(&stream, fs, core_base, RAW);

	uint64_t magic = stream.read64();
	stream.close();

	if(__periodic_magic_match(MAGIC_PERIOD, magic)) {
		init_stream(&stream, fs, master_base, RAW);

		fs->log("Reading partition table...");

		stream.read(fs->reverse_partition_table, NUM_BLOCKS);
		stream.close();

		for(uint32_t i = 0; i < NUM_BLOCKS; i++) {
			// Reverse bits to increase the lifetime of NOR flash memories (do not do this if the targeted device is a NAND flash).
			fs->partition_table[i] = ~fs->reverse_partition_table[i];
		}

		rfs_init_block_management(fs); // in block_management.c

		fs->mounted = true;

		fs->log("Filesystem mounted.");
	} else {
		// TODO (a) Check redundant magic number.
		// TODO (b) Write corrupted partition to a free backup slot.
		fs->log("Mounting filesystem for the first time or filesystem corrupted.");

		rocket_fs_format(fs);

		rocket_fs_mount(fs);
	}
}

void rocket_fs_unmount(FileSystem* fs) {
	fs->log("Unmounting FileSystem...");

	fs_check_mounted(fs);
	rocket_fs_flush(fs);
	fs->mounted = false;

	fs->log("FileSystem unmounted.");
}


void rocket_fs_format(FileSystem* fs) {
	fs->log("Formatting FileSystem...");

	uint32_t core_base = rfs_get_block_base_address(fs, 0);
	uint32_t master_base = rfs_get_block_base_address(fs, 1);

	fs->erase_block(core_base);   // Core block
	fs->erase_block(master_base); // Master partition block


	Stream stream;
	init_stream(&stream, fs, master_base, RAW);

	/*
	 * Blocks 0 to 7 are reserved anyways
	 */

	stream.write8(~0b00001110); // Core block (used as internal relative clock)
	stream.write8(~0b00001111); // Master partition block
	stream.write8(~0b00001111); // Recovery partition block
	stream.write8(~0b00001111); // Backup partition block 1
	stream.write8(~0b00001111); // Backup partition block 2
	stream.write8(~0b00001111); // Backup partition block 3
	stream.write8(~0b00001111); // Backup partition block 4
	stream.write8(~0b00001111); // Journal block

	stream.close();

	rfs_block_write_header(fs, 0, 0, 0);
	rfs_block_write_header(fs, 1, 0, 0);
	rfs_block_write_header(fs, 2, 0, 0);
	rfs_block_write_header(fs, 3, 0, 0);
	rfs_block_write_header(fs, 4, 0, 0);
	rfs_block_write_header(fs, 5, 0, 0);
	rfs_block_write_header(fs, 6, 0, 0);
	rfs_block_write_header(fs, 7, 0, 0);

	init_stream(&stream, fs, core_base, RAW);

	uint64_t magic = __generate_periodic(MAGIC_PERIOD);
	stream.write64(magic);

	/*
	 * ... write heuristic magic number and metadata
	 */

	stream.close();

	fs->total_used_blocks = 8;

	fs->log("FileSystem formatted.");
}

/*
 * Flushes the partition table
 */
void rocket_fs_flush(FileSystem* fs) {
	if(fs->mounted && fs->partition_table_modified) {
		fs->log("Flushing partition table...");

		fs->partition_table_modified = false;

		fs->erase_block(fs->block_size); // Erase the master partition block

		fs->log("Master partition block erased.");

		rfs_block_write_header(fs, 1, 0, 0);

		uint32_t master_base = rfs_get_block_base_address(fs, 1);

		Stream stream;
		init_stream(&stream, fs, master_base, RAW);

		for(uint32_t i = 0; i < NUM_BLOCKS; i++) {
			// Reverse bits to increase the lifetime of NOR flash memories (do not do this if the targeted device is a NAND flash).
		   fs->reverse_partition_table[i] = ~fs->partition_table[i];
		}

		fs->log("Partition data encoded.");

		stream.write(fs->reverse_partition_table, NUM_BLOCKS);
		stream.close();

		fs->log("Partition table flushed.");
	}
}

/*
 * Names at most 15 characters long.
 * Storing file names in a hashtable.
 */
File* rocket_fs_newfile(FileSystem* fs, const char* name, FileType type) {
   static char filename[16];

   fs_check_mounted(fs);

	fs->log("Creating new file...");

	filename_copy(name, filename);

	File* file;
	uint32_t hash = hash_filename(filename);
	uint8_t bucket = hash % NUM_FILES;

	for(uint8_t file_id = bucket; file_id < bucket + NUM_FILES; file_id++) {
		file = &(fs->files[file_id % NUM_FILES]);

		if(filename_equals(file->filename, name)) {
			fs->log("File with the given filename already exists:");
			fs->log(name);
			return 0;
		}

		if(file->first_block == 0) {
			// Yey! We found an available file identifier

			uint16_t first_block_id = rfs_block_alloc(fs, type);
			rfs_block_write_header(fs, first_block_id, file_id, 0);
			rfs_set_file_root(fs, first_block_id);

			uint32_t address = rfs_get_block_base_address(fs, first_block_id);

			fs->write(address, (uint8_t*) filename, 16); // Write the filename

			filename_copy(filename, file->filename);
			file->hash = hash;
			file->first_block = first_block_id;
			file->last_block = first_block_id;
			file->used_blocks = 1;
			file->length = 0;

			rocket_fs_flush(fs);

			fs->log("File created.");

			return file;
		}
	}

	fs->log("Maximal number of files reached.");
	return 0;
}

void rocket_fs_delfile(FileSystem* fs, File* file) {
	fs_check_mounted(fs);

	fs->log("Deleting file...");

	uint16_t block_id = file->first_block;
	DataBlock* block;

	if(block_id) {
		do {
			rfs_block_free(fs, block_id);

			block = &(fs->data_blocks[block_id]);

			block_id = block->successor;
			block->successor = 0;
		} while(block_id);

		file->hash = 0;
		file->first_block = 0;
		file->last_block = 0;
		file->length = 0;
		file->used_blocks = 0;

		rocket_fs_flush(fs);

		fs->log("File deleted.");
	} else {
		fs->log("File does not exist.");
	}
}

File* rocket_fs_getfile(FileSystem* fs, const char* name) {
	fs_check_mounted(fs);

	char filename[16] = { 0 };
	filename_copy(name, filename);

	File* file;
	uint32_t hash = hash_filename(filename);
	uint8_t bucket = hash % NUM_FILES;

	for(uint8_t file_id = bucket; file_id < bucket + NUM_FILES; file_id++) {
		file = &(fs->files[file_id % NUM_FILES]);

		if(file->first_block && filename_equals(file->filename, filename)) {
		   //rfs_load_file_meta(fs, file);
			return file;
		}
	}

	fs->log("File with the given filename was not found in the filesystem:");
	fs->log(filename);

	return 0;
}

bool rocket_fs_touch(FileSystem* fs, File* file) {
	fs_check_mounted(fs);

	rfs_load_file_meta(fs, file);

	return true;
}


bool rocket_fs_stream(Stream* stream, FileSystem* fs, File* file, StreamMode mode) {
	fs_check_mounted(fs);

	switch(mode) {
	case OVERWRITE: {
		uint16_t first_block = file->first_block;
		uint32_t base_address = rfs_get_block_base_address(fs, first_block) + 16; // Do not overwrite the 16-characters long identifier

		FileType type = static_cast<FileType>(fs->partition_table[first_block] >> 4);

		return init_stream(stream, fs, base_address, type);
	}

	case APPEND: {
		uint16_t last_block = file->last_block;
		uint32_t base_address = last_block * fs->block_size + rfs_compute_block_length(fs, last_block);

		FileType type = static_cast<FileType>(fs->partition_table[last_block] >> 4);

		return init_stream(stream, fs, base_address, type);
	}

	default:
		fs->log("Unsupported stream mode");
		return false;
	}

	return true;
}


static void fs_check_mounted(FileSystem *fs) {
	if(!fs->mounted) {
		fs->log("Error: FileSystem not mounted");
	}
}


/*
 * Corruption utility functions
 */

/*
 * Use Gaussian filter on binary magic number to remove impulsional noise.
 * Decompose magic signal in wave form and compare it with the given periodicity
 * (we could use Fourier transforms but that would probably be overkill for embedded systems).
 * Sigma: 1.5
 */

static uint8_t __clamp(uint8_t input, uint8_t start, uint8_t end) {
	if(input <= start) {
		return start;
	} else if(input >= end) {
		return end;
	} else {
		return input;
	}
}

static uint64_t __signed_shift(int64_t input, int8_t amount) {
	if(amount > 0) {
		return input >> amount;
	} else if(amount < 0) {
		return input << (-amount);
	} else {
		return input;
	}
}

static uint64_t __generate_periodic(uint8_t period) {
	uint64_t periodic = 0;
	uint64_t period_generator = (-1ULL) >> (64 - period / 2);

	for(uint8_t i = 0; i < 64; i += period) {
		periodic <<= period;
		periodic |= period_generator;
	}

	return periodic;
}

static bool __periodic_magic_match(uint8_t period, uint64_t testable_magic) {
	static const uint16_t __gaussian_kernel[] = { 614, 2447, 3877, 2447, 614 };
	static const uint16_t __gaussian_divider[] = { 3470, 4693, 5000 };

	uint8_t i, j;
	uint64_t weighted = 0;
	uint64_t hard_coded_magic = __generate_periodic(period);

	uint64_t filtered = 0;

	for(i = 0; i < 64; i++) {
		weighted = 0;
		filtered <<= 1;

		for(j = 0; j < 5; j++) {
			weighted += __gaussian_kernel[j] * (__signed_shift(testable_magic, 65 - i - j) & 0b1);
		}

		if(i < 32) {
			filtered |=  weighted / __gaussian_divider[__clamp(i, 0, 2)];
		} else {
			filtered |= weighted / __gaussian_divider[__clamp(64 - i, 0, 2)];
		}
	}

	uint8_t delta = 0;

	filtered ^= hard_coded_magic;

	for(uint8_t i = 0; i < 64; i++) {
		delta += (filtered >> i) & 0b1;
	}

	return delta < CORRUPTION_THRESHOLD;
}

static void __no_log(const char* _) {}
