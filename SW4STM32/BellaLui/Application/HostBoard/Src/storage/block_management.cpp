/*
 * block_management.c
 *
 *  Created on: 17 Oct 2019
 *      Author: Arion
 */

#include "block_management.h"

#include "file.h"
#include "rocket_fs.h"
#include "stream.h"


#define BLOCK_MAGIC_NUMBER 0xC0FFEE00
#define BLOCK_HEADER_SIZE 16
/*
 * 0...3:  Magic number
 * 4...5:  Related file ID
 * 6...7:  Predecessor block ID
 * 8...15: Usage table
 */

/*
 * Non-exported function prototypes
 */
static void rfs_block_update_usage_table(FileSystem* fs, uint32_t write_begin, uint32_t write_end);

static void rfs_update_relative_time(FileSystem* fs);
static void rfs_decrease_relative_time(FileSystem* fs);

static uint32_t __compute_block_length(FileSystem* fs, uint64_t usage_table);



static bool allow_unsafe_access = false;


void rfs_init_block_management(FileSystem* fs) {
	static char identifier[16];
	Stream stream;
	File* selected_file;

	/*
	 * First pass: Detect all files.
	 */
	fs->log("Detecting files...");

	for(uint16_t block_id = PROTECTED_BLOCKS; block_id < NUM_BLOCKS; block_id++) {
		uint8_t meta_data = fs->partition_table[block_id];

		if(meta_data) {
			uint32_t address = fs->block_size * block_id;

			allow_unsafe_access = true;
			init_stream(&stream, fs, address, RAW);

			uint32_t magic = stream.read32();
			uint16_t file_id = stream.read16();
			uint16_t predecessor = stream.read16();

			stream.read64(); // Skip the usage table

			if(magic == BLOCK_MAGIC_NUMBER) {
				if(predecessor) {
					// Normal block detected
					fs->data_blocks[predecessor].successor = block_id;
				} else if((meta_data & 0b11110000) != 0b11110000) {
					// File detected
					stream.read((uint8_t*) identifier, 16);

					fs->log(identifier);

					selected_file = &(fs->files[file_id]);

					uint32_t hash = hash_filename(identifier);

					selected_file->first_block = block_id;
					filename_copy(identifier, selected_file->filename);
					selected_file->hash = hash;
					selected_file->used_blocks = 0;
					selected_file->length = 0;
				} else {
					// Lost block detected
					fs->log("Lost block recovered");

					selected_file = &(fs->files[file_id]);
					fs->data_blocks[selected_file->first_block].successor = block_id;
				}
			} else {
				fs->log("Warning: Invalid magic number. Ignoring block");
			}

			stream.close();
			allow_unsafe_access = false;
		}
	}

	/*
	 * Second pass: Resolve all block links and compute storage statistics.
	 * Warning: This function traverses all blocks and does not check for cyclicity.
	 * This may cause infinite loops in extreme data corruption cases. TODO: Check cyclicity.
	 */
	fs->log("Resolving block hierarchy...");

	for(uint8_t file_id = 0; file_id < NUM_FILES; file_id++) {
		selected_file = &(fs->files[file_id]);
		uint32_t used_blocks = rfs_load_file_meta(fs, selected_file);
		fs->total_used_blocks += used_blocks;
	}
}

/*
 * Storage allocation functions
 */

/*
 * Returns the allocated block ID
 *
 * No block header is written.
 * Only the partition table is modified.
 */
uint16_t rfs_block_alloc(FileSystem* fs, FileType type) {
	uint16_t oldest_block_id = PROTECTED_BLOCKS;
   uint16_t oldest_block_age = 0xF;

	for(uint16_t block_id = PROTECTED_BLOCKS; block_id < NUM_BLOCKS; block_id++) {
		uint8_t* meta = &(fs->partition_table[block_id]);
		uint8_t age = (*meta) & 0xF;

		if(*meta == 0) {
			// We found a free block!

			fs->total_used_blocks++;

			*meta = (type << 4) | 0b1100;

			uint32_t address = rfs_get_block_base_address(fs, 1) + block_id;
		    fs->write(address, meta, 8);

			fs->partition_table_modified = true;

			fs->data_blocks[block_id].successor = 0;

			rfs_update_relative_time(fs);

			fs->erase_block(fs->block_size * block_id); // Prepare for writing

			return block_id;
		} else if(age < oldest_block_age){
		   oldest_block_id = block_id;
		   oldest_block_age = age;
		}
	}

	/* Device is full! Realloc oldest block. */

	if(oldest_block_age > 0) { // Some correction for a better relative time repartition
		rfs_decrease_relative_time(fs);
	}

    fs->partition_table[oldest_block_id] = (type << 4) | 0b1100; // Reset the entry in the partition table
	fs->partition_table_modified = true;

	// Now, we have to update the predecessor/successor references to avoid inconsistencies in the filesystem.
    uint8_t header[8];

    fs->read(fs->block_size * oldest_block_id, header, 8);

   uint16_t successor_block_id = fs->data_blocks[oldest_block_id].successor;

   File* old_file = &fs->files[(header[5] << 8) | header[4]];

   old_file->length -= 4096;
   old_file->used_blocks--;


   if(successor_block_id) {
  	 uint8_t lost_predecessor[2];

  	 fs->write(fs->block_size * successor_block_id + 6, lost_predecessor, 2);
     fs->partition_table[successor_block_id] |= 0b11110000; // Set the successor block as a lost block
     fs->partition_table_modified = true;

     fs->data_blocks[old_file->first_block].successor = successor_block_id;
   }

   fs->data_blocks[oldest_block_id].successor = 0;

   rfs_update_relative_time(fs);

	fs->erase_block(fs->block_size * oldest_block_id); // Prepare reallocated block for writing

	return oldest_block_id;
}


void rfs_block_free(FileSystem* fs, uint16_t block_id) {
	if(block_id >= PROTECTED_BLOCKS) {
		fs->partition_table[block_id] = 0;
		fs->partition_table_modified = true;
		fs->total_used_blocks--;
	} else {
		fs->log("Error: Cannot free a protected block");
	}
}


/*
 * This function transforms the memory access operation to ensure that no block is overwritten.
 *
 * This only provides forward memory protection.
 * Do not attempt to access memory before the address provided by rfs_block_alloc().
 * Implementing full memory protection would cost memory and is not absolutely necessary.
 * Returns the number of readable bytes.
 */
int32_t rfs_access_memory(FileSystem* fs, uint32_t* address, uint32_t length, AccessType access_type) {
	if(allow_unsafe_access) {
		return length; // Bypass software protection mechanism
	}

   uint32_t internal_address = 1 + (*address - 1) % fs->block_size;
   uint16_t block_id = (*address - internal_address) / fs->block_size;

   if(internal_address < BLOCK_HEADER_SIZE) {
      // Correction of the address when it is too low
      *address += BLOCK_HEADER_SIZE - internal_address;
   } else if(internal_address == fs->block_size) {
      // Correction of the address when it is at the end of a block
      uint16_t successor_block = fs->data_blocks[block_id].successor;

      if(!successor_block) {
         switch(access_type) {
         case READ:
            return -1; // End of file
         case WRITE: {
			uint8_t buffer[2];
			fs->read(block_id * fs->block_size + 4, buffer, 2); // Read the file identifier
			uint16_t file_id = (buffer[1] << 8) | buffer[0];
			File* file = &(fs->files[file_id]);

			FileType file_type = (FileType) (fs->partition_table[block_id] >> 4);
			uint16_t new_block_id = rfs_block_alloc(fs, file_type); // Allocate a new block

			rfs_block_write_header(fs, new_block_id, file_id, block_id);

			file->used_blocks += 1;
			file->last_block = new_block_id;
			file->length += fs->block_size;

			fs->data_blocks[block_id].successor = new_block_id;

			successor_block = new_block_id;

			break;
         }
         default:
            return -1; // Not implemented
         }
      }

      *address = successor_block * fs->block_size + BLOCK_HEADER_SIZE;
      internal_address = BLOCK_HEADER_SIZE;
   }

   uint32_t max_length = fs->block_size;
   uint32_t new_length = length;

   if(access_type == READ && *address >= fs->block_size * PROTECTED_BLOCKS) {
	   max_length = rfs_compute_block_length(fs, block_id);
   }

   if(internal_address + length > max_length) {
	   new_length = max_length - internal_address; // Readable/Writable length correction
   }

   if(internal_address != fs->block_size && new_length == 0) { // Goto next block
	   *address = (block_id + 1) * fs->block_size;
	   return rfs_access_memory(fs, address, length, access_type);
   }

   if(access_type == WRITE) {
	   rfs_block_update_usage_table(fs, *address, *address + new_length - 1);
   }

   return new_length;
}



/*
 * Returns the number of blocks used by this file.
 */
uint16_t rfs_load_file_meta(FileSystem* fs, File* file) {
   uint32_t block_id = file->first_block;

   file->length = 0;
   file->used_blocks = 0;

   uint16_t counter = 0;

   while(block_id) {
      file->length += rfs_compute_block_length(fs, block_id);
      file->used_blocks++;
      file->last_block = block_id;

      block_id = fs->data_blocks[block_id].successor;

      if(counter++ > NUM_BLOCKS) {
    	  break;
      }
   }

   return file->used_blocks;
}

void rfs_set_file_root(FileSystem* fs, uint16_t block_id) {
	uint32_t address = rfs_get_block_base_address(fs, block_id);

	fs->partition_table[block_id] |= 0b00001111; // Set the file base block immortal
	rfs_block_update_usage_table(fs, address, address + 16);
}

/*
 * Block statistics functions
 */
uint32_t rfs_compute_block_length(FileSystem* fs, uint16_t block_id) {
	uint32_t address = block_id * fs->block_size;
	uint8_t usage_table[8];

	fs->read(address + 8, usage_table, 8); // Skip the file id and predecessor block id

	uint64_t composition = 0ULL;

	composition |= (uint64_t) usage_table[7] << 56;
	composition |= (uint64_t) usage_table[6] << 48;
	composition |= (uint64_t) usage_table[5] << 40;
	composition |= (uint64_t) usage_table[4] << 32;
	composition |= (uint64_t) usage_table[3] << 24;
	composition |= (uint64_t) usage_table[2] << 16;
	composition |= (uint64_t) usage_table[1] << 8;
	composition |= (uint64_t) usage_table[0];

	return __compute_block_length(fs, composition);
}

/*
 * Written length is not an actual length but must be regarded as a bit chain.
 * Consider those examples:
 *
 * 11111111 11111111 11111111 11111111 11111111 11111111 11111111 11111111: Block completely empty
 * 11111111 11111111 11111111 11111111 11111111 11111111 11110000 00000000: Block uses 18.75% of subsector_size
 * 11111111 11111111 00000000 00000000 00000000 00000000 00000000 00000000: Block uses 75% of subsector_size
 * 10000000 00000000 00000000 00000000 00000000 00000000 00000000 00000000: Block uses 98.4% of subsector_size
 *
 */
static uint32_t __compute_block_length(FileSystem* fs, uint64_t usage_table) {
	uint32_t length = 0;

	usage_table = ~usage_table;

	while(usage_table) {
		length += usage_table & 0b1;
		usage_table >>= 1;
	}

	return fs->block_size * length / 64;
}


uint32_t rfs_get_block_base_address(FileSystem* fs, uint16_t block_id) {
	return block_id * fs->block_size + BLOCK_HEADER_SIZE;
}



/*
 * Header update functions
 */
void rfs_block_write_header(FileSystem* fs, uint16_t block_id, uint16_t file_id, uint16_t predecessor) {
	uint8_t buffer[8];

	buffer[0] = (uint8_t) (BLOCK_MAGIC_NUMBER);
	buffer[1] = (uint8_t) (BLOCK_MAGIC_NUMBER >> 8);
	buffer[2] = (uint8_t) (BLOCK_MAGIC_NUMBER >> 16);
	buffer[3] = (uint8_t) (BLOCK_MAGIC_NUMBER >> 24);
	buffer[4] = (uint8_t) file_id;
	buffer[5] = (uint8_t) (file_id >> 8);
	buffer[6] = (uint8_t) predecessor;
	buffer[7] = (uint8_t) (predecessor >> 8);

	fs->write(block_id * fs->block_size, buffer, 8);

	rfs_block_update_usage_table(fs, block_id * fs->block_size, block_id * fs->block_size + 16);
}

/*
 * To understand this function, please remember how NOR flash memories work :-)
 *
 *   normalised_end       normalised_begin
 * 		   | 			         |
 * 1111111100000000000000000000000111111111 =
 *
 * 0000000011111111111111111111111000000000 NOT
 *
 * 1111111100000000000000000000000000000000 = (~0ULL << (normalised_end + 1));
 * 0000000000000000000000000000000111111111 = (1ULL << normalised_begin) - 1;
 */
static void rfs_block_update_usage_table(FileSystem* fs, uint32_t write_begin, uint32_t write_end) {
	uint16_t block_id = write_begin / fs->block_size;

	uint8_t lsb = fs->block_size / 64;

	uint8_t normalised_begin = (write_begin % fs->block_size) / lsb;
	uint8_t normalised_end = (write_end % fs->block_size) / lsb;


	uint64_t begin_bit_mask = (1ULL << normalised_begin) - 1;
	uint64_t end_bit_mask = (~0ULL << (normalised_end + 1));

	uint64_t usage_bit_mask = normalised_end < 63 ? (begin_bit_mask | end_bit_mask) : begin_bit_mask;

	/*
	 * We cannot use the stream API because this function is called by rfs_access_memory(),
	 * which is itself called by all Stream read and write operations.
	 */
	uint8_t buffer[8];

	buffer[0] = usage_bit_mask;
	buffer[1] = usage_bit_mask >> 8;
	buffer[2] = usage_bit_mask >> 16;
	buffer[3] = usage_bit_mask >> 24;
	buffer[4] = usage_bit_mask >> 32;
	buffer[5] = usage_bit_mask >> 40;
	buffer[6] = usage_bit_mask >> 48;
	buffer[7] = usage_bit_mask >> 56;

	fs->write(block_id * fs->block_size + 8, buffer, 8);
}

/*
 * Relative time update functions
 *
 * The relative time ranges from 0 to 16 and describes more or less the age of a block.
 * Birth age is 14, greatest age is 0.
 */
static void rfs_update_relative_time(FileSystem* fs) {
	uint8_t anchor = fs->partition_table[0] & 0xF; // Core block meta is used as a time reference
	uint8_t available_space = 16 - (fs->total_used_blocks * 16UL) / NUM_BLOCKS; // Ranges from 0 to 15

	if(available_space < anchor) {
		rfs_decrease_relative_time(fs);
	}
}

static void rfs_decrease_relative_time(FileSystem* fs) {
	for(uint16_t block_id = 0; block_id < NUM_BLOCKS; block_id++) {
		uint8_t* meta = &(fs->partition_table[block_id]);

		if((*meta & 0b00001111) > 0 && (*meta & 0b00001111) < 0xF) {
			(*meta)--;
		}
	}
}
