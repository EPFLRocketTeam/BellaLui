/*
 * flash_runtime.c
 *
 *  Created on: 27 Nov 2019
 *      Author: Arion
 */

#include <storage/flash_runtime.h>

#include <flash.h>
#include <stdbool.h>

#include <debug/console.h>



static FileSystem fs = { 0 };
static volatile bool ready = false;





void __debug(const char *message) {
	rocket_boot_log("%s\n", message);
}

void init_filesystem() {
   rocket_fs_debug(&fs, &__debug);
   rocket_fs_device(&fs, "NOR Flash", 4096 * 4096, 4096);
   rocket_fs_bind(&fs, &flash_read, &flash_write, &flash_erase_subsector);

   rocket_fs_mount(&fs);

   ready = true;
}

FileSystem* get_flash_fs() {
   if(!ready) {
      return 0;
   }

   return &fs;
}
