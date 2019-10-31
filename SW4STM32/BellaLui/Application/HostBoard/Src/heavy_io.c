/*
 * heavy_io.c
 *
 *  Created on: 30 Oct 2019
 *      Author: Arion
 */

#include "heavy_io.h"

#include "flash.h"

#include <cmsis_os.h>
#include <stdbool.h>


#define MAX_TASKS 256



struct Node {
	const void* arg;
	const int32_t (*task)(const void* arg);
	const void (*feedback)(int32_t);
	struct Node* next;
};

struct FIFO {
	struct Node* first;
	struct Node* last;
};


static FileSystem *fs;

static volatile SemaphoreHandle_t scheduler_mutex;
static volatile SemaphoreHandle_t task_semaphore;

static volatile struct FIFO queue = { 0 };


void init_heavy_scheduler() {
	scheduler_mutex = xSemaphoreCreateMutex();
	task_semaphore = xSemaphoreCreateCounting(0, MAX_TASKS);
}


FileSystem* get_flash_fs() {
	static bool ready = false;

	if(!ready) {
		xSemaphoreTake(scheduler_mutex, 1);
		ready = true;
	}

	return fs;
}

/*
 * TODO: Better concurrent implementation of FIFO queue.
 */
void schedule_heavy_task(int32_t (*task)(const void*), const void* arg, void (*feedback)(int32_t)) {
	struct Node* node = pvPortMalloc(sizeof(struct Node));

	node->task = task;
	node->arg = arg;
	node->feedback = feedback;

	if(queue.first != 0) {
		queue.last->next = node;
	} else {
		queue.first = node;
	}

	queue.last = node;

	xSemaphoreGive(task_semaphore);
}



void __debug(const char *message) {
	// printf("%s\n", message);
}

void TK_heavy_io_scheduler() {
	fs = pvPortMalloc(sizeof(FileSystem));


	// rocket_fs_debug(fs, &__debug);
	rocket_fs_device(fs, "NOR Flash", 4096 * 4096, 4096);
	rocket_fs_bind(fs, &flash_read, &flash_write, &flash_erase_subsector);

	rocket_fs_mount(fs);


	xSemaphoreGive(scheduler_mutex);

	while(true) {
		xSemaphoreTake(task_semaphore, 10);

		struct Node* task = queue.first;
		queue.first = task->next;

		task->feedback(task->task(task->arg));
	}
}
