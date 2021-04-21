/*
 * heavy_io.c
 *
 *  Created on: 30 Oct 2019
 *      Author: Arion
 */

#include "storage/heavy_io.h"

#include "debug/led.h"
#include "debug/console.h"
#include "debug/profiler.h"

#include <cmsis_os.h>


#define MAX_TASKS 256


struct Node {
	const void* arg;
	int32_t (*task)(const void* arg);
	void (*feedback)(int32_t);
	struct Node* next;
};

struct FIFO {
	struct Node* first;
	struct Node* last;
};


static volatile SemaphoreHandle_t task_semaphore;
static volatile struct FIFO queue = { 0 };



static struct Node node_heap[MAX_TASKS];
static volatile uint8_t heap_counter = 0;
static volatile uint8_t scheduled_tasks = 0;


void init_heavy_scheduler() {
	task_semaphore = xSemaphoreCreateCounting(256, 0);
}




/*
 * TODO: Better concurrent implementation of FIFO queue.
 */
void schedule_heavy_task(int32_t (*task)(void*), const void* arg, void (*feedback)(int32_t)) {
	struct Node* node = &node_heap[heap_counter++]; // To avoid using malloc from interrupts

	node->task = task;
	node->arg = arg;
	node->feedback = feedback;
	node->next = 0;

	if(queue.first != 0) {
		queue.last->next = node;
	} else {
		queue.first = node;
	}

	queue.last = node;

	scheduled_tasks++;
}

void TK_heavy_io_scheduler() {
	uint32_t led_identifier = led_register_TK();

	while(true) {
		while(!scheduled_tasks);

		//rocket_log("Launching task\n");

		led_set_TK_rgb(led_identifier, 0xFF, 0xAA, 0);

		struct Node* task = queue.first;
		queue.first = task->next;

		task->feedback(task->task(task->arg));

		if(!queue.first) { // Avoid inconsistent FIFO queue state
			queue.last = queue.first;
		}

		//rocket_log("Task finished\n");

		led_set_TK_rgb(led_identifier, 0, 0, 0);

		scheduled_tasks--;
	}
}
