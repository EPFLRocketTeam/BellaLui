/*
 * heavy_io.c
 *
 *  Created on: 30 Oct 2019
 *      Author: Arion
 */

#include <storage/heavy_io.h>

#include <debug/led.h>
#include <debug/console.h>

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


void init_heavy_scheduler() {
	task_semaphore = xSemaphoreCreateCounting(256, 0);
}


/*
 * TODO: Better concurrent implementation of FIFO queue.
 */
void schedule_heavy_task(int32_t (*task)(void*), const void* arg, void (*feedback)(int32_t)) {
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

void TK_heavy_io_scheduler() {
	uint32_t led_identifier = led_register_TK();


	//on_dump_request();
	//on_fullsd_dump_request();


   /*CAN_msg msg;


	uint32_t start = HAL_GetTick();

	uint32_t num_messages = 8192;

	for(uint32_t i = 0; i < num_messages; i++) {
      msg.data = 1;
      msg.id = 2;
      msg.id_CAN = 0xCAFEBABE;
      msg.timestamp = HAL_GetTick();

      flash_log(msg);
	}

	printf("CAN message processing time: %ldµs\n", 1000 * (HAL_GetTick() - start) / num_messages);*/


	while(true) {
		xSemaphoreTake(task_semaphore, portMAX_DELAY);

		printf("Launching task\n");

		struct Node* task = queue.first;
		queue.first = task->next;

		task->feedback(task->task(task->arg));

		if(!queue.first) { // Avoid inconsistent FIFO queue state
			queue.first = queue.last;
		}

		led_set_TK_rgb(led_identifier, 0xFF, 0xAA, 0);
	}
}
