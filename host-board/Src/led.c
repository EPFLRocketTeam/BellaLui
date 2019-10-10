#include "tim.h"
#include "cmsis_os.h"

#include "led.h"

#define TIM_LED (TIM8)

#define LED_TK_BREAK   100  // [ms] off time in between thread
#define LED_TK_ON      300  // [ms] on time of each thread
#define LED_LOOP_BREAK 500  // [ms] time in between each start of thread sequence
#define LED_INIT_DELAY 1000 // [ms] initial time to display a solid color

#define MAX_N_THREADS 10

volatile int n_threads = 0;
volatile uint16_t r_list[MAX_N_THREADS] = {0};
volatile uint16_t g_list[MAX_N_THREADS] = {0};
volatile uint16_t b_list[MAX_N_THREADS] = {0};


void TK_led_handler(void const *arg) {
	led_init();

#ifdef BOARD_LED_R // check if default color is defined
	led_set_rgb(BOARD_LED_R, BOARD_LED_G, BOARD_LED_B);
	osDelay(LED_INIT_DELAY);
	led_set_rgb(0, 0, 0);
	osDelay(LED_TK_BREAK);
#endif

	while (1) {
		for (int i=0 ; i<n_threads ; i++) {
			led_set_rgb(r_list[i], g_list[i], b_list[i]);
			osDelay(LED_TK_ON);
			led_set_rgb(0, 0, 0);
			osDelay(LED_TK_BREAK);
		}
		osDelay(LED_LOOP_BREAK);
	}
}

// return id if sucessfull, else -1
int led_register_TK() {
	int val = 0;

	portDISABLE_INTERRUPTS();
	if (n_threads < MAX_N_THREADS) {
		val = n_threads++;
	} else {
		return -1;
	}
	portENABLE_INTERRUPTS();

	return val;
}


void led_set_TK_rgb(int tk_id, uint16_t r, uint16_t g, uint16_t b) {
	if (tk_id>=0 && tk_id<n_threads) {
		r_list[tk_id] = r;
		g_list[tk_id] = g;
		b_list[tk_id] = b;
	}
}


void led_set_rgb(uint16_t r, uint16_t g, uint16_t b) {
	LL_TIM_OC_SetCompareCH1(TIM_LED, r);
	LL_TIM_OC_SetCompareCH2(TIM_LED, g);
	LL_TIM_OC_SetCompareCH3(TIM_LED, b);
}

void led_set_r(uint16_t r) {
	LL_TIM_OC_SetCompareCH1(TIM_LED, r);
}

void led_set_g(uint16_t g) {
	LL_TIM_OC_SetCompareCH2(TIM_LED, g);
}

void led_set_b(uint16_t b) {
	LL_TIM_OC_SetCompareCH3(TIM_LED, b);
}

void led_init() {
	LL_TIM_CC_EnableChannel(TIM8, LL_TIM_CHANNEL_CH1N);
	LL_TIM_CC_EnableChannel(TIM8, LL_TIM_CHANNEL_CH2N);
	LL_TIM_CC_EnableChannel(TIM8, LL_TIM_CHANNEL_CH3N);
	LL_TIM_EnableAutomaticOutput(TIM8);
	LL_TIM_EnableCounter(TIM8);

	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_7);
	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_14);
	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_15);

	led_set_rgb(0,0,0);
}


