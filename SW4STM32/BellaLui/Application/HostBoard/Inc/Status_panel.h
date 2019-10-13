#include "gpio.h"
#include "cmsis_os.h"

void Status_panel_init(uint32_t slot);
void setBuzzerPanel(uint32_t timeOn);
void setLedPanel(uint32_t Led_number);
void resetLedPanel(uint32_t Led_number);
