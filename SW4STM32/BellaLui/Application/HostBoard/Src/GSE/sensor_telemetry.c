void telemetry_init(void)
{
	//Initialise telemetry

}
void sensors_init(void)
{
	//Initialize sensors
}

void TK_telemetry_control(void const * argument)
{
	//Order status, check each state and send appropriate data
	for(;;)
	{
		osDelay(1000);
	}
}
void TK_sensors_control(void const * argument)
{
	for(;;)
	{
//		float tank_temp = 0;
//		float hose_pressure = 0;
//		float host_temp = 0;
//




//		//Tank Temperature
//		tank_temp = read_tank_temp();
//		can_setFrame(tank_temp, DATA_ID_TANK_TEMPERATURE, HAL_GetTick());
//
//
		osDelay(1000);
	}
}
