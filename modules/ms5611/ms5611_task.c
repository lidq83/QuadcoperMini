/*
 * task.c
 *
 *  Created on: December 23, 2019
 *      Author: lidq
 */

#include <ms5611.h>
#include <ms5611_task.h>

extern I2C_HandleTypeDef hi2c2;

double alt_press = 0;

void* ms5611_pthread(void* arg)
{
	MS5611_t ms5611 = { 0 };
	MS5611_init(&hi2c2, &ms5611);

	double P = 0;
	double P_pre = 0;
	double P_filter = 0.1;
	double vel = 0;

	double vel_pre = 0;
	double vel_filter = 0.1;
	int first = 1;
	while (1)
	{
		MS5611_read_press(&hi2c2, &ms5611, MS5611_CMD_CONVERT_D1_4096);
		MS5611_read_temp(&hi2c2, &ms5611, MS5611_CMD_CONVERT_D2_4096);

		MS5611_calculate(&ms5611);

		alt_press = -ms5611.P * 0.05;
		// printf("%8d %8d %8d\n", (int)ms5611.P, (int)(alt_press * 1000.0));

		sleep_ticks(25);
	}
	return NULL;
}

void ms5611_task(void)
{
	pcb_create(PCB_MS5611_PRIO, &ms5611_pthread, NULL, PCB_MS5611_SIZE);
}
