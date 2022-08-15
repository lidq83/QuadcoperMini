/*
 * task.c
 *
 *  Created on: December 23, 2019
 *      Author: lidq
 */

#include <ms5611.h>
#include <ms5611_task.h>

extern I2C_HandleTypeDef hi2c2;

void* ms5611_pthread(void* arg)
{
	MS5611_t ms5611 = { 0 };
	MS5611_init(&hi2c2, &ms5611);

	double v = 0;
	double vp = 0;
	double f = 0.03;
	while (1)
	{
		// NB_MS5611_request_press(&hi2c2, &ms5611, MS5611_CMD_CONVERT_D1_4096);
		// NB_MS5611_pull_press(&hi2c2, &ms5611);

		// NB_MS5611_request_temp(&hi2c2, &ms5611, MS5611_CMD_CONVERT_D2_4096);
		// NB_MS5611_pull_temp(&hi2c2, &ms5611);


		MS5611_read_press(&hi2c2, &ms5611, MS5611_CMD_CONVERT_D1_2048);
		MS5611_read_temp(&hi2c2, &ms5611, MS5611_CMD_CONVERT_D2_2048);

		MS5611_calculate(&ms5611);

		v = ms5611.P * f + vp * (1.0 - f);
		vp = v;

		printf("ms5611 %6d %6d %7d \n", ms5611.P, ms5611.TEMP, (int)(v * 10));
		sleep_ticks(10);
	}
	return NULL;
}

void ms5611_task(void)
{
	pcb_create(PCB_MS5611_PRIO, &ms5611_pthread, NULL, PCB_MS5611_SIZE);
}
