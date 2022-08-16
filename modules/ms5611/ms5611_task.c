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

	double v = 0;
	double vp = 0;
	double filter = 0.03;
	double first_total = 0;
	double first_val = 0;
	int first_cnt = 0;
	double alt_p = -0.0001;

	while (1)
	{
		MS5611_read_press(&hi2c2, &ms5611, MS5611_CMD_CONVERT_D1_4096);
		MS5611_read_temp(&hi2c2, &ms5611, MS5611_CMD_CONVERT_D2_4096);

		MS5611_calculate(&ms5611);


		double P = ms5611.P;

		if (first_cnt < 100)
		{
			first_total += P;
			first_cnt++;
		}
		else if (first_cnt == 100)
		{
			first_val = first_total / first_cnt;
			first_cnt++;
		}
		else
		{
			double press = P - first_val;

			v = press * filter + vp * (1.0 - filter);
			vp = v;
			alt_press = v * alt_p;
		}

		sleep_ticks(10);
	}
	return NULL;
}

void ms5611_task(void)
{
	pcb_create(PCB_MS5611_PRIO, &ms5611_pthread, NULL, PCB_MS5611_SIZE);
}
