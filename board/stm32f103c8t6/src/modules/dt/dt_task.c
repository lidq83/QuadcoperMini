/*
 * task.c
 *
 *  Created on: December 23, 2019
 *      Author: lidq
 */

#include <dt_task.h>
#include <buzzer_task.h>

#define MAX_NUM (99)
#define TIME_GOD (30)

uint8_t kick_num = 0;

uint8_t st = 0;
uint32_t time = 0;

void *dt_pthread(void *arg)
{
	dt_init();

	while (1)
	{
		if (time == 0)
		{
			dt_show_num(kick_num / 10, 0, 0);
			sleep_ticks(5);

			dt_show_num(kick_num % 10, 1, 0);
			sleep_ticks(5);

			continue;
		}

		if (time % 2 < 1)
		{
			dt_show_num(kick_num / 10, 0, 0);
			sleep_ticks(5);

			dt_show_num(kick_num % 10, 1, 0);
			sleep_ticks(5);
		}
		else
		{
			dt_hide_num(0);
			sleep_ticks(5);

			dt_hide_num(1);
			sleep_ticks(5);
		}
	}

	return NULL;
}

void *dt_tm_pthread(void *arg)
{
	while (1)
	{
		if (time != 0)
		{
			time--;
		}
		sleep_ticks(100);
	}
	return NULL;
}

void dt_add_num(void)
{
	if (time != 0)
	{
		return;
	}

	kick_num++;
	if (kick_num > MAX_NUM)
	{
		kick_num = MAX_NUM;
	}

	time = TIME_GOD;
	buzzer_set_status(1);
}

void dt_task(void)
{
	pcb_create(PROI_DT, &dt_pthread, NULL, 800);
	pcb_create(PROI_DT_TM, &dt_tm_pthread, NULL, 500);
}
