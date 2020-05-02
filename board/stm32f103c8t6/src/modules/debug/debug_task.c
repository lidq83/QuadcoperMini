/*
 * task.c
 *
 *  Created on: December 23, 2019
 *      Author: lidq
 */

#include <debug_task.h>

void debug_pthread(void)
{
	float num = 0.0;
	while (1)
	{
		k_printf("debug %.2f\n", num);
		num += 0.01;
		led_blink(1);
		sleep_ticks(500);
	}
}

void debug_task(void)
{
	pcb_create(26, &debug_pthread, NULL, 1024);
}
