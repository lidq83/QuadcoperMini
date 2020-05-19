/*
 * task.c
 *
 *  Created on: December 23, 2019
 *      Author: lidq
 */

#include <debug_task.h>

void debug_pthread(void *arg)
{
	uint32_t i = 0;
	while (1)
	{
		k_printf("debug %04u\n", i++);
		sleep_ticks(100);
	}
}

void debug_task(void)
{
	pcb_create(26, &debug_pthread, NULL, 1024);
}
