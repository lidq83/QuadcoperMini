/*
 * controller_task.c
 *
 *  Created on: December 23, 2019
 *      Author: lidq
 */

#include <controller_task.h>
#include <main.h>
#include <stdio.h>

void* controller_pthread(void* arg)
{
	double timepre = 0;

	while (1)
	{
		// 计算当前时间戳
		uint64_t cnt = get_count();
		double timestamp = cnt / 1000000.0;
		double dt = timestamp - timepre;
		timepre = timestamp;

		// printf("time=%d\n", (int)(timestamp));

		msleep(100);
	}
	return NULL;
}

void controller_task(void)
{
	pcb_create(PCB_CTL_PRIO, &controller_pthread, NULL, PCB_CTL_SIZE);
}
