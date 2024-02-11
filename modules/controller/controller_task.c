/*
 * controller_task.c
 *
 *  Created on: December 23, 2019
 *      Author: lidq
 */

#include <controller_task.h>
#include <main.h>
#include <stdio.h>

extern float ctl_thro;
extern float ctl_pitch;
extern float ctl_roll;
extern float ctl_yaw;
extern uint8_t ctl_sw[16];

void* controller_pthread(void* arg)
{
	double timepre = 0;
	uint32_t tk = 0;
	while (1)
	{
		// 计算当前时间戳
		uint64_t cnt = get_count();
		double timestamp = cnt / 1000000.0;
		double dt = timestamp - timepre;
		timepre = timestamp;

		// if (tk % 10 == 0)
		// {
		// 	printf("%04d %04d %04d %04d \n",
		// 		   (int)(ctl_thro * 1000),
		// 		   (int)(ctl_pitch * 1000),
		// 		   (int)(ctl_roll * 1000),
		// 		   (int)(ctl_yaw * 1000));
		// }

		tk++;
		msleep(10);
	}
	return NULL;
}

void controller_task(void)
{
	pcb_create(PCB_CTL_PRIO, &controller_pthread, NULL, PCB_CTL_SIZE);
}
