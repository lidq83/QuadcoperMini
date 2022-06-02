/*
 * controller_task.c
 *
 *  Created on: December 23, 2019
 *      Author: lidq
 */

#include <controller_task.h>
#include <mpu6050.h>
#include <stdio.h>

void* controller_pthread(void* arg)
{
	mpu6050_setup();

	float x = 0;
	float y = 0;
	float z = 0;
	float gx = 0;
	float gy = 0;
	float gz = 0;
	float ax = 0;
	float ay = 0;
	float az = 0;

	float sx = 0;
	float sy = 0;
	float sz = 0;

	uint32_t tk = 0;

	while (tk++ < 20)
	{
		mpu6050_value(&x, &y, &z, &gx, &gy, &gz, &ax, &ay, &az);
		sx += x;
		sy += y;
		sz += z;
		sleep_ticks(5);
	}

	if (sx == 0 && sy == 0 && sz == 0)
	{
		printf("Init mpu6050 error, reset mpu.\n");
		HAL_NVIC_SystemReset();
		while (1)
		{
		}
	}

	while (1)
	{
		mpu6050_value(&x, &y, &z, &gx, &gy, &gz, &ax, &ay, &az);
		if (tk % 5 == 0)
		{
			printf("%+6d %+6d %+6d\n", (int) (x * 1000), (int) (y * 1000), (int) (z * 1000));
		}
		tk++;
		sleep_ticks(10);
	}
	return NULL;
}

void controller_task(void)
{
	pcb_create(PCB_CTL_PRIO, &controller_pthread, NULL, PCB_CTL_SIZE);
}
