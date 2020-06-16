/*
 * task.c
 *
 *  Created on: December 23, 2019
 *      Author: lidq
 */

#include <mpu6050_task.h>

static void mpu6050_pthread(void *arg)
{
	I2C_init();

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

	while (1)
	{
		mpu6050_value(&x, &y, &z, &gx, &gy, &gz, &ax, &ay, &az);

		sleep_ticks(50);
	}
}

void mpu6050_task(void)
{
	pcb_create(PROI_MOTOR, &mpu6050_pthread, NULL, 1024);
}