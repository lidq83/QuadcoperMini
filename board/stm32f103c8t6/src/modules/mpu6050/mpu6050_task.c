/*
 * task.c
 *
 *  Created on: December 23, 2019
 *      Author: lidq
 */

#include <mpu6050_task.h>
#include <mpu6050.h>
#include <IOI2C.h>
#include <k_printf.h>

float att_angle[3] = {0};
float att_gyro[3] = {0};

static float filter = 0.5f;

static void mpu6050_pthread(void *arg)
{
	float values_filt[6] = {0};
	float values_read[6] = {0};
	float values_last[6] = {0};

	IIC_Init();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	DMP_Init();

	while (1)
	{
		int st = Read_DMP(&values_read[0], &values_read[1], &values_read[2], &values_read[3], &values_read[4], &values_read[5]);
		if (st == 0)
		{
			for (int i = 0; i < 6; i++)
			{
				values_filt[i] = values_read[i] * filter + values_last[i] * (1.0f - filter);
				values_last[i] = values_filt[i];
			}

			att_angle[0] = values_filt[0];
			att_angle[1] = values_filt[1];
			att_angle[2] = values_filt[2];
			att_gyro[0] = values_filt[3];
			att_gyro[1] = values_filt[4];
			att_gyro[2] = values_filt[5];
		}
		sleep_ticks(20);
	}
}

void mpu6050_task(void)
{
	pcb_create(PROI_MPU6050, &mpu6050_pthread, NULL, 2000);
}