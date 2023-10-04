/*
 * task.c
 *
 *  Created on: December 23, 2019
 *      Author: lidq
 */

#include <ms5611.h>
#include <ms5611_task.h>
#include <math.h>

extern I2C_HandleTypeDef hi2c2;

double alt_press = 0;

// 根据实际情况修改以下参数
#define SEA_LEVEL_PRESSURE 101325.0 // 海平面标准大气压力值（Pa）
#define TEMPERATURE_CORRECTION 1.0 // 温度修正值

// 将浮点数大气压力值转换为相对高度
double convertToAltitude(double pressure)
{
	return (1 - pow(pressure / SEA_LEVEL_PRESSURE, 0.190284)) * TEMPERATURE_CORRECTION * 44330.7692;
}

void* ms5611_pthread(void* arg)
{
	MS5611_t ms5611 = { 0 };
	MS5611_init(&hi2c2, &ms5611);

	while (1)
	{
		MS5611_read_press(&hi2c2, &ms5611, MS5611_CMD_CONVERT_D1_4096);
		MS5611_read_temp(&hi2c2, &ms5611, MS5611_CMD_CONVERT_D2_4096);

		MS5611_calculate(&ms5611);

		alt_press = convertToAltitude((double)ms5611.P);
		
		// printf("%+8d\n", (int)(alt_press * 1000.0));
		sleep_ticks(20);
	}
	return NULL;
}

void ms5611_task(void)
{
	pcb_create(PCB_MS5611_PRIO, &ms5611_pthread, NULL, PCB_MS5611_SIZE);
}
