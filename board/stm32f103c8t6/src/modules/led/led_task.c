/*
 * task.c
 *
 *  Created on: December 23, 2019
 *      Author: lidq
 */

#include <led_task.h>

static led_s led = {0, 0x05};

int led_vals[5] = {0x1, 0x5, 0x15, 0x55, 0x155};

extern int power_level;

void led_pthread(void *arg)
{
	for (uint8_t i = 0;; i++)
	{
		if (i % 16 == 0)
		{
			led.led_val = led_vals[power_level];
		}

		if ((led.led_val >> (i % 16)) & 0x1)
		{
			led_on(led.led_num);
		}
		else
		{
			led_off(led.led_num);
		}
		sleep_ticks(150);
	}
}

void led_task(void)
{
	pcb_create(28, &led_pthread, NULL, 512);
}
