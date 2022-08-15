/*
 * task.c
 *
 *  Created on: December 23, 2019
 *      Author: lidq
 */

#include <led.h>
#include <led_task.h>

led_s led = { 0, 0x05 };
extern int ctl_arming;
extern int ctl_calibrate;

void* led_pthread(void* arg)
{
	led_on(0);
	led_off(1);
	led_off(2);

	for (uint8_t i = 0;; i++)
	{
		if (ctl_arming)
		{
			if ((led.led_val >> (i % 8)) & 0x1)
			{
				led_on(0);
			}
			else
			{
				led_off(0);
			}
		}
		else
		{
			if (ctl_calibrate == 0)
			{
				led_blink(0);
			}
			else
			{
				if (i % 16 < 8)
				{
					led_on(0);
				}
				else
				{
					led_off(0);
				}
			}
		}

		sleep_ticks(125);
	}
	return NULL;
}

void led_task(void)
{
	pcb_create(PCB_LED_PRIO, &led_pthread, NULL, PCB_LED_SIZE);
}
