/*
 * task.c
 *
 *  Created on: December 23, 2019
 *      Author: lidq
 */

#include <led.h>
#include <led_task.h>

led_s led = { 0, 0x05 };

void* led_pthread(void* arg)
{
	for (uint8_t i = 0;; i++)
	{
		if ((led.led_val >> (i % 8)) & 0x1)
		{
			led_on(0);
		}
		else
		{
			led_off(0);
		}
		led_blink(1);
		
		sleep_ticks(100);
	}
	return NULL;
}

void led_task(void)
{
	pcb_create(PCB_LED_PRIO, &led_pthread, NULL, PCB_LED_SIZE);
}
