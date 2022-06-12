#ifndef __DRIVERS_LED_H
#define __DRIVERS_LED_H

#include <board.h>
#include <stdint.h>

typedef struct led_s
{
	uint8_t led_num;
	uint8_t led_val;
} led_s;

void led_init(void);

void led_on(int num);

void led_off(int num);

void led_blink(int num);

#endif
