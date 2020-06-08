/*
 * task.c
 *
 *  Created on: December 23, 2019
 *      Author: lidq
 */

#include <nrf_task.h>

#define CTL_PWM_MAX (2200)
#define CTL_PWM_MIN (800)
#define CTL_PWM_SCALE (CTL_PWM_MAX - CTL_PWM_MIN)

extern float motor_ctl[MOTOR_CNT];

extern int power_level;

void nrf_pthread(void *arg)
{
	uint16_t ctl[4] = {0};
	uint8_t status = 0;

	nrf_init();

	if (NRF_Check() != SUCCESS)
	{
		k_printf("NRF init error.\n");
		return;
	}

	NRF_RX_Mode();

	k_printf("[ OK ] NRF init finished.\n");

	while (1)
	{
		if (protocol_parse(ctl) == 0)
		{
			float ctl0 = ((float)(ctl[1] - CTL_PWM_MIN)) / CTL_PWM_SCALE;
			float ctl1 = ((float)(ctl[3] - CTL_PWM_MIN)) / CTL_PWM_SCALE;

			ctl0 = 1.0f - ctl0 * 2.0f;
			ctl1 = 1.0f - ctl1 * 2.0f;

#ifdef MOTOR_TYPE_CAR
			// float pl = (power_level + 1.0f) * 0.2f;
			// ctl0 *= pl;
			// ctl1 *= pl;
#endif

			motor_ctl[0] = ctl0;
			motor_ctl[1] = ctl1;

			led_blink(1);
		}

		sleep_ticks(5);
	}
}

void nrf_task(void)
{
	pcb_create(22, &nrf_pthread, NULL, 1024);
}
