/*
 * task.c
 *
 *  Created on: December 23, 2019
 *      Author: lidq
 */

#include <nrf_task.h>

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

	while (1)
	{
		if (protocol_parse(ctl) == 0)
		{
			float ctl0 = ((float)(ctl[1] - CTL_PWM_MIN)) / CTL_PWM_SCALE;
			float ctl1 = ((float)(ctl[3] - CTL_PWM_MIN)) / CTL_PWM_SCALE;

			motor_set_value(0, ctl0);
			motor_set_value(1, ctl1);
		}

		sleep_ticks(5);
	}
}

void nrf_task(void)
{
	pcb_create(22, &nrf_pthread, NULL, 1024);
}
