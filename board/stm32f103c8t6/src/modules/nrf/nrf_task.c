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

void nrf_pthread(void *arg)
{
	uint8_t RF24L01RxBuffer[128] = {0};
	uint16_t ctl[4] = {0};

	drv_spi_init();
	NRF24L01_Gpio_Init();
	NRF24L01_check();
	RF24L01_Init();
	RF24LL01_Write_Hopping_Point(64);
	NRF24L01_Set_Power(POWER_F18DBM);
	NRF24L01_Set_Speed(SPEED_250K);

	while (1)
	{
		RF24L01_Set_Mode(MODE_RX);
		int len = NRF24L01_RxPacket(RF24L01RxBuffer);
		if (len > 0)
		{
			protocol_append(RF24L01RxBuffer, len);

			if (protocol_parse(ctl) == 0)
			{
				float ctl0 = ((float)(ctl[1] - CTL_PWM_MIN)) / CTL_PWM_SCALE;
				float ctl1 = ((float)(ctl[3] - CTL_PWM_MIN)) / CTL_PWM_SCALE;

				ctl0 = 1.0f - ctl0 * 2.0f;
				ctl1 = 1.0f - ctl1 * 2.0f;

				ctl0 *= 0.5;
				ctl1 *= 0.5;

				motor_ctl[0] = ctl0;
				motor_ctl[1] = ctl1;

				led_blink(1);
			}
		}

		sleep_ticks(5);
	}
}

void nrf_task(void)
{
	pcb_create(PROI_NRF, &nrf_pthread, NULL, 1024);
}
