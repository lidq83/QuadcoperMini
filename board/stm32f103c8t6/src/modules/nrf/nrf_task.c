/*
 * task.c
 *
 *  Created on: December 23, 2019
 *      Author: lidq
 */

#include <nrf_task.h>

#define CTL_PWM_MAX (2000)
#define CTL_PWM_MIN (1000)
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
				float ctl_power = ((float)(ctl[1] - CTL_PWM_MIN)) / CTL_PWM_SCALE;
				float ctl_dir = ((float)(ctl[2] - CTL_PWM_MIN)) / CTL_PWM_SCALE;

				ctl_power = 1.0f - ctl_power * 2.0f;
				ctl_dir = 1.0f - ctl_dir * 2.0f;

				ctl_power *= 0.25;

				float offset_left = 1.0f;
				float offset_right = 1.0f;

				if (ctl_dir > 0.1f)
				{
					offset_left = 1.0f - ctl_dir;
				}
				else if (ctl_dir < -0.1f)
				{
					offset_right = 1.0f - (-ctl_dir);
				}

				motor_ctl[0] = ctl_power * offset_left;
				motor_ctl[1] = ctl_power * offset_right;

				//k_printf("%4.2f %4.2f %4.2f %4.2f\n", ctl_power, ctl_dir, motor_ctl[0], motor_ctl[1]);

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
