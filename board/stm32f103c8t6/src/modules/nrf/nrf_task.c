/*
 * task.c
 *
 *  Created on: December 23, 2019
 *      Author: lidq
 */

#include <nrf_task.h>
#include <k_printf.h>
#include <led.h>
#include <tim1.h>

#define CTL_PWM_MAX (2000)
#define CTL_PWM_MIN (1000)
#define CTL_PWM_SCALE (CTL_PWM_MAX - CTL_PWM_MIN)

float ctl_yaw = 0;
float ctl_thro = 0;
float ctl_roll = 0;
float ctl_pitch = 0;

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

	float filter = 0.5f;

	float ctl_yaw_last = 0;
	float ctl_thro_last = 0;
	float ctl_roll_last = 0;
	float ctl_pitch_last = 0;

	uint64_t rt = current_time();

	while (1)
	{
		uint64_t r = current_time();
		if (r - rt > 1000)
		{
			ctl_thro = 0;
			ctl_roll = 0;
			ctl_pitch = 0;
			ctl_yaw = 0;

			rt = r;
		}

		RF24L01_Set_Mode(MODE_RX);
		int len = NRF24L01_RxPacket(RF24L01RxBuffer);
		if (len > 0)
		{
			protocol_append(RF24L01RxBuffer, len);

			if (protocol_parse(ctl) == 0)
			{
				float roll = ((float)(ctl[0] - CTL_PWM_MIN)) / CTL_PWM_SCALE;
				float pitch = ((float)(ctl[1] - CTL_PWM_MIN)) / CTL_PWM_SCALE;
				float yaw = ((float)(ctl[2] - CTL_PWM_MIN)) / CTL_PWM_SCALE;
				float thro = ((float)(ctl[3] - CTL_PWM_MIN)) / CTL_PWM_SCALE;

				roll = 1.0f - roll * 2.0f;
				pitch = 1.0f - pitch * 2.0f;
				yaw = 1.0f - yaw * 2.0f;

				ctl_thro = thro * filter + ctl_thro_last * (1.0f - filter);
				ctl_roll = roll * filter + ctl_roll_last * (1.0f - filter);
				ctl_pitch = pitch * filter + ctl_pitch_last * (1.0f - filter);
				ctl_yaw = yaw * filter + ctl_yaw_last * (1.0f - filter);

				ctl_thro_last = ctl_thro;
				ctl_roll_last = ctl_roll;
				ctl_pitch_last = ctl_pitch;
				ctl_yaw_last = ctl_yaw;

				rt = current_time();

				led_blink(1);
			}
		}

		sleep_ticks(5);
	}
}

void nrf_task(void)
{
	pcb_create(PROI_NRF, &nrf_pthread, NULL, 1200);
}
