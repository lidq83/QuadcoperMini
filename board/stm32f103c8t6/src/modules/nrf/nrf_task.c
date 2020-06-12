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
	// 	uint16_t ctl[4] = {0};
	// 	uint8_t status = 0;

	// 	nrf_init();

	// 	if (NRF_Check() != SUCCESS)
	// 	{
	// 		k_printf("NRF init error.\n");
	// 		return;
	// 	}

	// 	NRF_RX_Mode();

	// 	k_printf("[ OK ] NRF init finished.\n");

	// 	while (1)
	// 	{
	// 		if (protocol_parse(ctl) == 0)
	// 		{
	// 			float ctl0 = ((float)(ctl[1] - CTL_PWM_MIN)) / CTL_PWM_SCALE;
	// 			float ctl1 = ((float)(ctl[2] - CTL_PWM_MIN)) / CTL_PWM_SCALE;

	// 			ctl0 = 1.0f - ctl0 * 2.0f;
	// 			ctl1 = 1.0f - ctl1 * 2.0f;

	// #ifdef MOTOR_TYPE_CAR
	// 			// float pl = (power_level + 1.0f) * 0.2f;
	// 			// ctl0 *= pl;
	// 			// ctl1 *= pl;
	// #endif

	// 			motor_ctl[0] = ctl0;
	// 			motor_ctl[1] = ctl1;

	// 			led_blink(1);
	// 		}

	// 		sleep_ticks(5);
	// 	}

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
#ifdef MOTOR_TYPE_CAR
				float ctl0 = ((float)(ctl[1] - CTL_PWM_MIN)) / CTL_PWM_SCALE;
				float ctl1 = ((float)(ctl[3] - CTL_PWM_MIN)) / CTL_PWM_SCALE;
#endif
#ifdef MOTOR_TYPE_BOAT
				float ctl0 = ((float)(ctl[1] - CTL_PWM_MIN)) / CTL_PWM_SCALE;
				float ctl1 = ((float)(ctl[2] - CTL_PWM_MIN)) / CTL_PWM_SCALE;
#endif
				ctl0 = 1.0f - ctl0 * 2.0f;
				ctl1 = 1.0f - ctl1 * 2.0f;

#ifdef MOTOR_TYPE_CAR
				float pl = (power_level + 1.0f) * 0.2f;
				ctl0 *= pl;
				ctl1 *= pl;
#endif

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
	pcb_create(22, &nrf_pthread, NULL, 1024);
}
