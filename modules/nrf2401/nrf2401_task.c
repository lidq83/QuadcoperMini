/*
 * nrf_task.c
 *
 *  Created on: December 23, 2019
 *      Author: lidq
 */

#include <nrf2401.h>
#include <nrf2401_task.h>
#include <stdio.h>

float ctl_yaw = 0;
float ctl_thro = 0;
float ctl_roll = 0;
float ctl_pitch = 0;

void* nrf2401_pthread(void* arg)
{
	uint8_t RF24L01RxBuffer[32] = { 0 };
	uint16_t ctl[4] = { 0 };

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


	while (1)
	{
		RF24L01_Set_Mode(MODE_RX);
		int len = NRF24L01_RxPacket(RF24L01RxBuffer);
		if (len > 0)
		{
			for (int i = 0; i < len; i++)
			{
				printf("%02x ", RF24L01RxBuffer[i]);
			}
			printf("\n");
			
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

				printf("%04d %04d %04d %04d\n", ctl[0], ctl[1], ctl[2], ctl[3]);
			}
		}

		sleep_ticks(20);
	}
	return NULL;
}

void nrf2401_task(void)
{
	pcb_create(PCB_NRF_PRIO, &nrf2401_pthread, NULL, PCB_NRF_SIZE);
}
