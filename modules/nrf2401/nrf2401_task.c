/*
 * nrf_task.c
 *
 *  Created on: December 23, 2019
 *      Author: lidq
 */

#include <nrf2401.h>
#include <nrf2401_task.h>
#include <stdio.h>

void* nrf2401_pthread(void* arg)
{
	uint8_t RF24L01RxBuffer[128] = { 0 };
	uint16_t ctl[4] = { 0 };

	NRF24L01_check();
	RF24L01_Init();
	RF24LL01_Write_Hopping_Point(64);
	NRF24L01_Set_Power(POWER_F18DBM);
	NRF24L01_Set_Speed(SPEED_250K);

	while (1)
	{
		RF24L01_Set_Mode(MODE_RX);
		int len = NRF24L01_RxPacket(RF24L01RxBuffer);
		// printf("nrf recv %d\n", len);

		sleep_ticks(50);
	}
	return NULL;
}

void nrf2401_task(void)
{
	pcb_create(PCB_NRF_PRIO, &nrf2401_pthread, NULL, PCB_NRF_SIZE);
}
