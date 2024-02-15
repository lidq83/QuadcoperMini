/*
 * nrf_task.c
 *
 *  Created on: December 23, 2019
 *      Author: lidq
 */

#include <led.h>
#include <math.h>
#include <nrf2401.h>
#include <nrf2401_task.h>
#include <protocol.h>
#include <stdint.h>
#include <stdio.h>

float ctl_thro	   = 0;
float ctl_pitch	   = 0;
float ctl_roll	   = 0;
float ctl_yaw	   = 0;
uint8_t ctl_sw[16] = { 0 };

static sem_t sem_sig	= { 0 };
static uint32_t tk_recv = 0;

void ctl_switch(uint16_t ctl_sw_ch)
{
	for (int i = 0; i < 16; i++)
	{
		ctl_sw[i] = (ctl_sw_ch >> i) & 1;
	}
}

static uint32_t recv_cnt = 0;

void* nrf2401_pthread(void* arg)
{
	protocol_init();

	sem_init(&sem_sig, 0);

	uint16_t ctl[10] = { 0 };

	uint8_t rx_buff[32] = { 0 };
	uint8_t rx_len		= 0;

	NRF24L01_check();
	RF24L01_Init();
	RF24LL01_Write_Hopping_Point(64);
	NRF24L01_Set_Power(POWER_0DBM);
	NRF24L01_Set_Speed(SPEED_250K);

	float filter = 1.0f; // 不使用滤波（在发射端已经滤波了）

	float ctl_yaw_last	 = 0;
	float ctl_thro_last	 = 0;
	float ctl_roll_last	 = 0;
	float ctl_pitch_last = 0;

	RF24L01_Set_Mode(MODE_RX);

	while (1)
	{
		sem_wait(&sem_sig);
		rx_len = NRF24L01_Read_Reg(R_RX_PL_WID);		 // 读取接收到的数据个数
		NRF24L01_Read_Buf(RD_RX_PLOAD, rx_buff, rx_len); // 接收到数据
		// printf("rx_len %d recv_cnt %u\n", rx_len, recv_cnt);
		if (rx_len > 0)
		{
			// for (int i = 0; i < rx_len; i++)
			// {
			// 	printf("%02x ", rx_buff[i]);
			// }
			// printf("\n");
			protocol_append(rx_buff, rx_len);
		}

		rx_len = 0;
		NRF24L01_Write_Reg(FLUSH_RX, 0xff); // 清除RX FIFO
		NRF24L01_Clear_IRQ_Flag(IRQ_ALL);

		RF24L01_Set_Mode(MODE_RX);
		int ret = protocol_parse(ctl);
		if (ret == 0)
		{
			tk_recv = HAL_GetTick();

			float roll	= ((float)(ctl[0] - CTL_PWM_MIN)) / CTL_PWM_SCALE;
			float pitch = ((float)(ctl[1] - CTL_PWM_MIN)) / CTL_PWM_SCALE;
			float yaw	= ((float)(ctl[2] - CTL_PWM_MIN)) / CTL_PWM_SCALE;
			float thro	= ((float)(ctl[3] - CTL_PWM_MIN)) / CTL_PWM_SCALE;

			pitch = (1.0f - pitch * 2.0f);
			roll  = (1.0f - roll * 2.0f);
			yaw	  = (1.0f - yaw * 2.0f);

			// 反向
			pitch *= -1.0f;
			roll *= -1.0f;
			yaw *= -1.0f;

			if (fabs(pitch) < 0.01)
			{
				pitch = 0;
			}
			if (fabs(roll) < 0.01)
			{
				roll = 0;
			}
			if (fabs(yaw) < 0.01)
			{
				yaw = 0;
			}
			if (thro < 0.01)
			{
				thro = 0;
			}

			ctl_thro  = thro * filter + ctl_thro_last * (1.0f - filter);
			ctl_roll  = roll * filter + ctl_roll_last * (1.0f - filter);
			ctl_pitch = pitch * filter + ctl_pitch_last * (1.0f - filter);
			ctl_yaw	  = yaw * filter + ctl_yaw_last * (1.0f - filter);

			ctl_thro_last  = ctl_thro;
			ctl_roll_last  = ctl_roll;
			ctl_pitch_last = ctl_pitch;
			ctl_yaw_last   = ctl_yaw;

			ctl_switch(ctl[9]);

			// static uint32_t tk = 0;
			// if (tk++ % 2 == 0)
			// {
			// 	printf("%04d %04d %04d %04d ", (int)(ctl_thro * 1000), (int)(ctl_pitch * 1000), (int)(ctl_roll * 1000), (int)(ctl_yaw * 1000));
			// 	for (int i = 0; i < 16; i++)
			// 	{
			// 		printf("%d ", ctl_sw[i]);
			// 	}
			// 	printf("\n");
			// }
		}
		// msleep(1);
	}
	return NULL;
}

void* nrf2401_protected_pthread(void* arg)
{
	msleep(1000);

	while (1)
	{
		uint32_t tk_now = HAL_GetTick();
		if (tk_now - tk_recv > 1000)
		{
			ctl_thro  = 0;
			ctl_pitch = 0;
			ctl_roll  = 0;
			ctl_yaw	  = 0;

			led_off(1);

			RF24L01_Init();
			RF24LL01_Write_Hopping_Point(64);
			NRF24L01_Set_Power(POWER_0DBM);
			NRF24L01_Set_Speed(SPEED_250K);
		}
		else
		{
			led_on(1);
		}

		sem_post(&sem_sig);
		msleep(2000);
	}
	return NULL;
}

void nrf2401_task(void)
{
	pcb_create(PCB_NRF_PRIO, &nrf2401_pthread, NULL, PCB_NRF_SIZE);
	pcb_create(PCB_NRF_PT_PRIO, &nrf2401_protected_pthread, NULL, PCB_NRF_PT_SIZE);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	recv_cnt++;
	if (GPIO_Pin == GPIO_PIN_10)
	{
		sem_post(&sem_sig);
	}
}
