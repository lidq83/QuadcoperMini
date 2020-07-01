/*
 * task.c
 *
 *  Created on: December 23, 2019
 *      Author: lidq
 */

#include <typedef.h>
#include <led.h>
#include <core.h>
#include <sysclk.h>
#include <serial1.h>
#include <std.h>
#include <tim1.h>
#include <tim2.h>
#include <IOI2C.h>
#include <pwm.h>
#include <led_task.h>
#include <nrf_task.h>
#include <mpu6050_task.h>
#include <controller_task.h>

/***************************************************************************************
 * 
 * libs/STM32_USB-FS-Device_Lib_V4.0.0/Libraries/CMSIS/Device/ST/STM32F10x/Include/stm32f10x.h
 * #define HSE_VALUE    ((uint32_t)16000000)
 * #define HSI_VALUE    ((uint32_t)16000000)
 *
 ***************************************************************************************/

static void rcc_config();

int main(int argc, char *argv[])
{
	SystemInit();
	sysclk_init();
	rcc_config();
	led_init();
	serial1_init();
	tim2_init();

	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	//GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

	kernel_startup();

	pwm_init();

	led_task();
	mpu6050_task();
	nrf_task();
	controller_task();

	tim1_init();

	while (1)
	{
	}
}

void rcc_config()
{
	ErrorStatus HSEStartUpStatus;

	RCC_DeInit();

	RCC_HSEConfig(RCC_HSE_ON);

	HSEStartUpStatus = RCC_WaitForHSEStartUp();

	if (HSEStartUpStatus == SUCCESS)
	{
		RCC_HCLKConfig(RCC_SYSCLK_Div1);
		RCC_PCLK1Config(RCC_HCLK_Div2);
		RCC_PCLK2Config(RCC_HCLK_Div1);

		// 16 / 2 * 9 = 72
		RCC_PLLConfig(RCC_PLLSource_HSE_Div2, RCC_PLLMul_9);

		RCC_PLLCmd(ENABLE);
		while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
		{
		}

		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
		while (RCC_GetSYSCLKSource() != 0x08)
		{
		}
	}
}
