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
#include <tim2.h>
#include <tim4.h>
#include <pwm.h>
#include <debug_task.h>
#include <led_task.h>
#include <motor_task.h>
#include <nrf_task.h>
#include <btn_task.h>

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

	rcc_config();

	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); //禁用JTAG

	kernel_startup();

	led_init();
	serial1_init();
	tim2_init();
	tim4_init();
	pwm_init();

	led_task();

	motor_task();
	nrf_task();
	btn_task();
	debug_task();

	sysclk_init();

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
