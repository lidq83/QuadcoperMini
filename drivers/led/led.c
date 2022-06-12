#include <led.h>

void led_init(void)
{
}

void led_on(int num)
{
	if (num == 0)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
	}
	else if (num == 1)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
	}
}

void led_off(int num)
{
	if (num == 0)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
	}
	else if (num == 1)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
	}
}

void led_blink(int num)
{
	if (num == 0)
	{
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_11);
	}
	else if (num == 1)
	{
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
	}
}