#include <led.h>

void led_init(void)
{
}

void led_on()
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
}

void led_off()
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
}

void led_blink()
{
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_11);
}