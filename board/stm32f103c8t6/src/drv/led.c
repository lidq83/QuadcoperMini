#include <led.h>

void led_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	led_off(0);
	led_off(1);
}

void led_on(int led)
{
	switch (led)
	{
	case 0:
		GPIO_WriteBit(GPIOA, GPIO_Pin_11, 0);
		break;

	case 1:
		GPIO_WriteBit(GPIOA, GPIO_Pin_12, 0);
		break;

	default:
		break;
	}
}

void led_off(int led)
{
	switch (led)
	{
	case 0:
		GPIO_WriteBit(GPIOA, GPIO_Pin_11, 1);
		break;

	case 1:
		GPIO_WriteBit(GPIOA, GPIO_Pin_12, 1);
		break;

	default:
		break;
	}
}

void led_blink(int led)
{
	static int led_0 = 0;
	static int led_1 = 0;

	switch (led)
	{
	case 0:
		GPIO_WriteBit(GPIOA, GPIO_Pin_11, led_0);
		led_0 = !led_0;
		break;

	case 1:
		GPIO_WriteBit(GPIOA, GPIO_Pin_12, led_1);
		led_1 = !led_1;
		break;

	default:
		break;
	}
}
