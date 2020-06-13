#include <led.h>

void led_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	led_off(0);
	led_off(1);
	led_off(2);
}

void led_on(int led)
{
	switch (led)
	{
	case 0:
		GPIO_WriteBit(GPIOB, GPIO_Pin_6, 0);
		break;

	case 1:
		GPIO_WriteBit(GPIOB, GPIO_Pin_7, 0);
		break;

	case 2:
		GPIO_WriteBit(GPIOB, GPIO_Pin_8, 0);
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
		GPIO_WriteBit(GPIOB, GPIO_Pin_6, 1);
		break;

	case 1:
		GPIO_WriteBit(GPIOB, GPIO_Pin_7, 1);
		break;

	case 2:
		GPIO_WriteBit(GPIOB, GPIO_Pin_8, 1);
		break;

	default:
		break;
	}
}

void led_blink(int led)
{
	static int led_0 = 0;
	static int led_1 = 0;
	static int led_2 = 0;

	switch (led)
	{
	case 0:
		GPIO_WriteBit(GPIOB, GPIO_Pin_6, led_0);
		led_0 = !led_0;
		break;

	case 1:
		GPIO_WriteBit(GPIOB, GPIO_Pin_7, led_1);
		led_1 = !led_1;
		break;

	case 2:
		GPIO_WriteBit(GPIOB, GPIO_Pin_8, led_2);
		led_2 = !led_2;
		break;

	default:
		break;
	}
}
