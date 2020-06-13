#include <digital_tube.h>

static dt_num_s DT_DOT = {GPIOB, GPIO_Pin_12};
static dt_num_s dt_digs[2] = {{GPIOB, GPIO_Pin_5}, {GPIOB, GPIO_Pin_15}};
static uint8_t num[10] = {DT_NUM_0, DT_NUM_1, DT_NUM_2, DT_NUM_3, DT_NUM_4, DT_NUM_5, DT_NUM_6, DT_NUM_7, DT_NUM_8, DT_NUM_9};
static dt_num_s dts[7] = {
	//A
	{GPIOB, GPIO_Pin_13},
	//B
	{GPIOB, GPIO_Pin_4},
	//C
	{GPIOB, GPIO_Pin_3},
	//D
	{GPIOA, GPIO_Pin_12},
	//E
	{GPIOA, GPIO_Pin_15},
	//F
	{GPIOB, GPIO_Pin_14},
	//G
	{GPIOB, GPIO_Pin_11}};

static void dt_init_gpio(dt_num_s *num);

void dt_init_gpio(dt_num_s *num)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = num->GPIO_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(num->GPIOx, &GPIO_InitStructure);

	GPIO_ResetBits(num->GPIOx, num->GPIO_Pin);
}

void dt_init()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	for (int i = 0; i < 7; i++)
	{
		dt_init_gpio(&dts[i]);
	}

	for (int i = 0; i < 2; i++)
	{
		dt_init_gpio(&dt_digs[i]);
	}

	dt_init_gpio(&DT_DOT);
}

void dt_show_num(uint8_t n, int dig, int dot)
{
	for (int i = 0; i < 2; i++)
	{
		if (i == dig)
		{
			GPIO_ResetBits(dt_digs[i].GPIOx, dt_digs[i].GPIO_Pin);
		}
		else
		{
			GPIO_SetBits(dt_digs[i].GPIOx, dt_digs[i].GPIO_Pin);
		}
	}

	GPIO_ResetBits(DT_DOT.GPIOx, DT_DOT.GPIO_Pin);
	if (dot)
	{
		GPIO_ResetBits(dt_digs[dig].GPIOx, dt_digs[dig].GPIO_Pin);
		GPIO_SetBits(DT_DOT.GPIOx, DT_DOT.GPIO_Pin);
	}

	if (n >= 10)
	{
		for (int i = 0; i < 7; i++)
		{
			GPIO_ResetBits(dts[i].GPIOx, dts[i].GPIO_Pin);
		}

		return;
	}

	uint8_t dig_num = num[n];
	for (int i = 0; i < 7; i++)
	{
		if (dig_num >> i & 1)
		{
			GPIO_SetBits(dts[i].GPIOx, dts[i].GPIO_Pin);
		}
		else
		{
			GPIO_ResetBits(dts[i].GPIOx, dts[i].GPIO_Pin);
		}
	}
}

void dt_hide_num(int dig)
{
	for (int i = 0; i < 2; i++)
	{
		if (i == dig)
		{
			GPIO_ResetBits(dt_digs[i].GPIOx, dt_digs[i].GPIO_Pin);
		}
		else
		{
			GPIO_SetBits(dt_digs[i].GPIOx, dt_digs[i].GPIO_Pin);
		}
	}

	for (int i = 0; i < 7; i++)
	{
		GPIO_ResetBits(dts[i].GPIOx, dts[i].GPIO_Pin);
	}
}