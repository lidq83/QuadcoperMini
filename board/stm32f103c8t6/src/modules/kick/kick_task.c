/*
 * task.c
 *
 *  Created on: December 23, 2019
 *      Author: lidq
 */

#include <kick_task.h>
#include <dt_task.h>

typedef struct kick_s
{
	uint8_t id;
	uint8_t st;
	uint8_t lp;
	void (*event_press)();
	void (*event_release)();
	void (*event_repeat)();
} kick_s;

static kick_s kicks[KICK_CNT] = {0};

static float kick_val[KICK_CNT] = {0};
static float last_val[KICK_CNT] = {0};
static float filter = 0.1;

static uint32_t kick_long_pre_tick[KICK_CNT] = {0};
static uint32_t kick_repeat_tick[KICK_CNT] = {0};

static void init_gpio(void);
static void init_kicks(void);
static void kicks_status(void);
static void kicks_events(void);

static void kick_event_press(kick_s *kick);
static void kick_event_release(kick_s *kick);
static void kick_event_repeat(kick_s *kick);

void kick_event_press(kick_s *kick)
{
	dt_add_num();
}

void kick_event_release(kick_s *kick)
{
}

void kick_event_repeat(kick_s *kick)
{
}

void init_gpio(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_SetBits(GPIOC, GPIO_Pin_13);
	GPIO_SetBits(GPIOC, GPIO_Pin_14);
	GPIO_SetBits(GPIOC, GPIO_Pin_15);
	GPIO_SetBits(GPIOA, GPIO_Pin_4);
}

void init_kicks(void)
{
	for (int i = 0; i < KICK_CNT; i++)
	{
		kicks[i].id = i;
		kicks[i].st = 0;
		kicks[i].lp = 0;
		kicks[i].event_press = kick_event_press;
		kicks[i].event_release = kick_event_release;
		kicks[i].event_repeat = kick_event_repeat;
	}
}

void kicks_status(void)
{
	if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13) == Bit_RESET)
	{
		kick_val[0] = 1.0f * filter + last_val[0] * (1.0f - filter);
		last_val[0] = kick_val[0];
	}
	else
	{
		kick_val[0] = 0.0f * filter + last_val[0] * (1.0f - filter);
		last_val[0] = kick_val[0];
	}

	if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_14) == Bit_RESET)
	{
		kick_val[1] = 1.0f * filter + last_val[1] * (1.0f - filter);
		last_val[1] = kick_val[1];
	}
	else
	{
		kick_val[1] = 0.0f * filter + last_val[1] * (1.0f - filter);
		last_val[1] = kick_val[1];
	}

	if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_15) == Bit_RESET)
	{
		kick_val[2] = 1.0f * filter + last_val[2] * (1.0f - filter);
		last_val[2] = kick_val[2];
	}
	else
	{
		kick_val[2] = 0.0f * filter + last_val[2] * (1.0f - filter);
		last_val[2] = kick_val[2];
	}

	if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4) == Bit_RESET)
	{
		kick_val[3] = 1.0f * filter + last_val[3] * (1.0f - filter);
		last_val[3] = kick_val[3];
	}
	else
	{
		kick_val[3] = 0.0f * filter + last_val[3] * (1.0f - filter);
		last_val[3] = kick_val[3];
	}
}

void kicks_events(void)
{
	for (int i = 0; i < KICK_CNT; i++)
	{
		if (kick_val[i] > 0.5f)
		{
			if (kicks[i].st == 0)
			{
				kicks[i].st = 1;
				kicks[i].event_press(&kicks[i]);
			}
		}
		else
		{
			if (kicks[i].st == 1)
			{
				kicks[i].st = 0;
				kicks[i].event_release(&kicks[i]);
			}
		}

		if (kicks[i].st == 1)
		{
			kick_long_pre_tick[i]++;
		}
		else
		{
			kick_long_pre_tick[i] = 0;
			kick_repeat_tick[i] = 0;
		}

		if (kicks[i].st == 1 && kick_long_pre_tick[i] > 100 * 2)
		{
			if (kick_repeat_tick[i]++ % 25 == 0)
			{
				kicks[i].lp = 1;
				kicks[i].event_repeat(&kicks[i]);
			}
		}
	}
}

void kick_pthread(void *arg)
{
	init_gpio();

	init_kicks();

	while (1)
	{
		kicks_status();

		kicks_events();

		sleep_ticks(10);
	}
}

void kick_task(void)
{
	pcb_create(PROI_KICK, &kick_pthread, NULL, 800);
}
