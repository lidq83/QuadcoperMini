/*
 * task.c
 *
 *  Created on: December 23, 2019
 *      Author: lidq
 */

#include <btn_task.h>

typedef struct btn_s
{
	uint8_t id;
	uint8_t st;
	uint8_t lp;
	void (*eventPress)();
	void (*eventRelease)();
	void (*eventRepeat)();
} btn_s;

static btn_s btns[BUTTON_CNT] = {0};

static float btn_val[BUTTON_CNT] = {0};
static float last_val[BUTTON_CNT] = {0};
static float filter = 0.1;

static uint32_t btn_long_pre_tick[BUTTON_CNT] = {0};
static uint32_t btn_repeat_tick[BUTTON_CNT] = {0};

static void init_gpio(void);
static void init_btns(void);
static void btns_status(void);
static void btns_events(void);

static void btn_event_press(btn_s *btn);
static void btn_event_release(btn_s *btn);
static void btn_event_repeat(btn_s *btn);

int power_level = 0;

void btn_event_press(btn_s *btn)
{

}

void btn_event_release(btn_s *btn)
{
	if (btn->lp == 1)
	{
		//长按后不执行release动作
		btn->lp = 0;
		return;
	}

	if (btn->id == 0)
	{
		power_level++;
		power_level %= 5;
	}
	else if (btn->id == 1)
	{
	}
}

void btn_event_repeat(btn_s *btn)
{
	if (btn->id == 0)
	{
	}
	else if (btn->id == 1)
	{
	}
}

void init_gpio(void)
{
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); //禁用JTAG

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_SetBits(GPIOB, GPIO_Pin_4);
	GPIO_SetBits(GPIOB, GPIO_Pin_5);
}

void init_btns(void)
{
	for (int i = 0; i < BUTTON_CNT; i++)
	{
		btns[i].id = i;
		btns[i].st = 0;
		btns[i].lp = 0;
		btns[i].eventPress = btn_event_press;
		btns[i].eventRelease = btn_event_release;
		btns[i].eventRepeat = btn_event_repeat;
	}
}

void btns_status(void)
{
	if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4) == Bit_RESET)
	{
		btn_val[0] = 1.0f * filter + last_val[0] * (1.0f - filter);
		last_val[0] = btn_val[0];
	}
	else
	{
		btn_val[0] = 0.0f * filter + last_val[0] * (1.0f - filter);
		last_val[0] = btn_val[0];
	}

	if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5) == Bit_RESET)
	{
		btn_val[1] = 1.0f * filter + last_val[1] * (1.0f - filter);
		last_val[1] = btn_val[1];
	}
	else
	{
		btn_val[1] = 0.0f * filter + last_val[1] * (1.0f - filter);
		last_val[1] = btn_val[1];
	}
}

void btns_events(void)
{
	for (int i = 0; i < BUTTON_CNT; i++)
	{
		if (btn_val[i] > 0.5f)
		{
			if (btns[i].st == 0)
			{
				btns[i].st = 1;
				btns[i].eventPress(&btns[i]);
			}
		}
		else
		{
			if (btns[i].st == 1)
			{
				btns[i].st = 0;
				btns[i].eventRelease(&btns[i]);
			}
		}

		if (btns[i].st == 1)
		{
			btn_long_pre_tick[i]++;
		}
		else
		{
			btn_long_pre_tick[i] = 0;
			btn_repeat_tick[i] = 0;
		}

		if (btns[i].st == 1 && btn_long_pre_tick[i] > 100 * 2)
		{
			if (btn_repeat_tick[i]++ % 25 == 0)
			{
				btns[i].lp = 1;
				btns[i].eventRepeat(&btns[i]);
			}
		}
	}
}

void btn_pthread(void *arg)
{
	init_gpio();

	init_btns();

	while (1)
	{
		btns_status();

		btns_events();

		sleep_ticks(10);
	}
}

void btn_task(void)
{
	pcb_create(25, &btn_pthread, NULL, 800);
}
