/*
 * task.c
 *
 *  Created on: December 23, 2019
 *      Author: lidq
 */

#include <buzzer_task.h>

#define S_DELAY_100 (100)
#define S_DELAY_150 (150)
#define S_DELAY_200 (200)
#define S_DELAY_500 (500)
#define S_DELAY_1000 (1000)
#define S_DELAY_2000 (2000)
#define S_DELAY_4000 (4000)

const static buzzer_s music[] = {
	//
	{
		.rate = BEE_C_Do_H, //
		.ms = S_DELAY_200	//
	},
	{
		.rate = BEE_C_Re_H, //
		.ms = S_DELAY_200	//
	},
	{
		.rate = BEE_C_Mi_H, //
		.ms = S_DELAY_200	//
	},
	{
		.rate = BEE_C_Do_H, //
		.ms = S_DELAY_200	//
	},
	{
		.rate = BEE_C_Re_H, //
		.ms = S_DELAY_200	//
	},
	{
		.rate = BEE_C_Mi_H, //
		.ms = S_DELAY_200	//
	},
	{
		.rate = BEE_MUTE, //
		.ms = S_DELAY_200 //
	}
	//
};

const static buzzer_s kick[] = {
	//
	{
		.rate = BEE_C_Do_H, //
		.ms = S_DELAY_150	//
	},
	{
		.rate = BEE_C_Re_H, //
		.ms = S_DELAY_150	//
	},
	{
		.rate = BEE_C_Mi_H, //
		.ms = S_DELAY_150	//
	},
	{
		.rate = BEE_C_Fa_H, //
		.ms = S_DELAY_150	//
	},
	{
		.rate = BEE_C_So_H, //
		.ms = S_DELAY_150	//
	},
	{
		.rate = BEE_C_La_H, //
		.ms = S_DELAY_150	//
	},
	{
		.rate = BEE_C_Si_H, //
		.ms = S_DELAY_150	//
	},
	{
		.rate = BEE_MUTE, //
		.ms = S_DELAY_150 //
	},
	{
		.rate = BEE_C_Do_H, //
		.ms = S_DELAY_150	//
	},
	{
		.rate = BEE_C_Re_H, //
		.ms = S_DELAY_150	//
	},
	{
		.rate = BEE_C_Mi_H, //
		.ms = S_DELAY_150	//
	},
	{
		.rate = BEE_C_Fa_H, //
		.ms = S_DELAY_150	//
	},
	{
		.rate = BEE_C_So_H, //
		.ms = S_DELAY_150	//
	},
	{
		.rate = BEE_C_La_H, //
		.ms = S_DELAY_150	//
	},
	{
		.rate = BEE_C_Si_H, //
		.ms = S_DELAY_150	//
	},
	{
		.rate = BEE_MUTE, //
		.ms = S_DELAY_150 //
	}
	//
};

static int status = -1;

void buzzer_set_status(int st)
{
	status = st;
}

void buzzer_play(const buzzer_s *music, int cnt)
{
	for (int i = 0; i < cnt; i++)
	{
		tim4_set_rate(music[i].rate);
		sleep_ticks(music[i].ms);
	}
}

void buzzer_pthread(void)
{
	buzzer_play(music, sizeof(music) / sizeof(buzzer_s));

	while (1)
	{
		if (status >= 0)
		{
			switch (status)
			{
			case 0:
				buzzer_play(music, sizeof(music) / sizeof(buzzer_s));
				break;

			case 1:
				buzzer_play(kick, sizeof(kick) / sizeof(buzzer_s));
				break;

			default:
				break;
			}
			status = -1;
		}
		sleep_ticks(100);
	}
}

void buzzer_task(void)
{
	pcb_create(PROI_BUZZER, &buzzer_pthread, NULL, 600);
}
