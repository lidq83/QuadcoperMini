/*
 * task.h
 *
 *  Created on: December 23, 2019
 *      Author: lidq
 */
#ifndef __SRC_BUZZER_TASK_H
#define __SRC_BUZZER_TASK_H

#include <typedef.h>
#include <sche.h>
#include <led.h>
#include <tim4.h>

#define BEE_MUTE 0
#define BEE_C_Do_L 131
#define BEE_C_Re_L 147
#define BEE_C_Mi_L 165
#define BEE_C_Fa_L 175
#define BEE_C_So_L 196
#define BEE_C_La_L 221
#define BEE_C_Si_L 248
#define BEE_C_Do_M 262
#define BEE_C_Re_M 294
#define BEE_C_Mi_M 330
#define BEE_C_Fa_M 350
#define BEE_C_So_M 393
#define BEE_C_La_M 441
#define BEE_C_Si_M 495
#define BEE_C_Do_H 525
#define BEE_C_Re_H 589
#define BEE_C_Mi_H 661
#define BEE_C_Fa_H 700
#define BEE_C_So_H 786
#define BEE_C_La_H 882
#define BEE_C_Si_H 990

typedef struct buzzer_s
{
	uint16_t rate;
	uint16_t ms;
} buzzer_s;

void buzzer_set_status(int st);

void buzzer_task(void);

#endif
