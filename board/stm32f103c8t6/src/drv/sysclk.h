/*
 * sysclk.h
 *
 *  Created on: Dec 24, 2019
 *      Author: lidq
 */

#ifndef __MODULES_SYSCLK_H
#define __MODULES_SYSCLK_H

#include <typedef.h>

//初始化系统时钟
void sysclk_init(void);

void delay_ms(u16 ms);

void delay_us(u32 us);

#endif
