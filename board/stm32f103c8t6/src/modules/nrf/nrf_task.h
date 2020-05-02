/*
 * task.h
 *
 *  Created on: December 23, 2019
 *      Author: lidq
 */
#ifndef __SRC_NRF_TASK_H
#define __SRC_NRF_TASK_H

#include <typedef.h>
#include <sche.h>
#include <sem.h>
#include <mm.h>
#include <nrf.h>
#include <protocol.h>
#include <motor_task.h>

#define CTL_PWM_MAX (2200)
#define CTL_PWM_MIN (800)
#define CTL_PWM_SCALE (CTL_PWM_MAX - CTL_PWM_MIN)

void nrf_task(void);

#endif
