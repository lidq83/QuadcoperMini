/*
 * task.h
 *
 *  Created on: December 23, 2019
 *      Author: lidq
 */
#ifndef __SRC_MOTOR_TASK_H
#define __SRC_MOTOR_TASK_H

#include <typedef.h>
#include <sche.h>
#include <sem.h>
#include <pwm.h>
#include <mm.h>

#define MOTOR_CNT (2)

void motor_task(void);

void motor_set_value(int motor, float value);

#endif
