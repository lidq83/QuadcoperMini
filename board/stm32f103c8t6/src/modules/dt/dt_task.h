/*
 * task.h
 *
 *  Created on: December 23, 2019
 *      Author: lidq
 */
#ifndef __SRC_TASK_DT_H
#define __SRC_TASK_DT_H

#include <typedef.h>
#include <sche.h>
#include <sem.h>
#include <mm.h>
#include <digital_tube.h>
#include <sysclk.h>
#include <tim2.h>

void dt_task(void);

void dt_add_num(void);

#endif
