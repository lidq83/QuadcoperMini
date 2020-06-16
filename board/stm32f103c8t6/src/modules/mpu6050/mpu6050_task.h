/*
 * task.h
 *
 *  Created on: December 23, 2019
 *      Author: lidq
 */
#ifndef __SRC_MPU6050_TASK_H
#define __SRC_MPU6050_TASK_H

#include <typedef.h>
#include <sche.h>
#include <sem.h>
#include <pwm.h>
#include <mm.h>
#include <i2c.h>
#include <mpu6050.h>

void mpu6050_task(void);

#endif
