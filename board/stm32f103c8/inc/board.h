#ifndef __BOARD_H_
#define __BOARD_H_

#include "stm32f1xx_hal.h"

#define PROCESS_CNT (8)
#define FNODE_CNT (8)

// 控制程序
#define PCB_CTL_PRIO (0)
#define PCB_CTL_SIZE (800)

#define PCB_ATTITUDE_PRIO (1)
#define PCB_ATTITUDE_SIZE (1200)

// 遥控信号
#define PCB_NRF_PRIO (3)
#define PCB_NRF_SIZE (800)
#define PCB_NRF_PT_PRIO (4)
#define PCB_NRF_PT_SIZE (600)

// LED灯
#define PCB_LED_PRIO (5)
#define PCB_LED_SIZE (600)

// 空闲进程
#define PCB_IDLE_PRIO (PROCESS_CNT - 2)
#define PCB_IDLE_SIZE (200)

// 清理进程
#define PCB_CLEANER_PRIO (PROCESS_CNT - 1)
#define PCB_CLEANER_SIZE (400)

#define CTL_PWM_MAX (9999)
#define CTL_PWM_MIN (0)
#define CTL_PWM_SCALE (CTL_PWM_MAX - CTL_PWM_MIN)

#define PWM_MAX (10000)
#define PWM_MIN (0)

#ifndef M_PI
#define M_PI (3.1415926535897932)
#endif

#define ONE_G (9.80665)

typedef struct
{
	double w, x, y, z;
} Quaternion;

typedef struct
{
	double x, y, z;
} Vector3f;

#endif
