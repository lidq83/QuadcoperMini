#ifndef __PWM_H_
#define __PWM_H_

#include <typedef.h>
#include <fcntl.h>
#include <fs.h>
#include <tim2.h>
#include <tim4.h>

#define PWM_CMD_SET_CH0_VALUE (0x0100)
#define PWM_CMD_SET_CH1_VALUE (0x0110)
#define PWM_CMD_SET_CH2_VALUE (0x0120)
#define PWM_CMD_SET_CH3_VALUE (0x0130)
#define PWM_CMD_SET_CH4_VALUE (0x0140)
#define PWM_CMD_SET_CH5_VALUE (0x0150)
#define PWM_CMD_SET_CH6_VALUE (0x0160)
#define PWM_CMD_SET_CH7_VALUE (0x0170)

void pwm_init(void);

#endif