#ifndef __PWM_H_
#define __PWM_H_

#include <typedef.h>
#include <fcntl.h>
#include <fs.h>
#include <tim2.h>

#define PWM_CMD_SET_CH0_VALUE (0x0100)
#define PWM_CMD_SET_CH1_VALUE (0x0110)
#define PWM_CMD_SET_CH2_VALUE (0x0120)
#define PWM_CMD_SET_CH3_VALUE (0x0130)

void pwm_init(void);

#endif