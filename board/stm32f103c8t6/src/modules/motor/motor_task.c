/*
 * task.c
 *
 *  Created on: December 23, 2019
 *      Author: lidq
 */

#include <motor_task.h>

static int _fd = -1;
static float power[MOTOR_CNT] = {0};

static void motor_pthread(void *arg)
{
	_fd = open("/dev/pwm", 0, 0);
	if (_fd < 0)
	{
		k_printf("open pwm error.\n");
		return;
	}
}

void motor_task(void)
{
	pcb_create(24, &motor_pthread, NULL, 1024);
}

void motor_set_value(int motor, float value)
{
	if (_fd < 0)
	{
		return;
	}

	if (value > 1.0f)
	{
		value = 1.0f;
	}
	if (value < -1.0f)
	{
		value = -1.0f;
	}

	if (motor == 0)
	{
		if (fabs(value) < 0.01)
		{
			ioctl(_fd, PWM_CMD_SET_CH0_VALUE, 0);
			ioctl(_fd, PWM_CMD_SET_CH1_VALUE, 0);
			return;
		}
		if (value > 0.0)
		{
			int pwm_val = (PWM_VAL_MAX - PWM_VAL_MIN) * value;
			ioctl(_fd, PWM_CMD_SET_CH0_VALUE, pwm_val);
			ioctl(_fd, PWM_CMD_SET_CH1_VALUE, 0);
			return;
		}
		if (value < 0.0)
		{
			int pwm_val = (PWM_VAL_MAX - PWM_VAL_MIN) * value;
			ioctl(_fd, PWM_CMD_SET_CH0_VALUE, 0);
			ioctl(_fd, PWM_CMD_SET_CH1_VALUE, pwm_val);
			return;
		}
	}

	if (motor == 1)
	{
		if (fabs(value) < 0.01)
		{
			ioctl(_fd, PWM_CMD_SET_CH2_VALUE, 0);
			ioctl(_fd, PWM_CMD_SET_CH3_VALUE, 0);
			return;
		}
		if (value > 0.0)
		{
			int pwm_val = (PWM_VAL_MAX - PWM_VAL_MIN) * value;
			ioctl(_fd, PWM_CMD_SET_CH2_VALUE, pwm_val);
			ioctl(_fd, PWM_CMD_SET_CH3_VALUE, 0);
			return;
		}
		if (value < 0.0)
		{
			int pwm_val = (PWM_VAL_MAX - PWM_VAL_MIN) * value;
			ioctl(_fd, PWM_CMD_SET_CH2_VALUE, 0);
			ioctl(_fd, PWM_CMD_SET_CH3_VALUE, pwm_val);
			return;
		}
	}
}
