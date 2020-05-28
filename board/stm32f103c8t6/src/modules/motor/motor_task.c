/*
 * task.c
 *
 *  Created on: December 23, 2019
 *      Author: lidq
 */

#include <motor_task.h>

static void motor_set_value(int fd, int motor, float value);

static void motor_pthread(void *arg)
{
	int fd = open("/dev/pwm", 0, 0);
	if (fd < 0)
	{
		k_printf("open pwm error.\n");
		return;
	}

	motor_set_value(fd, 0, 1.0f);
	motor_set_value(fd, 1, 1.0f);

	while (1)
	{
		led_blink(1);
		sleep_ticks(100);
	}

	close(fd);
}

void motor_set_value(int fd, int motor, float value)
{
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
			ioctl(fd, PWM_CMD_SET_CH0_VALUE, 0);
			ioctl(fd, PWM_CMD_SET_CH1_VALUE, 0);
			return;
		}
		if (value > 0.0)
		{
			int pwm_val = (PWM_VAL_MAX - PWM_VAL_MIN) * value;

			ioctl(fd, PWM_CMD_SET_CH1_VALUE, 0);
			ioctl(fd, PWM_CMD_SET_CH0_VALUE, pwm_val);

			return;
		}
		if (value < 0.0)
		{
			int pwm_val = (PWM_VAL_MAX - PWM_VAL_MIN) * value;

			ioctl(fd, PWM_CMD_SET_CH0_VALUE, 0);
			ioctl(fd, PWM_CMD_SET_CH1_VALUE, pwm_val);

			return;
		}
	}

	if (motor == 1)
	{
		if (fabs(value) < 0.01)
		{
			ioctl(fd, PWM_CMD_SET_CH2_VALUE, 0);
			ioctl(fd, PWM_CMD_SET_CH3_VALUE, 0);
			return;
		}
		if (value > 0.0)
		{
			int pwm_val = (PWM_VAL_MAX - PWM_VAL_MIN) * value;

			ioctl(fd, PWM_CMD_SET_CH3_VALUE, 0);
			ioctl(fd, PWM_CMD_SET_CH2_VALUE, pwm_val);

			return;
		}
		if (value < 0.0)
		{
			int pwm_val = (PWM_VAL_MAX - PWM_VAL_MIN) * value;

			ioctl(fd, PWM_CMD_SET_CH2_VALUE, 0);
			ioctl(fd, PWM_CMD_SET_CH3_VALUE, pwm_val);

			return;
		}
	}
}

void motor_task(void)
{
	pcb_create(24, &motor_pthread, NULL, 1024);
}