/*
 * task.c
 *
 *  Created on: December 23, 2019
 *      Author: lidq
 */

#include <motor_task.h>

float motor_ctl[MOTOR_CNT] = {0};

static void motor_set_value(int fd, int motor, float value);

static void motor_pthread(void *arg)
{
	int fd = open("/dev/pwm", 0, 0);
	if (fd < 0)
	{
		k_printf("open pwm error.\n");
		return;
	}

	for (int i = 0; i < MOTOR_CNT; i++)
	{
		motor_set_value(fd, i, 0.0f);
	}

	while (1)
	{
		for (int i = 0; i < MOTOR_CNT; i++)
		{
			motor_set_value(fd, i, motor_ctl[i]);
		}

		sleep_ticks(10);
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

#ifdef MOTOR_TYPE_CAR
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
			int pwm_val = (PWM_VAL_MAX - PWM_VAL_MIN) * fabs(value);
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
			int pwm_val = (PWM_VAL_MAX - PWM_VAL_MIN) * fabs(value);
			ioctl(fd, PWM_CMD_SET_CH2_VALUE, 0);
			ioctl(fd, PWM_CMD_SET_CH3_VALUE, pwm_val);

			return;
		}
	}
#endif

#ifdef MOTOR_TYPE_BOAT
	if (motor == 0)
	{
		if (fabs(value) < 0.01)
		{
			ioctl(fd, PWM_CMD_SET_CH4_VALUE, PWM_VAL_MIN);
			return;
		}
		if (value < 0.0)
		{
			int pwm_val = PWM_VAL_MIN + (PWM_VAL_MAX - PWM_VAL_MIN) * fabs(value);
			ioctl(fd, PWM_CMD_SET_CH4_VALUE, pwm_val);
			return;
		}
	}
	if (motor == 1)
	{
		if (fabs(value) < 0.01)
		{
			ioctl(fd, PWM_CMD_SET_CH5_VALUE, PWM_VAL_MID);
			return;
		}

		int pwm_val = PWM_VAL_MID + (PWM_VAL_MAX - PWM_VAL_MIN) / 2 * value;
		ioctl(fd, PWM_CMD_SET_CH5_VALUE, pwm_val);
		return;
	}
#endif
}

void motor_task(void)
{
	pcb_create(24, &motor_pthread, NULL, 1024);
}