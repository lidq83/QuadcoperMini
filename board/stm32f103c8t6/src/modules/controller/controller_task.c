/*
 * task.c
 *
 *  Created on: December 23, 2019
 *      Author: lidq
 */

#include <controller_task.h>
#include <k_printf.h>

const float param_angle_p = 12.7f;

const float param_gyro_p = 0.037;
const float param_gyro_i = 0.0023;
const float param_gyro_d = 0.007;

extern float ctl_thro;
extern float ctl_roll;
extern float ctl_pitch;
extern float ctl_yaw;

extern float att_angle[3];
extern float att_gyro[3];

float angle_pid(float x)
{
	return x * param_angle_p;
}

float gyro_pid(float x, float x_last, float *gyro_integral)
{
	float val_p = x * param_gyro_p;
	float val_d = (x - x_last) * param_gyro_d;
	*gyro_integral += x * param_gyro_i;
	if (*gyro_integral > 0 && *gyro_integral > val_p)
	{
		*gyro_integral = val_p;
	}
	if (*gyro_integral < 0 && *gyro_integral < val_p)
	{
		*gyro_integral = val_p;
	}
	return val_p + (*gyro_integral) + val_d;
}

void ctl_limit(float *val, float limit)
{
	if (*val > limit)
	{
		*val = limit;
	}
	if (*val < -limit)
	{
		*val = -limit;
	}
}

void motor_set_value(int fd, int motor, float value)
{
	if (value > 1.0f)
	{
		value = 1.0f;
	}
	if (value < 0.0f)
	{
		value = 0.0f;
	}

	int pwm_val = PWM_VAL_MIN + (PWM_VAL_MAX - PWM_VAL_MIN) * value;
	if (motor == 0)
	{
		ioctl(fd, PWM_CMD_SET_CH0_VALUE, pwm_val);
		return;
	}
	if (motor == 1)
	{
		ioctl(fd, PWM_CMD_SET_CH1_VALUE, pwm_val);
		return;
	}
	if (motor == 2)
	{
		ioctl(fd, PWM_CMD_SET_CH2_VALUE, pwm_val);
		return;
	}
	if (motor == 3)
	{
		ioctl(fd, PWM_CMD_SET_CH3_VALUE, pwm_val);
		return;
	}
}

void controller_pthread(void *arg)
{
	float motor_ctl[MOTOR_CNT] = {0};

	float att_angle_offset[3] = {0};
	float att_gyro_offset[3] = {0};

	float att_angle_offset_last[3] = {0};
	float att_gyro_offset_last[3] = {0};

	float ctl_roll_offset = 0;
	float ctl_pitch_offset = 0;
	float ctl_yaw_offset = 0;

	float ctl_roll_offset_last = 0;
	float ctl_pitch_offset_last = 0;
	float ctl_yaw_offset_last = 0;

	float filter_offset = 0.05;

	float rate_dval_roll_last = 0;
	float rate_dval_pitch_last = 0;
	float rate_dval_yaw_last = 0;

	float gyro_integral_roll = 0.0;
	float gyro_integral_pitch = 0.0;
	float gyro_integral_yaw = 0.0;

	int fd = open("/dev/pwm", 0, 0);
	if (fd < 0)
	{
		k_printf("[ERROR] could not open /dev/pwm\n");
		return;
	}

	for (int i = 0; i < MOTOR_CNT; i++)
	{
		motor_set_value(fd, i, 0.0f);
	}

	while (1)
	{
		// k_printf("%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n", att_angle[0], att_angle[1], att_angle[2], att_gyro[0], att_gyro[1], att_gyro[2]);
		// k_printf("%.3f\t%.3f\t%.3f\t%.3f\n", ctl_thro, ctl_roll, ctl_pitch, ctl_yaw);

		//外环控制
		//姿态误差 * P = 角速度期望
		float exp_rate_roll = angle_pid((ctl_roll - ctl_roll_offset) - (att_angle[1] - att_angle_offset[1]));
		float exp_rate_pitch = angle_pid((ctl_pitch - ctl_pitch_offset) - (att_angle[0] - att_angle_offset[0]));
		float exp_rate_yaw = angle_pid((ctl_yaw - ctl_yaw_offset) - (att_angle[2] - att_angle_offset[2]));

		//内环控制
		//角速度期望 - 实际 = 误差
		float rate_dval_roll = exp_rate_roll - (att_gyro[0] - att_gyro_offset[0]);
		float rate_dval_pitch = exp_rate_pitch - (att_gyro[1] - att_gyro_offset[1]);
		float rate_dval_yaw = exp_rate_yaw - (att_gyro[2] - att_gyro_offset[2]);

		//PID控制：比例 + 积分 + 微分 = 控制量
		float out_control_roll = gyro_pid(rate_dval_roll, rate_dval_roll_last, &gyro_integral_roll);
		float out_control_pitch = gyro_pid(rate_dval_pitch, rate_dval_pitch_last, &gyro_integral_pitch);
		float out_control_yaw = gyro_pid(rate_dval_yaw, rate_dval_yaw_last, &gyro_integral_yaw);

		rate_dval_roll_last = rate_dval_roll;
		rate_dval_pitch_last = rate_dval_pitch;
		rate_dval_yaw_last = rate_dval_yaw;

		// k_printf("%.4f\t %.4f\t%.4f\t%.4f\t%.4f\n", att_angle[1], exp_rate_roll, att_gyro[0], rate_dval_roll, out_control_roll);

		// motor_ctl[0] = ctl_thro - out_control_roll;
		// motor_ctl[1] = ctl_thro + out_control_roll;
		// motor_ctl[2] = ctl_thro + out_control_roll;
		// motor_ctl[3] = ctl_thro - out_control_roll;
		motor_ctl[0] = ctl_thro - out_control_roll + out_control_pitch;
		motor_ctl[1] = ctl_thro + out_control_roll + out_control_pitch;
		motor_ctl[2] = ctl_thro + out_control_roll - out_control_pitch;
		motor_ctl[3] = ctl_thro - out_control_roll - out_control_pitch;

		// for (int i = 0; i < MOTOR_CNT; i++)
		// {
		// 	k_printf("%.4f\t", motor_ctl[i]);
		// }
		// k_printf("\n");

		// motor_ctl[0] = 0;
		// motor_ctl[1] = 0;
		// motor_ctl[2] = 0;
		// motor_ctl[3] = 0;

		for (int i = 0; i < MOTOR_CNT; i++)
		{
			motor_set_value(fd, i, motor_ctl[i]);
		}

		if (ctl_thro < 0.1)
		{
			gyro_integral_roll = 0;
			gyro_integral_pitch = 0;
			gyro_integral_yaw = 0;

			motor_ctl[0] = 0;
			motor_ctl[1] = 0;
			motor_ctl[2] = 0;
			motor_ctl[3] = 0;

			for (int i = 0; i < MOTOR_CNT; i++)
			{
				motor_set_value(fd, i, motor_ctl[i]);
			}

			att_angle_offset[0] = att_angle[0] * filter_offset + att_angle_offset_last[0] * (1.0f - filter_offset);
			att_angle_offset[1] = att_angle[1] * filter_offset + att_angle_offset_last[1] * (1.0f - filter_offset);
			att_angle_offset[2] = att_angle[2] * filter_offset + att_angle_offset_last[2] * (1.0f - filter_offset);
			att_gyro_offset[0] = att_gyro[0] * filter_offset + att_gyro_offset_last[0] * (1.0f - filter_offset);
			att_gyro_offset[1] = att_gyro[1] * filter_offset + att_gyro_offset_last[1] * (1.0f - filter_offset);
			att_gyro_offset[2] = att_gyro[2] * filter_offset + att_gyro_offset_last[2] * (1.0f - filter_offset);

			ctl_roll_offset = ctl_roll * filter_offset + ctl_roll_offset_last * (1.0f - filter_offset);
			ctl_pitch_offset = ctl_pitch * filter_offset + ctl_pitch_offset_last * (1.0f - filter_offset);
			ctl_yaw_offset = ctl_yaw * filter_offset + ctl_yaw_offset_last * (1.0f - filter_offset);

			att_angle_offset_last[0] = att_angle_offset[0];
			att_angle_offset_last[1] = att_angle_offset[1];
			att_angle_offset_last[2] = att_angle_offset[2];

			att_gyro_offset_last[0] = att_gyro_offset[0];
			att_gyro_offset_last[1] = att_gyro_offset[1];
			att_gyro_offset_last[2] = att_gyro_offset[2];

			ctl_roll_offset_last = ctl_roll_offset;
			ctl_pitch_offset_last = ctl_pitch_offset;
			ctl_yaw_offset_last = ctl_yaw_offset;
		}

		sleep_ticks(20);
	}
}

void controller_task(void)
{
	pcb_create(PROI_CONTROLLER, &controller_pthread, NULL, 1200);
}