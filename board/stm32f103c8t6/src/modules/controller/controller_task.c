/*
 * task.c
 *
 *  Created on: December 23, 2019
 *      Author: lidq
 */

#include <controller_task.h>
#include <k_printf.h>

float param_angle_p = 0.5;

float param_gyro_p = 0.4;
float param_gyro_i = 0.005;
float param_gyro_d = 0.08;

extern float motor_ctl[MOTOR_CNT];

extern float att_angle[3];
extern float att_gyro[3];

extern float ctl_thro;
extern float ctl_roll;
extern float ctl_pitch;
extern float ctl_yaw;

float rate_dval_roll_last;
float rate_dval_pitch_last;
float rate_dval_yaw_last;

float gyro_integral_roll = 0.0;
float gyro_integral_pitch = 0.0;
float gyro_integral_yaw = 0.0;

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

static void controller_pthread(void *arg)
{
	while (1)
	{
		// k_printf("%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n", att_angle[0], att_angle[1], att_angle[2], att_gyro[0], att_gyro[1], att_gyro[2], ctl_thro, ctl_roll, ctl_pitch, ctl_yaw);
		if (ctl_thro < 0.3)
		{
			gyro_integral_roll = 0;
			gyro_integral_pitch = 0;
			gyro_integral_yaw = 0;
		}
		//外环控制
		//姿态误差 * P = 角速度期望
		float exp_rate_roll = angle_pid(ctl_roll - att_angle[1]);
		float exp_rate_pitch = angle_pid(ctl_pitch - att_angle[0]);
		float exp_rate_yaw = angle_pid(ctl_yaw - att_angle[2]);

		//内环控制
		//角速度期望 - 实际 = 误差
		float rate_dval_roll = exp_rate_roll - att_gyro[0];
		float rate_dval_pitch = exp_rate_pitch - att_gyro[1];
		float rate_dval_yaw = exp_rate_yaw - att_gyro[2];

		//PID控制：比例 + 积分 + 微分 = 控制量
		float out_control_roll = gyro_pid(rate_dval_roll, rate_dval_roll_last, &gyro_integral_roll);
		float out_control_pitch = gyro_pid(rate_dval_pitch, rate_dval_pitch_last, &gyro_integral_pitch);
		float out_control_yaw = gyro_pid(rate_dval_yaw, rate_dval_yaw_last, &gyro_integral_yaw);

		rate_dval_roll_last = rate_dval_roll;
		rate_dval_pitch_last = rate_dval_pitch;
		rate_dval_yaw_last = rate_dval_yaw;

		ctl_limit(&ctl_thro, 1.0f);
		ctl_limit(&out_control_roll, 0.5f);
		ctl_limit(&out_control_pitch, 0.5f);
		ctl_limit(&out_control_yaw, 0.5f);

		//k_printf("%.4f\t%.4f\t%.4f\n", out_control_roll, out_control_pitch, out_control_yaw);
		motor_ctl[0] = ctl_thro + out_control_roll + out_control_pitch;
		motor_ctl[1] = ctl_thro - out_control_roll + out_control_pitch;
		motor_ctl[2] = ctl_thro - out_control_roll - out_control_pitch;
		motor_ctl[3] = ctl_thro + out_control_roll - out_control_pitch;

		for (int i = 0; i < MOTOR_CNT; i++)
		{
			ctl_limit(&motor_ctl[i], 1.5f);
			motor_ctl[i] /= 1.5f;
			if (motor_ctl[i] < 0)
			{
				motor_ctl[i] = 0;
			}
		}

		k_printf("%.4f\t%.4f\t%.4f\t%.4f\n", motor_ctl[0], motor_ctl[1], motor_ctl[2], motor_ctl[3]);

		sleep_ticks(20);
	}
}

void controller_task(void)
{
	pcb_create(PROI_CONTROLLER, &controller_pthread, NULL, 1200);
}