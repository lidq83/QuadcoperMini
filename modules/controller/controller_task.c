/*
 * controller_task.c
 *
 *  Created on: December 23, 2019
 *      Author: lidq
 */

#include <controller_task.h>
#include <main.h>
#include <stdio.h>

extern TIM_HandleTypeDef htim2;

extern double ctl_thro;
extern double ctl_pitch;
extern double ctl_roll;
extern double ctl_yaw;
extern uint8_t ctl_sw[16];

#define CTL_ANGLE (0.261799388) //(15.0 * M_PI / 180.0);

double sqrt_2_2 = 0.707106781; // sqrt(2)/2

// 航向期望角（总和）
double yaw_expect_rate	= 0.01; // 0.1弧度/10ms
double yaw_expect_total = 0;

// [角度参数
// 俯仰 - 滚转
double ctl_param_pitch_roll_angle_p = 15.0;
// 航向
double ctl_param_yaw_angle_p = 13.0;
// 角度参数]

// [角速度参数
// 俯仰 - 滚转
const double ctl_param_pitch_roll_rate_p = 0.02;
const double ctl_param_pitch_roll_rate_i = 0.0004;
const double ctl_param_pitch_roll_rate_d = 0.01;
// 航向
const double ctl_param_yaw_rate_p = 0.01;
const double ctl_param_yaw_rate_i = 0.0006;
const double ctl_param_yaw_rate_d = 0.016;
// 角速度参数]

// [积分项
double ctl_integral_rate_pitch = 0;
double ctl_integral_rate_roll  = 0;
double ctl_integral_rate_yaw   = 0;
// 积分项]

// [上一次误差
double devi_pitch_angle_pre = 0;
double devi_roll_angle_pre	= 0;
double devi_yaw_angle_pre	= 0;

double devi_pitch_rate_pre = 0;
double devi_roll_rate_pre  = 0;
double devi_yaw_rate_pre   = 0;
// 上一次误差]

// 电机控制量
double ctl_motor[4] = { 0 };

// 最终PWM信号值
uint32_t ctl_pwm[4] = { 0 };

extern Vector3f gyro;
extern Vector3f angle;
extern double yaw;

int ctl_arming = 0;

void ctl_value_limit(double* value, double max, double min)
{
	if (*value > max)
	{
		*value = max;
	}
	if (*value < min)
	{
		*value = min;
	}
}

double ctl_pid(double devi, double devi_pre, double p, double i, double d, double* integral, double integral_limit)
{
	if (integral != NULL)
	{
		*integral += devi * i;
		ctl_value_limit(integral, integral_limit, -integral_limit);
	}

	double val_p = devi * p;
	double val_i = integral == NULL ? 0 : *integral;
	double val_d = (devi - devi_pre) * d;

	return val_p + val_i + val_d;
}

double ctl_mixer(double ctl_t, double ctl_p, double ctl_r, double ctl_y, double* ctl_motor)
{
	ctl_motor[0] = ctl_t + (ctl_p * sqrt_2_2) - (ctl_r * sqrt_2_2) + ctl_y;
	ctl_motor[1] = ctl_t + (ctl_p * sqrt_2_2) + (ctl_r * sqrt_2_2) - ctl_y;
	ctl_motor[2] = ctl_t - (ctl_p * sqrt_2_2) + (ctl_r * sqrt_2_2) + ctl_y;
	ctl_motor[3] = ctl_t - (ctl_p * sqrt_2_2) - (ctl_r * sqrt_2_2) - ctl_y;

	for (int i = 0; i < 4; i++)
	{
		ctl_value_limit(&ctl_motor[i], 1.0, 0.0);
	}
}

void ctl_lock_zero(void)
{
	devi_pitch_angle_pre = 0;
	devi_roll_angle_pre	 = 0;
	devi_yaw_angle_pre	 = 0;

	devi_pitch_rate_pre = 0;
	devi_roll_rate_pre	= 0;
	devi_yaw_rate_pre	= 0;

	ctl_integral_rate_pitch = 0;
	ctl_integral_rate_roll	= 0;
	ctl_integral_rate_yaw	= 0;

	yaw_expect_total = 0;
}

void ctl_output(void)
{
	for (int i = 0; i < 4; i++)
	{
		ctl_pwm[i] = ctl_motor[i] * (PWM_MAX - PWM_MIN) + PWM_MIN;
	}

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ctl_pwm[0]);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, ctl_pwm[1]);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, ctl_pwm[2]);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, ctl_pwm[3]);
}

void* controller_pthread(void* arg)
{
	double timepre		= 0;
	uint32_t tk			= 0;
	double yaw_offset	= 0;
	double yaw_opposite = 0;
	while (1)
	{
		// 计算当前时间戳
		uint64_t cnt	 = get_count();
		double timestamp = cnt / 1000000.0;
		double dt		 = timestamp - timepre;
		timepre			 = timestamp;

		//
		if (ctl_sw[2] == 1)
		{
			ctl_arming = 1;
		}
		else
		{
			ctl_arming = 0;
		}

		// 已解锁
		if (ctl_arming)
		{
			yaw_opposite = yaw + yaw_offset;
			// 角速度期望转为航向角速期望
			yaw_expect_total += ctl_yaw * yaw_expect_rate;

			// [外环PID控制
			// 根据角度期望计算角度误差
			double devi_pitch_angle = ctl_pitch * CTL_ANGLE - angle.x;
			double devi_roll_angle	= ctl_roll * CTL_ANGLE - angle.y;
			double devi_yaw_angle	= 0 - yaw_opposite; // yaw_expect_total - yaw_opposite;
			// PID得到角速度期望
			double ctl_pitch_angle = ctl_pid(devi_pitch_angle, devi_pitch_angle_pre, ctl_param_pitch_roll_angle_p, 0, 0, NULL, 0);
			double ctl_roll_angle  = ctl_pid(devi_roll_angle, devi_roll_angle_pre, ctl_param_pitch_roll_angle_p, 0, 0, NULL, 0);
			double ctl_yaw_angle   = ctl_pid(devi_yaw_angle, devi_yaw_angle_pre, ctl_param_yaw_angle_p, 0, 0, NULL, 0);
			// 外环PID控制]

			// [内环PID控制
			// 根据角速度期望计算角度误差
			double devi_pitch_rate = ctl_pitch_angle - gyro.y;
			double devi_roll_rate  = ctl_roll_angle - gyro.x;
			double devi_yaw_rate   = ctl_yaw_angle - gyro.z;
			// PID得到控制量
			double ctl_pitch_rate = ctl_pid(devi_pitch_rate, devi_pitch_rate_pre, ctl_param_pitch_roll_rate_p, ctl_param_pitch_roll_rate_i, ctl_param_pitch_roll_rate_d, &ctl_integral_rate_pitch, ctl_thro * 2);
			double ctl_roll_rate  = ctl_pid(devi_roll_rate, devi_roll_rate_pre, ctl_param_pitch_roll_rate_p, ctl_param_pitch_roll_rate_i, ctl_param_pitch_roll_rate_d, &ctl_integral_rate_roll, ctl_thro * 2);
			double ctl_yaw_rate	  = ctl_pid(devi_yaw_rate, devi_yaw_rate_pre, ctl_param_yaw_rate_p, ctl_param_yaw_rate_i, ctl_param_yaw_rate_d, &ctl_integral_rate_yaw, ctl_thro * 2);
			// 内环PID控制]

			// [更新误差项
			devi_pitch_angle_pre = devi_pitch_angle;
			devi_roll_angle_pre	 = devi_roll_angle;
			devi_yaw_angle_pre	 = devi_yaw_angle;

			devi_pitch_rate_pre = devi_pitch_rate;
			devi_roll_rate_pre	= devi_roll_rate;
			devi_yaw_rate_pre	= devi_yaw_rate;
			// 更新误差项]

			// 混控
			ctl_mixer(ctl_thro, ctl_pitch_rate, ctl_roll_rate, ctl_yaw_rate, ctl_motor);
			// ctl_mixer(0, 0, 0, 0, ctl_motor);
		}
		// 未解锁
		else
		{
			yaw_offset = -yaw;
			ctl_lock_zero();
			ctl_mixer(0, 0, 0, 0, ctl_motor);
		}

		// if (tk % 10 == 0)
		// {
		// 	printf("%+6d %+6d\n",
		// 		   (int)(angle.z * 57.3 * 10),
		// 		   (int)(gyro.z * 57.3 * 10));
		// }

		ctl_output();

		tk++;
		msleep(5);
	}
	return NULL;
}

void controller_task(void)
{
	pcb_create(PCB_CTL_PRIO, &controller_pthread, NULL, PCB_CTL_SIZE);
}
