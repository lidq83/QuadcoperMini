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

extern float ctl_thro;
extern float ctl_pitch;
extern float ctl_roll;
extern float ctl_yaw;
extern uint8_t ctl_sw[16];

#define CTL_ANGLE (15.0f * M_PI / 180.0);

float sqrt_2_2 = 0.707106781; // sqrt(2)/2

// 航向期望角（总和）
float yaw_expect_rate  = 0.01; // 0.1弧度/10ms
float yaw_expect_total = 0;

// [角度参数
// 俯仰 - 滚转
float ctl_param_pitch_roll_angle_p = 20.0;
// 航向
float ctl_param_yaw_angle_p = 13.0;
// 角度参数]

// [角速度参数
// 俯仰 - 滚转
const float ctl_param_pitch_roll_rate_p = 0.01;
const float ctl_param_pitch_roll_rate_i = 0.0015;
const float ctl_param_pitch_roll_rate_d = 0.02;
// 航向
const float ctl_param_yaw_rate_p = 0.02;
const float ctl_param_yaw_rate_i = 0.0012;
const float ctl_param_yaw_rate_d = 0.037;
// 角速度参数]

// [积分项
float ctl_integral_rate_pitch = 0;
float ctl_integral_rate_roll  = 0;
float ctl_integral_rate_yaw	  = 0;
// 积分项]

// [上一次误差
float devi_pitch_angle_pre = 0;
float devi_roll_angle_pre  = 0;
float devi_yaw_angle_pre   = 0;

float devi_pitch_rate_pre = 0;
float devi_roll_rate_pre  = 0;
float devi_yaw_rate_pre	  = 0;
// 上一次误差]

// 电机控制量
float ctl_motor[4] = { 0 };

// 最终PWM信号值
uint32_t ctl_pwm[4] = { 0 };

extern Vector3f gyro;
extern Vector3f angle;
extern double yaw;

int ctl_arming = 0;

void ctl_value_limit(float* value, float max, float min)
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

float ctl_pid(float devi, float devi_pre, float p, float i, float d, float* integral, float integral_limit)
{
	if (integral != NULL)
	{
		*integral += devi * i;
		ctl_value_limit(integral, integral_limit, -integral_limit);
	}

	float val_p = devi * p;
	float val_i = integral == NULL ? 0 : *integral;
	float val_d = (devi - devi_pre) * d;

	return val_p + val_i + val_d;
}

float ctl_mixer(float ctl_t, float ctl_p, float ctl_r, float ctl_y, float* ctl_motor)
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
	double timepre = 0;
	uint32_t tk	   = 0;
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
			// 角速度期望转为航向角速期望
			yaw_expect_total += ctl_yaw * yaw_expect_rate;

			// [外环PID控制
			// 根据角度期望计算角度误差
			float devi_pitch_angle = (-ctl_pitch) * CTL_ANGLE - angle.x;
			float devi_roll_angle  = (-ctl_roll) * CTL_ANGLE - angle.y;
			float devi_yaw_angle   = yaw_expect_total - yaw;
			// PID得到角速度期望
			float ctl_pitch_angle = ctl_pid(devi_pitch_angle, devi_pitch_angle_pre, ctl_param_pitch_roll_angle_p, 0, 0, NULL, 0);
			float ctl_roll_angle  = ctl_pid(devi_roll_angle, devi_roll_angle_pre, ctl_param_pitch_roll_angle_p, 0, 0, NULL, 0);
			float ctl_yaw_angle	  = ctl_pid(devi_yaw_angle, devi_yaw_angle_pre, ctl_param_yaw_angle_p, 0, 0, NULL, 0);
			// 外环PID控制]

			// [内环PID控制
			// 根据角速度期望计算角度误差
			float devi_pitch_rate = ctl_pitch_angle - gyro.x;
			float devi_roll_rate  = ctl_roll_angle - gyro.y;
			float devi_yaw_rate	  = ctl_yaw_angle - gyro.z;
			// PID得到控制量
			float ctl_pitch_rate = ctl_pid(devi_pitch_rate, devi_pitch_rate_pre, ctl_param_pitch_roll_rate_p, ctl_param_pitch_roll_rate_i, ctl_param_pitch_roll_rate_d, &ctl_integral_rate_pitch, ctl_thro);
			float ctl_roll_rate	 = ctl_pid(devi_roll_rate, devi_roll_rate_pre, ctl_param_pitch_roll_rate_p, ctl_param_pitch_roll_rate_i, ctl_param_pitch_roll_rate_d, &ctl_integral_rate_roll, ctl_thro);
			float ctl_yaw_rate	 = ctl_pid(devi_yaw_rate, devi_yaw_rate_pre, ctl_param_yaw_rate_p, ctl_param_yaw_rate_i, ctl_param_yaw_rate_d, &ctl_integral_rate_yaw, ctl_thro);
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
			// ctl_mixer(ctl_thro, ctl_pitch_rate, ctl_roll_rate, ctl_yaw_rate, ctl_motor);
			ctl_mixer(0, 0, 0, 0, ctl_motor);
		}
		// 未解锁
		else
		{
			ctl_lock_zero();
			ctl_mixer(0, 0, 0, 0, ctl_motor);
		}

		if (tk % 10 == 0)
		{
			printf("arming %d\n", ctl_arming);
		}

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
