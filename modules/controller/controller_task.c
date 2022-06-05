/*
 * controller_task.c
 *
 *  Created on: December 23, 2019
 *      Author: lidq
 */

#include <controller_task.h>
#include <mpu6050.h>
#include <stdio.h>

extern TIM_HandleTypeDef htim2;

extern float ctl_yaw;
extern float ctl_thro;
extern float ctl_roll;
extern float ctl_pitch;

// [外环角度控制参数
float ctl_param_roll_angle_p = 1.5;
float ctl_param_pitch_angle_p = 1.5;
float ctl_param_yaw_angle_p = 1.5;
// 外环角度控制参数]

// [内环角速度参数
// 俯仰
float ctl_param_pitch_rate_p = 0.3;
float ctl_param_pitch_rate_i = 0.001;
float ctl_param_pitch_rate_d = 0.03;
// 滚转
float ctl_param_roll_rate_p = 0.3;
float ctl_param_roll_rate_i = 0.001;
float ctl_param_roll_rate_d = 0.03;
// 航向
float ctl_param_yaw_rate_p = 0.1;
float ctl_param_yaw_rate_i = 0.0003;
float ctl_param_yaw_rate_d = 0.01;

float offset_x = 0;
float offset_y = 0;
float offset_z = 0;

float offset_gx = 0;
float offset_gy = 0;
float offset_gz = 0;

// 内环角速度参数]

// [积分项
float ctl_integral_pitch = 0;
float ctl_integral_roll = 0;
float ctl_integral_yaw = 0;
// 积分项]

// [上一次误差
float devi_pitch_angle_pre = 0;
float devi_roll_angle_pre = 0;
float devi_yaw_angle_pre = 0;

float devi_pitch_rate_pre = 0;
float devi_roll_rate_pre = 0;
float devi_yaw_rate_pre = 0;
// 上一次误差]

// 电机控制量
float ctl_motor[4] = { 0 };

uint32_t ctl_pwm[4] = { 0 };

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
	float val_ctl = val_p + val_i + val_d;

	return val_ctl;
}

float ctl_mixer(float ctl_t, float ctl_r, float ctl_p, float ctl_y, float* ctl_motor)
{
	ctl_motor[0] = ctl_t - ctl_r + ctl_p - ctl_y;
	ctl_motor[1] = ctl_t + ctl_r + ctl_p + ctl_y;
	ctl_motor[2] = ctl_t + ctl_r - ctl_p - ctl_y;
	ctl_motor[3] = ctl_t - ctl_r - ctl_p + ctl_y;

	// ctl_motor[0] = ctl_t;
	// ctl_motor[1] = ctl_t;
	// ctl_motor[2] = ctl_t;
	// ctl_motor[3] = ctl_t;

	for (int i = 0; i < 4; i++)
	{
		ctl_value_limit(&ctl_motor[i], 1.0, 0.0);
	}
}

void* controller_pthread(void* arg)
{
	mpu6050_setup();

	float x = 0;
	float y = 0;
	float z = 0;
	float gx = 0;
	float gy = 0;
	float gz = 0;
	float ax = 0;
	float ay = 0;
	float az = 0;

	float sx = 0;
	float sy = 0;
	float sz = 0;

	uint32_t tk = 0;

	while (tk++ < 20)
	{
		mpu6050_value(&x, &y, &z, &gx, &gy, &gz, &ax, &ay, &az);
		sx += x;
		sy += y;
		sz += z;
		sleep_ticks(5);
	}

	if (sx == 0 && sy == 0 && sz == 0)
	{
		printf("Init mpu6050 error, reset mpu.\n");
		HAL_NVIC_SystemReset();
		while (1)
		{
		}
	}


	while (1)
	{
		//读取姿态信息
		mpu6050_value(&x, &y, &z, &gx, &gy, &gz, &ax, &ay, &az);

		if (ctl_thro < 0.1)
		{
			offset_x = 0;
			offset_y = 0;
			offset_z = -z;

			offset_gx = -gx;
			offset_gy = -gy;
			offset_gz = -gz;

			ctl_integral_pitch = 0;
			ctl_integral_roll = 0;
			ctl_integral_yaw = 0;

			devi_pitch_angle_pre = 0;
			devi_roll_angle_pre = 0;
			devi_yaw_angle_pre = 0;

			devi_pitch_rate_pre = 0;
			devi_pitch_rate_pre = 0;
			devi_pitch_rate_pre = 0;

			ctl_mixer(0, 0, 0, 0, ctl_motor);
		}
		else
		{
			//外环PID
			float ctl_pitch_angle = ctl_pid(ctl_pitch - (offset_x + x), devi_pitch_angle_pre, ctl_param_pitch_angle_p, 0, 0, NULL, 0);
			float ctl_roll_angle = ctl_pid(ctl_roll - (offset_y + y), devi_roll_angle_pre, ctl_param_roll_angle_p, 0, 0, NULL, 0);
			float ctl_yaw_angle = ctl_pid(0 - (offset_z + z), devi_yaw_angle_pre, ctl_param_yaw_angle_p, 0, 0, NULL, 0);
			//内环PID
			float ctl_pitch_rate = ctl_pid(ctl_pitch_angle - (offset_gx + gx), devi_pitch_rate_pre, ctl_param_pitch_rate_p, ctl_param_pitch_rate_i, ctl_param_pitch_rate_d, &ctl_integral_pitch, ctl_thro);
			float ctl_roll_rate = ctl_pid(ctl_roll_angle - (offset_gy + gy), devi_roll_rate_pre, ctl_param_roll_rate_p, ctl_param_roll_rate_i, ctl_param_roll_rate_d, &ctl_integral_roll, ctl_thro);
			float ctl_yaw_rate = ctl_pid(ctl_yaw_angle - (offset_gz + gz) - ctl_yaw, devi_yaw_rate_pre, ctl_param_yaw_rate_p, ctl_param_yaw_rate_i, ctl_param_yaw_rate_d, &ctl_integral_yaw, ctl_thro);

			// [更新误差项
			// 角度
			devi_pitch_angle_pre = ctl_pitch - (offset_x + x);
			devi_roll_angle_pre = ctl_roll - (offset_y + y);
			devi_yaw_angle_pre = ctl_yaw - (offset_z + z);
			// 角速度
			devi_pitch_rate_pre = ctl_pitch_angle - (offset_gx + gx);
			devi_roll_rate_pre = ctl_roll_angle - (offset_gy + gy);
			devi_yaw_rate_pre = ctl_yaw_angle - (offset_gz + gz);
			// 更新误差项]

			//混控
			ctl_mixer(ctl_thro, ctl_roll_rate, ctl_pitch_rate, ctl_yaw_rate, ctl_motor);
		}
		if (tk % 10 == 0)
		{
			printf("%+6d %+6d %+6d ", (int)((offset_x + x) * 1000), (int)((offset_y + y) * 1000), (int)((offset_z + z) * 1000));
			printf("%+6d %+6d %+6d ", (int)((offset_gx + gx) * 1000), (int)((offset_gy + gy) * 1000), (int)((offset_gz + gz) * 1000));
			printf("%04d %04d %04d %04d\n", (int)(ctl_motor[0] * 1000), (int)(ctl_motor[1] * 1000), (int)(ctl_motor[2] * 1000), (int)(ctl_motor[3] * 1000));
			// printf("%04d %04d %04d %04d\n", (int)(ctl_thro * 1000), (int)(ctl_roll * 1000), (int)(ctl_pitch * 1000), (int)(ctl_yaw * 1000));
		}

		for (int i = 0; i < 4; i++)
		{
			ctl_pwm[i] = ctl_motor[i] * (PWM_MAX - PWM_MIN) + PWM_MIN;
		}

		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ctl_pwm[0]);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, ctl_pwm[1]);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, ctl_pwm[2]);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, ctl_pwm[3]);

		tk++;
		sleep_ticks(10);
	}
	return NULL;
}

void controller_task(void)
{
	pcb_create(PCB_CTL_PRIO, &controller_pthread, NULL, PCB_CTL_SIZE);
}
