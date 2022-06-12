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

float ctl_angle = M_PI / 12.0; // 15度

// [零偏
float offset_x = 0;
float offset_y = 0;
float offset_z = 0;
// 零偏]

// [角度参数
// 俯仰
float ctl_param_pitch_angle_p = 0.9;
float ctl_param_pitch_angle_i = 0.012;
float ctl_param_pitch_angle_d = 0.1;
// 滚转
float ctl_param_roll_angle_p = 0.9;
float ctl_param_roll_angle_i = 0.012;
float ctl_param_roll_angle_d = 0.1;
// 航向
float ctl_param_yaw_angle_p = 0.4;
float ctl_param_yaw_angle_i = 0.003;
float ctl_param_yaw_angle_d = 0.02;
// 角度参数]

// [积分项
float ctl_integral_pitch = 0;
float ctl_integral_roll = 0;
float ctl_integral_yaw = 0;
// 积分项]

// [上一次误差
float devi_pitch_angle_pre = 0;
float devi_roll_angle_pre = 0;
float devi_yaw_angle_pre = 0;
// 上一次误差]

// 电机控制量
float ctl_motor[4] = { 0 };

uint32_t ctl_pwm[4] = { 0 };

int ctl_armed = 1;
float ctl_armed_v1 = 0.15;
float ctl_armed_v2 = 0.85;
float ctl_armed_val = 0;
float ctl_armed_val_pre = 0;
float ctl_armed_val_filter = 0.03;

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

float ctl_mixer(float ctl_t, float ctl_p, float ctl_r, float ctl_y, float* ctl_motor)
{
	ctl_motor[0] = ctl_t - ctl_r + ctl_p + ctl_y;
	ctl_motor[1] = ctl_t + ctl_r + ctl_p - ctl_y;
	ctl_motor[2] = ctl_t + ctl_r - ctl_p + ctl_y;
	ctl_motor[3] = ctl_t - ctl_r - ctl_p - ctl_y;

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

		//已解锁
		if (ctl_armed)
		{
			if (ctl_thro < ctl_armed_v1 && ctl_yaw > ctl_armed_v2 && ctl_pitch < -ctl_armed_v2 && ctl_roll < -ctl_armed_v2)
			{
				ctl_armed_val = 1.0 * ctl_armed_val_filter + ctl_armed_val_pre * (1.0 - ctl_armed_val_filter);
				ctl_armed_val_pre = ctl_armed_val;

				if (ctl_armed_val > ctl_armed_v2)
				{
					ctl_armed_val = 0;
					ctl_armed_val_pre = 0;

					//锁定
					ctl_armed = 0;
				}
			}
			else
			{
				ctl_armed_val = 0;
				ctl_armed_val_pre = 0;
			}

			if (ctl_thro < ctl_armed_v1)
			{
				offset_x = 0;
				offset_y = 0;
				offset_z = -z;

				ctl_integral_pitch = 0;
				ctl_integral_roll = 0;
				ctl_integral_yaw = 0;

				devi_pitch_angle_pre = 0;
				devi_roll_angle_pre = 0;
				devi_yaw_angle_pre = 0;
				ctl_mixer(0, 0, 0, 0, ctl_motor);
			}
			else
			{

				float devi_pitch = ctl_pitch * ctl_angle - (offset_x + x);
				float devi_roll = ctl_roll * ctl_angle - (offset_y + y);
				float devi_yaw = 0 - (offset_z + z);

				// PID控制
				float ctl_pitch_angle = ctl_pid(devi_pitch, devi_pitch_angle_pre, ctl_param_pitch_angle_p, ctl_param_pitch_angle_i, ctl_param_pitch_angle_d, &ctl_integral_pitch, ctl_thro);
				float ctl_roll_angle = ctl_pid(devi_roll, devi_roll_angle_pre, ctl_param_roll_angle_p, ctl_param_roll_angle_i, ctl_param_roll_angle_d, &ctl_integral_roll, ctl_thro);
				float ctl_yaw_angle = ctl_pid(devi_yaw, devi_yaw_angle_pre, ctl_param_yaw_angle_p, ctl_param_yaw_angle_i, ctl_param_yaw_angle_d, &ctl_integral_yaw, ctl_thro);

				// [更新误差项
				devi_pitch_angle_pre = devi_pitch;
				devi_roll_angle_pre = devi_roll;
				devi_yaw_angle_pre = devi_yaw;
				// 更新误差项]

				//混控
				ctl_mixer(ctl_thro, ctl_pitch_angle, ctl_roll_angle, ctl_yaw_angle, ctl_motor);
			}
		}
		//未解锁
		else
		{
			if (ctl_thro < ctl_armed_v1 && ctl_yaw < -ctl_armed_v2 && ctl_pitch < -ctl_armed_v2 && ctl_roll > ctl_armed_v2)
			{
				ctl_armed_val = 1.0 * ctl_armed_val_filter + ctl_armed_val_pre * (1.0 - ctl_armed_val_filter);
				ctl_armed_val_pre = ctl_armed_val;

				if (ctl_armed_val > ctl_armed_v2)
				{
					ctl_armed_val = 0;
					ctl_armed_val_pre = 0;

					//解锁
					ctl_armed = 1;
				}
			}
			else
			{
				ctl_armed_val = 0;
				ctl_armed_val_pre = 0;
			}


			offset_x = 0;
			offset_y = 0;
			offset_z = -z;

			ctl_integral_pitch = 0;
			ctl_integral_roll = 0;
			ctl_integral_yaw = 0;

			devi_pitch_angle_pre = 0;
			devi_roll_angle_pre = 0;
			devi_yaw_angle_pre = 0;

			ctl_mixer(0, 0, 0, 0, ctl_motor);
		}

		for (int i = 0; i < 4; i++)
		{
			ctl_pwm[i] = ctl_motor[i] * (PWM_MAX - PWM_MIN) + PWM_MIN;
		}

		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ctl_pwm[0]);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, ctl_pwm[1]);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, ctl_pwm[2]);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, ctl_pwm[3]);

		if (tk % 5 == 0)
		{
			printf("%+6d %+6d %+6d ", (int)((offset_x + x) * 1000), (int)((offset_y + y) * 1000), (int)((offset_z + z) * 1000));
			printf("%04d %04d %04d %04d\n", (int)(ctl_motor[0] * 1000), (int)(ctl_motor[1] * 1000), (int)(ctl_motor[2] * 1000), (int)(ctl_motor[3] * 1000));
			// printf("%04d %04d %04d %04d\n", (int)(ctl_thro * 1000), (int)(ctl_pitch * 1000), (int)(ctl_roll * 1000), (int)(ctl_yaw * 1000));
		}

		tk++;
		sleep_ticks(10);
	}
	return NULL;
}

void controller_task(void)
{
	pcb_create(PCB_CTL_PRIO, &controller_pthread, NULL, PCB_CTL_SIZE);
}
