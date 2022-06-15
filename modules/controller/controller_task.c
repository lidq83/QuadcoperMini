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

extern float ctl_thro;
extern float ctl_pitch;
extern float ctl_roll;
extern float ctl_yaw;

float ctl_angle = M_PI / 6.0; // 30度
float sqrt_2_2 = 0.707106781; // sqrt(2)/2

// [角度参数
// 俯仰
float ctl_param_pitch_angle_p = 7.0;
// 滚转
float ctl_param_roll_angle_p = 7.0;
// 航向
float ctl_param_yaw_angle_p = 7.0;
// 角度参数]

// [角速度参数
// 俯仰
float ctl_param_pitch_rate_p = 0.017;
float ctl_param_pitch_rate_i = 0.0005;
float ctl_param_pitch_rate_d = 0.03;
// 滚转
float ctl_param_roll_rate_p = 0.017;
float ctl_param_roll_rate_i = 0.0005;
float ctl_param_roll_rate_d = 0.03;
// 航向
float ctl_param_yaw_rate_p = 0.01;
float ctl_param_yaw_rate_i = 0.0002;
float ctl_param_yaw_rate_d = 0.015;
// 角速度参数]

// [积分项
float ctl_integral_rate_pitch = 0;
float ctl_integral_rate_roll = 0;
float ctl_integral_rate_yaw = 0;
// 积分项]

// [上一次误差
float devi_pitch_angle_pre = 0;
float devi_roll_angle_pre = 0;

float devi_pitch_rate_pre = 0;
float devi_roll_rate_pre = 0;
float devi_yaw_rate_pre = 0;

// 上一次误差]

// [零偏
float offset_x = 0;
float offset_y = 0;
float offset_z = 0;
float offset_x_pre = 0;
float offset_y_pre = 0;
float offset_z_pre = 0;
float offset_x_filter = 0.03;
float offset_y_filter = 0.03;
float offset_z_filter = 0.03;

float offset_gx = 0;
float offset_gy = 0;
float offset_gz = 0;
float offset_gx_pre = 0;
float offset_gy_pre = 0;
float offset_gz_pre = 0;
float offset_gx_filter = 0.03;
float offset_gy_filter = 0.03;
float offset_gz_filter = 0.03;
// 零偏]

// 电机控制量
float ctl_motor[4] = { 0 };

// 最终PWM信号值
uint32_t ctl_pwm[4] = { 0 };

int ctl_armed = 1;
float ctl_armed_v1 = 0.1;
float ctl_armed_v2 = 0.9;
float ctl_armed_val = 0;
float ctl_armed_val_pre = 0;
float ctl_armed_val_filter = 0.03;

float mpu_value[9] = { 0 };
float xyz_value[9] = { 0 };
float xyz_value_pre[9] = { 0 };
float xyz_value_filter[9] = { 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5 };

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
	devi_roll_angle_pre = 0;

	devi_pitch_rate_pre = 0;
	devi_roll_rate_pre = 0;
	devi_yaw_rate_pre = 0;

	ctl_integral_rate_pitch = 0;
	ctl_integral_rate_roll = 0;
	ctl_integral_rate_yaw = 0;
}

void ctl_offset(float x, float y, float z, float gx, float gy, float gz)
{
	//角度
	offset_x = x * offset_x_filter + offset_x_pre * (1.0 - offset_x_filter);
	offset_y = y * offset_y_filter + offset_y_pre * (1.0 - offset_y_filter);
	offset_z = z * offset_z_filter + offset_z_pre * (1.0 - offset_z_filter);
	offset_x_pre = offset_x;
	offset_y_pre = offset_y;
	offset_z_pre = offset_z;

	//角速度
	offset_gx = gx * offset_gx_filter + offset_gx_pre * (1.0 - offset_gx_filter);
	offset_gy = gy * offset_gy_filter + offset_gy_pre * (1.0 - offset_gy_filter);
	offset_gz = gz * offset_gz_filter + offset_gz_pre * (1.0 - offset_gz_filter);
	offset_gx_pre = offset_gx;
	offset_gy_pre = offset_gy;
	offset_gz_pre = offset_gz;
}

void* controller_pthread(void* arg)
{
	mpu6050_setup();

	float sx = 0;
	float sy = 0;
	float sz = 0;

	uint32_t tk = 0;

	while (tk++ < 20)
	{
		mpu6050_value(&mpu_value[0], &mpu_value[1], &mpu_value[2], &mpu_value[3], &mpu_value[4], &mpu_value[5], &mpu_value[6], &mpu_value[7], &mpu_value[8]);
		sx += mpu_value[0];
		sy += mpu_value[1];
		sz += mpu_value[2];
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
		mpu6050_value(&mpu_value[0], &mpu_value[1], &mpu_value[2], &mpu_value[3], &mpu_value[4], &mpu_value[5], &mpu_value[6], &mpu_value[7], &mpu_value[8]);

		for (int i = 0; i < 9; i++)
		{
			xyz_value[i] = mpu_value[i] * xyz_value_filter[i] + xyz_value_pre[i] * (1.0 - xyz_value_filter[i]);
			xyz_value_pre[i] = xyz_value[i];
		}

		float x = xyz_value[0];
		float y = xyz_value[1];
		float z = xyz_value[2];

		float gx = xyz_value[3];
		float gy = xyz_value[4];
		float gz = xyz_value[5];


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
				ctl_offset(-x, -y, -z, -gx, -gy, -gz);
				ctl_lock_zero();
				ctl_mixer(0, 0, 0, 0, ctl_motor);
			}
			else
			{
				// [外环PID控制
				//根据角度期望计算角度误差
				float devi_pitch_angle = (-ctl_pitch) * ctl_angle - (offset_x + x);
				float devi_roll_angle = (-ctl_roll) * ctl_angle - (offset_y + y);
				// PID得到角速度期望
				float ctl_pitch_angle = ctl_pid(devi_pitch_angle, devi_pitch_angle_pre, ctl_param_pitch_angle_p, 0, 0, NULL, 0);
				float ctl_roll_angle = ctl_pid(devi_roll_angle, devi_roll_angle_pre, ctl_param_roll_angle_p, 0, 0, NULL, 0);
				// 外环PID控制]

				// [内环PID控制
				//根据角速度期望计算角度误差
				float devi_pitch_rate = ctl_pitch_angle - (offset_gx + gx);
				float devi_roll_rate = ctl_roll_angle - (offset_gy + gy);
				float devi_yaw_rate = ctl_yaw * ctl_param_yaw_angle_p - (offset_gz + gz);
				// PID得到控制量
				float ctl_pitch_rate = ctl_pid(devi_pitch_rate, devi_pitch_rate_pre, ctl_param_pitch_rate_p, ctl_param_pitch_rate_i, ctl_param_pitch_rate_d, &ctl_integral_rate_pitch, ctl_thro);
				float ctl_roll_rate = ctl_pid(devi_roll_rate, devi_roll_rate_pre, ctl_param_roll_rate_p, ctl_param_roll_rate_i, ctl_param_roll_rate_d, &ctl_integral_rate_roll, ctl_thro);
				float ctl_yaw_rate = ctl_pid(devi_yaw_rate, devi_yaw_rate_pre, ctl_param_yaw_rate_p, ctl_param_yaw_rate_i, ctl_param_yaw_rate_d, &ctl_integral_rate_yaw, ctl_thro);
				// 内环PID控制]

				// [更新误差项
				devi_pitch_angle_pre = devi_pitch_angle;
				devi_roll_angle_pre = devi_roll_angle;

				devi_pitch_rate_pre = devi_pitch_rate;
				devi_roll_rate_pre = devi_roll_rate;
				devi_yaw_rate_pre = devi_yaw_rate;
				// 更新误差项]

				//混控
				ctl_mixer(ctl_thro, ctl_pitch_rate, ctl_roll_rate, ctl_yaw_rate, ctl_motor);
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

			ctl_offset(-x, -y, -z, -gx, -gy, -gz);
			ctl_lock_zero();
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
			// printf("%+6d %+6d %+6d ", (int)((offset_x + x) * 1000), (int)((offset_y + y) * 1000), (int)((offset_z + z) * 1000));
			// printf("%+6d %+6d %+6d \n", (int)((offset_gx + gx) * 1000), (int)((offset_gy + gy) * 1000), (int)((offset_gz + gz) * 1000));
			// printf("%04d %04d %04d %04d\n", (int)(ctl_motor[0] * 1000), (int)(ctl_motor[1] * 1000), (int)(ctl_motor[2] * 1000), (int)(ctl_motor[3] * 1000));
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
