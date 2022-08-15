/*
 * controller_task.c
 *
 *  Created on: December 23, 2019
 *      Author: lidq
 */

#include <controller_task.h>
#include <flash.h>
#include <mpu6050.h>
#include <stdio.h>

#define MAGIC_NUM (0x1F28E9C4)

extern TIM_HandleTypeDef htim2;

extern float ctl_thro;
extern float ctl_pitch;
extern float ctl_roll;
extern float ctl_yaw;
extern uint8_t ctl_sw[4];

float ctl_angle = 7.5f * M_PI / 180.0f;
float ctl_angle_p = 1.0f;

float sqrt_2_2 = 0.707106781; // sqrt(2)/2

//航向期望角（总和）
float yaw_expect_rate = 0.03; // 0.3弧度/10ms
float yaw_expect_total = 0;

// [角度参数
// 俯仰 - 滚转
float ctl_param_pitch_roll_angle_p = 20.0;
// 航向
float ctl_param_yaw_angle_p = 13.0;
// 角度参数]

// [角速度参数
// 俯仰 - 滚转
const float ctl_param_pitch_roll_rate_p = 0.011;
const float ctl_param_pitch_roll_rate_i = 0.0007;
const float ctl_param_pitch_roll_rate_d = 0.02;
// 航向
const float ctl_param_yaw_rate_p = 0.02;
const float ctl_param_yaw_rate_i = 0.0006;
const float ctl_param_yaw_rate_d = 0.04;
// 角速度参数]

// [积分项
float ctl_integral_rate_pitch = 0;
float ctl_integral_rate_roll = 0;
float ctl_integral_rate_yaw = 0;
// 积分项]

// [上一次误差
float devi_pitch_angle_pre = 0;
float devi_roll_angle_pre = 0;
float devi_yaw_angle_pre = 0;

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

float mpu_value[9] = { 0 };
float xyz_value[9] = { 0 };
float xyz_value_pre[9] = { 0 };
float xyz_value_filter[9] = { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 }; //暂不启用低通滤波

int ctl_arming = 0;
int ctl_calibrate = 0;

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
	devi_yaw_angle_pre = 0;

	devi_pitch_rate_pre = 0;
	devi_roll_rate_pre = 0;
	devi_yaw_rate_pre = 0;

	ctl_integral_rate_pitch = 0;
	ctl_integral_rate_roll = 0;
	ctl_integral_rate_yaw = 0;

	yaw_expect_total = 0;
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

void ctl_offset_z(float z, float gz)
{
	//角度
	offset_z = z * offset_z_filter + offset_z_pre * (1.0 - offset_z_filter);
	offset_z_pre = offset_z;
	//角速度
	offset_gz = gz * offset_gz_filter + offset_gz_pre * (1.0 - offset_gz_filter);
	offset_gz_pre = offset_gz;
}

void ctl_offset_save(void)
{
	uint32_t value[8] = { 0 };

	value[0] = MAGIC_NUM;

	float* p = (float*)&value[1];
	p[0] = offset_x;
	p[1] = offset_y;
	p[2] = offset_z;
	p[3] = offset_gx;
	p[4] = offset_gy;
	p[5] = offset_gz;

	flash_erase(FLASH_USER_DEF);
	flash_write(FLASH_USER_DEF, value, 8);
}

void ctl_offset_load(void)
{
	uint32_t value[8] = { 0 };

	flash_read(FLASH_USER_DEF, value, 8);

	float* p = (float*)&value[1];

	offset_x = p[0];
	offset_y = p[1];
	offset_z = p[2];
	offset_gx = p[3];
	offset_gy = p[4];
	offset_gz = p[5];
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
	uint32_t value[8] = { 0 };
	flash_read(FLASH_USER_DEF, value, 8);

	if (value[0] != MAGIC_NUM)
	{
		value[0] = MAGIC_NUM;

		offset_x = 0;
		offset_y = 0;
		offset_z = 0;
		offset_gx = 0;
		offset_gy = 0;
		offset_gz = 0;

		ctl_offset_save();
	}

	ctl_offset_load();

	sleep_ticks(20);

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
	printf("Init mpu6050 OK.\n");

	while (1)
	{
		if (ctl_sw[2] == 1)
		{
			if (ctl_arming == 0 && ctl_thro < 0.15)
			{
				ctl_arming = 1;
				ctl_calibrate = 0;
			}
		}
		else
		{
			ctl_arming = 0;
		}

		if (ctl_sw[3] == 1)
		{
			ctl_angle_p = 2.0f;
		}
		else
		{
			ctl_angle_p = 1.0f;
		}

		//读取姿态信息
		if (mpu6050_value(&mpu_value[0], &mpu_value[1], &mpu_value[2], &mpu_value[3], &mpu_value[4], &mpu_value[5], &mpu_value[6], &mpu_value[7], &mpu_value[8]) == 0)
		{
			for (int i = 0; i < 9; i++)
			{
				xyz_value[i] = mpu_value[i] * xyz_value_filter[i] + xyz_value_pre[i] * (1.0 - xyz_value_filter[i]);
				xyz_value_pre[i] = xyz_value[i];
			}
		}

		float x = xyz_value[0];
		float y = xyz_value[1];
		float z = xyz_value[2];

		float gx = xyz_value[3];
		float gy = xyz_value[4];
		float gz = xyz_value[5];

		float t_x = offset_x + x;
		float t_y = offset_y + y;
		float t_z = offset_z + z;

		float t_gx = offset_gx + gx;
		float t_gy = offset_gy + gy;
		float t_gz = offset_gz + gz;

		//已解锁
		if (ctl_arming)
		{
			// printf("%d %d %d | ", (int)(x * 1000), (int)(y * 1000), (int)(z * 1000));
			// printf("%d %d %d | ", (int)(offset_x * 1000), (int)(offset_y * 1000), (int)(offset_z * 1000));
			// printf("%d %d %d \n", (int)(t_x * 1000), (int)(t_y * 1000), (int)(t_z * 1000));

			//角速度期望转为航向角速期望
			yaw_expect_total += ctl_yaw * yaw_expect_rate;

			// [外环PID控制
			//根据角度期望计算角度误差
			float devi_pitch_angle = (-ctl_pitch) * ctl_angle - t_x;
			float devi_roll_angle = (-ctl_roll) * ctl_angle - t_y;
			float devi_yaw_angle = yaw_expect_total - t_z;
			// PID得到角速度期望
			float ctl_pitch_angle = ctl_pid(devi_pitch_angle, devi_pitch_angle_pre, ctl_param_pitch_roll_angle_p, 0, 0, NULL, 0);
			float ctl_roll_angle = ctl_pid(devi_roll_angle, devi_roll_angle_pre, ctl_param_pitch_roll_angle_p, 0, 0, NULL, 0);
			float ctl_yaw_angle = ctl_pid(devi_yaw_angle, devi_yaw_angle_pre, ctl_param_yaw_angle_p, 0, 0, NULL, 0);
			// 外环PID控制]

			// [内环PID控制
			//根据角速度期望计算角度误差
			float devi_pitch_rate = ctl_pitch_angle - t_gx;
			float devi_roll_rate = ctl_roll_angle - t_gy;
			float devi_yaw_rate = ctl_yaw_angle - t_gz;
			// PID得到控制量
			float ctl_pitch_rate = ctl_pid(devi_pitch_rate, devi_pitch_rate_pre, ctl_param_pitch_roll_rate_p, ctl_param_pitch_roll_rate_i, ctl_param_pitch_roll_rate_d, &ctl_integral_rate_pitch, ctl_thro);
			float ctl_roll_rate = ctl_pid(devi_roll_rate, devi_roll_rate_pre, ctl_param_pitch_roll_rate_p, ctl_param_pitch_roll_rate_i, ctl_param_pitch_roll_rate_d, &ctl_integral_rate_roll, ctl_thro);
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
		//未解锁
		else
		{
			//未校准时校准按钮被按下
			if (ctl_sw[0] == 1 && ctl_calibrate == 0)
			{
				//开始校准
				ctl_calibrate = 1;
			}

			//校准
			if (ctl_calibrate >= 1)
			{
				ctl_offset(-x, -y, -z, -gx, -gy, -gz);
				ctl_calibrate++;
			}
			if (ctl_calibrate > 1000)
			{
				ctl_calibrate = 0;

				ctl_offset_save();
			}
			if (ctl_calibrate == 0)
			{
				ctl_offset_z(-z, -gz);
			}

			ctl_lock_zero();
			ctl_mixer(0, 0, 0, 0, ctl_motor);
		}

		ctl_output();

		tk++;
		sleep_ticks(10);
	}
	return NULL;
}

void controller_task(void)
{
	pcb_create(PCB_CTL_PRIO, &controller_pthread, NULL, PCB_CTL_SIZE);
}
