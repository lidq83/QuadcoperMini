/*
 * task.c
 *
 *  Created on: December 23, 2019
 *      Author: lidq
 */

#include <attitude_task.h>
#include <bmi160.h>
#include <hmc5883.h>
#include <main.h>
#include <math.h>
#include <ms5611.h>
#include <stdio.h>

extern TIM_HandleTypeDef htim1;

#define MS (5)
#define CALI_CNT (300)

#ifndef M_PI
#define M_PI (3.1415926535897932)
#endif

#define ONE_G (9.8)

typedef struct
{
	double w, x, y, z;
} Quaternion;

typedef struct
{
	double x, y, z;
} Vector3f;

static double exInt = 0;
static double eyInt = 0;
static double ezInt = 0;

static double angle[3] = { 0 };
static double rate[3] = { 0 };
static double acc[3] = { 0 };

double acc_t[3] = { 0 };

double Kp = 0.6;
double Ki = 0.01;

double _w_gyro_bias = 0.1;
double _bias_max = 0.05;

double upper_accel_limit = ONE_G * 1.1f;
double lower_accel_limit = ONE_G * 0.9f;

Vector3f _gyro_bias = { 0 };

static Quaternion q_atti = { 1.0, 0, 0, 0 };

///////////////
/////
// offset[0] = 0.012283777265;
// offset[1] = -0.017036401283;
// offset[2] = -0.016685183598;
/////
///////////////

// 归一化四元数
void normalizeQuaternion(Quaternion* q)
{
	double norm = sqrt(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
	q->w /= norm;
	q->x /= norm;
	q->y /= norm;
	q->z /= norm;
}

void rotateVector(Quaternion* q, double* vector, double* result)
{
	// 四元数乘法，将矢量旋转到机体坐标系
	double qx = q->x;
	double qy = q->y;
	double qz = q->z;
	double qw = q->w;

	double vx = vector[0];
	double vy = vector[1];
	double vz = vector[2];

	// 四元数乘法公式
	result[0] = (1 - 2 * qy * qy - 2 * qz * qz) * vx + (2 * qx * qy - 2 * qz * qw) * vy + (2 * qx * qz + 2 * qy * qw) * vz;
	result[1] = (2 * qx * qy + 2 * qz * qw) * vx + (1 - 2 * qx * qx - 2 * qz * qz) * vy + (2 * qy * qz - 2 * qx * qw) * vz;
	result[2] = (2 * qx * qz - 2 * qy * qw) * vx + (2 * qy * qz + 2 * qx * qw) * vy + (1 - 2 * qx * qx - 2 * qy * qy) * vz;
}

void convertToBodyFrame(Quaternion* q, double mag_x, double mag_y, double mag_z, double mag_offset_x, double mag_offset_y, double mag_offset_z, double* result)
{
	// 将磁罗盘读数转换到机体坐标系
	double mag_vector[3] = { mag_x - mag_offset_x, mag_y - mag_offset_y, mag_z - mag_offset_z };
	rotateVector(q, mag_vector, result);
}

// 角度增量转换为四元数增量（欧拉角）
Quaternion eulerToQuaternion(double roll, double pitch, double yaw)
{
	Quaternion q;

	// 将角度转换为弧度
	roll = roll * M_PI / 180.0;
	pitch = pitch * M_PI / 180.0;
	yaw = yaw * M_PI / 180.0;

	// 计算旋转角的一半的正弦值和余弦值
	double sr = sin(roll / 2);
	double cr = cos(roll / 2);
	double sp = sin(pitch / 2);
	double cp = cos(pitch / 2);
	double sy = sin(yaw / 2);
	double cy = cos(yaw / 2);

	// 计算四元数的分量
	q.w = cr * cp * cy + sr * sp * sy;
	q.x = sr * cp * cy - cr * sp * sy;
	q.y = cr * sp * cy + sr * cp * sy;
	q.z = cr * cp * sy - sr * sp * cy;

	return q;
}

// 角度增量转换为四元数增量（轴角）
Quaternion axisAngleToQuaternion(double gx, double gy, double gz)
{
	Quaternion q;
	double angle = sqrt(gx * gx + gy * gy + gz * gz);
	if (angle < 1.e-10)
	{
		q.w = 1;
		q.x = 0;
		q.y = 0;
		q.z = 0;
	}
	else
	{
		double magnitude = sin(angle / 2) / angle;
		q.w = cos(angle / 2);
		q.x = gx * magnitude;
		q.y = gy * magnitude;
		q.z = gz * magnitude;
	}
	return q;
}

// 四元数的增量更新
Quaternion quaternionMultiply(Quaternion q1, Quaternion q2)
{
	Quaternion q;
	// 执行四元数更新
	q.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
	q.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
	q.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
	q.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
	return q;
}

void quaternionToEuler(Quaternion q, double* roll, double* pitch, double* yaw)
{
	// 计算滚转角（Roll）
	*roll = atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y));

	// 计算俯仰角（Pitch）
	double sinp = 2 * (q.w * q.y - q.z * q.x);
	if (fabs(sinp) >= 1)
	{
		// 避免极限情况下的除零错误
		*pitch = copysign(M_PI / 2, sinp);
	}
	else
	{
		*pitch = asin(sinp);
	}

	// 计算偏航角（Yaw）
	*yaw = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
}

// 将欧拉角转换为初始姿态四元数
Quaternion convertToQuaternion(double roll, double pitch, double yaw)
{
	Quaternion q;

	// 计算对应的三角函数值
	float cosRoll = cos(roll);
	float sinRoll = sin(roll);
	float cosPitch = cos(pitch);
	float sinPitch = sin(pitch);
	float cosYaw = cos(yaw);
	float sinYaw = sin(yaw);

	// 计算欧拉角转换为四元数的公式
	q.w = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
	q.x = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
	q.y = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
	q.z = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;

	return q;
}

// 计算初始角度
void computeInitialAngle(double ax, double ay, double az, double* x, double* y, double* z)
{
	// 计算归一化的加速度向量长度
	float norm = sqrt(ax * ax + ay * ay + az * az);

	// 检查除零情况
	if (norm != 0.0f)
	{
		// 计算俯仰角（pitch）
		*y = asin(ax / norm);

		// 计算滚转角（roll）
		*x = atan2(-ay, -az);

		// 计算偏航角（yaw），这里默认为0，可以根据实际情况进行调整
		*z = 0.0f;
	}
	else
	{
		// 若加速度向量长度为零，无法计算角度
		// 可以根据实际情况进行错误处理或默认值设置
		*x = 0.0f;
		*y = 0.0f;
		*z = 0.0f;
	}
}

void limit_value(double* value, double min, double max)
{
	if (*value < min)
	{
		*value = min;
	}
	if (*value > max)
	{
		*value = max;
	}
}

// 使用加速计补偿角速度
Quaternion complementaryFilter(double dt, Quaternion q, double ax, double ay, double az, double* g_x, double* g_y, double* g_z)
{
	normalizeQuaternion(&q);

	Vector3f _corr = { 0 };

	// 3轴加速度归一化
	double norm = sqrt(ax * ax + ay * ay + az * az);

	if (norm > lower_accel_limit && norm < upper_accel_limit)
	{
		ax = ax / norm;
		ay = ay / norm;
		az = az / norm;

		// 用当前姿态计算出重力在3轴上的分量，策略在ned系下的[0, 0, g]，乘以转换矩阵得到body系加速度。
		// 用参考坐标第ned系转化到机体坐标body系，用四元数表示的方向余弦矩阵第3列
		double vx = 2 * (q.x * q.z - q.w * q.y);
		double vy = 2 * (q.w * q.x + q.y * q.z);
		double vz = q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z;

		// 计算测得的重力与计算得重力间的误差，这个误差是通过微量外积（叉乘）求出
		double ex = (ay * vz - az * vy);
		double ey = (az * vx - ax * vz);
		double ez = (ax * vy - ay * vx);

		// 对误差进行积分
		exInt += ex * Ki;
		eyInt += ey * Ki;
		ezInt += ez * Ki;

		// 将误差进行PI（比例和积分）补偿到陀螺仪角速度
		_corr.x = Kp * ex + exInt;
		_corr.y = Kp * ey + eyInt;
		_corr.z = Kp * ez + ezInt; // 需要通过磁罗盘来补偿偏航角，如果没有磁罗盘则无法补偿
	}

	double gx = *g_x;
	double gy = *g_y;
	double gz = *g_z;

	// 计算角速度向量的长度，也称为模或范数。
	double spinRate = sqrt(gx * gx + gy * gy + gz * gz);
	if (spinRate < 0.175f)
	{
		_gyro_bias.x += _corr.x * _w_gyro_bias * dt;
		_gyro_bias.y += _corr.y * _w_gyro_bias * dt;
		_gyro_bias.z += _corr.z * _w_gyro_bias * dt;

		limit_value(&_gyro_bias.x, -_bias_max, _bias_max);
		limit_value(&_gyro_bias.y, -_bias_max, _bias_max);
		limit_value(&_gyro_bias.z, -_bias_max, _bias_max);
	}

	Vector3f _rates = { 0 };

	_rates.x = gx + _gyro_bias.x;
	_rates.y = gy + _gyro_bias.y;
	_rates.z = gz + _gyro_bias.z;

	_corr.x += _rates.x;
	_corr.y += _rates.y;
	_corr.z += _rates.z;

	Quaternion delta = axisAngleToQuaternion(_corr.x * dt, _corr.y * dt, _corr.z * dt);
	q = quaternionMultiply(q, delta);
	return q;
}

extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi2;

static uint8_t chip_id = 0;

// int8_t bmi160_write_i2c(uint8_t dev_addr, uint8_t reg_addr, uint8_t* write_data, uint16_t len)
// {
// 	HAL_I2C_Mem_Write(&hi2c1, dev_addr, reg_addr, 1, write_data, len, HAL_MAX_DELAY);
// }

// int8_t bmi160_read_i2c(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, uint16_t len)
// {
// 	HAL_I2C_Mem_Read(&hi2c1, dev_addr, reg_addr, 1, data, len, HAL_MAX_DELAY);
// }

int8_t bmi160_write_spi(uint8_t dev_addr, uint8_t reg_addr, uint8_t* write_data, uint16_t len)
{
	uint8_t d_read = 0;
	uint8_t d_write = reg_addr;

	// CS LOW
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

	// HAL_SPI_Transmit(&hspi2, &reg_addr, 1, HAL_MAX_DELAY);
	HAL_SPI_TransmitReceive(&hspi2, &d_write, &d_read, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi2, write_data, len, HAL_MAX_DELAY);

	// CS HIGH
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

	return 0;
}

int8_t bmi160_read_spi(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, uint16_t len)
{
	uint8_t d_read = 0;
	uint8_t d_write = reg_addr;

	// CS LOW
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

	// HAL_SPI_Transmit(&hspi2, &reg_addr, 1, HAL_MAX_DELAY);
	HAL_SPI_TransmitReceive(&hspi2, &d_write, &d_read, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi2, data, len, HAL_MAX_DELAY);

	// CS HIGH
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

	return 0;
}

void bmi160_delay_msec(uint32_t period)
{
	msleep(period);
}

/*********************************************************************/
/* local macro definitions */
/*! I2C interface communication, 1 - Enable; 0- Disable */
#define BMI160_INTERFACE_I2C 0

/*! SPI interface communication, 1 - Enable; 0- Disable */
#define BMI160_INTERFACE_SPI 1

#if (!((BMI160_INTERFACE_I2C == 1) && (BMI160_INTERFACE_SPI == 0)) && (!((BMI160_INTERFACE_I2C == 0) && (BMI160_INTERFACE_SPI == 1))))
#error "Invalid value given for the macros BMI160_INTERFACE_I2C / BMI160_INTERFACE_SPI"
#endif

/*! bmi160 shuttle id */
#define BMI160_SHUTTLE_ID 0x38

/*! bmi160 Device address */
#define BMI160_DEV_ADDR BMI160_I2C_ADDR

/*********************************************************************/
/* global variables */
/*********************************************************************/

/*! @brief This structure containing relevant bmi160 info */
struct bmi160_dev bmi160dev;

/*! @brief variable to hold the bmi160 accel data */
struct bmi160_sensor_data bmi160_accel;

/*! @brief variable to hold the bmi160 gyro data */
struct bmi160_sensor_data bmi160_gyro;

/*********************************************************************/
/* static function declarations */
/*********************************************************************/

/*!
 * @brief   This internal API is used to initialize the bmi160 sensor with default
 */
static int init_bmi160(void);

/*!
 * @brief   This internal API is used to initialize the sensor driver interface
 */
static void init_bmi160_sensor_driver_interface(void);

/*********************************************************************/
/* functions */
/*********************************************************************/

/*!
 *  @brief This internal API is used to initializes the bmi160 sensor
 *  settings like power mode and OSRS settings.
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
static int init_bmi160(void)
{
	int8_t rslt;

	rslt = bmi160_init(&bmi160dev);

	if (rslt == BMI160_OK)
	{
		printf("BMI160 initialization success !\n");
		printf("Chip ID 0x%X\n", bmi160dev.chip_id);
	}
	else
	{
		printf("BMI160 initialization failure %d!\n", rslt);
		return rslt;
	}

	/* Select the Output data rate, range of accelerometer sensor */
	bmi160dev.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
	bmi160dev.accel_cfg.range = BMI160_ACCEL_RANGE_16G;
	bmi160dev.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;

	/* Select the power mode of accelerometer sensor */
	bmi160dev.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

	/* Select the Output data rate, range of Gyroscope sensor */
	bmi160dev.gyro_cfg.odr = BMI160_GYRO_ODR_3200HZ;
	bmi160dev.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
	bmi160dev.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;

	/* Select the power mode of Gyroscope sensor */
	bmi160dev.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

	/* Set the sensor configuration */
	rslt = bmi160_set_sens_conf(&bmi160dev);

	return rslt;
}

/*!
 *  @brief This internal API is used to set the sensor driver interface to
 *  read/write the data.
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
static void init_bmi160_sensor_driver_interface(void)
{
#if BMI160_INTERFACE_I2C == 1

	/* I2C setup */

	/* link read/write/delay function of host system to appropriate
	 * bmi160 function call prototypes */
	bmi160dev.write = bmi160_write_i2c;
	bmi160dev.read = bmi160_read_i2c;
	bmi160dev.delay_ms = bmi160_delay_msec;

	/* set correct i2c address */
	bmi160dev.id = BMI160_DEV_ADDR;
	bmi160dev.intf = BMI160_I2C_INTF;
#endif
#if BMI160_INTERFACE_SPI == 1

	/* SPI setup */

	/* link read/write/delay function of host system to appropriate
	 *  bmi160 function call prototypes */
	bmi160dev.write = bmi160_write_spi;
	bmi160dev.read = bmi160_read_spi;
	bmi160dev.delay_ms = bmi160_delay_msec;
	bmi160dev.id = BMI160_SHUTTLE_ID;
	bmi160dev.intf = BMI160_SPI_INTF;
#endif
}

void* attitude_pthread(void* arg)
{
_restart:

	msleep(100);
	printf("BMI160 init\n");
	init_bmi160_sensor_driver_interface();
	msleep(100);
	int ret = init_bmi160();
	if (ret != 0)
	{
		goto _restart;
	}
	printf("BMI160 init ok\n");

	Barometer_init();
	msleep(100);
	Barometer_setOSR(OSR_256);
	msleep(100);
	printf("MS5611 init ok\n");

	HMC5883L_setRange(HMC5883L_RANGE_8_1GA);
	HMC5883L_setMeasurementMode(HMC5883L_CONTINOUS);
	HMC5883L_setDataRate(HMC5883L_DATARATE_75HZ);
	HMC5883L_setSamples(HMC5883L_SAMPLES_8);
	printf("HMC5883 init ok\n");

	uint32_t tk = 0;

	angle[0] = 0;
	angle[1] = 0;
	angle[2] = 0;

	double timepre = 0;
	int init_angle_cnt = 100;

	while (1)
	{
		// 计算当前时间戳
		uint64_t cnt = get_count();
		double timestamp = cnt / 1000000.0;
		double dt = timestamp - timepre;
		timepre = timestamp;

		bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL), &bmi160_accel, &bmi160_gyro, &bmi160dev);

		float altitude = Barometer_getAltitude(true);

		Vector mag = HMC5883L_readData();

		// if (tk % 10 == 0)
		// {
		// 	printf("acc %d %d %d gyro %d %d %d\n", //
		// 		   bmi160_accel.x,
		// 		   bmi160_accel.y,
		// 		   bmi160_accel.z,
		// 		   bmi160_gyro.x,
		// 		   bmi160_gyro.y,
		// 		   bmi160_gyro.z);
		// }

		// 转为弧度制
		rate[0] = (bmi160_gyro.x / 32768.0 * 2000.0) * M_PI / 180.0;
		rate[1] = (bmi160_gyro.y / 32768.0 * 2000.0) * M_PI / 180.0;
		rate[2] = (bmi160_gyro.z / 32768.0 * 2000.0) * M_PI / 180.0;

		// 转为加速度
		acc[0] = bmi160_accel.x / 32768.0f * (ONE_G * 16.0);
		acc[1] = bmi160_accel.y / 32768.0f * (ONE_G * 16.0);
		acc[2] = bmi160_accel.z / 32768.0f * (ONE_G * 16.0);

		// 互补滤波
		q_atti = complementaryFilter(dt, q_atti, acc[0], acc[1], acc[2], &rate[0], &rate[1], &rate[2]);

		// 姿态四元数转欧拉角
		quaternionToEuler(q_atti, &angle[0], &angle[1], &angle[2]);

		// 转角度制
		for (int i = 0; i < 3; i++)
		{
			angle[i] = angle[i] * 180.0 / M_PI;
		}

		if (tk % 10 == 0)
		{
			printf("angle %4d %4d %4d alt %d mag %4d %4d %4d\n", //
				   (int)(angle[0] * 10),
				   (int)(angle[1] * 10),
				   (int)(angle[2] * 10),
				   (int)(altitude * 1000),
				   (int)(mag.XAxis * 10),
				   (int)(mag.YAxis * 10),
				   (int)(mag.ZAxis * 10));
		}

		tk++;

		msleep(MS);
	}
	return NULL;
}

void attitude_task(void)
{
	pcb_create(PCB_ATTITUDE_PRIO, &attitude_pthread, NULL, PCB_ATTITUDE_SIZE);
}
