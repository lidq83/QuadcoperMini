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
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart1;

#define _LOG	 (0)

#define MS		 (1)
#define CALI_CNT (300)

static uint8_t chip_id = 0;

int8_t bmi160_write_spi(uint8_t dev_addr, uint8_t reg_addr, uint8_t* write_data, uint16_t len)
{
	uint8_t d_read	= 0;
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
	uint8_t d_read	= 0;
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
#define BMI160_DEV_ADDR	  BMI160_I2C_ADDR

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
	bmi160dev.accel_cfg.odr	  = BMI160_ACCEL_ODR_1600HZ;
	bmi160dev.accel_cfg.range = BMI160_ACCEL_RANGE_16G;
	bmi160dev.accel_cfg.bw	  = BMI160_ACCEL_BW_NORMAL_AVG4;

	/* Select the power mode of accelerometer sensor */
	bmi160dev.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

	/* Select the Output data rate, range of Gyroscope sensor */
	bmi160dev.gyro_cfg.odr	 = BMI160_GYRO_ODR_3200HZ;
	bmi160dev.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
	bmi160dev.gyro_cfg.bw	 = BMI160_GYRO_BW_NORMAL_MODE;

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
	bmi160dev.write	   = bmi160_write_i2c;
	bmi160dev.read	   = bmi160_read_i2c;
	bmi160dev.delay_ms = bmi160_delay_msec;

	/* set correct i2c address */
	bmi160dev.id   = BMI160_DEV_ADDR;
	bmi160dev.intf = BMI160_I2C_INTF;
#endif
#if BMI160_INTERFACE_SPI == 1

	/* SPI setup */

	/* link read/write/delay function of host system to appropriate
	 *  bmi160 function call prototypes */
	bmi160dev.write	   = bmi160_write_spi;
	bmi160dev.read	   = bmi160_read_spi;
	bmi160dev.delay_ms = bmi160_delay_msec;
	bmi160dev.id	   = BMI160_SHUTTLE_ID;
	bmi160dev.intf	   = BMI160_SPI_INTF;
#endif
}

void calculateRealAcceleration(double accel_raw[3], double accel_offset[3], double accel_T[3][3], double accel_real[3])
{
	// Subtract offset
	double accel_corrected[3];
	for (int i = 0; i < 3; i++)
	{
		accel_corrected[i] = accel_raw[i] - accel_offset[i];
	}

	// Apply scaling and cross-axis compensation
	for (int i = 0; i < 3; i++)
	{
		accel_real[i] = accel_T[i][0] * accel_corrected[0] + accel_T[i][1] * accel_corrected[1] + accel_T[i][2] * accel_corrected[2];
	}
}

void calibrate_magnetometer(double* real, double* raw, double* offset, double* scale)
{
	// Apply offset correction
	double calibrated[3];
	calibrated[0] = raw[0] - offset[0];
	calibrated[1] = raw[1] - offset[1];
	calibrated[2] = raw[2] - offset[2];

	// Apply scale correction
	real[0] = calibrated[0] / scale[0];
	real[1] = calibrated[1] / scale[1];
	real[2] = calibrated[2] / scale[2];
}

#define Kp	  25.0f	  // 比例增益
#define Ki	  1.2f	  // 积分增益
#define halfT 0.0025f // 采样周期的一半

static Quaternion q_atti = { 1.0, 0, 0, 0 };
static Vector3f angle	 = { 0, 0, 0 };
static Vector3f eInt	 = { 0, 0, 0 };

void IMUupdate(Quaternion* q, Vector3f gyro, Vector3f accel, Vector3f mag)
{
	double norm = 1;

	Vector3f h = { 0 };
	Vector3f b = { 0 };
	Vector3f w = { 0 };
	Vector3f v = { 0 };
	Vector3f e = { 0 };

	// 先把这些用得到的值算好
	double q0q0 = q->w * q->w;
	double q0q1 = q->w * q->x;
	double q0q2 = q->w * q->y;
	double q0q3 = q->w * q->z;
	double q1q1 = q->x * q->x;
	double q1q2 = q->x * q->y;
	double q1q3 = q->x * q->z;
	double q2q2 = q->y * q->y;
	double q2q3 = q->y * q->z;
	double q3q3 = q->z * q->z;

	if (accel.x * accel.y * accel.z == 0)
	{
		return;
	}

	if (mag.x * mag.y * mag.z == 0)
	{
		return;
	}

	norm = sqrt(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z); // accel数据归一化
	accel.x /= norm;
	accel.y /= norm;
	accel.z /= norm;

	norm = sqrt(mag.x * mag.x + mag.y * mag.y + mag.z * mag.z); // mag数据归一化
	mag.x /= norm;
	mag.y /= norm;
	mag.z /= norm;

	h.x = 2 * mag.x * (0.5 - q2q2 - q3q3) + 2 * mag.y * (q1q2 - q0q3) + 2 * mag.z * (q1q3 + q0q2);
	h.y = 2 * mag.x * (q1q2 + q0q3) + 2 * mag.y * (0.5 - q1q1 - q3q3) + 2 * mag.z * (q2q3 - q0q1);
	h.z = 2 * mag.x * (q1q3 - q0q2) + 2 * mag.y * (q2q3 + q0q1) + 2 * mag.z * (0.5 - q1q1 - q2q2);
	b.x = sqrt((h.x * h.x) + (h.y * h.y));
	b.y = 0;
	b.z = h.z;

	// estimated direction of gravity and flux (v and w)              估计重力方向和流量/变迁
	v.x = 2 * (q1q3 - q0q2); // 四元素中xyz的表示
	v.y = 2 * (q0q1 + q2q3);
	v.z = q0q0 - q1q1 - q2q2 + q3q3;

	w.x = 2 * b.x * (0.5 - q2q2 - q3q3) + 2 * b.z * (q1q3 - q0q2);
	w.y = 2 * b.x * (q1q2 - q0q3) + 2 * b.z * (q0q1 + q2q3);
	w.z = 2 * b.x * (q0q2 + q1q3) + 2 * b.z * (0.5 - q1q1 - q2q2);

	e.x = (accel.y * v.z - accel.z * v.y) + (mag.y * w.z - mag.z * w.y);
	e.y = (accel.z * v.x - accel.x * v.z) + (mag.z * w.x - mag.x * w.z);
	e.z = (accel.x * v.y - accel.y * v.x) + (mag.x * w.y - mag.y * w.x);

	eInt.z += e.x * Ki; // 对误差进行积分
	eInt.y += e.y * Ki;
	eInt.z += e.z * Ki;

	// adjusted gyroscope measurements
	gyro.x += Kp * e.x + eInt.x; // 将误差PI后补偿到陀螺仪，即补偿零点漂移
	gyro.y += Kp * e.y + eInt.y;
	gyro.z += Kp * e.z + eInt.z; // 这里的gz由于没有观测者进行矫正会产生漂移，表现出来的就是积分自增或自减

	// integrate quaternion rate and normalise                           //四元素的微分方程
	q->w = q->w + (-q->x * gyro.x - q->y * gyro.y - q->z * gyro.z) * halfT;
	q->x = q->x + (q->w * gyro.x + q->y * gyro.z - q->z * gyro.y) * halfT;
	q->y = q->y + (q->w * gyro.y - q->x * gyro.z + q->z * gyro.x) * halfT;
	q->z = q->z + (q->w * gyro.z + q->x * gyro.y - q->y * gyro.x) * halfT;

	// normalise quaternion
	norm = sqrt(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
	q->w = q->w / norm;
	q->x = q->x / norm;
	q->y = q->y / norm;
	q->z = q->z / norm;

	angle.x = asin(-2 * q->x * q->z + 2 * q->w * q->y);											// pitch
	angle.y = atan2(2 * q->y * q->z + 2 * q->w * q->x, -2 * q->x * q->x - 2 * q->y * q->y + 1); // roll
	angle.z = atan2(2 * q->x * q->y + 2 * q->w * q->z, -2 * q->y * q->y - 2 * q->z * q->z + 1); // yaw
}

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

Vector3f RotateVectorByQuaternion(Vector3f vector, Quaternion q)
{
	// 将磁力计数据转换为四元数表示
	Quaternion quat;
	quat.w = 0.0f;
	quat.x = vector.x;
	quat.y = vector.y;
	quat.z = vector.z;

	// 四元数旋转变换
	Quaternion q_conjugate = { q.w, -q.x, -q.y, -q.z };
	Quaternion ned		   = quaternionMultiply(quaternionMultiply(q, quat), q_conjugate);

	// 提取转换后的磁力计数据
	Vector3f ret;
	ret.x = ned.x;
	ret.y = ned.y;
	ret.z = ned.z;
	return ret;
}

#define Q_accel	   0.03 // 加速计噪声协方差
#define R_accel	   0.5	// 加速计测量噪声协方差
#define Q_altitude 0.1	// 气压计噪声协方差
#define R_altitude 1.0	// 气压计测量噪声协方差

// 系统状态向量
typedef struct
{
	double height;	 // 高度
	double velocity; // 垂直速度
} StateVector;

typedef struct output_s
{
	float dt;			// 时间戳约5ms
	float gyro[3];		// 角速度
	float accel[3];		// 加速度
	float mag[3];		// 磁罗盘
	float q[4];			// 姿态四元数
	float angle[3];		// 欧拉角
	float accel_ned[3]; // 北东地坐标系加速度
	float altitude[2];	// 气压计高度[0]为原始读数，[1]为低通滤波后数值
} output_s;

void output_data()
{
}

void* attitude_pthread(void* arg)
{
_restart:
	msleep(100);
	printf("BMI160 init\n");
	init_bmi160_sensor_driver_interface();
	int ret = init_bmi160();
	if (ret != 0)
	{
		goto _restart;
	}
	printf("BMI160 init ok\n");

	msleep(100);
	Barometer_init();
	Barometer_setOSR(OSR_256);
	printf("MS5611 init ok\n");

	HMC5883L_setRange(HMC5883L_RANGE_8_1GA);
	HMC5883L_setMeasurementMode(HMC5883L_CONTINOUS);
	HMC5883L_setDataRate(HMC5883L_DATARATE_75HZ);
	HMC5883L_setSamples(HMC5883L_SAMPLES_8);
	printf("HMC5883 init ok\n");

	uint32_t tk = 0;

	double timepre = 0;

	double alt_f   = 0.1;
	double alt_val = 0;
	double alt_pre = 0;

	// 手工校准后的零偏和标度因数
	double accel_offset[3] = { -1.653500 - 0.449500 - 0.311000 };
	double accel_T[3][3]   = {
		  { 1.000163, -0.006901, -0.064262 },
		  { -0.019924, 1.002784, 0.005600 },
		  { -0.007842, -0.006224, 0.983009 }
	};
	//

	double acc[3]		 = { 0 };
	double accel_corr[3] = { 0 };

	// 手工校准后的零偏和标度因数
	double mag_offset[3] = { -144.322078, -99.051393, 58.861521 };
	double mag_scale[3]	 = { 653.915949, 664.681395, 631.992458 };
	//
	double mag_raw[3]  = { 0 };
	double mag_corr[3] = { 0 };

	double altitude_offset = 0;
	double altitude		   = 0;
	double accel_z_offset  = 0;
	double accel_z		   = 0;

	Vector3f gyro	   = { 0 };
	Vector3f accel	   = { 0 };
	Vector3f mag	   = { 0 };
	Vector3f accel_ned = { 0 };

	StateVector st = { 0, 0 };

	output_s log = { 0 };

	while (1)
	{
		// 计算当前时间戳
		uint64_t cnt	 = get_count();
		double timestamp = cnt / 1000000.0;
		double dt		 = timestamp - timepre;
		timepre			 = timestamp;

		// 取得陀螺和加速度读数
		bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL), &bmi160_accel, &bmi160_gyro, &bmi160dev);
		// 转为弧度制
		gyro.x = (bmi160_gyro.x / 32768.0 * 2000.0) * M_PI / 180.0;
		gyro.y = (bmi160_gyro.y / 32768.0 * 2000.0) * M_PI / 180.0;
		gyro.z = (bmi160_gyro.z / 32768.0 * 2000.0) * M_PI / 180.0;
		// 转为加速度
		acc[0] = bmi160_accel.x / 32768.0f * (ONE_G * 16.0);
		acc[1] = bmi160_accel.y / 32768.0f * (ONE_G * 16.0);
		acc[2] = bmi160_accel.z / 32768.0f * (ONE_G * 16.0);
		// 根据offset和scale得到真实值
		calculateRealAcceleration(acc, accel_offset, accel_T, accel_corr);
		accel.x = accel_corr[0];
		accel.y = accel_corr[1];
		accel.z = accel_corr[2];

		// 取得气压计读数
		double alt_origin = Barometer_calculate();
		alt_val			  = alt_origin * alt_f + alt_pre * (1.0 - alt_f);
		alt_pre			  = alt_val;

		// 取得磁罗盘读数
		Vector m   = HMC5883L_readData();
		mag_raw[0] = m.XAxis;
		mag_raw[1] = m.YAxis;
		mag_raw[2] = m.ZAxis;
		// 根据offset和scale得到真实值
		calibrate_magnetometer(mag_corr, mag_raw, mag_offset, mag_scale);
		mag.x = mag_corr[0];
		mag.y = mag_corr[1];
		mag.z = mag_corr[2];

		IMUupdate(&q_atti, gyro, accel, mag);

		// 将加速度由机体坐标系转为ned坐标系
		accel_ned = RotateVectorByQuaternion(accel, q_atti);

#if !_LOG
		if (tk % 10 == 0)
		{
			printf("dt %d angle %4d %4d %4d accel_ned %4d %4d %4d\n", //
				   (int)(dt * 10000),
				   (int)(angle.x * 180 / M_PI * 10),
				   (int)(angle.y * 180 / M_PI * 10),
				   (int)(angle.z * 180 / M_PI * 10),
				   (int)(accel_ned.x * 10),
				   (int)(accel_ned.y * 10),
				   (int)(accel_ned.z * 10));
		}
#else
		log.dt			 = dt;
		log.gyro[0]		 = gyro.x;
		log.gyro[1]		 = gyro.y;
		log.gyro[2]		 = gyro.z;
		log.accel[0]	 = accel.x;
		log.accel[1]	 = accel.y;
		log.accel[2]	 = accel.z;
		log.mag[0]		 = mag.x;
		log.mag[1]		 = mag.y;
		log.mag[2]		 = mag.z;
		log.q[0]		 = q_atti.w;
		log.q[1]		 = q_atti.x;
		log.q[2]		 = q_atti.y;
		log.q[3]		 = q_atti.z;
		log.angle[0]	 = angle.x;
		log.angle[1]	 = angle.y;
		log.angle[2]	 = angle.z;
		log.accel_ned[0] = accel_ned.x;
		log.accel_ned[1] = accel_ned.y;
		log.accel_ned[2] = accel_ned.z;
		log.altitude[0]	 = alt_origin;
		log.altitude[1]	 = alt_val;

		uint8_t head[2] = { 0x55, 0xaa };
		HAL_UART_Transmit(&huart1, &head[0], 1, 0xFFFF); // 阻塞方式打印
		HAL_UART_Transmit(&huart1, &head[1], 1, 0xFFFF); // 阻塞方式打印
		uint8_t* p = (uint8_t*)&log;
		for (int i = 0; i < sizeof(output_s); i++)
		{
			HAL_UART_Transmit(&huart1, &p[i], 1, 0xFFFF);
		}
#endif
		tk++;

		msleep(MS);
	}
	return NULL;
}

void attitude_task(void)
{
	pcb_create(PCB_ATTITUDE_PRIO, &attitude_pthread, NULL, PCB_ATTITUDE_SIZE);
}
