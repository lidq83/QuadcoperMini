#include <math.h>
#include <ms5611.h>
#include <sche.h>

#define SEA_LEVEL_PRESSURE 101325.0 // 海平面标准大气压力值（Pa）
#define TEMPERATURE_CORRECTION 1.0 // 温度修正值

// TODO : set a proper timing
#define SPI_TIMEOUT 50 // in ms

#define MS5611_EN HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
#define MS5611_DIS HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);

#define CMD_RESET 0x1E
#define CMD_PROM_C1 0xA2
#define CMD_PROM_C2 0xA4
#define CMD_PROM_C3 0xA6
#define CMD_PROM_C4 0xA8
#define CMD_PROM_C5 0xAA
#define CMD_PROM_C6 0xAC

#define PRESSURE_OSR_256 0x40
#define PRESSURE_OSR_512 0x42
#define PRESSURE_OSR_1024 0x44
#define PRESSURE_OSR_2048 0x46
#define PRESSURE_OSR_4096 0x48

#define TEMP_OSR_256 0x50
#define TEMP_OSR_512 0x52
#define TEMP_OSR_1024 0x54
#define TEMP_OSR_2048 0x56
#define TEMP_OSR_4096 0x58

#define CONVERSION_OSR_256 1
#define CONVERSION_OSR_512 2
#define CONVERSION_OSR_1024 3
#define CONVERSION_OSR_2048 5
#define CONVERSION_OSR_4096 10

static uint16_t prom[6];
extern SPI_HandleTypeDef hspi2;

// min OSR by default
static uint8_t pressAddr = PRESSURE_OSR_256;
static uint8_t tempAddr = TEMP_OSR_256;
static uint32_t convDelay = CONVERSION_OSR_256;

static int32_t temperature;
static int32_t pressure;
static double altitude;

/**
 * @brief init the pressure sensor with default parameters
 */
static void ms5611_init();

/**
 * @brief write the command passed in parameters to the SPI Bus
 *
 * @param data the command to write
 */
static void ms5611_write(uint8_t data);

/**
 * @brief read the n bits of the SPI Bus on  register reg
 *
 * @return the value read on the SPI bus
 */
static uint16_t ms5611_read16bits(uint8_t reg);
static uint32_t ms5611_read24bits(uint8_t reg);

/**
 * @brief read the raw value
 *
 * @return the value read on the SPI bus
 */
static uint32_t ms5611_readRawTemp();
static uint32_t ms5611_readRawPressure();


static void ms5611_init()
{
	MS5611_DIS
	msleep(100);

	ms5611_write(CMD_RESET);
	msleep(100);

	prom[0] = ms5611_read16bits(CMD_PROM_C1);
	prom[1] = ms5611_read16bits(CMD_PROM_C2);
	prom[2] = ms5611_read16bits(CMD_PROM_C3);
	prom[3] = ms5611_read16bits(CMD_PROM_C4);
	prom[4] = ms5611_read16bits(CMD_PROM_C5);
	prom[5] = ms5611_read16bits(CMD_PROM_C6);
}

static void ms5611_write(uint8_t data)
{
	MS5611_EN
	HAL_SPI_Transmit(&hspi2, &data, 1, SPI_TIMEOUT);
	MS5611_DIS
}

static uint16_t ms5611_read16bits(uint8_t reg)
{
	uint8_t byte[3];
	uint16_t return_value;
	MS5611_EN
	HAL_SPI_TransmitReceive(&hspi2, &reg, byte, 3, SPI_TIMEOUT);
	MS5611_DIS
	/**
	 * We dont care about byte[0] because that is what was recorded while
	 * we were sending the first byte of the cmd. Since the baro wasn't sending
	 * actual data at that time (it was listening for command), data[0] will
	 * contain garbage data (probably all 0's).
	 */
	return_value = ((uint16_t)byte[1] << 8) | (byte[2]);
	return return_value;
}

static uint32_t ms5611_read24bits(uint8_t reg)
{
	uint8_t byte[4];
	uint32_t return_value;
	MS5611_EN
	HAL_SPI_TransmitReceive(&hspi2, &reg, byte, 4, SPI_TIMEOUT);
	MS5611_DIS
	return_value = ((uint32_t)byte[1] << 16) | ((uint32_t)(byte[2] << 8)) | (byte[3]);
	return return_value;
}

static uint32_t ms5611_readRawTemp()
{
	uint32_t D2;
	// Convert temp
	ms5611_write(tempAddr);
	// Conversion Time
	msleep(convDelay);
	// Read ADC
	D2 = ms5611_read24bits(0x00);
	return D2;
}

static uint32_t ms5611_readRawPressure()
{
	uint32_t D1;
	// Convert pressure
	ms5611_write(pressAddr);
	// Conversion time
	msleep(convDelay);
	// Read ADC
	D1 = ms5611_read24bits(0x00);
	return D1;
}

void Barometer_init()
{
	ms5611_init();
}

void Barometer_setOSR(OSR osr)
{
	switch (osr)
	{
	default:
	case OSR_256:
		pressAddr = PRESSURE_OSR_256;
		tempAddr = TEMP_OSR_256;
		convDelay = CONVERSION_OSR_256;
		break;
	case OSR_512:
		pressAddr = PRESSURE_OSR_512;
		tempAddr = TEMP_OSR_512;
		convDelay = CONVERSION_OSR_512;
		break;
	case OSR_1024:
		pressAddr = PRESSURE_OSR_1024;
		tempAddr = TEMP_OSR_1024;
		convDelay = CONVERSION_OSR_1024;
		break;
	case OSR_2048:
		pressAddr = PRESSURE_OSR_2048;
		tempAddr = TEMP_OSR_2048;
		convDelay = CONVERSION_OSR_2048;
		break;
	case OSR_4096:
		pressAddr = PRESSURE_OSR_4096;
		tempAddr = TEMP_OSR_4096;
		convDelay = CONVERSION_OSR_4096;
		break;
	}
}

int32_t Barometer_getTemp(bool calculate)
{
	if (calculate)
	{
		Barometer_calculate();
	}
	return temperature;
}

int32_t Barometer_getPressure(bool calculate)
{
	if (calculate)
	{
		Barometer_calculate();
	}
	return pressure;
}

double Barometer_getAltitude(bool calculate)
{
	if (calculate)
	{
		Barometer_calculate();
	}
	return altitude;
}

double Barometer_calculate()
{
	int32_t dT;
	int64_t TEMP, OFF, SENS, P;
	uint32_t D1, D2;
	double press, r, c;

	D1 = ms5611_readRawPressure();
	D2 = ms5611_readRawTemp();

	dT = D2 - ((long)prom[4] * 256);
	TEMP = 2000 + ((int64_t)dT * prom[5]) / 8388608;
	OFF = (int64_t)prom[1] * 65536 + ((int64_t)prom[3] * dT) / 128;
	SENS = (int64_t)prom[0] * 32768 + ((int64_t)prom[2] * dT) / 256;

	if (TEMP < 2000)
	{
		// second order temperature compensation
		int64_t T2 = (((int64_t)dT) * dT) >> 31;
		int64_t Aux_64 = (TEMP - 2000) * (TEMP - 2000);
		int64_t OFF2 = (5 * Aux_64) >> 1;
		int64_t SENS2 = (5 * Aux_64) >> 2;
		TEMP = TEMP - T2;
		OFF = OFF - OFF2;
		SENS = SENS - SENS2;
	}

	P = (D1 * SENS / 2097152 - OFF) / 32768;
	temperature = TEMP;
	pressure = P;

	press = (double)pressure;
	r = press / 101325.0;
	c = 1.0 / 5.255;
	altitude = (1 - pow(r, c)) * 44330.77;
}
