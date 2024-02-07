
#include <hmc5883.h>

static double mgPerDigit = 0.73f;
static Vector v = { 0 };

extern I2C_HandleTypeDef hi2c1;

Vector HMC5883L_readRaw(void)
{
	v.XAxis = HMC5883L_readRegister16(HMC5883L_REG_OUT_X_M);
	v.ZAxis = HMC5883L_readRegister16(HMC5883L_REG_OUT_Z_M);
	v.YAxis = HMC5883L_readRegister16(HMC5883L_REG_OUT_Y_M);

	return v;
}

Vector HMC5883L_readNormalize(void)
{
	v.XAxis = (double)HMC5883L_readRegister16(HMC5883L_REG_OUT_X_M) * mgPerDigit;
	v.ZAxis = (double)HMC5883L_readRegister16(HMC5883L_REG_OUT_Z_M) * mgPerDigit;
	v.YAxis = (double)HMC5883L_readRegister16(HMC5883L_REG_OUT_Y_M) * mgPerDigit;

	return v;
}

Vector HMC5883L_readData(void)
{
	uint8_t buff[6];
	uint8_t reg = HMC5883L_REG_OUT_X_M;
	HAL_I2C_Mem_Read(&hi2c1, HMC5883L_DEFAULT_ADDRESS, reg, 1, buff, 6, 3000);

	int16_t x = buff[0] << 8 | buff[1];
	int16_t z = buff[2] << 8 | buff[3];
	int16_t y = buff[4] << 8 | buff[5];

	v.XAxis = x * mgPerDigit;
	v.YAxis = y * mgPerDigit;
	v.ZAxis = z * mgPerDigit;

	return v;
}

void HMC5883L_setRange(hmc5883l_range_t range)
{
	switch (range)
	{
	case HMC5883L_RANGE_0_88GA:
		mgPerDigit = 0.73f;
		break;

	case HMC5883L_RANGE_1_3GA:
		mgPerDigit = 0.92f;
		break;

	case HMC5883L_RANGE_1_9GA:
		mgPerDigit = 1.22f;
		break;

	case HMC5883L_RANGE_2_5GA:
		mgPerDigit = 1.52f;
		break;

	case HMC5883L_RANGE_4GA:
		mgPerDigit = 2.27f;
		break;

	case HMC5883L_RANGE_4_7GA:
		mgPerDigit = 2.56f;
		break;

	case HMC5883L_RANGE_5_6GA:
		mgPerDigit = 3.03f;
		break;

	case HMC5883L_RANGE_8_1GA:
		mgPerDigit = 4.35f;
		break;

	default:
		mgPerDigit = 0.73f;
		break;
	}

	HMC5883L_writeRegister8(HMC5883L_REG_CONFIG_B, range << 5);
}

hmc5883l_range_t HMC5883L_getRange(void)
{
	return (hmc5883l_range_t)((HMC5883L_readRegister8(HMC5883L_REG_CONFIG_B) >> 5));
}

void HMC5883L_setMeasurementMode(hmc5883l_mode_t mode)
{
	uint8_t value;

	value = HMC5883L_readRegister8(HMC5883L_REG_MODE);
	value &= 0xFC;
	value |= mode;

	HMC5883L_writeRegister8(HMC5883L_REG_MODE, value);
}

hmc5883l_mode_t HMC5883L_getMeasurementMode(void)
{
	uint8_t value;

	value = HMC5883L_readRegister8(HMC5883L_REG_MODE);
	value &= 0x03;

	return (hmc5883l_mode_t)value;
}

void HMC5883L_setDataRate(hmc5883l_dataRate_t dataRate)
{
	uint8_t value;

	value = HMC5883L_readRegister8(HMC5883L_REG_CONFIG_A);
	value &= 0xE3;
	value |= (dataRate << 2);

	HMC5883L_writeRegister8(HMC5883L_REG_CONFIG_A, value);
}

hmc5883l_dataRate_t HMC5883L_getDataRate(void)
{
	uint8_t value;

	value = HMC5883L_readRegister8(HMC5883L_REG_CONFIG_A);
	value &= 0x1C;
	value >>= 2;

	return (hmc5883l_dataRate_t)value;
}

void HMC5883L_setSamples(hmc5883l_samples_t samples)
{
	uint8_t value;

	value = HMC5883L_readRegister8(HMC5883L_REG_CONFIG_A);
	value &= 0x9F;
	value |= (samples << 5);

	HMC5883L_writeRegister8(HMC5883L_REG_CONFIG_A, value);
}

hmc5883l_samples_t HMC5883L_getSamples(void)
{
	uint8_t value;

	value = HMC5883L_readRegister8(HMC5883L_REG_CONFIG_A);
	value &= 0x60;
	value >>= 5;

	return (hmc5883l_samples_t)value;
}

// Write byte to register
void HMC5883L_writeRegister8(uint8_t reg, uint8_t value)
{
	HAL_I2C_Mem_Write(&hi2c1, HMC5883L_DEFAULT_ADDRESS, reg, 1, &value, 1, 3000);
}

// Read byte to register
uint8_t HMC5883L_fastRegister8(uint8_t reg)
{
	uint8_t value;
	//  HAL_I2C_Mem_Write(&hi2c1, HMC5883L_ADDRESS, reg, 1 ,value,1,500)
	HAL_I2C_Mem_Read(&hi2c1, HMC5883L_DEFAULT_ADDRESS, reg, 1, &value, 1, 3000);
	return value;
}

// Read byte from register
uint8_t HMC5883L_readRegister8(uint8_t reg)
{
	uint8_t value;
	HAL_I2C_Mem_Read(&hi2c1, HMC5883L_DEFAULT_ADDRESS, reg, 1, &value, 1, 3000);
	return value;
}

// Read word from register
int16_t HMC5883L_readRegister16(uint8_t reg)
{
	int16_t value;

	uint8_t vha[2];

	HAL_I2C_Mem_Read(&hi2c1, HMC5883L_DEFAULT_ADDRESS, reg, 1, vha, 2, 3000);

	value = vha[0] << 8 | vha[1];
	return value;
}
