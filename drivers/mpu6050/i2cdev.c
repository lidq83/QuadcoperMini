/*
 * i2cdev.c
 *
 *  Created on: May 22, 2022
 *      Author: lidq
 */

#include "stm32f1xx_hal.h"
#include "i2cdev.h"

/** Read a single bit from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-7)
 * @param data Container for single bit value
 * @return Status of read operation (1 = success)
 */
char i2cdev_readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data)
{
	uint8_t b;
	uint8_t count = i2cdev_readByte(devAddr, regAddr, &b);
	*data = b & (1 << bitNum);
	return count;
}

/** Read a single bit from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-15)
 * @param data Container for single bit value
 * @return Status of read operation (1 = success)
 */
char i2cdev_readBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t *data)
{
	uint16_t b;
	uint8_t count = i2cdev_readWord(devAddr, regAddr, &b);
	*data = b & (1 << bitNum);
	return count;
}

/** Read multiple bits from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-7)
 * @param length Number of bits to read (not more than 8)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @return Status of read operation (1 = success)
 */
char i2cdev_readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data)
{
	// 01101001 read byte
	// 76543210 bit numbers
	//    xxx   args: bitStart=4, length=3
	//    010   masked
	//   -> 010 shifted
	uint8_t count, b;
	if ((count = i2cdev_readByte(devAddr, regAddr, &b)) != 0)
	{
		uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
		b &= mask;
		b >>= (bitStart - length + 1);
		*data = b;
	}
	return count;
}

/** Read multiple bits from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-15)
 * @param length Number of bits to read (not more than 16)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @return Status of read operation (1 = success, 0 = failure)
 */
char i2cdev_readBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t *data)
{
	// 1101011001101001 read byte
	// fedcba9876543210 bit numbers
	//    xxx           args: bitStart=12, length=3
	//    010           masked
	//           -> 010 shifted
	uint8_t count;
	uint16_t w;
	if ((count = i2cdev_readWord(devAddr, regAddr, &w)) != 0)
	{
		uint16_t mask = ((1 << length) - 1) << (bitStart - length + 1);
		w &= mask;
		w >>= (bitStart - length + 1);
		*data = w;
	}
	return count;
}

/** Read single byte from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param data Container for byte value read from device
 * @return Status of read operation (1 = success)
 */
char i2cdev_readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data)
{
	return i2cdev_readBytes(devAddr, regAddr, 1, data);
}

/** Read single word from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param data Container for word value read from device
 * @return Status of read operation (1 = success)
 */
char i2cdev_readWord(uint8_t devAddr, uint8_t regAddr, uint16_t *data)
{
	return i2cdev_readWords(devAddr, regAddr, 1, data);
}

/** Read multiple bytes from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register regAddr to read from
 * @param length Number of bytes to read
 * @param data Buffer to store read data in
 * @return Number of bytes read (-1 indicates failure)
 */
char i2cdev_readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data)
{
	extern I2C_HandleTypeDef hi2c1;
	HAL_I2C_Mem_Read(&hi2c1, devAddr, regAddr, I2C_MEMADD_SIZE_8BIT, data, length, 0xfff);

	return length;
}

/** Read multiple words from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register regAddr to read from
 * @param length Number of words to read
 * @param data Buffer to store read data in
 * @return Number of words read (0 indicates failure)
 */
char i2cdev_readWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data)
{
	char count = 1;

	extern I2C_HandleTypeDef hi2c1;
	HAL_I2C_Mem_Read(&hi2c1, devAddr, regAddr, I2C_MEMADD_SIZE_8BIT, (uint8_t*) data, 2, 0xfff);

	return count;
}

/** write a single bit in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-7)
 * @param value New bit value to write
 * @return Status of operation (1 = success)
 */
int i2cdev_writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data)
{
	uint8_t b;
	i2cdev_readByte(devAddr, regAddr, &b);
	b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
	return i2cdev_writeByte(devAddr, regAddr, b);
}

/** write a single bit in a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-15)
 * @param value New bit value to write
 * @return Status of operation (1 = success)
 */
int i2cdev_writeBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t data)
{
	uint16_t w;
	i2cdev_readWord(devAddr, regAddr, &w);
	w = (data != 0) ? (w | (1 << bitNum)) : (w & ~(1 << bitNum));
	return i2cdev_writeWord(devAddr, regAddr, w);
}

/** Write multiple bits in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-7)
 * @param length Number of bits to write (not more than 8)
 * @param data Right-aligned value to write
 * @return Status of operation (1 = success)
 */
int i2cdev_writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data)
{
	//      010 value to write
	// 76543210 bit numbers
	//    xxx   args: bitStart=4, length=3
	// 00011100 mask byte
	// 10101111 original value (sample)
	// 10100011 original & ~mask
	// 10101011 masked | value
	uint8_t b;
	if (i2cdev_readByte(devAddr, regAddr, &b) != 0)
	{
		uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
		data <<= (bitStart - length + 1); // shift data into correct position
		data &= mask; // zero all non-important bits in data
		b &= ~(mask); // zero all important bits in existing byte
		b |= data; // combine data with existing byte
		return i2cdev_writeByte(devAddr, regAddr, b);
	}
	else
	{
		return -1;
	}
}

/** Write multiple bits in a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-15)
 * @param length Number of bits to write (not more than 16)
 * @param data Right-aligned value to write
 * @return Status of operation (1 = success)
 */
int i2cdev_writeBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t data)
{
	//              010 value to write
	// fedcba9876543210 bit numbers
	//    xxx           args: bitStart=12, length=3
	// 0001110000000000 mask byte
	// 1010111110010110 original value (sample)
	// 1010001110010110 original & ~mask
	// 1010101110010110 masked | value
	uint16_t w;
	if (i2cdev_readWord(devAddr, regAddr, &w) != 0)
	{
		uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
		data <<= (bitStart - length + 1); // shift data into correct position
		data &= mask; // zero all non-important bits in data
		w &= ~(mask); // zero all important bits in existing word
		w |= data; // combine data with existing word
		return i2cdev_writeWord(devAddr, regAddr, w);
	}
	else
	{
		return -1;
	}
}

/** Write single byte to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param data New byte value to write
 * @return Status of operation (1 = success)
 */
int i2cdev_writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data)
{
	return i2cdev_writeBytes(devAddr, regAddr, 1, &data);
}

/** Write single word to a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param data New word value to write
 * @return Status of operation (1 = success)
 */
int i2cdev_writeWord(uint8_t devAddr, uint8_t regAddr, uint16_t data)
{
	return i2cdev_writeWords(devAddr, regAddr, 1, &data);
}

/** Write multiple bytes to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register address to write to
 * @param length Number of bytes to write
 * @param data Buffer to copy new data from
 * @return Status of operation (1 = success)
 */
int i2cdev_writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data)
{
	extern I2C_HandleTypeDef hi2c1;
	HAL_I2C_Mem_Write(&hi2c1, devAddr, regAddr, I2C_MEMADD_SIZE_8BIT, data, length, 0xfff);

	return 1;
}

/** Write multiple words to a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register address to write to
 * @param length Number of words to write
 * @param data Buffer to copy new data from
 * @return Status of operation (1 = success)
 */
int i2cdev_writeWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data)
{
	uint8_t buff[length * 2];
	for (int i = 0; i < length; i++)
	{
		buff[i * 2] = data[i] >> 8;
		buff[i * 2 + 1] = data[i];
	}

	extern I2C_HandleTypeDef hi2c1;
	HAL_I2C_Mem_Write(&hi2c1, devAddr, regAddr, I2C_MEMADD_SIZE_8BIT, data, length * 2, 0xfff);

	return 1;
}
