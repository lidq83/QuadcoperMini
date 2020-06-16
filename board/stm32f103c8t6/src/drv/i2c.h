/*
 * i2c.h
 *
 *  Created on: Jul 14, 2017
 *      Author: lidq
 */

#ifndef SRC_INCLUDE_I2C_H_
#define SRC_INCLUDE_I2C_H_

#include <typedef.h>

void I2C_init();

void I2C_single_write(I2C_TypeDef* I2Cx, uint8_t HW_address, uint8_t addr, uint8_t data);

void i2cdev_writeBit(u8 devAddr, u8 regAddr, u8 bitNum, u8 data);

void i2cdev_writeBits(u8 devAddr, u8 regAddr, u8 bitStart, u8 length, u8 data);

void i2cdev_readBit(u8 devAddr, u8 regAddr, u8 bitNum, u8* data);

void i2cdev_readBits(u8 devAddr, u8 regAddr, u8 bitStart, u8 length, u8* data);

void i2cdev_readBytes(u8 devAddr, u8 regAddr, u8 length, u8* data);

void i2cdev_readByte(u8 devAddr, u8 regAddr, u8* data);

void i2cdev_writeByte(u8 devAddr, u8 regAddr, u8 data);

void i2cdev_writeWord(u8 devAddr, u8 regAddr, u16 data);

void i2cdev_writeBytes(u8 devAddr, u8 regAddr, u8 length, u8* data);

#endif /* SRC_INCLUDE_I2C_H_ */
