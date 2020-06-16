#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_i2c.h"
#include "i2c.h"

void I2C_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;       //两个引脚都加 4.7K 上拉电阻
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	I2C_InitTypeDef I2C_InitStructure;

	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0xc0; // MPU6050 7-bit adress = 0x68, 8-bit adress = 0xD0;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = 100000;

	/* Apply I2C configuration after enabling it */
	I2C_Init(I2C1, &I2C_InitStructure);
	/* I2C Peripheral Enable */
	I2C_Cmd(I2C1, ENABLE);
}

void MPU6050_I2C_ByteWrite(u8 slaveAddr, u8 pBuffer, u8 writeAddr)
{
//  ENTR_CRT_SECTION();

	/* Send START condition */
	I2C_GenerateSTART(I2C1, ENABLE);

	/* Test on EV5 and clear it */
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
		;

	/* Send MPU6050 address for write */
	I2C_Send7bitAddress(I2C1, slaveAddr, I2C_Direction_Transmitter);

	/* Test on EV6 and clear it */
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
		;

	/* Send the MPU6050's internal address to write to */
	I2C_SendData(I2C1, writeAddr);

	/* Test on EV8 and clear it */
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		;

	/* Send the byte to be written */
	I2C_SendData(I2C1, pBuffer);

	/* Test on EV8 and clear it */
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		;

	/* Send STOP condition */
	I2C_GenerateSTOP(I2C1, ENABLE);

	// EXT_CRT_SECTION();

}

/**
 * @brief  Reads a block of data from the MPU6050.
 * @param  slaveAddr  : slave address MPU6050_DEFAULT_ADDRESS
 * @param  pBuffer : pointer to the buffer that receives the data read from the MPU6050.
 * @param  readAddr : MPU6050's internal address to read from.
 * @param  NumByteToRead : number of bytes to read from the MPU6050 ( NumByteToRead >1  only for the Mgnetometer readinf).
 * @return None
 */

void MPU6050_I2C_BufferRead(u8 slaveAddr, u8* pBuffer, u8 readAddr, u16 NumByteToRead)
{
	// ENTR_CRT_SECTION();

	/* While the bus is busy */

	while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY))
		;

	/* Send START condition */
	I2C_GenerateSTART(I2C1, ENABLE);

	/* Test on EV5 and clear it */
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
		;

	/* Send MPU6050 address for write */
	I2C_Send7bitAddress(I2C1, slaveAddr, I2C_Direction_Transmitter);

	/* Test on EV6 and clear it */
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
		;

	/* Clear EV6 by setting again the PE bit */
	I2C_Cmd(I2C1, ENABLE);

	/* Send the MPU6050's internal address to write to */
	I2C_SendData(I2C1, readAddr);

	/* Test on EV8 and clear it */
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		;

	/* Send STRAT condition a second time */
	I2C_GenerateSTART(I2C1, ENABLE);

	/* Test on EV5 and clear it */
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
		;

	/* Send MPU6050 address for read */
	I2C_Send7bitAddress(I2C1, slaveAddr, I2C_Direction_Receiver);

	/* Test on EV6 and clear it */
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
		;

	/* While there is data to be read */
	while (NumByteToRead)
	{
		if (NumByteToRead == 1)
		{
			/* Disable Acknowledgement */
			I2C_AcknowledgeConfig(I2C1, DISABLE);

			/* Send STOP Condition */
			I2C_GenerateSTOP(I2C1, ENABLE);
		}

		/* Test on EV7 and clear it */
		if (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED))
		{
			/* Read a byte from the MPU6050 */
			*pBuffer = I2C_ReceiveData(I2C1);

			/* Point to the next location where the byte read will be saved */
			pBuffer++;

			/* Decrement the read bytes counter */
			NumByteToRead--;
		}
	}

	/* Enable Acknowledgement to be ready for another reception */
	I2C_AcknowledgeConfig(I2C1, ENABLE);
//  EXT_CRT_SECTION();

}

void I2C_single_write(I2C_TypeDef* I2Cx, uint8_t HW_address, uint8_t addr, uint8_t data)
{
	MPU6050_I2C_ByteWrite(0xd0, data, addr);
}

uint8_t I2C_single_read(I2C_TypeDef* I2Cx, uint8_t HW_address, uint8_t addr)
{
	u8 data;
	MPU6050_I2C_BufferRead(0xd0, &data, addr, 1);
	return data;
}

void i2cdev_writeBit(u8 devAddr, u8 regAddr, u8 bitNum, u8 data)
{
	u8 b = I2C_single_read(I2C1, devAddr, regAddr);
	b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
	I2C_single_write(I2C1, devAddr, regAddr, b);
}

void i2cdev_writeBits(u8 devAddr, u8 regAddr, u8 bitStart, u8 length, u8 data)
{
	//      010 value to write
	// 76543210 bit numbers
	//    xxx   args: bitStart=4, length=3
	// 00011100 mask byte
	// 10101111 original value (sample)
	// 10100011 original & ~mask
	// 10101011 masked | value
	u8 b = I2C_single_read(I2C1, devAddr, regAddr);
	u8 mask = ((1 << length) - 1) << (bitStart - length + 1);
	data <<= (bitStart - length + 1); // shift data into correct position
	data &= mask;			  // zero all non-important bits in data
	b &= ~(mask);			  // zero all important bits in existing byte
	b |= data;			  // combine data with existing byte

	I2C_single_write(I2C1, devAddr, regAddr, b);
}

void i2cdev_readBit(u8 devAddr, u8 regAddr, u8 bitNum, u8* data)
{
	u8 b = I2C_single_read(I2C1, devAddr, regAddr);
	*data = b & (1 << bitNum);
}

void i2cdev_readBits(u8 devAddr, u8 regAddr, u8 bitStart, u8 length, u8* data)
{
	// 01101001 read byte
	// 76543210 bit numbers
	//    xxx   args: bitStart=4, length=3
	//    010   masked
	//   -> 010 shifted
	u8 b = I2C_single_read(I2C1, devAddr, regAddr);
	u8 mask = ((1 << length) - 1) << (bitStart - length + 1);
	b &= mask;
	b >>= (bitStart - length + 1);
	*data = b;
}

void i2cdev_readBytes(u8 devAddr, u8 regAddr, u8 length, u8* data)
{
	MPU6050_I2C_BufferRead(0xd0, data, regAddr, length);
}

void i2cdev_readByte(u8 devAddr, u8 regAddr, u8* data)
{
	MPU6050_I2C_BufferRead(0xd0, data, regAddr, 1);
}

void i2cdev_writeByte(u8 devAddr, u8 regAddr, u8 data)
{
	I2C_single_write(I2C1, devAddr, regAddr, data);
}

void i2cdev_writeWord(u8 devAddr, u8 regAddr, u16 data)
{
	I2C_single_write(I2C1, devAddr, regAddr, data >> 8);
	I2C_single_write(I2C1, devAddr, regAddr, data);
}

void i2cdev_writeBytes(u8 devAddr, u8 regAddr, u8 length, u8* data)
{
	for (int i = 0; i < length; i++)
	{
		I2C_single_write(I2C1, devAddr, regAddr, data[i]);
	}
}
