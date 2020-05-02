/*
 * crc.c
 *
 *  Created on: Aug 2, 2019
 *      Author: lidq
 */

#include <crc.h>

uint16_t crc16value(uint16_t i)
{
	uint16_t j;
	uint16_t crc;
	crc = i;
	for (j = 8; j > 0; j--)
	{
		if (crc & 1)
		{
			crc = (crc >> 1) ^ CRC16_SEED;
		}
		else
		{
			crc >>= 1;
		}
	}
	return crc;
}

uint16_t calc16crc(uint8_t *p_buff, uint16_t len)
{
	if (len <= 0)
	{
		return 0;
	}
	uint16_t temp1;
	uint16_t temp2;
	uint16_t crc = 0;
	while (len-- != 0)
	{
		temp1 = (crc >> 8) & 0xffff;
		temp2 = crc16value(((uint16_t) crc ^ *p_buff++) & 0xff);
		crc = temp1 ^ temp2;
	}
	return crc;
}
