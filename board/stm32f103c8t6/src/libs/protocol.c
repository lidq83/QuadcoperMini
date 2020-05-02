/*
 * protocol.c
 *
 *  Created on: Aug 2, 2019
 *      Author: lidq
 */

#include <protocol.h>
#include <crc.h>

static buff_s bs = { 0 };

void protocol_init(void)
{
	memset(&bs, 0, sizeof(buff_s));
}

void protocol_append(char *buff, int size)
{
	for (int i = 0; i < size; i++)
	{
		bs.buff[bs.head] = buff[i];
		bs.head++;
		bs.head %= BUFF_SIZE;
		if (bs.head == bs.foot)
		{
			bs.foot++;
			bs.foot %= BUFF_SIZE;
		}
	}
}

int protocol_size(void)
{
	int size = bs.head - bs.foot;
	if (size < 0)
	{
		size += BUFF_SIZE;
	}
	return size;
}

int protocol_parse(uint16_t *ctl)
{
	int cnt = protocol_size();
	if (cnt < 12)
	{
		return -1;
	}

	char buff[12] = { 0 };
	uint16_t crc = 0;
	int ind = 0;
	int step = 0;
	int foot = bs.foot;
	for (int i = 0; i < cnt; i++)
	{
		char ch = bs.buff[foot];
		foot++;
		foot %= BUFF_SIZE;

		switch (step)
		{
			case 0: //HEAD_0
			{
				ind = 0;
				step = 0;
				if (ch == P_HEAD_0)
				{
					buff[ind] = ch;
					ind++;
					step = 1;
				}
				break;
			}
			case 1: //HEAD_1
			{
				step = 0;
				if (ch == P_HEAD_1)
				{
					buff[ind] = ch;
					ind++;
					step = 2;
				}
				break;
			}
			case 2: //DATA
			{
				step = 2;
				buff[ind] = ch;
				ind++;
				if (ind >= P_SIZE_HEAD + P_SIZE_DATA)
				{
					step = 3;
				}
				break;
			}
			case 3: //CRC_0
			{
				crc = (ch << 0);
				step = 4;
				break;
			}
			case 4: //CRC_1
			{
				crc |= (ch << 8);
				step = 5;
				break;
			}
			default:
			{
				break;
			}
		}
		if (step >= 5)
		{
			break;
		}
	}

	if (step < 5)
	{
		return -2;
	}

	bs.foot = foot;

	uint16_t crc_calc = calc16crc((uint8_t *) buff, 10);
	if (crc != crc_calc || crc == 0 || crc_calc == 0)
	{
		return -3;
	}

	ctl[0] = buff[2];
	ctl[0] |= (buff[3] << 8) & 0xff00;

	ctl[1] = buff[4];
	ctl[1] |= (buff[5] << 8) & 0xff00;

	ctl[2] = buff[6];
	ctl[2] |= (buff[7] << 8) & 0xff00;

	ctl[3] = buff[8];
	ctl[3] |= (buff[9] << 8) & 0xff00;

	return 0;
}
