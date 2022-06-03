/*
 * protocol.h
 *
 *  Created on: Aug 2, 2019
 *      Author: lidq
 */

#ifndef SRC_INCLUDE_PROTOCOL_H_
#define SRC_INCLUDE_PROTOCOL_H_

#include <stdint.h>
#include <string.h>

#define BUFF_SIZE (256)

#define P_HEAD_0				0x55
#define P_HEAD_1				0xAA
#define P_SIZE_HEAD				2
#define P_SIZE_DATA				8
#define P_SIZE_CRC				2

typedef struct buff_s
{
	char buff[BUFF_SIZE];
	int head;
	int foot;
} buff_s;

void protocol_init(void);

void protocol_append(char *buff, int size);

int protocol_size(void);

int protocol_parse(uint16_t *ctl);

#endif /* SRC_INCLUDE_PROTOCOL_H_ */
