/*
 * crc.h
 *
 *  Created on: Aug 2, 2019
 *      Author: lidq
 */

#ifndef SRC_INCLUDE_CRC_H_
#define SRC_INCLUDE_CRC_H_

#include <stdint.h>

#define CRC16_SEED				(0xC72A)

uint16_t crc16value(uint16_t i);

uint16_t calc16crc(uint8_t *p_buff, uint16_t len);

#endif /* SRC_INCLUDE_CRC_H_ */
