#ifndef __DRIVERS_FLASH_H
#define __DRIVERS_FLASH_H

#include <board.h>
#include <stdint.h>

#define FLASH_ADDR_START (0x8000000)
#define FLASH_SIZE (0x10000) // 64k
#define FLASH_USER_SIZE (0x1000)
#define FLASH_USER_DEF (FLASH_ADDR_START + FLASH_SIZE - FLASH_USER_SIZE)

void flash_init(void);

void flash_erase(uint32_t addr);

void flash_write(uint32_t addr, uint32_t* data, int size);

void flash_read(uint32_t addr, uint32_t* data, int size);

#endif
