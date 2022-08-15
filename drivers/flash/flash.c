#include <board.h>
#include <flash.h>
#include <stm32f1xx_hal_def.h>

void flash_init(void)
{
}

void flash_erase(uint32_t addr)
{
	FLASH_EraseInitTypeDef f;
	f.TypeErase = FLASH_TYPEERASE_PAGES;
	f.PageAddress = addr;
	f.NbPages = 1;
	uint32_t PageError = 0;
	
	HAL_FLASH_Unlock();
	HAL_FLASHEx_Erase(&f, &PageError);
	HAL_FLASH_Lock();
}
void flash_write(uint32_t addr, uint32_t* data, int size)
{
	HAL_FLASH_Unlock();
	for (int i = 0; i < size; i++)
	{
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, data[i]);
		addr += sizeof(uint32_t);
	}
	HAL_FLASH_Lock();
}

void flash_read(uint32_t addr, uint32_t* data, int size)
{
	for (int i = 0; i < size; i++)
	{
		data[i] = *(uint32_t*)addr;
		addr += sizeof(uint32_t);
	}
}