#ifndef __DIGITAL_TUBE_H
#define __DIGITAL_TUBE_H

#include <typedef.h>

#define DT_NUM_0	0x3F
#define DT_NUM_1	0x06
#define DT_NUM_2	0x5B
#define DT_NUM_3	0x4F
#define DT_NUM_4	0x66
#define DT_NUM_5	0x6D
#define DT_NUM_6	0x7D
#define DT_NUM_7	0x07
#define DT_NUM_8	0x7F
#define DT_NUM_9	0x6F

typedef struct dt_num_s
{
	GPIO_TypeDef *GPIOx;
	uint16_t GPIO_Pin;
} dt_num_s;

typedef struct dt_s
{
} dt_s;

void dt_init();

void dt_show_num(uint8_t n, int dig, int dot);

void dt_hide_num(int dig);

#endif
