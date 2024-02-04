/*
 * kernel.c
 *
 *  Created on: December 20, 2019
 *      Author: lidq
 */

#include <core.h>
#include <pcb.h>
#include <sche.h>
#include <vfs.h>
#include <stdlib.h>

extern pcb_s *pcb_next;

extern void sche_switch_first(void);

//空闲进程
void process_idle(void)
{
	static uint32_t idle_ind = 0;
	while (1)
	{
		idle_ind++;
	}
}

//启动内核程序
void kernel_startup(void)
{
	vfs_init();
	pcb_init();
	sche_init();
	//创建空闲进程，优先级为最低
	pcb_s *pcb_idle = pcb_create(PCB_IDLE_PRIO, &process_idle, NULL, PCB_IDLE_SIZE);
	pcb_next = pcb_idle;
	pcb_clear_process();
}
