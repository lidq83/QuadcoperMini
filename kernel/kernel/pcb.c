/*
 * pcb.c
 *
 *  Created on: December 20, 2019
 *      Author: lidq
 */

#include <stdlib.h>

#include <pcb.h>
#include <prio_map.h>
#include <sche.h>
//进程就绪队列位图，bit位中为1表示已就绪，0表示未就绪 PROCESS_CNT / sizeof(uint8_t)
uint8_t pcb_ready_map[PROCESS_CNT / 8] = { 0 };
//内核中静态pcb
pcb_s* pcbs = NULL;

//关中断
extern void sche_interrupt_disable(void);
//开中断
extern void sche_interrupt_enable(void);

//当前正在运行的pcb
extern pcb_s* pcb_current;
//下一个需要运行的pcb
extern pcb_s* pcb_next;
//下一个需要运行的pcb
extern pcb_s* pcb_need_kill;

//初始化进程栈
extern void* stack_init(uint32_t* stack, void* runner);

// pcb运行函数
static void pcb_runner(void);

//清除已经结束的进程资源
static void pcb_clear_stoped(void);

//进程pcb初始化
void pcb_init(void)
{
	pcbs = malloc(sizeof(pcb_s) * PROCESS_CNT);
	memset(pcbs, 0 , sizeof(pcb_s) * PROCESS_CNT);
	memset(pcb_ready_map, 0, sizeof(pcb_ready_map));
}

//创建清理pcb资源进程
void pcb_clear_process(void)
{
	// pcb资源清理进行
	pcb_create(PCB_CLEANER_PRIO, &pcb_clear_stoped, NULL, PCB_CLEANER_SIZE);
}

//创建一个进程
pcb_s* pcb_create(uint8_t prio, void* p_entry, void* p_arg, uint32_t stack_size)
{
	uint8_t* stack = malloc(stack_size);
	if (stack == NULL)
	{
		return NULL;
	}
	//设置栈内存默认值以便统计
	memset(stack, 0xFF, stack_size);
	//初始化pcb状态
	pcbs[prio].status = PCB_ST_INIT;
	//初始化栈
	pcbs[prio].p_stack = stack_init((uint32_t*)&stack[stack_size], pcb_runner);
	//栈内存地址
	pcbs[prio].p_stack_mem = stack;
	//栈内存大小
	pcbs[prio].stack_size = stack_size;
	//优先级
	pcbs[prio].prio = prio;
	//休眠tick数
	pcbs[prio].sleep_tick = 0;
	//进程入口函数
	pcbs[prio].task_entry = p_entry;
	//入口函数参数
	pcbs[prio].task_arg = p_arg;
	//文件使用位图
	memset(pcbs[prio].f_use_map, 0xff, FNODE_CNT / 8);
	pcbs[prio].f_use_map[0] = ~0x7;
	//初始化文件描述符
	pcbs[prio].fnodes = malloc(sizeof(void*) * FNODE_CNT);
	if (pcbs[prio].fnodes == NULL)
	{
		return NULL;
	}
	memset(pcbs[prio].fnodes, 0, sizeof(void*) * FNODE_CNT);

	//获取标准IO的节点指针
	vfs_node_s* node_stdin = vfs_find_node("/dev/stdin");
	vfs_node_s* node_stdout = vfs_find_node("/dev/stdout");
	vfs_node_s* node_stderr = vfs_find_node("/dev/stderr");
	//设置标准IO的设备节点
	pcbs[prio].fnodes[0] = node_stdin;
	pcbs[prio].fnodes[1] = node_stdout;
	pcbs[prio].fnodes[2] = node_stderr;
	//将空闲进程放入就绪队列
	pcb_ready(&pcbs[prio]);
	//返回pcb地址
	return &pcbs[prio];
}

void pcb_runner(void)
{
	//执行进程主函数
	pcb_current->task_entry(pcb_current->task_arg);
	//关中断
	sche_interrupt_disable();
	//挂起当前进程
	pcb_block(pcb_current);
	//当前进程运行结束
	pcb_current->status = PCB_ST_STOPED;
	//取得下一个需要运行的pcb
	pcb_next = pcb_get_highest_pcb();
	//设置清理资源的pcb指针
	sche_switch();
	//开中断
	sche_interrupt_enable();
}

//将进程加入就绪队列
void pcb_ready(pcb_s* pcb)
{
	int index = pcb->prio / 8;
	int bit = pcb->prio % 8;

	pcb_ready_map[index] |= 1 << bit;
	pcb->status = PCB_ST_READ;
}

//将进程由就绪队列挂起
void pcb_block(pcb_s* pcb)
{
	int index = pcb->prio / 8;
	int bit = pcb->prio % 8;

	pcb_ready_map[index] &= ~(1 << bit);
	pcb->status = PCB_ST_BLOCK;
}

//将进程结束
void pcb_kill(pcb_s* pcb)
{
	//关中断
	sche_interrupt_disable();
	//挂起进程
	pcb_block(pcb);
	//进程结束
	pcb->status = PCB_ST_STOPED;
	//开中断
	sche_interrupt_enable();
}

//获取优先级最高的进程索引
uint32_t pcb_get_highest_prio(void)
{
	int size = PROCESS_CNT / 8;
	for (int i = 0; i < size; i++)
	{
		if (pcb_ready_map[i] & 0xff)
		{
			int prio = map_proi[pcb_ready_map[i] & 0xff];
			return i * 8 + prio;
		}
	}
	return PROCESS_CNT - 1;
}

//获取优先级最高的进程
pcb_s* pcb_get_highest_pcb(void)
{
	return &pcbs[pcb_get_highest_prio()];
}

void pcb_clear_stoped(void)
{
	while (1)
	{
		sleep_ticks(1);
		for (uint8_t i = 0; i < PROCESS_CNT - 2; i++)
		{
			if (pcbs[i].status == PCB_ST_STOPED)
			{
				free(pcbs[i].p_stack_mem);
				memset(&pcbs[i], 0, sizeof(pcb_s));
			}
		}
	}
}
