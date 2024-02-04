/*
 * fcntl.c
 *
 *  Created on: January 1, 2020
 *      Author: lidq
 */
#include <fcntl.h>
#include <sche.h>
#include <fs.h>

extern const uint8_t map_proi[256];

//找到第一个空闲位置的索引号
static uint32_t fcntl_first_empty(pcb_s *pcb);

//根据索引号分配节点
static int fcntl_alloc(pcb_s *pcb, uint32_t ind);

//根据索引号释放节点
static int fcntl_free(pcb_s *pcb, uint32_t ind);

int open(char *path, int oflag, int mode)
{
	vfs_node_s *node = fs_get_node(path);
	if (node == NULL)
	{
		return -1;
	}

	pcb_s *pcb = sche_curr_pcb();
	if (node->ops.open == NULL)
	{
		return -1;
	}
	node->ops.open(NULL);

	uint32_t ind = fcntl_first_empty(pcb);
	if (ind >= FNODE_CNT)
	{
		return -1;
	}

	//申请节点
	fcntl_alloc(pcb, ind);
	pcb->fnodes[ind] = node;
	return ind;
}

int close(int fd)
{
	if (fd < 0 || fd >= FNODE_CNT)
	{
		return -1;
	}

	pcb_s *pcb = sche_curr_pcb();
	vfs_node_s *node = pcb->fnodes[fd];
	if (node == NULL)
	{
		return -1;
	}

	if (node->ops.close == NULL)
	{
		return -1;
	}

	int ret = node->ops.close(NULL);

	pcb->fnodes[fd] = NULL;
	fcntl_free(pcb, fd);

	return ret;
}

size_t read(int fd, void *buf, size_t count)
{
	if (fd < 0 || fd >= FNODE_CNT)
	{
		return -1;
	}

	pcb_s *pcb = sche_curr_pcb();
	vfs_node_s *node = pcb->fnodes[fd];
	if (node == NULL)
	{
		return -1;
	}

	if (node->ops.read == NULL)
	{
		return -1;
	}

	return node->ops.read(NULL, buf, count);
}

size_t write(int fd, void *buf, size_t count)
{
	if (fd < 0 || fd >= FNODE_CNT)
	{
		return -1;
	}

	pcb_s *pcb = sche_curr_pcb();
	vfs_node_s *node = pcb->fnodes[fd];
	if (node == NULL)
	{
		return -1;
	}

	if (node->ops.write == NULL)
	{
		return -1;
	}

	return node->ops.write(NULL, buf, count);
}

int ioctl(int fd, unsigned int cmd, unsigned long arg)
{
	if (fd < 0 || fd >= FNODE_CNT)
	{
		return -1;
	}

	pcb_s *pcb = sche_curr_pcb();
	vfs_node_s *node = pcb->fnodes[fd];
	if (node == NULL)
	{
		return -1;
	}

	if (node->ops.ioctl == NULL)
	{
		return -1;
	}

	return node->ops.ioctl(NULL, cmd, arg);
}

//找到第一个空闲位置的索引号
uint32_t fcntl_first_empty(pcb_s* pcb)
{
	int size = FNODE_CNT / 8;
	for (int i = 0; i < size; i++)
	{
		if (pcb->f_use_map[i] & 0xff)
		{
			int bit = map_proi[pcb->f_use_map[i] & 0xff];
			return i * 8 + bit;
		}
	}
	return FNODE_CNT - 1;
}

//根据索引号分配节点
int fcntl_alloc(pcb_s* pcb, uint32_t ind)
{
	if (pcb == NULL)
	{
		return -1;
	}

	int index = ind / 8;
	int bit = ind % 8;
	//将使用位图中指定的索引号置为0,表示此位置占用
	pcb->f_use_map[index] &= ~(1 << bit);

	return 0;
}

//根据索引号释放节点
int fcntl_free(pcb_s* pcb, uint32_t ind)
{
	if (pcb == NULL)
	{
		return -1;
	}

	int index = pcb->prio / 8;
	int bit = pcb->prio % 8;
	//将使用位图中指定的索引号置为1,表示此位置空闲
	pcb->f_use_map[index] |= 1 << bit;

	return 0;
}