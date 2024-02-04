/*
 * list.c
 *
 *  Created on: December 23, 2019
 *      Author: lidq
 */
#include "list.h"

#include <stdlib.h>
//初始化链表
int list_init(list_s* list)
{
	if (list == NULL)
	{
		return -1;
	}
	// memset(list, 0, sizeof(list_s));
	list->head = NULL;
	return 0;
}

//追加节点到链表
int list_append(list_s* list, void* key, void* value)
{
	if (list == NULL)
	{
		return -1;
	}

	//设置新节点的值
	list_node_s* node_new = malloc(sizeof(list_node_s));
	if (node_new == NULL)
	{
		return -1;
	}
	//关键字
	node_new->key = key;
	//值
	node_new->value = value;
	//下一个节点
	node_new->next = NULL;

	//二级指针
	list_node_s** p = &list->head;
	//找到最后一个节点
	while ((*p) != NULL)
	{
		p = &(*p)->next;
	}
	//插入新节点
	*p = node_new;

	return 0;
}

//从链表中移除节点
int list_remove(list_s* list, list_node_s* node)
{
	if (list == NULL)
	{
		return -1;
	}
	if (node == NULL)
	{
		return -1;
	}

	list_node_s** p = &list->head;
	while ((*p) != NULL && (*p) != node)
	{
		p = &(*p)->next;
	}

	if (*p == NULL || (*p) != node)
	{
		return -1;
	}

	//等待释放节点
	list_node_s* del = *p;
	*p = (*p)->next;
	//释放内存
	free(del);

	return 0;
}
