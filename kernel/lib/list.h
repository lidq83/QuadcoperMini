/*
 * list.h
 *
 *  Created on: December 23, 2019
 *      Author: lidq
 */
#ifndef __SRC_LIB_LIST_H
#define __SRC_LIB_LIST_H

#include <stdint.h>
#include <string.h>
#include <stdlib.h>

//链表节点结构体
typedef struct list_node_s
{
	//下一个结节
	struct list_node_s *next;
	//关键字
	void *key;
	//值
	void *value;
} list_node_s;

//链表结构体
typedef struct list_s
{
	//链表头节点
	list_node_s *head;
} list_s;

//初始化链表
int list_init(list_s *list);

//追加节点到链表
int list_append(list_s *list, void *key, void *value);

//从链表中移除节点
int list_remove(list_s *list, list_node_s *node);

#endif