/*
  Description: This file is for the use of the queue
  Author: Green
  Date: 16/6/2
*/

#ifndef _Heap_Queue_H
#define _Heap_Queue_H

struct t_HeapStruct;
typedef struct t_HeapStruct *t_PriorityQueuePtr;
struct t_EnvMember;
typedef struct t_EnvMember *t_EnvironmentMemberPtr;
typedef t_EnvironmentMemberPtr t_ElementTypePtr; /* store the ptr to the environment inside the queue */

t_PriorityQueuePtr InitializePriorityQueue(void);
void DestroyPriorityQueue(t_PriorityQueuePtr queue);
t_PriorityQueuePtr InsertPriorityQueue(int (* cmp)(t_ElementTypePtr, t_ElementTypePtr), t_ElementTypePtr X, t_PriorityQueuePtr queue);
t_ElementTypePtr DeleteMinPriorityQueue(int (* cmp)(t_ElementTypePtr, t_ElementTypePtr), t_PriorityQueuePtr queue);
#endif
