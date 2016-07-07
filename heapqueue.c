/*
  Description: This file is all about declarition and functions 4 Heap;
  Author: Green
  Date: 16/06/02
 */

#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include "heapqueue.h"
#include "publicfun.h"
#include "pretreatment.h"


#define c_priorityQueueSize 3


struct t_HeapStruct
{
  int m_size;
  int m_index;
  t_ElementTypePtr *m_elementsPtr;
};

static t_PriorityQueuePtr InitializeWithSize(int elementSize);
static int IsQueueEmpty(t_PriorityQueuePtr queue);
static int IsQueueFull(t_PriorityQueuePtr queue);
static t_PriorityQueuePtr MakeQueueBigger(t_PriorityQueuePtr queue);
static int GetFather(int childIndex);
static int GetLeftChild(int fatherIndex);

/* count the times that call the PriorityQueue */
static int sg_priorityQueueCount = 0;

t_PriorityQueuePtr
InitializePriorityQueue(void)
{
  sg_priorityQueueCount++;

  return InitializeWithSize(c_priorityQueueSize);
}

t_PriorityQueuePtr
InitializeWithSize(int elementSize)
{
  t_PriorityQueuePtr queue;

  queue = (t_PriorityQueuePtr)Malloc(sizeof(struct t_HeapStruct));
  queue->m_elementsPtr = (t_ElementTypePtr *)Malloc((elementSize + 1) * sizeof(t_ElementTypePtr)); /* the thing that store inside is the ptr to the environment */

  queue->m_size = elementSize;
  queue->m_index = 0;

  return queue;
}

static int
IsQueueEmpty(t_PriorityQueuePtr queue)
{
  return queue->m_index == 0;
}

static int
IsQueueFull(t_PriorityQueuePtr queue)
{
  return queue->m_index == queue->m_size;
}

t_PriorityQueuePtr
InsertPriorityQueue(int (* cmp)(t_ElementTypePtr, t_ElementTypePtr), t_ElementTypePtr X, t_PriorityQueuePtr queue)
{
  int i;

  assert(X != NULL);
  if (IsQueueFull(queue)) {
    queue = MakeQueueBigger(queue);
  }

  if (IsQueueEmpty(queue)) {
    queue->m_index++;
    queue->m_elementsPtr[0] = X;
    return queue;
  }

  for (i = queue->m_index++; i > 0 && cmp(queue->m_elementsPtr[GetFather(i)], X); i = GetFather(i)) {
    queue->m_elementsPtr[i] = queue->m_elementsPtr[GetFather(i)];
  }
  queue->m_elementsPtr[i] = X;

  DebugCodeDetail (
		   printf("InsertPriorityQueue : The %d times PriorityQueue of x %d y %d priority %d cost %d\n", sg_priorityQueueCount, GetEnvMemberX(X), GetEnvMemberY(X), GetEnvMemberPriority(X), GetEnvMemberCost(X));
		   fflush(stdout);
		   );
  return queue;
}

static t_PriorityQueuePtr
MakeQueueBigger(t_PriorityQueuePtr queue)
{
  t_PriorityQueuePtr priorityQueueNew;
  int i;

  priorityQueueNew = InitializeWithSize(queue->m_size * 2);
  priorityQueueNew->m_index = queue->m_index;
  for (i = 0; i < queue->m_index; i++) {
    priorityQueueNew->m_elementsPtr[i] = queue->m_elementsPtr[i];
  }

  t_ElementTypePtr *elementsp;
  elementsp = queue->m_elementsPtr;
  Free(elementsp);
  Free(queue);
  return  priorityQueueNew;
}

t_ElementTypePtr
DeleteMinPriorityQueue(int (* cmp)(t_ElementTypePtr, t_ElementTypePtr), t_PriorityQueuePtr queue)
{
  int i, child;
  t_ElementTypePtr elementPrev, elementMin;

  if (IsQueueEmpty(queue)) {
    DebugCode (
	       printf("DeleteMinPriorityQueue : Queue is empty\n");
	       fflush(stdout);
	       );
    return NULL;		/* return the responsibility of handling no way existence to the upper */
  }
  elementMin = queue->m_elementsPtr[0];
  assert(elementMin != NULL);	/* the elementMin can not be NULL */
  queue->m_index--;
  elementPrev = queue->m_elementsPtr[queue->m_index];

  for (i = 0; GetLeftChild(i) < queue->m_index; i = child) {
    child = GetLeftChild(i);
    if (child != queue->m_index &&
         (cmp (queue->m_elementsPtr[child],
               queue->m_elementsPtr[child + 1])))
      child++;

    if (cmp (elementPrev, queue->m_elementsPtr[child]))
      queue->m_elementsPtr[i] = queue->m_elementsPtr[child];
    else
      break;
  }
  queue->m_elementsPtr[i] = elementPrev;

  DebugCodeDetail (
		   printf("DeleteMinPriorityQueue : The %d times PriorityQueue of Min x %d y %d priority %d cost %d\n", sg_priorityQueueCount, GetEnvMemberX(elementMin), GetEnvMemberY(elementMin), GetEnvMemberPriority(elementMin), GetEnvMemberCost(elementMin));
		   fflush(stdout);
		   );

  return elementMin;
}

void
DestroyPriorityQueue(t_PriorityQueuePtr queue)
{
  t_ElementTypePtr *elementsp;

  elementsp = queue->m_elementsPtr;
  Free(elementsp);
  Free(queue);
}

static int
GetLeftChild(int fatherIndex)
{
  assert(fatherIndex >= 0);
  return 2 * fatherIndex + 1;
}

static int
GetFather(int childIndex)
{
  assert(childIndex >= 1);
  return (childIndex - 1) / 2;
}
