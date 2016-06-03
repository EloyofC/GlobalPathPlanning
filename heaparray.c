/*
  Description: This file is all about declarition and functions 4 Heap;
  Author: Green
  Date: 16/06/02
 */
#include "HeapQueue.h"
#include "PublicFun.h"

#define c_priorityQueueSize 800

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

t_PriorityQueuePtr
InitializePriorityQueue()
{
  return InitializeWithSize(c_priorityQueueSize);
}

t_PriorityQueuePtr
InitializeWithSize(int elementSize)
{
  t_PriorityQueuePtr queue;

  queue = Malloc(sizeof(struct t_HeapStruct));
  queue->m_elementsPtr = Malloc((elementSize + 1) * sizeof(t_ElementTypePtr));

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

  if (IsQueueFull(queue)) {
    queue = MakeQueueBigger(queue);
  }

  if (IsQueueEmpty(queue)) {
    queue->m_index++;
    queue->m_elementsPtr[0] = X;
    return queue;
  }

  for (i = queue->m_index++; cmp(queue->m_elementsPtr[GetFather(i)], X) && i > 0; i = GetFather(i)) {
    queue->m_elementsPtr[i] = queue->m_elementsPtr[GetFather(i)];
  }
  queue->m_elementsPtr[i] = X;
  return queue;
}

static t_PriorityQueuePtr
MakeQueueBigger(t_PriorityQueuePtr queue)
{
  t_PriorityQueuePtr priorityQueueNew;
  int i;

  priorityQueueNew = InitializeWithSize(queue->m_size * 2);
  for (i = 0; i < queue->m_index; i++) {
    priorityQueueNew->m_elementsPtr[i] = queue->m_elementsPtr[i];
  }

  free(queue);
  return  priorityQueueNew;
}

t_ElementTypePtr
DeleteMinPriorityQueue(int (* cmp)(t_ElementTypePtr, t_ElementTypePtr), t_PriorityQueuePtr queue)
{
  int i, child;
  t_ElementTypePtr elementPrev, elementMin;

  if (IsQueueEmpty(queue)) {
    fprintf(stderr, "Priority queue is empty");
    exit(1);
  }
  elementMin = queue->m_elementsPtr[0];
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

  return elementMin;
}

void
DestroyPriorityQueue(t_PriorityQueuePtr queue)
{
  t_ElementTypePtr *elementsp;

  elementsp = queue->m_elementsPtr;
  free(elementsp);
  free(queue);
}

static int
GetLeftChild(int fatherIndex)
{
  return 2 * fatherIndex + 1;
}

static int
GetFather(int childIndex)
{
  return (childIndex - 1) / 2;
}
