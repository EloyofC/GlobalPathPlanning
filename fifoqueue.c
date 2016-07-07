#include <stdio.h>
#include <assert.h>
#include "publicfun.h"
#include "fifoqueue.h"

#define c_queueSizeMin 2

static t_FifoQueuePtr MakeQueueBigger(t_FifoQueuePtr queue);
static int IsQueueFull(t_FifoQueuePtr queue);
static int IsQueueEmpty(t_FifoQueuePtr queue);
static t_FifoQueuePtr InitialFifoQueueWithSize(int size);

typedef struct t_FifoQueue
{
  int m_size;
  int m_index;
  int m_frontptr;
  int m_tailptr;
  t_EnvironmentMemberPtr *m_queuePtr;
} *t_FifoQueuePtr;

static t_FifoQueuePtr
InitialFifoQueueWithSize(int size)
{
  t_FifoQueuePtr queue;

  queue = Malloc(sizeof(struct t_FifoQueue));
  queue->m_queuePtr = Malloc(sizeof(t_EnvironmentMemberPtr) * size);
  queue->m_size = c_queueSizeMin;
  queue->m_index = queue->m_frontptr = queue->m_tailptr = 0;
  return queue;
}

t_FifoQueuePtr
InitialFifoQueue(void)
{
  return InitialFifoQueueWithSize(c_queueSizeMin);

}

static int
IsQueueFull(t_FifoQueuePtr queue)
{
  return queue->m_size == queue->m_index;
}

static int
IsQueueEmpty(t_FifoQueuePtr queue)
{
  return queue->m_index == 0;
}

void
FreeFifoQueue(t_FifoQueuePtr queue)
{
  Free(queue->m_queuePtr);
  Free(queue);
}

t_EnvironmentMemberPtr
DeFifoQueue(t_FifoQueuePtr queue)
{
  t_EnvironmentMemberPtr memberOut;

  assert(!IsQueueEmpty(queue));
  memberOut = (queue->m_queuePtr)[queue->m_frontptr];
  queue->m_index -= 1;
  queue->m_frontptr += 1;
  if (queue->m_frontptr == queue->m_size) {
    queue->m_frontptr = 0;
  }
  return memberOut;
}

t_FifoQueuePtr
EnFifoQueue(t_EnvironmentMemberPtr memberNew, t_FifoQueuePtr queue)
{
  if (IsQueueFull(queue)) {
    queue = MakeQueueBigger(queue);
  }

  (queue->m_queuePtr)[queue->m_tailptr] = memberNew;
  queue->m_index += 1;
  queue->m_tailptr += 1;
  if (queue->m_tailptr == queue->m_size) {
    queue->m_tailptr = 0;
  }
  return queue;
}

static t_FifoQueuePtr
MakeQueueBigger(t_FifoQueuePtr queue)
{
  t_EnvironmentMemberPtr *newQueue;
  int newSize = (queue->m_size) * 2;
  int newPtr;


  newQueue = Malloc(sizeof(t_EnvironmentMemberPtr) * newSize);
  for (newPtr = 0; newPtr < queue->m_size; newPtr++) {
    newQueue[newPtr] = (queue->m_queuePtr)[newPtr];
  }

  Free(queue->m_queuePtr);
  queue->m_queuePtr = newQueue;
  queue->m_frontptr = 0;
  queue->m_tailptr = queue->m_index;
  queue->m_size = newSize;
  return queue;
}
