/*
  Descripition: This file is the interferce of the First in and First out queue
  Author: green
  Date: 2016/07/06
*/

#ifndef _Fifo_Queue_H
#define _Fifo_Queue_H

struct t_FifoQueue;
typedef struct t_FifoQueue *t_FifoQueuePtr;
struct t_EnvMember;
typedef struct t_EnvMember *t_EnvironmentMemberPtr;

t_FifoQueuePtr EnFifoQueue(t_EnvironmentMemberPtr memberNew, t_FifoQueuePtr queue);
t_EnvironmentMemberPtr DeFifoQueue(t_FifoQueuePtr queue);
void FreeFifoQueue(t_FifoQueuePtr queue);
t_FifoQueuePtr InitialFifoQueue(void);

#endif
