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
typedef t_EnvironmentMemberPtr t_PQElementTypePtr; /* store the ptr to the environment inside the queue */

t_PriorityQueuePtr CreatePriorityQueue( void );
void FreePriorityQueue( t_PriorityQueuePtr queue );
t_PriorityQueuePtr InsertPriorityQueue( int ( * cmp )( t_PQElementTypePtr, t_PQElementTypePtr ), t_PQElementTypePtr X, t_PriorityQueuePtr queue );
t_PQElementTypePtr DeleteMinPriorityQueue( int ( * cmp )( t_PQElementTypePtr, t_PQElementTypePtr ), t_PriorityQueuePtr queue );
#endif
