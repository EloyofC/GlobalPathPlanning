#include <stdio.h>
#include <assert.h>
#include "publicfun.h"
#include "fifoqueue.h"

#define c_queueSizeMin 1

/*
 The First In First Out queue use the array as its basic data structure
It has m_size to imply that how many members it can contain
and it has m_index to imply that how many members it has
and it has m_frontptr to imply the head place
and it has m_tailptr to imply the tail place
This FifoQueue is implemented with the technique of dynamic growing, so it's
space is only limited to the VM
*/

typedef struct t_FifoQueue
{
    int m_size;
    int m_index;
    int m_frontptr;
    int m_tailptr;
    t_FifoElementTypePtr *m_queuePtr;
} *t_FifoQueuePtr;

static t_FifoQueuePtr CreateFifoQueueWithSize(
    int size
    ) {
    t_FifoQueuePtr queue = Malloc( sizeof( struct t_FifoQueue ) );
    queue->m_queuePtr = Malloc( sizeof( t_FifoElementTypePtr ) * size );
    queue->m_size = size;
    queue->m_index = 0;
    queue->m_frontptr = 0;
    queue->m_tailptr = 0;
    return queue;
}

t_FifoQueuePtr CreateFifoQueue(
   void
   ) {
    return CreateFifoQueueWithSize( c_queueSizeMin );
}

static int IsQueueFull(
   t_FifoQueuePtr queue
   ) {
    return queue->m_size == queue->m_index;
}

static int IsQueueEmpty(
   t_FifoQueuePtr queue
   ) {
    return queue->m_index == 0;
}

void FreeFifoQueue(
   t_FifoQueuePtr queue
   ) {
    Free( queue->m_queuePtr );
    Free( queue );
}

t_FifoElementTypePtr DeFifoQueue(
   t_FifoQueuePtr queue
   ) {
    assert( !IsQueueEmpty( queue ) );

    t_FifoElementTypePtr memberOut = ( queue->m_queuePtr )[ queue->m_frontptr ];

    /* delete the member and update the frontptr */
    queue->m_index -= 1;
    queue->m_frontptr += 1;
    if ( queue->m_frontptr == queue->m_size ) {
        queue->m_frontptr = 0;
    }

    return memberOut;
}


/* This routine returns a new queue with the double size of the old one, and contains the same member of the old one */
static t_FifoQueuePtr MakeQueueBigger(
   t_FifoQueuePtr queue
   ) {
    int newSize = ( queue->m_size ) * 2;
    int count = queue->m_index;
    t_FifoElementTypePtr *newQueue = Malloc( sizeof( t_FifoElementTypePtr ) * newSize );

    /* copy the members from the old queue to the big new one */
    for ( int i = 0; i < count; i++ ) {
       newQueue[ i ] = DeFifoQueue( queue );
    }

    Free( queue->m_queuePtr );

    queue->m_queuePtr = newQueue;
    queue->m_frontptr = 0;
    queue->m_index = count;
    queue->m_tailptr = count;
    queue->m_size = newSize;
    return queue;
}

t_FifoQueuePtr EnFifoQueue(
   t_FifoElementTypePtr memberNew,
   t_FifoQueuePtr queue
   ) {
    if ( IsQueueFull( queue ) ) {
       queue = MakeQueueBigger( queue );
    }

    /* insert the member to the queue and update the m_tailptr */
    ( queue->m_queuePtr )[ queue->m_tailptr ] = memberNew;
    queue->m_index += 1;
    queue->m_tailptr += 1;
    if ( queue->m_tailptr == queue->m_size ) {
        queue->m_tailptr = 0;
    }

    return queue;
}
