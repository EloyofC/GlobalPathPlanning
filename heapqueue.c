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
#include "envoperate.h"


#define c_priorityQueueSize 3

/* The Priority Queue implements the Binary Heap */

struct t_HeapStruct
{
   int m_size;
   int m_index;
   t_PQElementTypePtr *m_elementsPtr;
};

static int sg_priorityQueueCount = 0; /* count the times that call the PriorityQueue */


static t_PriorityQueuePtr CreatePriorityQueueWithSize(
   int elementSize
   ) {
   t_PriorityQueuePtr queue = ( t_PriorityQueuePtr )Malloc( sizeof( struct t_HeapStruct ) );
   queue->m_elementsPtr = ( t_PQElementTypePtr * )Malloc( elementSize * sizeof( t_PQElementTypePtr ));
   queue->m_size = elementSize;
   queue->m_index = 0;

   return queue;
}

t_PriorityQueuePtr CreatePriorityQueue(
   void
   ) {
   sg_priorityQueueCount++;

   return CreatePriorityQueueWithSize( c_priorityQueueSize );
}


static int IsQueueEmpty(
   t_PriorityQueuePtr queue
   ) {
   return queue->m_index == 0;
}

static int IsQueueFull(
   t_PriorityQueuePtr queue
   ) {
   return queue->m_index == queue->m_size;
}

/* This routine returns a new priority queue with the size of the twice of the old one, and
 contines the same element of the old one */
static t_PriorityQueuePtr MakeQueueBigger(
   t_PriorityQueuePtr queue
   ) {
   t_PriorityQueuePtr priorityQueueNew = CreatePriorityQueueWithSize( queue->m_size * 2 );
   DebugCodeDetail(
      printf( "MakeQueueBigger\n");
      fflush( stdout );
      )
   priorityQueueNew->m_index = queue->m_index;
   for ( int i = 0; i < queue->m_index; i++ ) {
      priorityQueueNew->m_elementsPtr[ i ] = queue->m_elementsPtr[ i ];
   }

   t_PQElementTypePtr *elementsp = queue->m_elementsPtr;
   Free( elementsp );
   Free( queue );
   return  priorityQueueNew;
}

static int GetLeftChild(
   int fatherIndex
   ) {
   assert( fatherIndex >= 0 );

   return 2 * fatherIndex + 1;
}

static int GetFather(
   int childIndex
   ) {
   assert( childIndex >= 1 );

   return ( childIndex - 1 ) / 2;
}

t_PriorityQueuePtr InsertPriorityQueue(
   int ( * cmp )( t_PQElementTypePtr, t_PQElementTypePtr ),
   t_PQElementTypePtr X,        /* Inserted Member can not be NULL */
   t_PriorityQueuePtr queue
   ) {
   assert( X != NULL );

   if ( IsQueueEmpty( queue ) ) {
      queue->m_index++;
      queue->m_elementsPtr[ 0 ] = X;
      return queue;
   } else {
      if ( IsQueueFull( queue ) ) {
         queue = MakeQueueBigger( queue );
      }

      /* using siftdown to find the appropriate place for insert member and keep the heap order  */
      int heapPlace;
      for ( heapPlace = queue->m_index++;
            heapPlace > 0 && cmp(queue->m_elementsPtr[ GetFather( heapPlace ) ], X );
            heapPlace = GetFather( heapPlace ) ) {
         queue->m_elementsPtr[ heapPlace ] = queue->m_elementsPtr[ GetFather( heapPlace ) ];
      }
      queue->m_elementsPtr[ heapPlace ] = X;

      DebugCodeDetail (
         printf( "InsertPriorityQueue : The %d times PriorityQueue of x %d y %d priority %d cost %d\n",
                 sg_priorityQueueCount, GetEnvMemberX( X ), GetEnvMemberY( X ), GetEnvMemberPriority( X ), GetEnvMemberCost( X ) );
         fflush( stdout );
         );
   }
   return queue;
}


t_PQElementTypePtr DeleteMinPriorityQueue(
   int ( * cmp )( t_PQElementTypePtr, t_PQElementTypePtr ),
   t_PriorityQueuePtr queue
   ) {
   if ( IsQueueEmpty( queue ) ) {
      DebugCode (
         printf( "DeleteMinPriorityQueue : Queue is empty\n" );
         fflush( stdout );
         );
      return NULL;
   } else {
      t_PQElementTypePtr elementMin = queue->m_elementsPtr[ 0 ];
      assert( elementMin != NULL );	/* the elementMin can not be NULL */
      queue->m_index--;
      t_PQElementTypePtr elementPrev = queue->m_elementsPtr[ queue->m_index ];

      /* take the lastelement to the head and using siftup to find the appropriate place for the last element */
      int child;
      int heapPlace;
      for ( heapPlace = 0; GetLeftChild( heapPlace ) < queue->m_index; heapPlace = child ) {
         child = GetLeftChild( heapPlace );
         if ( child != queue->m_index &&
              ( cmp ( queue->m_elementsPtr[ child ],
                      queue->m_elementsPtr[ child + 1 ] ) ) ) {
            child++;
         }
         if ( cmp ( elementPrev, queue->m_elementsPtr[ child ] ) ) {
            queue->m_elementsPtr[ heapPlace ] = queue->m_elementsPtr[ child ];
         } else {
            break;
         }
      }
      queue->m_elementsPtr[ heapPlace ] = elementPrev;

      DebugCodeDetail (
         printf( "DeleteMinPriorityQueue : The %d times PriorityQueue of Min x %d y %d priority %d cost %d\n",
                 sg_priorityQueueCount, GetEnvMemberX( elementMin ), GetEnvMemberY( elementMin ), GetEnvMemberPriority( elementMin ), GetEnvMemberCost( elementMin ) );
         fflush( stdout );
         );

      return elementMin;
   }
}

void FreePriorityQueue(
   t_PriorityQueuePtr queue
   ) {
   t_PQElementTypePtr *elementsp = queue->m_elementsPtr;
   Free( elementsp );
   Free( queue );
}

