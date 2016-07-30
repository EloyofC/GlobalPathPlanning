/*
  Description: This file is all about some functions related to the basic parts of pathplanning;
  Author: Green
  Date: 16/06/02
*/

#include <assert.h>
#include <stdio.h>
#include "pretreatment.h"
#include "envoperate.h"
#include "advancedplan.h"
#include "localplanning.h"
#include "heapqueue.h"
#include "publicfun.h"


/* This routine shows whether the member should update the cost to the priority queue. */
static int IsEnvMemberUpdateNewCost(
   int costNew,
   t_EnvironmentMemberPtr member
   ) {
   return IsEnvMemberUnvisited( member ) ||
      ( IsEnvMemberAlive( member ) && ( costNew < GetEnvMemberCost( member )) );
}

/* This routine take down the the previous point of memberSon for backtracking */
static void SayFather(
   t_EnvironmentMemberPtr memberFather,
   t_EnvironmentMemberPtr memberSon
   ) {
   SetEnvMemberPrev( memberFather, memberSon );
}

static void PrintPathPlan(
   t_EnvironmentMemberPtr memberLast
   ) {
   for (t_EnvironmentMemberPtr memberPrev = memberLast;
        memberPrev != NULL; memberPrev = GetEnvMemberPrev( memberPrev ) )
      printf( "%d %d %d\n",
              GetEnvMemberX( memberPrev ),
              GetEnvMemberY( memberPrev ),
              GetEnvMemberCost( memberPrev ) );
}

/* This routine insert the new(updated) member to the PriorityQueue*/
static t_PriorityQueuePtr UpdateMember2Queue(
   int costNew,
   int ( * compute ) ( int, t_EnvironmentMemberPtr, t_EnvironmentPtr ),
   int ( * cmp ) ( t_EnvironmentMemberPtr, t_EnvironmentMemberPtr  ),
   t_EnvironmentMemberPtr memberPrev,
   t_EnvironmentMemberPtr memberInSearch,
   t_EnvironmentPtr environment,
   t_PriorityQueuePtr queue
   ) {
   SetEnvMemberCost( costNew, memberInSearch );
   SetEnvMemberPriority( compute( costNew, memberInSearch, environment ), memberInSearch );
   SayFather( memberPrev, memberInSearch );
   SetEnvMemberAlive( memberInSearch );
   queue = InsertPriorityQueue( cmp, memberInSearch, queue );

   return queue;
}

int ComparePriority(
   t_EnvironmentMemberPtr memberFirst,
   t_EnvironmentMemberPtr memberSecond
   ) {
   return GetEnvMemberPriority( memberFirst ) > GetEnvMemberPriority( memberSecond );
}

/* This routine initial the search with the start point */
static t_EnvironmentMemberPtr GetEnvMemberInitial(
   int xStart,
   int yStart,
   t_EnvironmentPtr environment
   ) {
   t_EnvironmentMemberPtr memberFirst = GetEnvMember( xStart, yStart, environment );
   SetEnvMemberDead( memberFirst );

   return memberFirst;
}

/* This routine get the next valid env member with the min cost, if couldn't find
   return NULL*/
static t_EnvironmentMemberPtr GetEnvMemberNextMin(
   int ( * cmp )( t_PQElementTypePtr, t_PQElementTypePtr  ),
   t_PriorityQueuePtr queue
   ) {
   /* skip the member until the next no dead member */
   t_EnvironmentMemberPtr memberNew = DeleteMinPriorityQueue( cmp, queue );
   while ( ( memberNew != NULL ) && IsEnvMemberDead( memberNew ) ) {
      memberNew = DeleteMinPriorityQueue( cmp, queue );
   }

   /* if no way exist DeleteMin -> memberNew -> return is all NULL */
   if ( memberNew != NULL ) {
      SetEnvMemberDead( memberNew );
   }
   return memberNew;
}

/* This routine calculate the cost of a node, and according to the cost to determine to
   update the member cost or not*/
static t_PriorityQueuePtr SearchANode(
   int ( * compute ) ( int, t_EnvironmentMemberPtr, t_EnvironmentPtr ),
   int ( * cmp ) ( t_EnvironmentMemberPtr, t_EnvironmentMemberPtr  ),
   t_EnvironmentMemberPtr memberPrev,
   t_EnvironmentMemberPtr memberInSearch,
   t_EnvironmentPtr environment,
   t_PriorityQueuePtr queue
   ) {
   assert( memberInSearch != NULL );

   int costNew = c_costNormal;
   costNew += GetEnvMemberCost( memberPrev );

   /* If the cost of the existing QueueMember is smaller than the previous, then update */
   if ( IsEnvMemberUpdateNewCost( costNew, memberInSearch ) ) {
      queue = UpdateMember2Queue( costNew, compute, cmp,
                                  memberPrev, memberInSearch, environment, queue );
   }
   return queue;
}

/* This routine search a node if it is a index should be searched.
   It return the updated queue(if searched) or old queue */
static t_PriorityQueuePtr SearchADirection(
   int xIndex,                  /* the x of the searching point, must be in environment and not searched before */
   int yIndex,
   int ( * compute ) ( int, t_EnvironmentMemberPtr, t_EnvironmentPtr ),
   int ( * cmp ) ( t_EnvironmentMemberPtr, t_EnvironmentMemberPtr  ),
   t_EnvironmentMemberPtr memberPrev,
   t_EnvironmentPtr environment,
   t_PriorityQueuePtr queue
   ) {
   /* If the index is in the environment and
      the corresponding environment member is not dead and
      not a obstacle
      then do the search calculation with the member */
   if ( IsEnvPointInEnv( xIndex, yIndex, environment ) ) {
      t_EnvironmentMemberPtr member = GetEnvMember( xIndex, yIndex, environment );
      if ( !IsEnvMemberDead( member ) &&
           !IsEnvMemberObstacle ( member ) ) {
         queue = SearchANode( compute, cmp, memberPrev,
                              GetEnvMember( xIndex, yIndex, environment ), environment, queue );
      }
   }
   return queue;
}

/* This routine search four neighbour point for shortest path */
static t_PriorityQueuePtr SearchNeighbour(
   int ( * compute ) ( int, t_EnvironmentMemberPtr, t_EnvironmentPtr ),
   int ( * cmp ) ( t_EnvironmentMemberPtr, t_EnvironmentMemberPtr  ),
   t_EnvironmentMemberPtr member,
   t_EnvironmentPtr environment,
   t_PriorityQueuePtr queue
   ) {
   int x[ ] = {
      -1, 1, 0, 0
   };	/* search in four directions */
   int y[ ] = {
      0, 0, -1, 1
   };
   int xCurrent = GetEnvMemberX( member );
   int yCurrent = GetEnvMemberY( member );
   int count = sizeof( x )/sizeof( int );

   for ( int i = 0; i < count; i++ ) {
      queue = SearchADirection( xCurrent + x[ i ], yCurrent + y[ i ],
                                compute, cmp, member, environment, queue );
   }
  
   return queue;
}

/* This routine do the shortest path search between the start point and end point ,
   with the method of function point computer.
   The planning route is stored in the prev of the environment's member.
   You can use backtracking with the end point(if existed).
   It returns a value indicating success of faliure  */
int DoSomePathPlanning(
   int xStart,
   int yStart,
   int xEnd,
   int yEnd,
   int ( * compute )( int, t_EnvironmentMemberPtr, t_EnvironmentPtr ),
   int ( * cmp )( t_PQElementTypePtr, t_PQElementTypePtr  ),
   t_EnvironmentPtr environment
   ) {
   /* confirm the start and end is no obstacle */
   assert( !IsEnvMemberObstacle( GetEnvMember( xStart, yStart, environment )) );
   assert( !IsEnvMemberObstacle( GetEnvMember( xEnd, yEnd, environment )) );

   t_EnvironmentMemberPtr member = GetEnvMemberInitial( xStart, yStart, environment );
   t_PriorityQueuePtr planningQueue = CreatePriorityQueue();
   SetEnvStartAndEnd( xStart, yStart, xEnd, yEnd, environment );

   /* Do the shortest path search until the search is finished or there is no way */
   for ( ; ( member != NULL ) && !IsSearchEnd( member, environment );
         member = GetEnvMemberNextMin( cmp, planningQueue ) ) {

      DebugCodeDetail (
         printf( "DoSomePathPlanning : Searched Member x %d y %d \n",
                 GetEnvMemberX( member ), GetEnvMemberY( member ) );
         fflush( stdout );
         );

      planningQueue = SearchNeighbour( compute, cmp, member, environment, planningQueue );
   }

   FreePriorityQueue( planningQueue );

   /* 0: no way exist may have several reasons ; 1: some way exist */
   return ( member == NULL ) ? 0 : 1;
}

