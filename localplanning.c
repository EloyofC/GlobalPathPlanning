/*
  Description: This file is all about some functions related to the basic parts of pathplanning;
  Author: Green
  Date: 16/06/02
 */

#include <assert.h>
#include <stdio.h>
#include "pretreatment.h"
#include "advancedplan.h"
#include "localplanning.h"
#include "heapqueue.h"
#include "publicfun.h"


static t_EnvironmentMemberPtr GetEnvMemberInitial(int xStart, int yStart, t_EnvironmentPtr environment);
static t_EnvironmentMemberPtr GetEnvMemberNextMin(int (* cmp)(t_ElementTypePtr, t_ElementTypePtr), t_PriorityQueuePtr queue);
static t_PriorityQueuePtr SearchNeighbour(int (* compute) (int, t_EnvironmentMemberPtr, t_EnvironmentPtr), int (* cmp) (t_EnvironmentMemberPtr, t_EnvironmentMemberPtr), t_EnvironmentMemberPtr member, t_EnvironmentPtr environment, t_PriorityQueuePtr queue);
static t_PriorityQueuePtr SearchADirection(int xIndex, int yIndex, int (* compute) (int, t_EnvironmentMemberPtr, t_EnvironmentPtr), int (* cmp) (t_EnvironmentMemberPtr, t_EnvironmentMemberPtr), t_EnvironmentMemberPtr memberPrev, t_EnvironmentPtr environment, t_PriorityQueuePtr queue);
static t_PriorityQueuePtr SearchANode(int (* compute) (int, t_EnvironmentMemberPtr, t_EnvironmentPtr), int (* cmp) (t_EnvironmentMemberPtr, t_EnvironmentMemberPtr), t_EnvironmentMemberPtr memberPrev, t_EnvironmentMemberPtr memberInSearch, t_EnvironmentPtr environment, t_PriorityQueuePtr queue);
static t_PriorityQueuePtr UpdateMember2Queue(int costNew, int (* compute) (int, t_EnvironmentMemberPtr, t_EnvironmentPtr), int (* cmp) (t_EnvironmentMemberPtr, t_EnvironmentMemberPtr), t_EnvironmentMemberPtr memberPrev, t_EnvironmentMemberPtr memberInSearch, t_EnvironmentPtr environment, t_PriorityQueuePtr queue);
static void SayFather(t_EnvironmentMemberPtr memberFather, t_EnvironmentMemberPtr memberSon);
static void PrintPathPlan(t_EnvironmentMemberPtr memberLast);
static int IsEnvMemberUpdateNewCost(int costNew, t_EnvironmentMemberPtr member);


int
DoSomePathPlanning(int xStart, int yStart, int xEnd, int yEnd, int (* compute)(int, t_EnvironmentMemberPtr, t_EnvironmentPtr), int (* cmp)(t_ElementTypePtr, t_ElementTypePtr), t_EnvironmentPtr environment)
{
  t_PriorityQueuePtr planningQueue;
  t_EnvironmentMemberPtr member;

  assert(!IsEnvMemberObstacle(GetEnvMember(xStart, yStart, environment))); /* confirm the start and end is no obstacle */
  assert(!IsEnvMemberObstacle(GetEnvMember(xEnd, yEnd, environment)));

  member = GetEnvMemberInitial(xStart, yStart, environment);
  planningQueue = InitializePriorityQueue();
  assert(planningQueue != NULL);
  assert(member != NULL);

  SetEnvStartAndEnd(xStart, yStart, xEnd, yEnd, environment);

  for (; (member != NULL) && !IsSearchEnd(member, environment); member = GetEnvMemberNextMin(cmp, planningQueue)) {
    DebugCodeDetail (
		     printf("DoSomePathPlanning : Searched Member x %d y %d \n", GetEnvMemberX(member), GetEnvMemberY(member));
		     fflush(stdout);
		     );
    planningQueue = SearchNeighbour(compute, cmp, member, environment, planningQueue);
  }
  DestroyPriorityQueue(planningQueue);
  return (member == NULL) ? 0 : 1; /* 0: no way exist may have several reasons ; 1: some way exist */
}
/**
 *  \brief function description
 This functon is to initial the for search with the start point
 *  \param start point info and the environment to be searched
 *  \return the start point envmember
 */
/* assume the initial space have no obstacle */
static t_EnvironmentMemberPtr
GetEnvMemberInitial(int xStart, int yStart, t_EnvironmentPtr environment)
{
  t_EnvironmentMemberPtr memberFirst;

  memberFirst = GetEnvMember(xStart, yStart, environment);
  SetEnvMemberDead(memberFirst);
  return memberFirst;
}

static t_EnvironmentMemberPtr
GetEnvMemberNextMin(int (* cmp)(t_ElementTypePtr, t_ElementTypePtr), t_PriorityQueuePtr queue)
{
  t_EnvironmentMemberPtr memberNew;

  for (memberNew = DeleteMinPriorityQueue(cmp, queue); (memberNew != NULL) && IsEnvMemberDead(memberNew); memberNew = DeleteMinPriorityQueue(cmp, queue)); /* To get the next not dead min member */
  if (memberNew != NULL) {
    SetEnvMemberDead(memberNew);
  }
  return memberNew;		/* if no way exist DeleteMin -> memberNew -> return is all NULL */
}

static t_PriorityQueuePtr
SearchNeighbour(int (* compute) (int, t_EnvironmentMemberPtr, t_EnvironmentPtr), int (* cmp) (t_EnvironmentMemberPtr, t_EnvironmentMemberPtr), t_EnvironmentMemberPtr member, t_EnvironmentPtr environment, t_PriorityQueuePtr queue)
{
  int xCurrent, yCurrent, i, count;
  int x[4] = {-1, 1, 0, 0};	/* search in four directions */
  int y[4] = {0, 0, -1, 1};

  xCurrent = GetEnvMemberX(member);
  yCurrent = GetEnvMemberY(member);
  count = sizeof(x)/sizeof(int);

  for (i=0; i <count; i++) {
      queue = SearchADirection(xCurrent + x[i], yCurrent + y[i], compute, cmp, member, environment, queue);
  }
  return queue;
}

static t_PriorityQueuePtr
SearchADirection(int xIndex, int yIndex, int (* compute) (int, t_EnvironmentMemberPtr, t_EnvironmentPtr), int (* cmp) (t_EnvironmentMemberPtr, t_EnvironmentMemberPtr), t_EnvironmentMemberPtr memberPrev, t_EnvironmentPtr environment, t_PriorityQueuePtr queue)
{
  DebugCodeDetail (
		   printf("SearchADirection : Search x %d y %d\n", xIndex, yIndex);
		   fflush(stdout);
		   );
  if (IsEnvMemberLegal(xIndex, yIndex, environment)) {
    DebugCodeDetail (
		     printf("SearchADirection : Search Legal x %d y %d\n", xIndex, yIndex);
		     fflush(stdout);
		     );
    queue = SearchANode(compute, cmp, memberPrev, GetEnvMember(xIndex, yIndex, environment), environment, queue);
  }
  return queue;
}

/* If the cost of the existing QueueMember is smaller than the previous one, then update and insert again  */
static t_PriorityQueuePtr
SearchANode(int (* compute) (int, t_EnvironmentMemberPtr, t_EnvironmentPtr), int (* cmp) (t_EnvironmentMemberPtr, t_EnvironmentMemberPtr), t_EnvironmentMemberPtr memberPrev, t_EnvironmentMemberPtr memberInSearch, t_EnvironmentPtr environment, t_PriorityQueuePtr queue)
{
  int costNew;

  assert(memberInSearch != NULL);
  if (!IsEnvMemberObstacle(memberInSearch))
    {
      DebugCodeDetail (
		       static int i = 0;
		       printf("SearchANode : %d's times Search a valid node x %d y %d\n", i++, GetEnvMemberX(memberInSearch), GetEnvMemberY(memberInSearch));
		       fflush(stdout);
		       );
      /* if not a obstacle: update or not according to visited and gcost */
      costNew = c_costNormal;
      costNew += GetEnvMemberCost(memberPrev);
      if (IsEnvMemberUpdateNewCost(costNew, memberInSearch)) {
	queue = UpdateMember2Queue(costNew, compute, cmp, memberPrev, memberInSearch, environment, queue);
      }
    }
  return queue;
}

static int
IsEnvMemberUpdateNewCost(int costNew, t_EnvironmentMemberPtr member)
{
  return IsEnvMemberUnvisited(member) || (IsEnvMemberAlive(member) && (costNew < GetEnvMemberCost(member)));
}

static t_PriorityQueuePtr
UpdateMember2Queue(int costNew, int (* compute) (int, t_EnvironmentMemberPtr, t_EnvironmentPtr), int (* cmp) (t_EnvironmentMemberPtr, t_EnvironmentMemberPtr), t_EnvironmentMemberPtr memberPrev, t_EnvironmentMemberPtr memberInSearch, t_EnvironmentPtr environment, t_PriorityQueuePtr queue)
{
  SetEnvMemberCost(costNew, memberInSearch);
  SetEnvMemberPriority(compute(costNew, memberInSearch, environment), memberInSearch);
  SayFather(memberPrev, memberInSearch);
  SetEnvMemberAlive(memberInSearch);
  queue = InsertPriorityQueue(cmp, memberInSearch, queue);
  return queue;
}

int
ComparePriority(t_EnvironmentMemberPtr memberFirst, t_EnvironmentMemberPtr memberSecond)
{
  return GetEnvMemberPriority(memberFirst) > GetEnvMemberPriority(memberSecond);
}

static void
SayFather(t_EnvironmentMemberPtr memberFather, t_EnvironmentMemberPtr memberSon)
{
  SetEnvMemberPrev(memberFather, memberSon);
}

static void
PrintPathPlan(t_EnvironmentMemberPtr memberLast)
{
  t_EnvironmentMemberPtr memberPrev;

  for (memberPrev = memberLast; memberPrev != NULL; memberPrev = GetEnvMemberPrev(memberPrev))
    printf("%d %d %d\n", GetEnvMemberX(memberPrev), GetEnvMemberY(memberPrev), GetEnvMemberCost(memberPrev));
}
