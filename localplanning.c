/*
  Description: This file is all about some functions related to the basic parts of pathplanning;
  Author: Green
  Date: 16/06/02
 */
#include "GetChangeEnv.h"
#include "ComPlan.h"
#include "HeapQueue.h"


static t_EnvironmentMemberPtr GetEnvMemberInitial(int xStart, int yStart, t_EnvironmentPtr environment);
static t_EnvironmentMemberPtr GetEnvMemberNextMin(int (* cmp)(t_ElementTypePtr, t_ElementTypePtr), t_PriorityQueuePtr queue);
static t_PriorityQueuePtr SearchNeighbour(int (* compute) (int, t_EnvironmentMemberPtr, t_EnvironmentPtr), int (* cmp) (t_EnvironmentMemberPtr, t_EnvironmentMemberPtr), t_EnvironmentMemberPtr member, t_EnvironmentPtr environment, t_PriorityQueuePtr queue);
static t_PriorityQueuePtr SearchADirection(int xIndex, int yIndex, int (* compute) (int, t_EnvironmentMemberPtr, t_EnvironmentPtr), int (* cmp) (t_EnvironmentMemberPtr, t_EnvironmentMemberPtr), t_EnvironmentMemberPtr memberPrev, t_EnvironmentPtr environment, t_PriorityQueuePtr queue);
static t_PriorityQueuePtr SearchANode(int (* compute) (int, t_EnvironmentMemberPtr, t_EnvironmentPtr), int (* cmp) (t_EnvironmentMemberPtr, t_EnvironmentMemberPtr), t_EnvironmentMemberPtr memberPrev, t_EnvironmentMemberPtr memberInSearch, t_EnvironmentPtr environment, t_PriorityQueuePtr queue);
static t_PriorityQueuePtr UpdateMember2Queue(int costNew, int (* compute) (int, t_EnvironmentMemberPtr, t_EnvironmentPtr), int (* cmp) (t_EnvironmentMemberPtr, t_EnvironmentMemberPtr), t_EnvironmentMemberPtr memberPrev, t_EnvironmentMemberPtr memberInSearch, t_EnvironmentPtr environment, t_PriorityQueuePtr queue);
static void SayFather(t_EnvironmentMemberPtr memberFather, t_EnvironmentMemberPtr memberSon);
static void PrintPathPlan(t_EnvironmentMemberPtr memberLast);

void
DoSomePathPlanning(int xStart, int yStart, int xGoal, int yGoal, int (* compute)(int, t_EnvironmentMemberPtr, t_EnvironmentPtr), int (* cmp)(t_ElementTypePtr, t_ElementTypePtr), t_EnvironmentPtr environment)
{
  t_PriorityQueuePtr planningQueue;
  t_EnvironmentMemberPtr member;

  member = GetEnvMemberInitial(xStart, yStart, environment);
  planningQueue = InitializePriorityQueue();
  for (; IsSearchEnd(xGoal, yGoal, member, environment) == 0; member = GetEnvMemberNextMin(cmp, planningQueue)) {
    planningQueue = SearchNeighbour(compute, cmp, member, environment, planningQueue);
  }

  DestroyPriorityQueue(planningQueue);
}

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

  for (memberNew = DeleteMinPriorityQueue(cmp, queue); IsEnvMemberDead(memberNew); memberNew = DeleteMinPriorityQueue(cmp, queue));
  SetEnvMemberDead(memberNew);
  return memberNew;
}

static t_PriorityQueuePtr
SearchNeighbour(int (* compute) (int, t_EnvironmentMemberPtr, t_EnvironmentPtr), int (* cmp) (t_EnvironmentMemberPtr, t_EnvironmentMemberPtr), t_EnvironmentMemberPtr member, t_EnvironmentPtr environment, t_PriorityQueuePtr queue)
{
  int xCurrent, yCurrent;

  xCurrent = GetEnvMemberX(member);
  yCurrent = GetEnvMemberY(member);

  queue = SearchADirection(xCurrent - 1, yCurrent, compute, cmp, member, environment, queue);
  queue = SearchADirection(xCurrent + 1, yCurrent, compute, cmp, member, environment, queue);
  queue = SearchADirection(xCurrent, yCurrent - 1, compute, cmp, member, environment, queue);
  queue = SearchADirection(xCurrent, yCurrent + 1, compute, cmp, member, environment, queue);
  return queue;
}

static t_PriorityQueuePtr
SearchADirection(int xIndex, int yIndex, int (* compute) (int, t_EnvironmentMemberPtr, t_EnvironmentPtr), int (* cmp) (t_EnvironmentMemberPtr, t_EnvironmentMemberPtr), t_EnvironmentMemberPtr memberPrev, t_EnvironmentPtr environment, t_PriorityQueuePtr queue)
{
  if (IsEnvMemberLegal(xIndex, yIndex, environment))
    queue = SearchANode(compute, cmp, memberPrev, GetEnvMember(xIndex, yIndex, environment), environment, queue);
  return queue;
}

/* If the cost of the existing QueueMember is smaller than the previous one, then update and insert again  */
static t_PriorityQueuePtr
SearchANode(int (* compute) (int, t_EnvironmentMemberPtr, t_EnvironmentPtr), int (* cmp) (t_EnvironmentMemberPtr, t_EnvironmentMemberPtr), t_EnvironmentMemberPtr memberPrev, t_EnvironmentMemberPtr memberInSearch, t_EnvironmentPtr environment, t_PriorityQueuePtr queue)
{
  int costNew;

  if (!IsEnvMemberObstacle(memberInSearch))
    { /* if not a obstacle: update or not according to visited and gcost */
      costNew = c_costNormal;
      costNew += GetEnvMemberCost(memberPrev);
      if (IsEnvMemberUnvisited(memberInSearch)) {
	queue = UpdateMember2Queue(costNew, compute, cmp, memberPrev, memberInSearch, environment, queue);
      } else if (IsEnvMemberAlive(memberInSearch) && (costNew < GetEnvMemberCost(memberInSearch)))
	queue = UpdateMember2Queue(costNew, compute, cmp, memberPrev, memberInSearch, environment, queue);
    }
  return queue;
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
