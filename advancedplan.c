/*
  Description: This file is for the advanced use of the pathplanning
  Author: Green
  Date: 16/06/02
*/
#include "GetChangeEnv.h"
#include "ComPlan.h"
#include "PublicFun.h"

#define c_queueSizeMin 500

typedef struct t_PathLineMember
{
  int m_xIndex;
  int m_yIndex;
  int m_cost;                     /* also used as index in Distributed */
  struct t_PathLineMember *m_prevPtr;
} *t_PathLineMemsPtr;

typedef struct t_NearQueue
{
  int m_size;
  int m_index;
  int m_frontptr;
  int m_tailptr;
  t_EnvironmentMemberPtr *m_queuePtr;
} *t_NearestQueuePtr;

static t_PathLineMemsPtr sg_pathLines = NULL; /* for getting the partial search path  */
static t_PathLineMemsPtr sg_distributedLines = NULL; /* for distribute points */
static t_NearestQueuePtr sg_nearQueue = NULL; /* for the search nearest free point */

static void AstarInlocal(int xStart, int yStart, int xGoal, int yGoal, t_EnvironmentPtr environment);
static t_PathLineMemsPtr StorePathLine(int xGoal, int yGoal, t_PathLineMemsPtr pathLine, t_EnvironmentPtr environment);
static t_PathLineMemsPtr StoreNewPathMem(int xIndex, int yIndex, int mem_cost, t_PathLineMemsPtr pathLine);
static int IsNumEven(int num);
static void PrintPathLine(t_PathLineMemsPtr pathLine);
static t_PathLineMemsPtr FreePathLine(t_PathLineMemsPtr pathLine);
static void PartialPathSearch(int xStart, int yStart, int xGoal, int yGoal, t_EnvironmentPtr environment);
static int IsQueueFull();
static int IsQueueEmpty();
static void FreeNearestQueue();
static t_EnvironmentMemberPtr DeNearQueue();
static void EnNearQueue(t_EnvironmentMemberPtr memberNew);
static void MakeQueueBigger();
static void InitialNearQueue();
static void GetDistributedPoints(int xStart, int yStart, int xEnd, int yEnd, int width, t_EnvironmentPtr environment);
static void GetDistributedFreePoints(int xStart, int yStart, int xEnd, int yEnd, int width, t_EnvironmentPtr environment);
static void UpdateNearestFreePoint(t_PathLineMemsPtr member, t_EnvironmentPtr environment);
static void StoreDistributedPoints(int xMin, int yMin, int xMax, int yMax, int xStart, int yStart, int width);
static void SearchFourNeighbour(t_EnvironmentMemberPtr member, t_EnvironmentPtr environment);
static void SearchOneNode(int xIndex, int yIndex, t_EnvironmentPtr environment);
static void SwapNum(int *a, int *b);
static t_PathLineMemsPtr AppendLine(t_PathLineMemsPtr pathLineFirst, t_PathLineMemsPtr pathLineSecond); /* performance bug */
static int DijskstraPriority(int costNew, t_EnvironmentMemberPtr member, t_EnvironmentPtr environment);
static int DepthFirstPriority(int costNew, t_EnvironmentMemberPtr member, t_EnvironmentPtr environment);
static int AStarPriority(int costNew, t_EnvironmentMemberPtr member, t_EnvironmentPtr environment);

static int
DijskstraPriority(int costNew, t_EnvironmentMemberPtr member, t_EnvironmentPtr environment)
{
  return costNew;
}

static int
DepthFirstPriority(int costNew, t_EnvironmentMemberPtr member, t_EnvironmentPtr environment)
{
  return GetEnvXGoal(environment) + GetEnvYGoal(environment) - GetEnvMemberX(member) -  GetEnvMemberY(member);
}

int
AStarPriority(int costNew, t_EnvironmentMemberPtr member, t_EnvironmentPtr environment)
{
  return DijskstraPriority(costNew, member, environment) +
    DepthFirstPriority(costNew, member, environment);
}

void
ScanSearch(int xStart, int yStart, int xGoal, int yGoal, int width, t_EnvironmentPtr environment)
{
  t_PathLineMemsPtr member;
  int xPrev, yPrev;

  if (sg_distributedLines != NULL) {
    sg_distributedLines = FreePathLine(sg_distributedLines);
  }
  GetDistributedFreePoints(xStart, yStart, xGoal, yGoal, width, environment);

  if (sg_distributedLines != NULL) {
    //   sg_distributedLines = FreePathLine(sg_distributedLines);
    member = sg_distributedLines;
    xPrev = member->m_xIndex;
    yPrev = member->m_yIndex;     /* store something in cache */
    for (member = member->m_prevPtr; member != NULL; member = member->m_prevPtr) {
      PartialPathSearch(xPrev, yPrev, member->m_xIndex, member->m_yIndex, environment);
      xPrev = member->m_xIndex;
      yPrev = member->m_yIndex;
    }
    sg_distributedLines = FreePathLine(sg_distributedLines);
  }
}

static void
GetDistributedPoints(int xStart, int yStart, int xEnd, int yEnd, int width, t_EnvironmentPtr environment)
{
  int min_x, min_y, max_x, max_y;

  if (!IsEnvPointValid(xStart, yStart, environment) && !IsEnvPointValid(xEnd, yEnd, environment)) {
    fprintf(stderr, "Not a valid Points in GetDistributedPoints\n");
    exit(0);
  }
  min_x = xStart < xEnd ? xStart : xEnd;
  min_y = yStart < yEnd ? yStart : yEnd;
  max_x = xStart + xEnd - min_x;
  max_y = yStart + yEnd - min_y;
  StoreDistributedPoints(min_x, min_y, max_x, max_y, xStart, yStart, width);
}

static void
GetDistributedFreePoints(int xStart, int yStart, int xEnd, int yEnd, int width, t_EnvironmentPtr environment)
{
  t_PathLineMemsPtr member;

  GetDistributedPoints(xStart, yStart, xEnd, yEnd, width, environment);
  //PrintPathLine(sg_distributedLines);
  for (member = sg_distributedLines; member != NULL; member = member->m_prevPtr) {
    UpdateNearestFreePoint(member, environment);
  }
  PrintPathLine(sg_distributedLines);
}

static void
UpdateNearestFreePoint(t_PathLineMemsPtr member, t_EnvironmentPtr environment)
{
  t_EnvironmentMemberPtr newFreeMember;
  int xIndex, yIndex;

  xIndex = member->m_xIndex;
  yIndex = member->m_yIndex;
  if (IsEnvMemberObstacle(GetEnvMember(xIndex, yIndex, environment))) {
    newFreeMember = SearchNearestFreePoint(xIndex, yIndex, environment);
    if (newFreeMember == NULL) {
      fprintf(stderr, "No Nearest Free\n");
      exit(0);
    }
    member->m_xIndex = GetEnvMemberX(newFreeMember);
    member->m_yIndex = GetEnvMemberY(newFreeMember);
  }
}

t_EnvironmentMemberPtr
SearchNearestFreePoint(int xIndex, int yIndex, t_EnvironmentPtr environment)
{
  t_EnvironmentMemberPtr member;

  if (environment == NULL)
    return NULL;

  ResetEnvAllFlag(environment);

  InitialNearQueue();

  for (member = GetEnvMember(xIndex, yIndex, environment); IsEnvMemberObstacle(member); member = DeNearQueue()) {
    SearchFourNeighbour(member, environment);
  }
  FreeNearestQueue();
  return member;
}

static void
SearchFourNeighbour(t_EnvironmentMemberPtr member, t_EnvironmentPtr environment)
{
  int xIndex = GetEnvMemberX(member);
  int yIndex = GetEnvMemberY(member);

  // printf("SFN: %d %d\n", xIndex, yIndex);
  //fflush(stdout);
  SearchOneNode(xIndex, yIndex + 1, environment);
  SearchOneNode(xIndex + 1, yIndex, environment);
  SearchOneNode(xIndex, yIndex - 1, environment);
  SearchOneNode(xIndex - 1, yIndex, environment);
}

static void
SearchOneNode(int xIndex, int yIndex, t_EnvironmentPtr environment)
{
  t_EnvironmentMemberPtr member;

  if (IsEnvMemberValid(xIndex, yIndex, environment)) {
    member = GetEnvMember(xIndex, yIndex, environment);
    if (IsEnvMemberFlagNotSet(member)) {
      SetEnvMemberFlag(member);
      EnNearQueue(member);
    }
  }
}

static void
InitialNearQueue()
{
  if (sg_nearQueue == NULL) {
    sg_nearQueue = Malloc(sizeof(struct t_NearQueue));
    sg_nearQueue->m_queuePtr = Malloc(sizeof(t_EnvironmentMemberPtr) * c_queueSizeMin);
    sg_nearQueue->m_size = c_queueSizeMin;
    sg_nearQueue->m_index = sg_nearQueue->m_frontptr = sg_nearQueue->m_tailptr = 0;
  }
}

static int
IsQueueFull()
{
  return sg_nearQueue->m_size == sg_nearQueue->m_index;
}

static int
IsQueueEmpty()
{
  return sg_nearQueue->m_index == 0;
}

static void
FreeNearestQueue()
{
  free(sg_nearQueue->m_queuePtr);
  free(sg_nearQueue);
  sg_nearQueue = NULL;
}

static t_EnvironmentMemberPtr
DeNearQueue()
{
  t_EnvironmentMemberPtr memberOut;

  if (IsQueueEmpty()) {
    fprintf(stderr, "NearQueue is Empty\n");
    exit(0);
  } else {
    memberOut = (sg_nearQueue->m_queuePtr)[sg_nearQueue->m_frontptr];
    sg_nearQueue->m_index -= 1;
    sg_nearQueue->m_frontptr += 1;
    if (sg_nearQueue->m_frontptr == sg_nearQueue->m_size) {
      sg_nearQueue->m_frontptr = 0;
    }
    return memberOut;
  }
}

static void
EnNearQueue(t_EnvironmentMemberPtr memberNew)
{
  if (IsQueueFull()) {
    MakeQueueBigger();
  }

  (sg_nearQueue->m_queuePtr)[sg_nearQueue->m_tailptr] = memberNew;
  sg_nearQueue->m_index += 1;
  sg_nearQueue->m_tailptr += 1;
  if (sg_nearQueue->m_tailptr == sg_nearQueue->m_size) {
    sg_nearQueue->m_tailptr = 0;
  }
}

static void
MakeQueueBigger()
{
  t_EnvironmentMemberPtr *newQueue;
  int newSize = (sg_nearQueue->m_size) * 2;
  int newPtr;


  newQueue = Malloc(sizeof(t_EnvironmentMemberPtr) * newSize);
  for (newPtr = 0; newPtr < sg_nearQueue->m_size; newPtr++) {
    newQueue[newPtr] = (sg_nearQueue->m_queuePtr)[newPtr];
  }

  free(sg_nearQueue->m_queuePtr);
  sg_nearQueue->m_queuePtr = newQueue;
  sg_nearQueue->m_frontptr = 0;
  sg_nearQueue->m_tailptr = sg_nearQueue->m_index;
  sg_nearQueue->m_size = newSize;
}

static t_PathLineMemsPtr                     /* from the end to the start point  */
StorePathLine(int xGoal, int yGoal, t_PathLineMemsPtr pathLine, t_EnvironmentPtr environment)
{
  t_EnvironmentMemberPtr memberPrev;
  t_PathLineMemsPtr pathLineNew = NULL;

  for (memberPrev = GetEnvMember(xGoal, yGoal, environment); memberPrev != NULL; memberPrev = GetEnvMemberPrev(memberPrev)) {
    DebugCode(
	      static int i = 1;
	      printf("This is for the %d nd internal of StorePathLine\n", i++);
	      printf("Searched Member has X: %d, Y: %d, Cost: %d\n", GetEnvMemberX(memberPrev), GetEnvMemberY(memberPrev), GetEnvMemberCost(memberPrev));
	      );
    pathLineNew = StoreNewPathMem(GetEnvMemberX(memberPrev), GetEnvMemberY(memberPrev), GetEnvMemberCost(memberPrev), pathLineNew);
  }

  pathLine = AppendLine(pathLine, pathLineNew);
  return pathLine;
}

/* !!This function may need to think it over again */
static void                     /* the last point is the start */
StoreDistributedPoints(int xMin, int yMin, int xMax, int yMax, int xStart, int yStart, int width)
{
  int i, j, yTemp, widthCount, isGoUp;
  int itemp_x, jtemp_x;

  widthCount = (yMax - yMin)/width;
  if (yStart == yMin)
    isGoUp = 1;
  else
    isGoUp = 0;
  if ((yMax - yMin) > width * widthCount)
    widthCount++;              /* need to compensate the rest of the width */
  if (IsNumEven(widthCount)) {      /* determines the initial x according to the widthCount even or odd */
    itemp_x = xStart;
    jtemp_x = xMin + xMax - itemp_x;
  } else {
    itemp_x = xMin + xMax - xStart;
    jtemp_x = xStart;
  }
  if (isGoUp) {
    for (i=j=0, yTemp = yMin; i < widthCount - 1; i++, j=j+2, yTemp += width) {
      sg_distributedLines = StoreNewPathMem(itemp_x, yTemp, j, sg_distributedLines);
      sg_distributedLines = StoreNewPathMem(jtemp_x, yTemp, j+1, sg_distributedLines);
      SwapNum(&itemp_x, &jtemp_x);
    }
    sg_distributedLines = StoreNewPathMem(itemp_x, yMax, j, sg_distributedLines); /* take care of the rest */
    sg_distributedLines = StoreNewPathMem(jtemp_x, yMax, j+1, sg_distributedLines);
  } else {
    for (i=j=0, yTemp = yMax; i < widthCount - 1; i++, j=j+2, yTemp -= width) {
      sg_distributedLines = StoreNewPathMem(itemp_x, yTemp, j, sg_distributedLines);
      sg_distributedLines = StoreNewPathMem(jtemp_x, yTemp, j+1, sg_distributedLines);
      SwapNum(&itemp_x, &jtemp_x);
    }
    sg_distributedLines = StoreNewPathMem(itemp_x, yMin, j, sg_distributedLines);
    sg_distributedLines = StoreNewPathMem(jtemp_x, yMin, j+1, sg_distributedLines);
  }
}

static int
IsNumEven(int num)
{
  int half;

  half = num/2;
  return half * 2 == num;
}

static void
SwapNum(int *a, int *b)
{
  int temp;

  temp = *a;
  *a = *b;
  *b = temp;
}

static t_PathLineMemsPtr
StoreNewPathMem(int xIndex, int yIndex, int cost, t_PathLineMemsPtr pathLine)
{
  t_PathLineMemsPtr memberNew;

  memberNew = Malloc(sizeof(struct t_PathLineMember));
  memberNew->m_xIndex = xIndex;
  memberNew->m_yIndex = yIndex;
  memberNew->m_cost = cost;
  memberNew->m_prevPtr = pathLine;

  return memberNew;
}

static void
PrintPathLine(t_PathLineMemsPtr pathLine)
{
  t_PathLineMemsPtr memberPrev;

  for (memberPrev = pathLine; memberPrev != NULL; memberPrev = memberPrev->m_prevPtr)
    printf("The Line is %d %d %d\n", memberPrev->m_xIndex, memberPrev->m_yIndex, memberPrev->m_cost);
}

static t_PathLineMemsPtr
FreePathLine(t_PathLineMemsPtr pathLine)
{
  t_PathLineMemsPtr memberPrev;
  t_PathLineMemsPtr memberTemp;

  for (memberPrev = pathLine; memberPrev != NULL;) {
    memberTemp = memberPrev;
    memberPrev = memberPrev->m_prevPtr;
    free(memberTemp);
  }
  return NULL;
}

void
PrintAndFreePathLine()
{
  PrintPathLine(sg_pathLines);
  sg_pathLines = FreePathLine(sg_pathLines);
}

static void
AstarInlocal(int xStart, int yStart, int xGoal, int yGoal, t_EnvironmentPtr environment)
{
  DoSomePathPlanning(xStart, yStart, xGoal, yGoal, AStarPriority, ComparePriority, environment);
}

void
PartialPathSearch(int xStart, int yStart, int xGoal, int yGoal, t_EnvironmentPtr environment)
{
  if (sg_pathLines != NULL) {
    ResetEnvironment(environment);
  }
  //  printf("xs%d, ys%d, xg%d, yg%d\n", xStart, yStart, xGoal, yGoal);
  AstarInlocal(xStart, yStart, xGoal, yGoal, environment);
  sg_pathLines = StorePathLine(xGoal, yGoal, sg_pathLines, environment);
}

static t_PathLineMemsPtr
AppendLine(t_PathLineMemsPtr pathLineFirst, t_PathLineMemsPtr pathLineSecond)
{
  t_PathLineMemsPtr linePrev;

  DebugCode (
	     static int i = 1;
	     printf("This is for %d nd the first part of AppendLine\n", i);
	     PrintPathLine(pathLineFirst);
	     printf("This is for %d nd the second part of AppendLine\n", i++);
	     PrintPathLine(pathLineSecond);
	     );
  if (pathLineFirst != NULL) {
    for (linePrev = pathLineFirst; linePrev->m_prevPtr != NULL; linePrev = linePrev->m_prevPtr) ;
    linePrev->m_prevPtr = pathLineSecond;
    return pathLineFirst;
  } else {
    return pathLineSecond;
  }
}
