/*
  Description: This file is for the advanced use of the pathplanning
  Author: Green
  Date: 16/06/02
*/

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "advancedplan.h"
#include "localplanning.h"
#include "pretreatment.h"
#include "publicfun.h"
#include "fifoqueue.h"
#include "mission.h"

typedef struct t_PathLineMember
{
  int m_xIndex;
  int m_yIndex;
  int m_cost;                     /* also used as index in Distributed */
  struct t_PathLineMember *m_prevPtr;
} *t_PathLineMemsPtr;

static t_PathLineMemsPtr sg_pathLines = NULL; /* for getting the partial search path  */
static t_PathLineMemsPtr sg_distributedPoints = NULL; /* for distribute points */
static t_FifoQueuePtr sg_nearQueue = NULL; /* for the search nearest free point */


static t_PathLinesPtr GetEnvPathLines(void);
static void TurnEnv2GpsPathLines(t_PathLinesPtr finalPathLines, t_EnvironmentPtr environment);
static void TurnEnvX2Gps(t_PathPointPtr finalPathPoint, t_EnvironmentPtr environment);
static void TurnEnvY2Gps(t_PathPointPtr finalPathPoint, t_EnvironmentPtr environment);
static int GetFinalPathPointX(t_PathPointPtr finalPathPoint);
static int GetFinalPathPointY(t_PathPointPtr finalPathPoint);
static void SetFinalPathPointX(int x, t_PathPointPtr finalPathPoint);
static void SetFinalPathPointY(int y, t_PathPointPtr finalPathPoint);
static int AstarInlocal(int xStart, int yStart, int xEnd, int yEnd, t_EnvironmentPtr environment);
static void NextPathLinePoint(void);
static int IsEnvPathLineEmpty(void);
static void InsertNewFinalPathPoint(int x, int y, t_PathLinesPtr finalPathLines);
static t_PathLinesPtr InitialFinalPathLines(void);
static t_PathLineMemsPtr StorePathLine(int xEnd, int yEnd, t_PathLineMemsPtr pathLine, t_EnvironmentPtr environment);
static t_PathLineMemsPtr StoreNewDistributedMembers(int xIndex, int yIndex, int mem_cost, t_PathLineMemsPtr pathLine);
static int IsNumEven(int num);
static void PrintPathFreePoints(t_PathLineMemsPtr pathLine);
static void PrintPathFinalPoints(t_PathLineMemsPtr pathLine);
static void PrintPathPoints(t_PathLineMemsPtr pathLine, char *str);
static t_PathLineMemsPtr FreePathLine(t_PathLineMemsPtr pathLine);


static void GetDistributedPoints(int xStart, int yStart, int xEnd, int yEnd, int width, t_EnvironmentPtr environment);
static int GetDistributedFreePoints(int xStart, int yStart, int xEnd, int yEnd, int width, t_EnvironmentPtr environment);
static int UpdateNearestFreePoint(t_PathLineMemsPtr member, t_EnvironmentPtr environment);
static void StoreDistributedPoints(int xMin, int yMin, int xMax, int yMax, int xStart, int yStart, int width);
static t_PathLineMemsPtr StoreNewPathMember(t_EnvironmentMemberPtr envMember, t_PathLineMemsPtr pathLine);
static void SearchFourNeighbour(t_EnvironmentMemberPtr member, t_EnvironmentPtr environment);
static void SearchOneNode(int xIndex, int yIndex, t_EnvironmentPtr environment);

static t_PathLineMemsPtr AppendLine(t_PathLineMemsPtr pathLineFirst, t_PathLineMemsPtr pathLineSecond); /* performance bug */
static int DijskstraPriority(int costNew);
static int DepthFirstPriority(t_EnvironmentMemberPtr member, t_EnvironmentPtr environment);
static int AStarPriority(int costNew, t_EnvironmentMemberPtr member, t_EnvironmentPtr environment);
static int GetPathLineMemberX(t_PathLineMemsPtr pathLineMember);
static int GetPathLineMemberY(t_PathLineMemsPtr pathLineMember);
static int GetPathLineMemberCost(t_PathLineMemsPtr pathLineMember);
static t_PathLineMemsPtr GetPathLineMemberPrev(t_PathLineMemsPtr pathLineMember);
static void FilterPathLinePoints(void);
static int IsThreePointInALine(int x1, int y1, int x2, int y2, int x3, int y3);
static int GetPathLineX(void);
static int GetPathLineY(void);
static int GetNextLineX(void);
static int GetNextLineY(void);
static int GetDoubleNextLineX(void);
static int GetDoubleNextLineY(void);
static int IsDoubleNextLineExist(void);
static int IsTwoCorClose(int x, int y);
static int IsTwoPointsClose(int x1, int y1, int x2, int y2);

static int
GetPathLineMemberX(t_PathLineMemsPtr pathLineMember)
{
  return pathLineMember->m_xIndex;
}

static int
GetPathLineMemberY(t_PathLineMemsPtr pathLineMember)
{
  return pathLineMember->m_yIndex;
}

static int
GetPathLineMemberCost(t_PathLineMemsPtr pathLineMember)
{
  return pathLineMember->m_cost;
}

static t_PathLineMemsPtr
GetPathLineMemberPrev(t_PathLineMemsPtr pathLineMember)
{
  return pathLineMember->m_prevPtr;
}

static int
DijskstraPriority(int costNew)
{
  return costNew;
}

static int
DepthFirstPriority(t_EnvironmentMemberPtr member, t_EnvironmentPtr environment)
{
  int xHeuristic, yHeuristic;

  xHeuristic = GetEnvEndX(environment) - GetEnvMemberX(member);
  yHeuristic = GetEnvEndY(environment)-  GetEnvMemberY(member);

  xHeuristic = abs(xHeuristic);
  yHeuristic = abs(yHeuristic);
  return xHeuristic + yHeuristic;
}

/* can add some restraint of keep the same direction with the previous and stay at the same y(or may be the line of start and the end) */
static int
AStarPriority(int costNew, t_EnvironmentMemberPtr member, t_EnvironmentPtr environment)
{
  return DijskstraPriority(costNew) +
    DepthFirstPriority(member, environment);
}

int
ScanSearch(int xStart, int yStart, int xEnd, int yEnd, int width, t_EnvironmentPtr environment)
{
  t_PathLineMemsPtr member;
  int xPrev, yPrev;

  assert(xStart >= 0 && xStart <= GetEnvLength(environment));
  assert(yStart >= 0 && yStart <= GetEnvWidth(environment));
  assert(xEnd >= 0 && xEnd <= GetEnvLength(environment));
  assert(yEnd >= 0 && yEnd <= GetEnvWidth(environment));
  assert(sg_distributedPoints == NULL);

  if (GetDistributedFreePoints(xStart, yStart, xEnd, yEnd, width, environment) == 0) {
    sg_distributedPoints = FreePathLine(sg_distributedPoints);
    return 0;			/* no free points exist */
  }
  assert(sg_distributedPoints != NULL);

  member = sg_distributedPoints;
  xPrev = GetPathLineMemberX(member);
  yPrev = GetPathLineMemberY(member);     /* store something in cache */
  for (member = GetPathLineMemberPrev(member); member != NULL; member = GetPathLineMemberPrev(member)) {
    if (PartialPathSearch(xPrev, yPrev, GetPathLineMemberX(member), GetPathLineMemberY(member), environment) == 0) { /* handle the situation of no way being */
      sg_distributedPoints = FreePathLine(sg_distributedPoints);
      return 0;
    }
    xPrev = GetPathLineMemberX(member);
    yPrev = GetPathLineMemberY(member);
  }
  sg_distributedPoints = FreePathLine(sg_distributedPoints);
  return 1;
}

static void
GetDistributedPoints(int xStart, int yStart, int xEnd, int yEnd, int width, t_EnvironmentPtr environment)
{
  int min_x, min_y, max_x, max_y;

  assert(IsEnvPointValid(xStart, yStart, environment)); /* assert valid Start Point */
  assert(IsEnvPointValid(xEnd, yEnd, environment));

  min_x = xStart < xEnd ? xStart : xEnd;
  min_y = yStart < yEnd ? yStart : yEnd;
  max_x = xStart + xEnd - min_x;
  max_y = yStart + yEnd - min_y;
  StoreDistributedPoints(min_x, min_y, max_x, max_y, xStart, yStart, width);
}

static int
GetDistributedFreePoints(int xStart, int yStart, int xEnd, int yEnd, int width, t_EnvironmentPtr environment)
{
  t_PathLineMemsPtr member;

  GetDistributedPoints(xStart, yStart, xEnd, yEnd, width, environment);
  for (member = sg_distributedPoints; member != NULL; member = GetPathLineMemberPrev(member)) {
    if (UpdateNearestFreePoint(member, environment) == 0)
      return 0;			/* No Free Points exist return the handle to the upper class */
  }
  DebugCode(
	    PrintPathFreePoints(sg_distributedPoints);
	    fflush(stdout);
	    );
  return 1;
}

static int
UpdateNearestFreePoint(t_PathLineMemsPtr member, t_EnvironmentPtr environment)
{
  t_EnvironmentMemberPtr newFreeMember;
  int xIndex, yIndex;

  xIndex = GetPathLineMemberX(member);
  yIndex = GetPathLineMemberY(member);
  if (IsEnvMemberObstacle(GetEnvMember(xIndex, yIndex, environment))) {
    newFreeMember = SearchNearestFreePoint(xIndex, yIndex, environment);
    if (newFreeMember == NULL) {
      return 0;			/* no nearest free points exist */
    }
    member->m_xIndex = GetEnvMemberX(newFreeMember);
    member->m_yIndex = GetEnvMemberY(newFreeMember);
  }
  return 1;
}

t_EnvironmentMemberPtr
SearchNearestFreePoint(int xIndex, int yIndex, t_EnvironmentPtr environment)
{
  t_EnvironmentMemberPtr member;

  if (environment == NULL)
    return NULL;

  ResetEnvAllFlag(environment);

  sg_nearQueue =  InitialFifoQueue();

  for (member = GetEnvMember(xIndex, yIndex, environment); IsEnvMemberObstacle(member); member = DeFifoQueue(sg_nearQueue)) {
    SearchFourNeighbour(member, environment);
  }
  FreeFifoQueue(sg_nearQueue);
  return member;
}

static void
SearchFourNeighbour(t_EnvironmentMemberPtr member, t_EnvironmentPtr environment)
{
  int xIndex = GetEnvMemberX(member);
  int yIndex = GetEnvMemberY(member);
  int x[4] = {-1, 1, 0, 0};	/* search in four directions */
  int y[4] = {0, 0, -1, 1};
  unsigned long i;

 for (i=0; i < sizeof(x)/sizeof(int); i++)
   SearchOneNode(xIndex + x[i], yIndex + y[i], environment);
}

static void
SearchOneNode(int xIndex, int yIndex, t_EnvironmentPtr environment)
{
  t_EnvironmentMemberPtr member;

  if (IsEnvMemberValid(xIndex, yIndex, environment)) {
    member = GetEnvMember(xIndex, yIndex, environment);
    if (IsEnvMemberFlagNotSet(member)) {
      SetEnvMemberFlag(member);
      sg_nearQueue = EnFifoQueue(member, sg_nearQueue);
    }
  }
}


static t_PathLineMemsPtr                     /* from the end to the start point  */
StorePathLine(int xEnd, int yEnd, t_PathLineMemsPtr pathLine, t_EnvironmentPtr environment)
{
  t_EnvironmentMemberPtr memberPrev;
  t_PathLineMemsPtr pathLineNew = NULL;

  for (memberPrev = GetEnvMember(xEnd, yEnd, environment); memberPrev != NULL; memberPrev = GetEnvMemberPrev(memberPrev)) {
    DebugCodeDetail(
	      static int i = 1;
	      printf("StorePathLine : %d nd search Searched Member has X %d, Y %d, Cost %d\n", i++, GetEnvMemberX(memberPrev), GetEnvMemberY(memberPrev), GetEnvMemberCost(memberPrev));
	      fflush(stdout);
	      );
    pathLineNew = StoreNewPathMember(memberPrev, pathLineNew);
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
      sg_distributedPoints = StoreNewDistributedMembers(itemp_x, yTemp, j, sg_distributedPoints);
      sg_distributedPoints = StoreNewDistributedMembers(jtemp_x, yTemp, j+1, sg_distributedPoints);
      SwapNum(&itemp_x, &jtemp_x);
    }
    sg_distributedPoints = StoreNewDistributedMembers(itemp_x, yMax, j, sg_distributedPoints); /* take care of the rest */
    sg_distributedPoints = StoreNewDistributedMembers(jtemp_x, yMax, j+1, sg_distributedPoints);
  } else {
    for (i=j=0, yTemp = yMax; i < widthCount - 1; i++, j=j+2, yTemp -= width) {
      sg_distributedPoints = StoreNewDistributedMembers(itemp_x, yTemp, j, sg_distributedPoints);
      sg_distributedPoints = StoreNewDistributedMembers(jtemp_x, yTemp, j+1, sg_distributedPoints);
      SwapNum(&itemp_x, &jtemp_x);
    }
    sg_distributedPoints = StoreNewDistributedMembers(itemp_x, yMin, j, sg_distributedPoints);
    sg_distributedPoints = StoreNewDistributedMembers(jtemp_x, yMin, j+1, sg_distributedPoints);
  }
}

static int
IsNumEven(int num)
{
  int half;

  half = num/2;
  return half * 2 == num;
}

static t_PathLineMemsPtr
StoreNewDistributedMembers(int xIndex, int yIndex, int cost, t_PathLineMemsPtr pathLine)
{
  t_PathLineMemsPtr memberNew;

  memberNew = Malloc(sizeof(struct t_PathLineMember));
  memberNew->m_xIndex = xIndex;
  memberNew->m_yIndex = yIndex;
  memberNew->m_cost = cost;
  memberNew->m_prevPtr = pathLine;

  return memberNew;
}

static t_PathLineMemsPtr
StoreNewPathMember(t_EnvironmentMemberPtr envMember, t_PathLineMemsPtr pathLine)
{
  t_PathLineMemsPtr memberNew;

  memberNew = Malloc(sizeof(struct t_PathLineMember));
  memberNew->m_xIndex = GetEnvMemberX(envMember);
  memberNew->m_yIndex = GetEnvMemberY(envMember);
  memberNew->m_cost = GetEnvMemberCost(envMember);
  memberNew->m_prevPtr = pathLine;

  return memberNew;
}

static void
PrintPathPoints(t_PathLineMemsPtr pathLine, char *str)
{
  t_PathLineMemsPtr memberPrev;
  static int i = 0;

  for (memberPrev = pathLine; memberPrev != NULL; memberPrev = memberPrev->m_prevPtr)
    printf("%s : the %d nd point x %d y %d cost %d\n", str, i++, memberPrev->m_xIndex, memberPrev->m_yIndex, memberPrev->m_cost);
}

static void
PrintPathFreePoints(t_PathLineMemsPtr pathLine)
{
  PrintPathPoints(pathLine, "The pass free point");
}

static void
PrintPathFinalPoints(t_PathLineMemsPtr pathLine)
{
  PrintPathPoints(pathLine, "The final pass point");
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
PrintEnvPathLine(void)
{
  PrintPathFinalPoints(sg_pathLines);
}

void
PrintAndFreePathLine(void)
{
  PrintPathFinalPoints(sg_pathLines);
  sg_pathLines = FreePathLine(sg_pathLines);
}

static int
GetPathLineX(void)
{
  return sg_pathLines->m_xIndex;
}

static int
GetPathLineY(void)
{
  return sg_pathLines->m_yIndex;
}

static int
GetNextLineX(void)
{
  return sg_pathLines->m_prevPtr->m_xIndex;
}

static int
GetNextLineY(void)
{
  return sg_pathLines->m_prevPtr->m_yIndex;
}

static int
GetDoubleNextLineX(void)
{
  return sg_pathLines->m_prevPtr->m_prevPtr->m_xIndex;
}

static int
GetDoubleNextLineY(void)
{
  return sg_pathLines->m_prevPtr->m_prevPtr->m_yIndex;
}

static int
IsDoubleNextLineExist(void)
{
  return (sg_pathLines->m_prevPtr != NULL) && (sg_pathLines->m_prevPtr->m_prevPtr != NULL);
}

static void
NextPathLinePoint(void)
{
  t_PathLineMemsPtr memberTemp;

  assert(sg_pathLines != NULL);
  memberTemp = sg_pathLines->m_prevPtr;
  free(sg_pathLines);
  sg_pathLines = memberTemp;
}

static int
IsEnvPathLineEmpty(void)
{
  return sg_pathLines == NULL;
}

static t_PathLinesPtr
GetEnvPathLines(void)
{
  t_PathLinesPtr finalPathLines;
  int x, y;

  finalPathLines = InitialFinalPathLines();
  for (; !IsEnvPathLineEmpty(); NextPathLinePoint()) {
    x = GetPathLineX();
    y = GetPathLineY();
    InsertNewFinalPathPoint(x, y, finalPathLines);
    FilterPathLinePoints();
  }
  return finalPathLines;
}

t_PathLinesPtr
GetGpsPathLines(t_EnvironmentPtr environment)
{
  t_PathLinesPtr finalPathLines;

  finalPathLines = GetEnvPathLines();
  DebugCode (
             PrintFinalPathLines(finalPathLines, "The env pass point");
             );
  TurnEnv2GpsPathLines(finalPathLines, environment);
  return finalPathLines;
}

static void
TurnEnv2GpsPathLines(t_PathLinesPtr finalPathLines, t_EnvironmentPtr environment)
{
  t_PathPointPtr finalPathPoints;

  for (finalPathPoints = finalPathLines->m_pathPoints; finalPathPoints != NULL; finalPathPoints = finalPathPoints->m_next) {
    TurnEnvX2Gps(finalPathPoints, environment);
    TurnEnvY2Gps(finalPathPoints, environment);
  }
}

/*
  the element x in the map is bigger than the topleft point and the y is smaller than the topleft point
  the formual: longitude = curlongi + x / (111.3 * math.cos(math.radians(curlat)))/ 1000(parameter is m)
  latitude = curlat - y/111.3/1000
  Remember that the answer is ought to inhance 10^7 like 1200291593, the remember that the angle is like 120.0291593
*/
static void
TurnEnvX2Gps(t_PathPointPtr finalPathPoint, t_EnvironmentPtr environment)
{
  int x, xGps;
  double shift, lona;

  x = GetFinalPathPointX(finalPathPoint) * GetEnvLengthOfUnit(environment);
  x *= pow(10, 4);
  lona = (double)GetEnvTopLeftY(environment)/pow(10,7);
  shift = x / (111 * cos(Angle2Radians(lona)));
  shift = fabs(shift);
  xGps = GetEnvTopLeftX(environment) + (int)shift;
  SetFinalPathPointX(xGps, finalPathPoint);
}

static void
TurnEnvY2Gps(t_PathPointPtr finalPathPoint, t_EnvironmentPtr environment)
{
  int y, yGps;
  double shift;

  y =  GetFinalPathPointY(finalPathPoint) * GetEnvWidthOfUnit(environment);
  y *= pow(10, 4);
  shift = y / 111;
  yGps = GetEnvTopLeftY(environment) - (int) shift;
  SetFinalPathPointY(yGps, finalPathPoint);
}

static void
FilterPathLinePoints(void)
{
  while (IsDoubleNextLineExist()) {
    if (IsThreePointInALine(GetPathLineX(), GetPathLineY(), GetNextLineX(), GetNextLineY(), GetDoubleNextLineX(), GetDoubleNextLineY())) {
      NextPathLinePoint();
    }else {
      break;
    }
  }
}

static int
IsTwoPointsClose(int x1, int y1, int x2, int y2)
{
  return IsTwoCorClose(x1, x2) && IsTwoCorClose(y1, y2);
}

static int
IsTwoCorClose(int x, int y)
{
  return abs(x-y) < 2;
}

static int
IsThreePointInALine(int x1, int y1, int x2, int y2, int x3, int y3)
{
  if ((x1 == x2) && (x2 == x3)) { /* just take care of the condition that delta of x is zero */
    return 1;
  } else if ((x1 == x2) || (x1 == x3) || (x2 == x3)) {
    return 0;
  } else {
    double delta1, delta2;

    delta1 = (double) abs(y1 - y2) / (double) (abs(x1 - x2));
    delta2 = (double) abs(y2 - y3) / (double) (abs(x2 - x3));
    return IsDoubleEqual(delta1, delta2);
  }
}


static int
AstarInlocal(int xStart, int yStart, int xEnd, int yEnd, t_EnvironmentPtr environment)
{
  return DoSomePathPlanning(xStart, yStart, xEnd, yEnd, AStarPriority, ComparePriority, environment);
}

int
PartialPathSearch(int xStart, int yStart, int xEnd, int yEnd, t_EnvironmentPtr environment)
{
  if (sg_pathLines != NULL) {
    ResetEnvironment(environment);
  }

  DebugCodeDetail (
	     printf("PartialPathSearch : Searching Between x %d y %d to x %d y %d\n", xStart, yStart, xEnd, yEnd);
	     fflush(stdout);
	     );
  if (AstarInlocal(xStart, yStart, xEnd, yEnd, environment) == 0) {
    DebugCodeDetail (
	       printf("PartialPathSearch : Search Between x %d y %d to x %d y %d is wrong\n", xStart, yStart, xEnd, yEnd);
	       fflush(stdout);
	       );
    sg_pathLines = FreePathLine(sg_pathLines);
    return 0;
  }
  sg_pathLines = StorePathLine(xEnd, yEnd, sg_pathLines, environment);
  return 1;
}

static t_PathLineMemsPtr
AppendLine(t_PathLineMemsPtr pathLineFirst, t_PathLineMemsPtr pathLineSecond)
{
  t_PathLineMemsPtr linePrev;

  DebugCodeDetail (
	     static int i = 1;
	     printf("AppendLine : This is for %d nd the first part of AppendLine\n", i);
	     PrintPathFinalPoints(pathLineFirst);
	     printf("AppendLine : This is for %d nd the second part of AppendLine\n", i++);
	     PrintPathFinalPoints(pathLineSecond);
	     fflush(stdout);
	     );
  if (pathLineFirst != NULL) {
    for (linePrev = pathLineFirst; linePrev->m_prevPtr != NULL; linePrev = linePrev->m_prevPtr) ;
    linePrev->m_prevPtr = pathLineSecond;
    return pathLineFirst;
  } else {
    return pathLineSecond;
  }
}

static t_PathLinesPtr
InitialFinalPathLines(void)
{
  t_PathLinesPtr finalPathLinesNew;

  finalPathLinesNew = Malloc(sizeof(struct t_PathLines));
  finalPathLinesNew->m_pointCounts = 0;
  finalPathLinesNew->m_pathPoints = NULL;
  return finalPathLinesNew;
}

static void
InsertNewFinalPathPoint(int x, int y, t_PathLinesPtr finalPathLines)
{
  t_PathPointPtr finalPointNew;

  finalPointNew = Malloc(sizeof(struct t_PathPoint));
  finalPointNew->m_lon = x;
  finalPointNew->m_lat = y;
  finalPointNew->m_next = finalPathLines->m_pathPoints;
  finalPathLines->m_pathPoints = finalPointNew;
  finalPathLines->m_pointCounts++;
}

static int
GetFinalPathPointX(t_PathPointPtr finalPathPoint)
{
  return finalPathPoint->m_lon;
}

static int
GetFinalPathPointY(t_PathPointPtr finalPathPoint)
{
  return finalPathPoint->m_lat;
}

static void
SetFinalPathPointX(int x, t_PathPointPtr finalPathPoint)
{
  finalPathPoint->m_lon = x;
}

static void
SetFinalPathPointY(int y, t_PathPointPtr finalPathPoint)
{
  finalPathPoint->m_lat = y;
}
