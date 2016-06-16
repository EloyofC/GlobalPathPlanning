/*
  Description: This file is about the declarition and functions about the pretreatment of pathplanning such as transform file to map, transform map to Environment model.
  Author: Green
  Date: 15/12/15
*/

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

#include "GetChangeEnv.h"
#include "PathAndShow.h"
#include "PublicFun.h"

#define c_neighbourNum 4

enum e_States{States_unvisited, States_alive, States_dead};
enum e_Obstacle{Obstacle_obstacle, Obstacle_free, Obstacle_edge};

struct t_EnvMember
{
  int m_xIndex;
  int m_yIndex;
  int m_cost;
  int m_priority;
  int m_flag;/* a m_flag for some function who need it  */
  enum e_Obstacle m_obstacle;
  enum e_States m_state;
  t_EnvironmentMemberPtr m_prevPtr;
};

struct t_EnvironmentInfo
{
  int m_envLength;
  int m_envWidth;
  int m_lengthOfUnit;
  int m_widthOfUnit;
  int m_xStart;
  int m_yStart;
  int m_xEnd;
  int m_yEnd;
  int m_topLeftX;
  int m_topleftY;
  int m_bottomRightX;
  int m_bottomRightY;
  t_EnvironmentMemberPtr m_envMembersPtr;
};

typedef struct t_AdaptPoint	/* just convert the index of the gps cor and add the first point to the end of the link */
{
  int m_xIndex;
  int m_yIndex;
  struct t_AdaptPoint * m_next;
} *t_AdaptPointPtr;


static void DrawEnvironmentPicture(t_EnvironmentPtr environment);
static int CalGpsPointYInEnv(t_PointCorPtr obstaclePointt, t_EnvironmentPtr environment);
static int CalGpsPointXInEnv(t_PointCorPtr obstaclePointt, t_EnvironmentPtr environment);
static t_AdaptPointPtr InitialAdaptPoint(int x, int y);
static t_AdaptPointPtr InsertAdaptPoint(int x, int y, t_AdaptPointPtr point);
static void FreeAdaptPoints(t_AdaptPointPtr points);
static t_AdaptPointPtr GetNextAdaptPoint(t_AdaptPointPtr point);
static int GetAdaptPointY(t_AdaptPointPtr point);
static int GetAdaptPointX(t_AdaptPointPtr point);
static void BoundaryFillFourInEnv(int xStart, int yStart, t_EnvironmentPtr environment);
static void BoundaryFillInEnv(int xStart, int yStart, int neighbourNum, t_EnvironmentPtr environment);
static void SetEnvMemberEdgeIfPossible(int x, int y, t_EnvironmentPtr environment);
static void DrawVerticalInEnv(int xStart, int yStart, int xEnd, int yEnd, t_EnvironmentPtr environment);
static void DrawHorizonInEnv(int xStart, int yStart, int xEnd, int yEnd, t_EnvironmentPtr environment);
static void DrawFUBUInEnv(int xStart, int yStart, int xEnd, int yEnd, int forwardBack, double gapRatio, t_EnvironmentPtr environment);
static void DrawBackUpInEnv(int xStart, int yStart, int xEnd, int yEnd, double gapRatio, t_EnvironmentPtr environment);
static void DrawForwardUpInEnv(int xStart, int yStart, int xEnd, int yEnd, double gapRatio, t_EnvironmentPtr environment);
static void DrawDiagonalInEnv(int xPointFirst, int yPointFirst, int xPointSecond, int yPointSecond, t_EnvironmentPtr environment);
static void DrawLineWithTwoPointsInEnv(t_AdaptPointPtr pointFirst, t_AdaptPointPtr pointSecond, t_EnvironmentPtr environment);
static void DrawLineInEnv(t_AdaptPointPtr points, t_EnvironmentPtr environment);
static t_AdaptPointPtr AdjustAndInsertAdapt(t_PointCorPtr obstaclePoint, t_EnvironmentPtr environment, t_AdaptPointPtr adaptPoints);
static t_AdaptPointPtr GetAdaptPoints(t_SingleObstaclePtr singleObstacle, t_EnvironmentPtr environment);
static void RevertEnvMemberObstacleState(t_EnvironmentMemberPtr member);
static void RevertRightColorExceptEdge(int x, int y, int length, t_EnvironmentPtr environment);
static void AreaFillWithScanSingleLine(int widthIndex, t_EnvironmentPtr environment);
static void AreaFillWithScanLine(t_EnvironmentPtr environment);
static void GetAdaptStartXY(int *xStart, int *yStart, t_AdaptPointPtr adaptPoints);
static void SetSingleObstacleInEnvironment(t_SingleObstaclePtr singleObstacle, t_EnvironmentPtr environment);
static t_EnvironmentPtr InitialEnvWithCell(int length, int width, int lengthOfUnit, int widthOfUnit, int xTopLeft, int yTopLeft, int xBottomRight, int yBottomRight);
static t_EnvironmentPtr CreateEnvironment(int length, int width);
static void InitialEnvironment(int length, int width, int xTopLeft, int yTopLeft, int xBottomRight, int yBottomRight, int lengthOfUnit, int widthOfUnit,  t_EnvironmentPtr environment);
static int CalRectangleIndex(int xIndex, int yIndex, int length, int width);
static void ResetEnvironmentMember(int xIndex, int yIndex, t_EnvironmentPtr environment);
static void PrintEnvironmentMember(int xIndex, int yIndex, t_EnvironmentPtr environment);
static void InitialEnvironmentMember(int xIndex, int yIndex, t_EnvironmentPtr environment);

static void SetEnvMemberEdge(t_EnvironmentMemberPtr member);
static void SetEnvMemberFree(t_EnvironmentMemberPtr member);
static int IsEnvMemberEdge(t_EnvironmentMemberPtr member);
static int IsEnvMemberFree(t_EnvironmentMemberPtr member);

static int GetObstaclesCounts(t_ObstaclesPtr obstacles);
static t_SingleObstaclePtr GetObstacleArray(t_ObstaclesPtr obstacles);
static t_SingleObstaclePtr GetSingleObstacle(t_SingleObstaclePtr obstacles, int index);
static int GetVertexCounts(t_SingleObstaclePtr singleObstacle);
static t_PointCorPtr GetPointArray(t_SingleObstaclePtr singleObstacle);
static t_PointCorPtr GetSinglePoint( t_PointCorPtr pointArray,int index);
static int GetPointCorLon(t_PointCorPtr point);
static int GetPointCorLat(t_PointCorPtr point);
static void PrintAdaptPoints(t_AdaptPointPtr points);



t_EnvironmentPtr
InitialEnvWithGps(int lengthOfUnit, int widthOfUnit, int lonTopLeft, int latTopLeft, int lonBottomRight, int latBottomRight)
{
  int length, width;
  t_EnvironmentPtr environmentNew;

  assert(lonTopLeft <= lonBottomRight); /* confirm the relation */
  assert(latTopLeft >= latBottomRight); /* need to confirm the number range */

  length = CalGpsDistanceLon(lonTopLeft, latTopLeft, lonBottomRight, latBottomRight);
  width = CalGpsDistanceLat(lonTopLeft, latTopLeft, lonBottomRight, latBottomRight);

  DebugCode (
	     printf("Length is %d, and Width is %d\n", length, width);
	     fflush(stdout);
	     );

  environmentNew = InitialEnvWithCell(length, width, lengthOfUnit, widthOfUnit, lonTopLeft, latTopLeft, lonBottomRight, latBottomRight);

  return environmentNew;
}

void
SetObstaclesInEnvironment(t_ObstaclesPtr obstacles, t_EnvironmentPtr newEnvironment)
{
  int i, obstacleCounts;
  t_SingleObstaclePtr singleObstacle;

  if (obstacles == NULL) {
    return;
  }

  obstacleCounts = GetObstaclesCounts(obstacles);
  singleObstacle = GetObstacleArray(obstacles);
  for (i=0; i < obstacleCounts; i++) {
    SetSingleObstacleInEnvironment(GetSingleObstacle(singleObstacle, i), newEnvironment);
  }
  return;
}

/* core algorithm for set environment */
static void
SetSingleObstacleInEnvironment(t_SingleObstaclePtr singleObstacle, t_EnvironmentPtr environment)
{
  t_AdaptPointPtr adaptPoints;
  int xStart, yStart;
  assert(singleObstacle != NULL);

  adaptPoints = GetAdaptPoints(singleObstacle, environment);
  GetAdaptStartXY(&xStart, &yStart, adaptPoints);
  DrawLineInEnv(adaptPoints, environment);
  DebugCode(
	    PrintAdaptPoints(adaptPoints);
	    printf("SetSingleObstacleInEnvironment : xStart %d yStart %d\n", xStart, yStart);
	    );
  FreeAdaptPoints(adaptPoints);
  BoundaryFillInEnv(xStart, yStart, c_neighbourNum, environment);
  DebugCode(
	    PrintEnvironment(environment);
	    DrawEnvironmentPicture(environment);
	    );
}

static void
GetAdaptStartXY(int *xStart, int *yStart, t_AdaptPointPtr adaptPoints)
{
  int i;
  double xMedium, yMedium;
  t_AdaptPointPtr currentPoint;

  xMedium = yMedium = 0;
  for (currentPoint = adaptPoints, i = 0; currentPoint != NULL; currentPoint = GetNextAdaptPoint(currentPoint), i++) {
    xMedium += GetAdaptPointX(currentPoint);
    yMedium += GetAdaptPointY(currentPoint);
  }
  *xStart = xMedium / i;
  *yStart = yMedium / i;
}

static void
AreaFillWithScanLine(t_EnvironmentPtr environment)
{
  int widthIndex, width;

  width = GetEnvWidth(environment);
  for (widthIndex = 0; widthIndex < width; widthIndex++) {
    AreaFillWithScanSingleLine(widthIndex, environment);
  }
}

static void
AreaFillWithScanSingleLine(int widthIndex, t_EnvironmentPtr environment)
{
  int lengthIndex, length;

  length = GetEnvLength(environment);
  for (lengthIndex = 0; lengthIndex < length; lengthIndex++) {
    if (IsEnvMemberEdge(GetEnvMember(lengthIndex, widthIndex, environment))) {
      RevertRightColorExceptEdge(lengthIndex, widthIndex, length, environment);
    }
  }
}

static void
RevertRightColorExceptEdge(int x, int y, int length, t_EnvironmentPtr environment)
{
  int i;
  t_EnvironmentMemberPtr member;

  for (i = x; i < length; i++) {
    member = GetEnvMember(i, y, environment);
    if (!IsEnvMemberEdge(member)) {
      RevertEnvMemberObstacleState(member);
    }
  }
}

static void
RevertEnvMemberObstacleState(t_EnvironmentMemberPtr member)
{
  assert(!IsEnvMemberEdge(member));

  if (IsEnvMemberFree(member)) {
    SetEnvMemberObstacle(member);
  } else {
    SetEnvMemberFree(member);
  }
}

/* to ensure the closure of the obstacle if the two crosspoint is in two different lines get the corner vertex into the pointsLink */
static t_AdaptPointPtr
GetAdaptPoints(t_SingleObstaclePtr singleObstacle, t_EnvironmentPtr environment)
{
  t_PointCorPtr pointArray, firstPoint, currentPoint;
  int i, pointCounts;
  t_AdaptPointPtr adaptPoints = NULL;

  assert(singleObstacle != NULL);
  pointCounts = GetVertexCounts(singleObstacle);
  pointArray = GetPointArray(singleObstacle);
  firstPoint = GetSinglePoint(pointArray, 0);
  for (i = 0; i < pointCounts; i++) {
    currentPoint = GetSinglePoint(pointArray, i);
    adaptPoints = AdjustAndInsertAdapt(currentPoint, environment, adaptPoints);
  } /* add the first one to the end */
  adaptPoints = AdjustAndInsertAdapt(firstPoint, environment, adaptPoints);
  return adaptPoints;
}

static t_AdaptPointPtr
AdjustAndInsertAdapt(t_PointCorPtr obstaclePoint, t_EnvironmentPtr environment, t_AdaptPointPtr adaptPoints)
{
  int x, y;
  t_AdaptPointPtr adaptPointsNew;

  x = CalGpsPointXInEnv(obstaclePoint, environment);
  y = CalGpsPointYInEnv(obstaclePoint, environment);
  adaptPointsNew = InsertAdaptPoint(x, y, adaptPoints);
  assert(adaptPointsNew != NULL);
  return adaptPointsNew;
}

/* This series of functions is to set the edge in the environment */
static void
DrawLineInEnv(t_AdaptPointPtr points, t_EnvironmentPtr environment)
{
  t_AdaptPointPtr pointNext, pointCurrent;

  pointCurrent = points;
  for (; pointCurrent != NULL; pointCurrent = pointNext) {
    pointNext = GetNextAdaptPoint(pointCurrent);
    if (pointNext != NULL) {
      DrawLineWithTwoPointsInEnv(pointCurrent, pointNext, environment);
    }
  }
}

static void
DrawLineWithTwoPointsInEnv(t_AdaptPointPtr pointFirst, t_AdaptPointPtr pointSecond, t_EnvironmentPtr environment)
{
  int xPointFirst, yPointFirst, xPointSecond, yPointSecond;

  xPointFirst = GetAdaptPointX(pointFirst);
  yPointFirst = GetAdaptPointY(pointFirst);
  xPointSecond = GetAdaptPointX(pointSecond);
  yPointSecond = GetAdaptPointY(pointSecond);
  DebugCodeDetail(
		  printf("DrawLineWithTwoPointsInEnv : x1 %d y1 %d x2 %d y2 %d\n", xPointFirst, yPointFirst, xPointSecond, yPointSecond);
		  );
  /* to confirm the obstacle is in the width we assert the following my need to convert to if  */
  assert(GetEnvLength(environment) >= xPointFirst && xPointFirst >= 0);
  assert(GetEnvWidth(environment) >= yPointFirst && yPointFirst >= 0);
  assert(GetEnvLength(environment) >= xPointSecond && xPointSecond >= 0);
  assert(GetEnvWidth(environment) >= yPointSecond && yPointSecond >= 0);
  if (xPointFirst == xPointSecond) {
    DrawVerticalInEnv(xPointFirst, yPointFirst, xPointSecond, yPointSecond, environment);
  } else if (yPointFirst == yPointSecond) {
    DrawHorizonInEnv(xPointFirst, yPointFirst, xPointSecond, yPointSecond, environment);
  } else {
    DrawDiagonalInEnv(xPointFirst, yPointFirst, xPointSecond, yPointSecond, environment);
  }
}

static void
DrawDiagonalInEnv(int xPointFirst, int yPointFirst, int xPointSecond, int yPointSecond, t_EnvironmentPtr environment)
{
  int xDistance, yDistance;
  double gapRatio;

   xDistance = SimpleIntAbs(xPointFirst - xPointSecond);
   yDistance = SimpleIntAbs(yPointFirst - yPointSecond);
   gapRatio = (double)yDistance / (double)xDistance;
   DebugCodeDetail(
		   printf("DrawDiagonalInEnv : xDis %d yDis %d gapRation %f\n", xDistance, yDistance, gapRatio);
		   );
   /*
     1. should ensure the y is getting bigger; so we divide into four conditions with the consideration of the relation of A and B
     2. remember that the zero point is in the topleft corner -- that is the gps cor relationship --unrelated to this problem
     3. should use the gapRatio in according to the relation length of A and B
   */
   if (xPointFirst < xPointSecond) {
     if (yPointFirst < yPointSecond) { /* A is in the leftdown of the B */
       DrawForwardUpInEnv(xPointFirst, yPointFirst, xPointSecond, yPointSecond, gapRatio, environment);
     } else {			/* A is in the leftup of the B */
       DrawBackUpInEnv(xPointSecond, yPointSecond, xPointFirst, yPointFirst, gapRatio, environment);
     }
   } else {
     if (yPointFirst < yPointSecond) { /* A is in the rightdown of the B */
       DrawBackUpInEnv(xPointFirst, yPointFirst, xPointSecond, yPointSecond, gapRatio, environment);
     } else {			/* A is in the rightup of the B */
       DrawForwardUpInEnv(xPointSecond, yPointSecond, xPointFirst, yPointFirst, gapRatio, environment);
     }
   }
}

static void
DrawForwardUpInEnv(int xStart, int yStart, int xEnd, int yEnd, double gapRatio, t_EnvironmentPtr environment)
{
  DrawFUBUInEnv(xStart, yStart, xEnd, yEnd, 1, gapRatio, environment);
}

static void
DrawBackUpInEnv(int xStart, int yStart, int xEnd, int yEnd, double gapRatio, t_EnvironmentPtr environment)
{
  DrawFUBUInEnv(xStart, yStart, xEnd, yEnd, -1, gapRatio, environment);
}

static void
DrawFUBUInEnv(int xStart, int yStart, int xEnd, int yEnd, int forwardBack, double gapRatio, t_EnvironmentPtr environment)
{
  int i, j, xIndex, yIndex, yPrev;
  double gap;

  yIndex = yPrev = yStart;
  i = gap = 0;
  xIndex = xStart;
  /* need a bandage for the situation of that the gapratio is bigger than 2 */
  for (; yIndex < yEnd; xIndex += forwardBack, i++) {
    DebugCodeDetail (
		     printf("DrawFUBUInEnv : x %d y %d gap %f\n", xIndex, yIndex, gapRatio);
		     );
    gap += gapRatio;
    yPrev = yIndex;
    yIndex = yStart + gap;
    for (j = yPrev + 1; j < yIndex; j++) {
      SetEnvMemberEdgeIfPossible(xIndex, j, environment);
    }
    SetEnvMemberEdgeIfPossible(xIndex, yIndex, environment);
  } /* must ensure the Node Point */
   SetEnvMemberEdgeIfPossible(xEnd, yEnd, environment);
}

static void
DrawHorizonInEnv(int xStart, int yStart, int xEnd, int yEnd, t_EnvironmentPtr environment)
{
  int xIndex;

  if (xStart > xEnd) {
    SwapNum(&xStart, &xEnd);
  }
  DebugCodeDetail(
		  printf("DrawHorizonInEnv : xStart %d xEnd %d y %d\n", xStart, xEnd, yStart);
		  );
  for (xIndex = xStart; xIndex <= xEnd; xIndex++) { /* this should include the end point */
    SetEnvMemberEdgeIfPossible(xIndex, yStart, environment);
  }
}

static void
DrawVerticalInEnv(int xStart, int yStart, int xEnd, int yEnd, t_EnvironmentPtr environment)
{
  int yIndex;

  if (yStart > yEnd) {
    SwapNum(&yStart, &yEnd);
  }
  DebugCodeDetail(
		  printf("DrawVerticalInEnv : x %d yStart %d yEnd %d\n", xStart, yStart, yEnd);
		  );
  for (yIndex = yStart; yIndex <= yEnd; yIndex++) { /* this should include the end point */
    SetEnvMemberEdgeIfPossible(xStart, yIndex, environment);
  }
}

static void
SetEnvMemberEdgeIfPossible(int x, int y, t_EnvironmentPtr environment)
{
  if (IsEnvPointValid(x, y, environment)) {
    DebugCodeDetail(
		    printf("SetEnvMemberEdgeIfPossible : x %d y %d\n", x, y);
		    );
    SetEnvMemberEdge(GetEnvMember(x, y, environment));
  }
}

static void
BoundaryFillInEnv(int xStart, int yStart, int neighbourNum, t_EnvironmentPtr environment)
{
  assert(neighbourNum == 4);
  assert(IsEnvPointValid(xStart, yStart, environment) && !IsEnvMemberObstacle(GetEnvMember(xStart, yStart, environment)));
  BoundaryFillFourInEnv(xStart, yStart, environment);
}

static void
BoundaryFillFourInEnv(int xStart, int yStart, t_EnvironmentPtr environment)
{
  int i;
  int x[4] = {-1, 1, 0, 0};
  int y[4] = {0, 0, -1, 1};

  if (IsEnvPointValid(xStart, yStart, environment) && !IsEnvMemberObstacle(GetEnvMember(xStart, yStart, environment))) {
    SetEnvMemberObstacle(GetEnvMember(xStart, yStart, environment));
    for (i=0; i < sizeof(x)/sizeof(int); i++) {
      BoundaryFillFourInEnv(xStart + x[i], yStart + y[i], environment);
    }
  }
}

static t_PointCorPtr
GetPointArray(t_SingleObstaclePtr singleObstacle)
{
  return singleObstacle->m_pointsPtr;
}

static int
GetPointCorLon(t_PointCorPtr point)
{
  return point->m_lon;
}

static int
GetPointCorLat(t_PointCorPtr point)
{
  return point->m_lat;
}

static int
GetObstaclesCounts(t_ObstaclesPtr obstacles)
{
  return obstacles->m_obstacleCounts;
}

static t_SingleObstaclePtr
GetObstacleArray(t_ObstaclesPtr obstacles)
{
  return obstacles->m_obstacleMembersPtr;
}

static t_SingleObstaclePtr
GetSingleObstacle(t_SingleObstaclePtr singleObstacle, int index)
{
  return singleObstacle + index;
}

static int
GetVertexCounts(t_SingleObstaclePtr singleObstacle)
{
  return singleObstacle->m_vertexCounts;
}

static t_PointCorPtr
GetSinglePoint(t_PointCorPtr pointArray, int index)
{
  return pointArray + index;
}

/* Assume the relation between cordinatation of the environment and index is top and left to 0 and right and down to 1 */
static t_EnvironmentPtr
CreateEnvironment(int length, int width)
{
  t_EnvironmentPtr p;

  p = Malloc(sizeof(struct t_EnvironmentInfo));
  p->m_envMembersPtr = Malloc(sizeof(struct t_EnvMember) * length * width);

  return p;
}

static void
InitialEnvironment(int length, int width, int xTopLeft, int yTopLeft, int xBottomRight, int yBottomRight, int lengthOfUnit, int widthOfUnit, t_EnvironmentPtr environment)
{
  t_EnvironmentPtr p;

  p = environment;
  p->m_envLength = length;
  p->m_envWidth = width;
  p->m_lengthOfUnit = lengthOfUnit;
  p->m_widthOfUnit = widthOfUnit;
  p->m_xStart = 0;
  p->m_yStart = 0;
  p->m_xEnd = 0;
  p->m_yEnd = 0;
  p->m_topLeftX = xTopLeft;
  p->m_topleftY = yTopLeft;
  p->m_bottomRightX = xBottomRight;
  p->m_bottomRightY = yBottomRight;
}

void
DeleteEnvironment(t_EnvironmentPtr environment)
{
  t_EnvironmentMemberPtr p;

  p = environment->m_envMembersPtr;
  free(p);
  free(environment);
}

/* If the size of environment is not equal length * width, the rest will in the last row and col, it means the last will be big
 and the TopLeft and BottomRight index is gps data */
t_EnvironmentPtr
InitialEnvWithCell(int length, int width, int lengthOfUnit, int widthOfUnit, int xTopLeft, int yTopLeft, int xBottomRight, int yBottomRight)
{
  int i, j;
  int xIndex, yIndex;
  t_EnvironmentPtr environment;

  xIndex = length / lengthOfUnit + 1; /* need to thinkthrough */
  yIndex = width / widthOfUnit + 1;   /* need to plus 1 or not? */

  environment = CreateEnvironment(xIndex, yIndex);
  InitialEnvironment(xIndex, yIndex, xTopLeft, yTopLeft, xBottomRight, yBottomRight, lengthOfUnit, widthOfUnit, environment);
  for (i = 0; i < xIndex; i++)
    for (j = 0; j < yIndex; j++)
      InitialEnvironmentMember(i, j, environment);

  return environment;
}

static void
InitialEnvironmentMember(int xIndex, int yIndex, t_EnvironmentPtr environment)
{
  t_EnvironmentMemberPtr member;

  member = GetEnvMember(xIndex, yIndex, environment);
  member->m_xIndex = xIndex;
  member->m_yIndex = yIndex;
  member->m_prevPtr = NULL;
  member->m_cost = 0;
  member->m_priority = 0;
  member->m_flag = 0;
  member->m_state = States_unvisited;
  member->m_obstacle = Obstacle_free;
}

static void
ResetEnvironmentMember(int xIndex, int yIndex, t_EnvironmentPtr environment)
{
  t_EnvironmentMemberPtr member;

  member = GetEnvMember(xIndex, yIndex, environment);
  member->m_prevPtr = NULL;
  member->m_cost = 0;
  member->m_priority = 0;
  member->m_flag = 0;
  member->m_state = States_unvisited;
}

static void
PrintEnvironmentMember(int xIndex, int yIndex, t_EnvironmentPtr environment)
{
  t_EnvironmentMemberPtr member;

  member = GetEnvMember(xIndex, yIndex, environment);
  printf("PrintEnvironmentMember : The EnvironmentMember of x %d y %d is ", xIndex, yIndex);
  if (IsEnvMemberObstacle(member))
    printf("with obstacle\n");
  else
    printf("no obstacle\n");
}

void
PrintEnvironment(t_EnvironmentPtr environment)
{
  int i,j;

  printf("PrintEnvironment : The Environment with Length:%d, Width:%d\n", environment->m_envLength, environment->m_envWidth);
  for (i=0; i < environment->m_envLength; i++)
    for (j=0; j < environment->m_envWidth; j++)
      PrintEnvironmentMember(i, j, environment);
}

/* may need to take care of the off-by-one errro */
void
ResetEnvironment(t_EnvironmentPtr environment)
{
  int i,j;

  for (i=0; i < environment->m_envLength; i++)
    for (j=0; j < environment->m_envWidth; j++)
      ResetEnvironmentMember(i, j, environment);
}

static int
CalRectangleIndex(int x, int y, int length, int width)
{
  return x + y * length;
}

t_EnvironmentMemberPtr
GetEnvMember(int xIndex, int yIndex, t_EnvironmentPtr environment)
{
  t_EnvironmentMemberPtr member;
  /* may need to add something to confirm the validation of environment and the index */
  assert(environment != NULL);
  member = environment->m_envMembersPtr + CalRectangleIndex(xIndex, yIndex, environment->m_envLength, environment->m_envWidth);
  assert(environment != NULL);
  return member;
}

int
IsEnvMemberObstacle(t_EnvironmentMemberPtr member)
{
  return (member->m_obstacle == Obstacle_obstacle) || (member->m_obstacle == Obstacle_edge);
}

static int
IsEnvMemberEdge(t_EnvironmentMemberPtr member)
{
  return member->m_obstacle == Obstacle_edge;
}
static void
SetEnvMemberEdge(t_EnvironmentMemberPtr member)
{
  member->m_obstacle = Obstacle_edge;
}

static int
IsEnvMemberFree(t_EnvironmentMemberPtr member)
{
  return member->m_obstacle == Obstacle_free;
}

static void
SetEnvMemberFree(t_EnvironmentMemberPtr member)
{
  member->m_obstacle = Obstacle_free;
}

int
IsEnvPointValid(int xIndex, int yIndex, t_EnvironmentPtr environment)
{
  if (xIndex >= 0 && xIndex < environment->m_envLength && yIndex >= 0 && yIndex < environment->m_envWidth) {
    return 1;
  } else {
    return 0;
  }
}

int
GetEnvLength(t_EnvironmentPtr environment)
{
  return environment->m_envLength;
}

int
GetEnvWidth(t_EnvironmentPtr environment)
{
  return environment->m_envWidth;
}

int
GetEnvStartX(t_EnvironmentPtr environment)
{
  return environment->m_xStart;
}

int
GetEnvEndX(t_EnvironmentPtr environment)
{
  return environment->m_xEnd;
}

int
GetEnvStartY(t_EnvironmentPtr environment)
{
  return environment->m_yStart;
}

int
GetEnvEndY(t_EnvironmentPtr environment)
{
  return environment->m_yEnd;
}

int
IsEnvMemberFlagNotSet(t_EnvironmentMemberPtr member)
{
  return member->m_flag == 0;
}

void
ResetEnvAllFlag(t_EnvironmentPtr environment)
{
  int i, j;
  t_EnvironmentMemberPtr member;

  for (i=0; i < environment->m_envLength; i++)
    for (j=0; j < environment->m_envWidth; j++) {
      member = GetEnvMember(i, j, environment);
      member->m_flag = 0;
    }
}

void
SetEnvMemberFlag(t_EnvironmentMemberPtr member)
{
  member->m_flag = 1;
}

int
GetEnvMemberX(t_EnvironmentMemberPtr member)
{
  return member->m_xIndex;
}

int
GetEnvMemberY(t_EnvironmentMemberPtr member)
{
  return member->m_yIndex;
}

int
GetEnvMemberCost(t_EnvironmentMemberPtr member)
{
  return member->m_cost;
}

int
GetEnvMemberPriority(t_EnvironmentMemberPtr member)
{
  return member->m_priority;
}

t_EnvironmentMemberPtr
GetEnvMemberPrev(t_EnvironmentMemberPtr member)
{
  return member->m_prevPtr;
}

int
GetEnvTopLeftX(t_EnvironmentPtr environment)
{
  return environment->m_topLeftX;
}

int
GetEnvTopLeftY(t_EnvironmentPtr environment)
{
  return environment->m_topleftY;
}

int
GetEnvBottomRightX(t_EnvironmentPtr environment)
{
  return environment->m_bottomRightX;
}

int
GetEnvBottomRightY(t_EnvironmentPtr environment)
{
  return environment->m_bottomRightY;
}

void
SetEnvStartX(int xStart, t_EnvironmentPtr environment)
{
  environment->m_xStart= xStart;
}

void
SetEnvStartY(int yStart, t_EnvironmentPtr environment)
{
  environment->m_yStart = yStart;
}

void
SetEnvEndX(int xEnd, t_EnvironmentPtr environment)
{
  environment->m_xEnd = xEnd;
}

void
SetEnvEndY(int yEnd, t_EnvironmentPtr environment)
{
  environment->m_yEnd = yEnd;
}

void
SetEnvStartAndEnd(int xStart, int yStart, int xEnd, int yEnd, t_EnvironmentPtr environment)
{
  SetEnvStartX(xStart, environment);
  SetEnvStartY(yStart, environment);
  SetEnvEndX(xEnd, environment);
  SetEnvEndY(yEnd, environment);
}

void
SetEnvMemberCost(int cost, t_EnvironmentMemberPtr member)
{
  member->m_cost = cost;
}

void
SetEnvMemberPriority(int priority, t_EnvironmentMemberPtr member)
{
  member->m_priority = priority;
}

void
SetEnvMemberDead(t_EnvironmentMemberPtr member)
{
  member->m_state = States_dead;
}

void
SetEnvMemberAlive(t_EnvironmentMemberPtr member)
{
  member->m_state = States_alive;
}

void
SetEnvMemberObstacle(t_EnvironmentMemberPtr member)
{
  member->m_obstacle = Obstacle_obstacle;
}


void
SetEnvMemberPrev(t_EnvironmentMemberPtr memberPrev, t_EnvironmentMemberPtr member)
{
  member->m_prevPtr = memberPrev;
}

/* IsEnvMemLegal : means the member is in environment and is unvisted */
int
IsEnvMemberLegal(int xIndex, int yIndex, t_EnvironmentPtr environment)
{
  return IsEnvMemberValid(xIndex, yIndex, environment) && (!IsEnvMemberDead(GetEnvMember(xIndex, yIndex, environment)));
}

int
IsEnvMemberValid(int xIndex, int yIndex, t_EnvironmentPtr environment)
{
  return xIndex >= 0 && xIndex < GetEnvLength(environment) && yIndex >= 0 && yIndex < GetEnvWidth(environment);
}

int
IsSearchEnd(t_EnvironmentMemberPtr member, t_EnvironmentPtr environment)
{
  return (member->m_xIndex == GetEnvEndX(environment)) && (member->m_yIndex == GetEnvEndY(environment));
}

int
IsEnvMemberUnvisited(t_EnvironmentMemberPtr member)
{
  return member->m_state == States_unvisited;
}

int
IsEnvMemberAlive(t_EnvironmentMemberPtr member)
{
  return member->m_state == States_alive;
}

int
IsEnvMemberDead(t_EnvironmentMemberPtr member)
{
  return member->m_state == States_dead;
}

static int
GetAdaptPointX(t_AdaptPointPtr point)
{
  return point->m_xIndex;
}

static int
GetAdaptPointY(t_AdaptPointPtr point)
{
  return point->m_yIndex;
}

static t_AdaptPointPtr
GetNextAdaptPoint(t_AdaptPointPtr point)
{
  return point->m_next;
}

static t_AdaptPointPtr
InsertAdaptPoint(int x, int y, t_AdaptPointPtr point)
{
  t_AdaptPointPtr pointNew;

  pointNew = InitialAdaptPoint(x, y);
  pointNew->m_next = point;
  return pointNew;
}

static void
FreeAdaptPoints(t_AdaptPointPtr points)
{
  t_AdaptPointPtr pointNext;

  for (; points != NULL; points = pointNext) {
    pointNext = points->m_next;
    free(points);
  }
}

static t_AdaptPointPtr
InitialAdaptPoint(int x, int y)
{
  t_AdaptPointPtr pointNew;

  pointNew = Malloc(sizeof(struct t_AdaptPoint));
  pointNew->m_xIndex = x;
  pointNew->m_yIndex = y;
  pointNew->m_next = NULL;
  return pointNew;
}

static void
PrintAdaptPoints(t_AdaptPointPtr points)
{
  t_AdaptPointPtr pointNext;

  for (; points != NULL; points = pointNext) {
    pointNext = points->m_next;
    printf("PrintAdaptPoints : x %d y %d\n", GetAdaptPointX(points), GetAdaptPointY(points));
  }
}
/* can be < 0 number */
static int
CalGpsPointXInEnv(t_PointCorPtr obstaclePoint, t_EnvironmentPtr environment)
{
  int lonPoint, latPoint, disLon, topLeftX, topLeftY;
  int lengthOfUnit;

  lengthOfUnit = GetEnvLengthOfUnit(environment);
  lonPoint = GetPointCorLon(obstaclePoint);
  latPoint = GetPointCorLat(obstaclePoint);
  topLeftX = GetEnvTopLeftX(environment);
  topLeftY =  GetEnvTopLeftY(environment);
  disLon = CalGpsDistanceLon(lonPoint, latPoint, topLeftX, topLeftY);
  if (lonPoint < topLeftX) {
    disLon = -1 *disLon;
  }
  return disLon / lengthOfUnit;
}

static int
CalGpsPointYInEnv(t_PointCorPtr obstaclePoint, t_EnvironmentPtr environment)
{
  int lonPoint, latPoint, disLat, topLeftX, topLeftY;
  int widthOfUnit;

  widthOfUnit = GetEnvWidthOfUnit(environment);
  lonPoint = GetPointCorLon(obstaclePoint);
  latPoint = GetPointCorLat(obstaclePoint);
  topLeftX = GetEnvTopLeftX(environment);
  topLeftY =  GetEnvTopLeftY(environment);
  disLat = CalGpsDistanceLat(lonPoint, latPoint, topLeftX, topLeftY);
  if (latPoint > topLeftY) {	/* if y is > 0 it should < the topleft */
    disLat = -1 *disLat;
  }
  return disLat / widthOfUnit;
}

int
GetEnvLengthOfUnit(t_EnvironmentPtr environment)
{
  return environment->m_lengthOfUnit;
}

int
GetEnvWidthOfUnit(t_EnvironmentPtr environment)
{
  return environment->m_widthOfUnit;
}

static void
DrawEnvironmentPicture(t_EnvironmentPtr environment)
{
  int i, j, length, width;

  length = GetEnvLength(environment);
  width = GetEnvWidth(environment);
  for (i = 0; i < length; i++) {
    for (j = 0; j < width; j++) {
      if (IsEnvMemberObstacle(GetEnvMember(i, j, environment))) {
	printf("*");
      } else {
	printf(" ");
      }
    }
    printf("\n");
  }
}
