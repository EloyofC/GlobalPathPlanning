#include <stdlib.h>
#include <stdio.h>
#include "GetChangeEnv.h"
#include "PathAndShow.h"
#include "PublicFun.h"
#include "ComPlan.h"

#define c_lengthOfUnit 100
#define c_widthOfUnit 100


static int GetFinalPointLon(t_PathPointPtr pathPoint);
static int GetFinalPointLat(t_PathPointPtr pathPoint);
static t_PathPointPtr GetFinalPointNext(t_PathPointPtr pathPoint);
static t_PathPointPtr GetFinalLinePoints(t_PathLinesPtr finalPathLines);
static int GetFinalLinePointCount(t_PathLinesPtr finalPathlines);


t_PathLinesPtr
DoCruiseGeneral(int lonStart, int latStart, int lonEnd, int latEnd, int lonTopLeft, int latTopLeft, int lonBottomRight, int latBottomRight, int width, t_ObstaclesPtr obstacles)
{
  int cellStartX, cellStartY, cellEndX, cellEndY, cellWidth; /* cellWidth is width convert to suite th cell */
  int endPointX, endPointY;
  int IsSearchSuccess;
  t_EnvironmentPtr newEnvironment;
  t_PathLinesPtr pathLines;

  cellStartX = CalGpsDistanceLon(lonTopLeft, latTopLeft, lonStart, latStart) / c_lengthOfUnit;
  cellStartY = CalGpsDistanceLat(lonTopLeft, latTopLeft, lonStart, latStart) / c_widthOfUnit;
  cellEndX =  CalGpsDistanceLon(lonTopLeft, latTopLeft, lonEnd, latEnd) / c_lengthOfUnit;
  cellEndY =  CalGpsDistanceLat(lonTopLeft, latTopLeft, lonEnd, latEnd) / c_widthOfUnit;
  cellWidth = width < c_widthOfUnit ? 1 : width / c_widthOfUnit;
  newEnvironment = InitialEnvWithGps(c_lengthOfUnit, c_widthOfUnit, lonTopLeft, latTopLeft, lonBottomRight, latBottomRight);
  SetObstaclesInEnvironment(obstacles, newEnvironment);

  endPointX = cellEndX - 1;	/* x need to be smaller, cos of end cal of multi and div */
  endPointY = cellEndY - 1;
  IsSearchSuccess = ScanSearch(cellStartX, cellStartY, endPointX, endPointY, cellWidth, newEnvironment);
  DebugCode(
 	    PrintEnvPathLine();
 	    );
  if (IsSearchSuccess) {
  pathLines = GetGpsPathLines(newEnvironment);
  DeleteEnvironment(newEnvironment);
  return pathLines;
  } else {
    DeleteEnvironment(newEnvironment);
    return NULL;
  }
}


static void
FreeFinalPathPoints(t_PathPointPtr pathPointHead)
{
  t_PathPointPtr pathPointTemp;

  for (; pathPointHead != NULL; pathPointHead = pathPointTemp) {
    pathPointTemp = pathPointHead->m_next;
    free(pathPointHead);
  }
}

void
FreeFinalPathLines(t_PathLinesPtr finalPathLines)
{
  FreeFinalPathPoints(GetFinalLinePoints(finalPathLines));
  free(finalPathLines);
}

static int
GetFinalLinePointCount(t_PathLinesPtr finalPathLines)
{
  return finalPathLines->m_pointCounts;
}

static t_PathPointPtr
GetFinalLinePoints(t_PathLinesPtr finalPathLines)
{
  return finalPathLines->m_pathPoints;
}

static int
GetFinalPointLon(t_PathPointPtr pathPoint)
{
  return pathPoint->m_lon;
}

static int
GetFinalPointLat(t_PathPointPtr pathPoint)
{
  return pathPoint->m_lat;
}

static t_PathPointPtr
GetFinalPointNext(t_PathPointPtr pathPoint)
{
  return pathPoint->m_next;
}

void
PrintGpsPathLines(t_PathLinesPtr finalPathLines)
{
  int i, count;
  t_PathPointPtr pathPoints;

  if (finalPathLines != NULL) {
    count = GetFinalLinePointCount(finalPathLines);
    pathPoints = GetFinalLinePoints(finalPathLines);
    for (i = 0; i < count; i++) {
      printf("The %d nd point is x %d y %d\n", i, GetFinalPointLon(pathPoints), GetFinalPointLat(pathPoints));
      pathPoints = GetFinalPointNext(pathPoints);
    }
  }
}
