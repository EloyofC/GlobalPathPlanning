#include <stdio.h>
#include "PathAndShow.h"

static t_ObstaclesPtr GetObstaclesInArea(int lonTopLeft, int latTopLeft, int lonBottomRight, int latBottomRight);
static void InsertPoint(int lon, int lat, t_PointCorPtr point);
static t_PointCorPtr CreatePointArray(int size);
static void SetSingleObstacleCount(int count, t_SingleObstaclePtr singleObstacle);
static t_SingleObstaclePtr CreateSingleObstacleArray(int size);
static t_ObstaclesPtr CreateObstacles(void);
static void SetObstaclesCount(int count, t_ObstaclesPtr obstacles);
static t_PointCorPtr GetNextPoint(t_PointCorPtr point);
static int TestSingleObstacle(void);
static int TestNoObstacle(void);
static int TestTemplate(int lonTopLeft, int latTopLeft, int lonBottomRight, int latBottomRight, int width, t_ObstaclesPtr obstacles);

int main(int argc, char *argv[])
{
  return TestNoObstacle();
}


static int
TestNoObstacle(void)
{
  int lonTopLeft, latTopLeft, lonBottomRight, latBottomRight, width;
  t_ObstaclesPtr obstacles;

  lonTopLeft = 1223012682;
  latTopLeft = 315632296;
  lonBottomRight = 1225999885;
  latBottomRight = 313792624;
  width = 50;
  printf("The start point is x %d y %d\n", lonTopLeft, latTopLeft);
  printf("The end point is x %d y %d\n", lonBottomRight, latBottomRight);
  obstacles = NULL;
  return TestTemplate(lonTopLeft, latTopLeft, lonBottomRight, latBottomRight, width, obstacles);
}
static int
TestSingleObstacle(void)
{
  int lonTopLeft, latTopLeft, lonBottomRight, latBottomRight, width;
  t_ObstaclesPtr obstacles;

  lonTopLeft = 1206480644;
  latTopLeft = 380044038;
  lonBottomRight = 1208053971;
  latBottomRight = 378817969;
  width = 50;
  printf("The start point is x %d y %d\n", lonTopLeft, latTopLeft);
  printf("The end point is x %d y %d\n", lonBottomRight, latBottomRight);
  obstacles = GetObstaclesInArea(lonTopLeft, latTopLeft, lonBottomRight, latBottomRight);
  return TestTemplate(lonTopLeft, latTopLeft, lonBottomRight, latBottomRight, width, obstacles);
}

static int
TestTemplate(int lonTopLeft, int latTopLeft, int lonBottomRight, int latBottomRight, int width, t_ObstaclesPtr obstacles)
{
  t_PathLinesPtr pathLines;

  pathLines = DoCruiseGeneral(lonTopLeft, latTopLeft, lonBottomRight, latBottomRight, lonTopLeft, latTopLeft, lonBottomRight, latBottomRight, width, obstacles);
  if (pathLines != NULL) {
    PrintGpsPathLines(pathLines);
    FreeFinalPathLines(pathLines);
    printf("Success\n");
  } else {
    printf("Sad\n");
  }
  return 0;
}
static t_ObstaclesPtr
GetObstaclesInArea(int lonTopLeft, int latTopLeft, int lonBottomRight, int latBottomRight)
{
  int obs1X[11] = {1206862015, 1206837643, 1207027422, 1207256519, 1207307561, 1207306124, 1207283199, 1207191355, 1206999547, 1206956616, 1206862015};
  int obs1Y[11] = {379789961, 379923562, 379909145, 379773635, 379710440, 379671831, 379638704, 379618101, 379647324, 379661093, 379789961};
  t_ObstaclesPtr obstacles;
  t_SingleObstaclePtr singleObstacle;
  t_PointCorPtr point;
  int i, pointCount;

  pointCount = sizeof(obs1X) / sizeof(int);
  obstacles = CreateObstacles();
  SetObstaclesCount(1, obstacles);
  singleObstacle = obstacles->m_obstacleMembersPtr = CreateSingleObstacleArray(pointCount);
  SetSingleObstacleCount(pointCount, singleObstacle);
  point = singleObstacle->m_pointsPtr = CreatePointArray(pointCount);
  for (i = 0; i < pointCount; i++) {
    InsertPoint(obs1X[i], obs1Y[i], point);
    point = GetNextPoint(point);
  }
  return obstacles;
}

static t_PointCorPtr
CreatePointArray(int size)
{
  return Malloc(size * sizeof(struct t_PointCor));
}

static void
SetSingleObstacleCount(int count, t_SingleObstaclePtr singleObstacle)
{
  singleObstacle->m_vertexCounts = count;
}

static t_SingleObstaclePtr
CreateSingleObstacleArray(int size)
{
  return Malloc(size * sizeof(struct t_SingleObstacle));
}

static t_ObstaclesPtr
CreateObstacles(void)
{
  return Malloc(sizeof(struct t_Obstacles));
}

static void
SetObstaclesCount(int count, t_ObstaclesPtr obstacles)
{
  obstacles->m_obstacleCounts = count;
}

static t_PointCorPtr
GetNextPoint(t_PointCorPtr point)
{
  return point + 1;
}

static void
InsertPoint(int lon, int lat, t_PointCorPtr point)
{
  point->m_lon = lon;
  point->m_lat = lat;
}
