#include <stdio.h>
#include "PublicFun.h"
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
static void InsertSingleObstacle(unsigned int size, int *x, int *y, t_SingleObstaclePtr singleObstacle);
static t_SingleObstaclePtr GetSingleObstacle(int index, t_ObstaclesPtr obstacles);
static int TestTwoObstacle(void);
static t_ObstaclesPtr GetSingleObstacleInArea(int lonTopLeft, int latTopLeft, int lonBottomRight, int latBottomRight);

int main(int argc, char *argv[])
{
  return TestSingleObstacle();
}


static int
TestNoObstacle(void)
{
  int lonTopLeft, latTopLeft, lonBottomRight, latBottomRight, width;
  t_ObstaclesPtr obstacles;

  lonTopLeft = 1200291593;
  latTopLeft = 356995731;
  lonBottomRight = 1201856045;
  latBottomRight = 356052480;
  width = 500;
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

  lonTopLeft = 1225604106;
  latTopLeft = 308890180;
  lonBottomRight =  1227319960;
  latBottomRight =  307873053;
  width = 500;
  printf("The start point is x %d y %d\n", lonTopLeft, latTopLeft);
  printf("The end point is x %d y %d\n", lonBottomRight, latBottomRight);
  obstacles = GetSingleObstacleInArea(lonTopLeft, latTopLeft, lonBottomRight, latBottomRight);
  return TestTemplate(lonTopLeft, latTopLeft, lonBottomRight, latBottomRight, width, obstacles);
}

static int
TestTwoObstacle(void)
{
  int lonTopLeft, latTopLeft, lonBottomRight, latBottomRight, width;
  t_ObstaclesPtr obstacles;

  lonTopLeft = 1225604106;
  latTopLeft = 308890180;
  lonBottomRight =  1227319960;
  latBottomRight =  307873053;
  width = 500;
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
  fflush(stdout);
  return 0;
}

static t_ObstaclesPtr
GetSingleObstacleInArea(int lonTopLeft, int latTopLeft, int lonBottomRight, int latBottomRight)
{
  int obs1X[] = {1226261133, 1226349743, 1226371716, 1226388919, 1226387886, 1226388613, 1226401468, 1226419803, 1226572984, 1226596097, 1226590914, 1226356696, 1226042259, 1226033024, 1226040696, 1226126296, 1226147344, 1226168796, 1226261133};
  int obs1Y[] = {308246224, 308204593, 308207694, 308218633, 308253659, 308271714, 308287948, 308288467, 308227486, 308198034, 308180667, 308120648, 308192785, 308211966, 308226106, 308278474, 308282459, 308283614, 308246224};

  t_ObstaclesPtr obstacles;
  obstacles = CreateObstacles();
  SetObstaclesCount(1, obstacles);
  obstacles->m_obstacleMembersPtr = CreateSingleObstacleArray(1);
  InsertSingleObstacle(sizeof(obs1X)/sizeof(int), obs1X, obs1Y,GetSingleObstacle(0, obstacles));
  return obstacles;
}

static t_ObstaclesPtr
GetTwoObstaclesInArea(int lonTopLeft, int latTopLeft, int lonBottomRight, int latBottomRight)
{
  int obs1X[] = {1226261133, 1226349743, 1226371716, 1226388919, 1226387886, 1226388613, 1226401468, 1226419803, 1226572984, 1226596097, 1226590914, 1226356696, 1226042259, 1226033024, 1226040696, 1226126296, 1226147344, 1226168796, 1226261133};
  int obs1Y[] = {308246224, 308204593, 308207694, 308218633, 308253659, 308271714, 308287948, 308288467, 308227486, 308198034, 308180667, 308120648, 308192785, 308211966, 308226106, 308278474, 308282459, 308283614, 308246224};
  int obs2X[] = {1226969706, 1226738282, 1226703778, 1226698388, 1226695271, 1226692477, 1226612887, 1226602727, 1226623666, 1226855081, 1227028061, 1227028681, 1227017793, 1227000492, 1226969706};
  int obs2Y[] = {308543448, 308620885, 308603333, 308585903, 308567125, 308530188, 308510634, 308495645, 308471222, 308385008, 308467029, 308505225, 308518346, 308528398, 308543448};

  t_ObstaclesPtr obstacles;
  obstacles = CreateObstacles();
  SetObstaclesCount(2, obstacles);
  obstacles->m_obstacleMembersPtr = CreateSingleObstacleArray(2);
  InsertSingleObstacle(sizeof(obs1X)/sizeof(int), obs1X, obs1Y,GetSingleObstacle(0, obstacles));
  InsertSingleObstacle(sizeof(obs2X)/sizeof(int), obs2X, obs2Y, GetSingleObstacle(1, obstacles));
  return obstacles;
}

static void
InsertSingleObstacle(unsigned int size, int *x, int *y, t_SingleObstaclePtr singleObstacle)
{
  t_PointCorPtr point;
  unsigned int i;

  SetSingleObstacleCount(size, singleObstacle);
  point = singleObstacle->m_pointsPtr = CreatePointArray(size);
  for (i = 0; i < size; i++) {
    InsertPoint(x[i], y[i], point);
    point = GetNextPoint(point);
  }
}

static t_PointCorPtr
CreatePointArray(int size)
{
  return (t_PointCorPtr)Malloc(size * sizeof(struct t_PointCor));
}

static void
SetSingleObstacleCount(int count, t_SingleObstaclePtr singleObstacle)
{
  singleObstacle->m_vertexCounts = count;
}

static t_SingleObstaclePtr
CreateSingleObstacleArray(int size)
{
  return (t_SingleObstaclePtr)Malloc(size * sizeof(struct t_SingleObstacle));
}

static t_SingleObstaclePtr
GetSingleObstacle(int index, t_ObstaclesPtr obstacles)
{
  return obstacles->m_obstacleMembersPtr + index;
}

static t_ObstaclesPtr
CreateObstacles(void)
{
  return (t_ObstaclesPtr)Malloc(sizeof(struct t_Obstacles));
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
