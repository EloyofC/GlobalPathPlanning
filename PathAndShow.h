/*
  Descripition : This file is the interface between pathplanning module and GUI part
  Author : Green
  Date : 16/06/02
*/
#ifndef _Path_And_Show_H
#define _Path_And_Show_H

typedef struct t_Obstacles
{
  int m_obstacleCounts;
  struct t_SingleObstacle *m_obstacleMembersPtr;
} *t_ObstaclesPtr;

typedef struct t_SingleObstacle
{
  int m_vertexCounts;
  struct t_PointCor *m_pointsPtr;
} *t_SingleObstaclePtr;

typedef struct t_PointCor
{
  int m_lon;
  int m_lat;
} *t_PointCorPtr;

typedef struct t_PathLines
{
int m_pointCounts;
struct t_PathPoint *m_pathPoints;
} *t_PathLinesPtr;

typedef struct t_PathPoint
{
int m_lon;
int m_lat;
struct t_PathPoint *m_next;
} *t_PathPointPtr;

t_PathLinesPtr DoCruiseGeneral(int xStart, int yStart, int xEnd, int yEnd, int xTopLeft, int yTopLeft, int xBottomRight, int yBottomRight, int width, t_ObstaclesPtr obstacles);

void FreeFinalPathLines(t_PathLinesPtr finalPathLines);

void PrintGpsPathLines(t_PathLinesPtr finalPathLines);

#endif
