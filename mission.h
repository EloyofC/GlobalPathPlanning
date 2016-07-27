/*
  Descripition : This file is the interface between pathplanning module and GUI part
  Author : Green
  Date : 16/06/02
*/
#ifndef _Mission_Interferce_H
#define _Mission_Interferce_H

typedef struct t_Obstacles
{
  int m_obstacleCounts;
  struct t_SingleObstacle *m_obstacleMembersPtr; /* an array of struct t_SingleObstacle */
} *t_ObstaclesPtr;

typedef struct t_SingleObstacle
{
  int m_vertexCounts;
  struct t_PointCor *m_pointsPtr; /* an array of struct t_PointCor */
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

struct t_ExpectedCruiseCricle
{
int lonCircleCenter;            /* type of gps */
int latCircleCenter;
int circleRadius;
};

t_PathLinesPtr GetScanLinesInRec( int lonStart, int latStart, int lonEnd, int latEnd, int lonTopLeft, int latTopLeft, int lonBottomRight, int latBottomRight, int width, t_ObstaclesPtr obstacles );
t_PathLinesPtr GetCruisePointsInCircle(struct t_ExpectedCruiseCricle circle, int lonTopLeft, int latTopLeft, int lonBottomRight, int latBottomRight, t_ObstaclesPtr obstacles);
t_PathLinesPtr GetPointsWithFixedMultiPosition(t_PathLinesPtr positions, int lonTopLeft, int latTopLeft, int lonBottomRight, int latBottomRight, t_ObstaclesPtr obstacles);
void FreeFinalPathLines( t_PathLinesPtr finalPathLines );
void PrintGpsPathLines( t_PathLinesPtr finalPathLines );
void PrintEnvPathLines( t_PathLinesPtr finalPathLines );

#endif
