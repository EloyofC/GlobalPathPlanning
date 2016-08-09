/*
  Descripition : This file is the interface between pathplanning module and GUI part
  Author : Green
  Date : 16/06/02
*/
#ifndef _Mission_Interferce_H
#define _Mission_Interferce_H

typedef struct t_Obstacles {
   int m_obstacleCounts;
   struct t_SingleObstacle *m_obstacleMembersPtr; /* an array of struct t_SingleObstacle */
} *t_ObstaclesPtr;

typedef struct t_SingleObstacle {
   int m_vertexCounts;
   struct t_PointCor *m_pointsPtr; /* an array of struct t_PointCor */
} *t_SingleObstaclePtr;

typedef struct t_PointCor {
   int m_lon;
   int m_lat;
} *t_PointCorPtr;

typedef struct t_PathLines {
   int m_pointCounts;
   struct t_PathPoint *m_pathPoints; /* the path point in the front is visited last? */
} *t_PathLinesPtr;

typedef struct t_PathPoint {
   int m_lon;
   int m_lat;
   struct t_PathPoint *m_next;
} *t_PathPointPtr;

struct t_RectangleArea {
   int m_lonTopLeft;
   int m_latTopLeft;
   int m_lonBottomRight;
   int m_latBottomRight;
};

struct t_ScanWidthInfo {
   int m_width;
   unsigned char m_isHorizon;             /* horizon with 1, vertical with 2 */
};

struct t_ExpectedCruiseCricle {
   int m_lonCircleCenter;            /* type of gps */
   int m_latCircleCenter;
   int m_circleRadius;
   unsigned char m_isClockWise;
};

t_PathLinesPtr GetScanLinesInRec( int lonStart, int latStart, int lonEnd, int latEnd, struct t_RectangleArea rectangle, struct t_ScanWidthInfo widthInfo, t_ObstaclesPtr obstacles );
t_PathLinesPtr GetCruisePointsInCircle(struct t_ExpectedCruiseCricle circle,  struct t_RectangleArea rectangle, t_ObstaclesPtr obstacles);
t_PathLinesPtr GetPointsWithFixedMultiPosition(t_PathLinesPtr positions, struct t_RectangleArea rectangle, t_ObstaclesPtr obstacles);
void FreeFinalPathLines( t_PathLinesPtr pathLine );
void PrintFinalGpsPathLines( t_PathLinesPtr pathLine );

#endif
