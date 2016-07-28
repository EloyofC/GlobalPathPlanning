/*
  Description: This file is for the interferce of advancedplanning;
  Author: Green
  Date: 16/06/02
 */
#ifndef _Advanced_Plan_H
#define _Advanced_Plan_H

#define c_costNormal 1

struct t_Environment;
typedef struct t_Environment *t_EnvironmentPtr;
struct t_PathLines;
typedef struct t_PathLines *t_PathLinesPtr;
struct t_PathPoint;
typedef struct t_PathPoint *t_PathPointPtr;

int GetGpsPathPointLon( t_PathPointPtr pathPoint );
int GetGpsPathPointLat( t_PathPointPtr pathPoint );
t_PathPointPtr GetGpsPathPointNext( t_PathPointPtr pathPoint );
t_PathPointPtr GetGpsPathLinePoints( t_PathLinesPtr pathLines );
int GetGpsPathLinePointCount( t_PathLinesPtr pathLines );
void InsertNewGpsPathPoint( int x, int y, t_PathLinesPtr pathLine );
t_PathLinesPtr CreateGpsPathLine( void );
void PrintGpsPathLines( t_PathLinesPtr pathLine, char *str );
void PrintAndFreePathLine( void );
t_PathLinesPtr ScanSearch( int xStart, int yStart, int xEnd, int yEnd, int width, t_EnvironmentPtr m_environment );
t_PathLinesPtr MultiGpsPosPathPlan( t_PathLinesPtr positions, t_EnvironmentPtr environment );

#endif
