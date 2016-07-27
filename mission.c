#include <stdlib.h>
#include <stdio.h>
#include "pretreatment.h"
#include "envoperate.h"
#include "mission.h"
#include "publicfun.h"
#include "advancedplan.h"

#define c_lengthOfUnit 100
#define c_widthOfUnit 100

static t_EnvironmentPtr InitialEnvWithObstacles(
   int lonTopLeft,
   int latTopLeft,
   int lonBottomRight,
   int latBottomRight,
   t_ObstaclesPtr obstacles
   ) {
   t_EnvironmentPtr newEnvironment = InitialEnvWithGps( c_lengthOfUnit, c_widthOfUnit,
                                                        lonTopLeft, latTopLeft,
                                                        lonBottomRight, latBottomRight );
   SetObstaclesInEnvironment( obstacles, newEnvironment );
   return newEnvironment;
}

t_PathLinesPtr GetScanLinesInRec(
   int lonStart,
   int latStart,
   int lonEnd,
   int latEnd,
   int lonTopLeft,
   int latTopLeft,
   int lonBottomRight,
   int latBottomRight,
   int width,
   t_ObstaclesPtr obstacles
   ) {
   int cellStartX = CalGpsDistanceLon( lonTopLeft, latTopLeft, lonStart ) / c_lengthOfUnit;
   int cellStartY = CalGpsDistanceLat( lonTopLeft, latTopLeft, latStart ) / c_widthOfUnit;
   int cellEndX =  CalGpsDistanceLon( lonTopLeft, latTopLeft, lonEnd ) / c_lengthOfUnit;
   int cellEndY =  CalGpsDistanceLat( lonTopLeft, latTopLeft, latEnd ) / c_widthOfUnit;
   /* ensure that the cellWidth is larger than 1 */
   int cellWidth = width < c_widthOfUnit ? 1 : width / c_widthOfUnit;
   t_EnvironmentPtr newEnvironment = InitialEnvWithObstacles( lonTopLeft, latTopLeft,
                                                              lonBottomRight, latBottomRight,
                                                              obstacles );

   int endPointX = cellEndX - 1;
   int endPointY = cellEndY - 1;
   t_PathLinesPtr pathLines = ScanSearch( cellStartX, cellStartY,
                                     endPointX, endPointY,
                                     cellWidth,
                                     newEnvironment );
   DeleteEnvironment( newEnvironment );
   return pathLines;
}

t_PathLinesPtr GetCruisePointsInCircle(
   struct t_ExpectedCruiseCricle circle,
   int lonTopLeft,
   int latTopLeft,
   int lonBottomRight,
   int latBottomRight,
   t_ObstaclesPtr obstacles
   ){
   
}

t_PathLinesPtr GetPointsWithFixedMultiPosition(
   t_PathLinesPtr positions,
   int lonTopLeft,
   int latTopLeft,
   int lonBottomRight,
   int latBottomRight,
   t_ObstaclesPtr obstacles
   ){
   t_EnvironmentPtr newEnvironment = InitialEnvWithObstacles( lonTopLeft, latTopLeft,
                                                              lonBottomRight, latBottomRight,
                                                              obstacles );
}

static void FreeFinalPathPoints(
   t_PathPointPtr pathPointHead
   ) {
   t_PathPointPtr pathPointTemp;

   for ( ; pathPointHead != NULL; pathPointHead = pathPointTemp ) {
      pathPointTemp = pathPointHead->m_next;
      free( pathPointHead );
   }
}

static t_PathPointPtr GetFinalLinePoints(
   t_PathLinesPtr finalPathLines
   ) {
   return finalPathLines->m_pathPoints;
}

void FreeFinalPathLines(
   t_PathLinesPtr finalPathLines
   ) {
   FreeFinalPathPoints( GetFinalLinePoints( finalPathLines ) );
   free( finalPathLines );
}

static int GetFinalLinePointCount(
   t_PathLinesPtr finalPathLines
   ) {
   return finalPathLines->m_pointCounts;
}

static int GetFinalPathPointLon(
   t_PathPointPtr pathPoint
   ) {
   return pathPoint->m_lon;
}

static int GetFinalPathPointLat(
   t_PathPointPtr pathPoint
   ) {
   return pathPoint->m_lat;
}

static t_PathPointPtr GetFinalPathPointNext(
   t_PathPointPtr pathPoint
   ) {
   return pathPoint->m_next;
}

static void PrintFinalPathLines(
   t_PathLinesPtr finalPathLines,
   char *str
   ) {
   if ( finalPathLines != NULL ) {
      int count = GetFinalLinePointCount( finalPathLines );
      t_PathPointPtr pathPoints = GetFinalLinePoints( finalPathLines );

      for ( int i = 0; i < count; i++ ) {
         printf( "%s : the %d nd point x %d y %d\n", str, i, GetFinalPathPointLon( pathPoints ), GetFinalPathPointLat( pathPoints ) );
         pathPoints = GetFinalPathPointNext( pathPoints );
      }
   } else {
      return;
   }
}

void PrintEnvPathLines(
   t_PathLinesPtr finalPathLines
   ) {
   PrintFinalPathLines( finalPathLines, "The env pass point");
}

void PrintGpsPathLines(
   t_PathLinesPtr finalPathLines
   ) {
   PrintFinalPathLines( finalPathLines, "The gps path line" );
}

