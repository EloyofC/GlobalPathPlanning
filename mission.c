#include <stdlib.h>
#include <stdio.h>
#include "pretreatment.h"
#include "envoperate.h"
#include "mission.h"
#include "publicfun.h"
#include "advancedplan.h"

#define c_lengthOfUnit 100
#define c_heightOfUnit 100

static t_EnvironmentPtr InitialEnvWithObstacles(
   int lonTopLeft,
   int latTopLeft,
   int lonBottomRight,
   int latBottomRight,
   t_ObstaclesPtr obstacles
   ) {
   t_EnvironmentPtr newEnvironment = InitialEnvWithGps( c_lengthOfUnit, c_heightOfUnit,
                                                        lonTopLeft, latTopLeft,
                                                        lonBottomRight, latBottomRight );
   SetObstaclesInEnvironment( obstacles, newEnvironment );
   return newEnvironment;
}

static int GetLonTopLeftFromRec(
   struct t_RectangleArea rectangle
   ) {
   return rectangle.m_lonTopLeft;
}

static int GetLatTopLeftFromRec(
   struct t_RectangleArea rectangle
   ) {
   return rectangle.m_latTopLeft;
}

static int GetLonBottomRightFromRec(
   struct t_RectangleArea rectangle
   ) {
   return rectangle.m_lonBottomRight;
}

static int GetLatBottomRightFromRec(
   struct t_RectangleArea rectangle
   ) {
   return rectangle.m_latBottomRight;
}

static int GetWidthFromScanWidthInfo(
   struct t_ScanWidthInfo widthInfo
   ) {
   return widthInfo.m_width;
}

static int IsScanWidthHorizon(
   struct t_ScanWidthInfo widthInfo
   ) {
   return widthInfo.m_isHorizon == 1;
}

t_PathLinesPtr GetScanLinesInRec(
   int lonStart,
   int latStart,
   int lonEnd,
   int latEnd,
   struct t_RectangleArea rectangle,
   struct t_ScanWidthInfo widthInfo,
   t_ObstaclesPtr obstacles
   ) {
   int lonTopLeft = GetLonTopLeftFromRec( rectangle );
   int latTopLeft = GetLatTopLeftFromRec( rectangle );
   int lonBottomRight = GetLonBottomRightFromRec( rectangle );
   int latBottomRight = GetLatBottomRightFromRec( rectangle );
   int cellStartX = CalGpsDistanceLon( lonTopLeft, latTopLeft, lonStart ) / c_lengthOfUnit;
   int cellStartY = CalGpsDistanceLat( lonTopLeft, latTopLeft, latStart ) / c_heightOfUnit;
   int cellEndX =  CalGpsDistanceLon( lonTopLeft, latTopLeft, lonEnd ) / c_lengthOfUnit;
   int cellEndY =  CalGpsDistanceLat( lonTopLeft, latTopLeft, latEnd ) / c_heightOfUnit;
   t_EnvironmentPtr newEnvironment = InitialEnvWithObstacles( lonTopLeft, latTopLeft,
                                                              lonBottomRight, latBottomRight,
                                                              obstacles );

   int endPointX = cellEndX - 1;
   int endPointY = cellEndY - 1;
   int width = GetWidthFromScanWidthInfo( widthInfo );
   t_PathLinesPtr finalPathLine = NULL;
   if ( IsScanWidthHorizon( widthInfo ) ) {
      /* ensure that the cellWidth is larger than 1 */
      int cellWidth = width < c_heightOfUnit ? 1 : width / c_heightOfUnit;
      finalPathLine = ScanSearch( cellStartX, cellStartY,
                                  endPointX, endPointY,
                                  cellWidth,
                                  newEnvironment );
   } else {
      int cellWidth = width < c_lengthOfUnit ? 1 : width / c_lengthOfUnit;
      finalPathLine = ScanSearch( cellStartX, cellStartY,
                                  endPointX, endPointY,
                                  cellWidth,
                                  newEnvironment );
   }
   DeleteEnvironment( newEnvironment );
   return finalPathLine;
}

t_PathLinesPtr GetCruisePointsInCircle(
   struct t_ExpectedCruiseCricle circle,
   struct t_RectangleArea rectangle,
   t_ObstaclesPtr obstacles
   ){
   int lonTopLeft = GetLonTopLeftFromRec( rectangle );
   int latTopLeft = GetLatTopLeftFromRec( rectangle );
   int lonBottomRight = GetLonBottomRightFromRec( rectangle );
   int latBottomRight = GetLatBottomRightFromRec( rectangle );
   t_EnvironmentPtr newEnvironment = InitialEnvWithObstacles( lonTopLeft, latTopLeft,
                                                              lonBottomRight,latBottomRight,
                                                              obstacles );
   t_PathLinesPtr finalPathLine = CircleCruisePathPlan( circle, newEnvironment );
   DeleteEnvironment( newEnvironment );
   return finalPathLine;
}

t_PathLinesPtr GetPointsWithFixedMultiPosition(
   t_PathLinesPtr positions,
   struct t_RectangleArea rectangle,
   t_ObstaclesPtr obstacles
   ){
   int lonTopLeft = GetLonTopLeftFromRec( rectangle );
   int latTopLeft = GetLatTopLeftFromRec( rectangle );
   int lonBottomRight = GetLonBottomRightFromRec( rectangle );
   int latBottomRight = GetLatBottomRightFromRec( rectangle );
   t_EnvironmentPtr newEnvironment = InitialEnvWithObstacles( lonTopLeft, latTopLeft,
                                                              lonBottomRight, latBottomRight,
                                                              obstacles );
   t_PathLinesPtr finalPathLine = MultiGpsPosPathPlan( positions, newEnvironment );
   DeleteEnvironment( newEnvironment );
   return finalPathLine;
}

static void FreeFinalPathPoints(
   t_PathPointPtr pathPointHead
   ) {
   t_PathPointPtr pathPointTemp;

   for ( ; pathPointHead != NULL; pathPointHead = pathPointTemp ) {
      pathPointTemp = GetGpsPathPointNext( pathPointHead );
      Free( pathPointHead );
   }
}

void FreeFinalPathLines(
   t_PathLinesPtr finalPathLines
   ) {
   FreeFinalPathPoints( GetGpsPathLinePoints( finalPathLines ) );
   Free( finalPathLines );
}

void PrintFinalGpsPathLines(
   t_PathLinesPtr pathLine
   ) {
   PrintGpsPathLines( pathLine, "The gps path line" );
}

