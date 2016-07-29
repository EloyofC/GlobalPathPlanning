/*
  Description: This file is for the advanced use of the pathplanning
  Author: Green
  Date: 16/06/02
*/

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "advancedplan.h"
#include "localplanning.h"
#include "pretreatment.h"
#include "envoperate.h"
#include "publicfun.h"
#include "fifoqueue.h"
#include "mission.h"

typedef struct t_EnvPathLine
{
   int m_xIndex;
   int m_yIndex;
   struct t_EnvPathLine *m_prevPtr;
} *t_EnvPathLinePtr;


static int GetEnvPathMemberX(
   t_EnvPathLinePtr envPathLineMember
   ) {
   return envPathLineMember->m_xIndex;
}

static int GetEnvPathMemberY(
   t_EnvPathLinePtr envPathLineMember
   ) {
   return envPathLineMember->m_yIndex;
}

static t_EnvPathLinePtr GetEnvPathMemberPrev(
   t_EnvPathLinePtr envPathLineMember
   ) {
   return envPathLineMember->m_prevPtr;
}

static void PrintEntirePathMembers(
   t_EnvPathLinePtr envPathLine
   ) {
   int i = 0;

   for ( t_EnvPathLinePtr memberPrev = envPathLine; memberPrev != NULL; memberPrev = memberPrev->m_prevPtr )
      printf( "The entire final pass point : the %d nd point x %d y %d\n", i++, memberPrev->m_xIndex, memberPrev->m_yIndex );
}


static void FreePathLine(
   t_EnvPathLinePtr envPathLine
   ) {
   for ( t_EnvPathLinePtr memberPrev = envPathLine; memberPrev != NULL; ) {
      t_EnvPathLinePtr memberCurrent = memberPrev;
      memberPrev = GetEnvPathMemberPrev( memberPrev );
      Free( memberCurrent );
   }
}

static int IsDoubleNextEnvPathMemberExist(
   t_EnvPathLinePtr envPathLine
   ) {
   t_EnvPathLinePtr envPathLinePrev = GetEnvPathMemberPrev( envPathLine );
   return ( envPathLinePrev != NULL ) &&
      ( GetEnvPathMemberPrev( envPathLinePrev ) != NULL );
}

static t_EnvPathLinePtr SkipPathLineMember(
   t_EnvPathLinePtr envPathLine
   ) {
   assert( envPathLine != NULL );
   t_EnvPathLinePtr memberPrev = GetEnvPathMemberPrev( envPathLine );
   return memberPrev;
}

static int IsEnvPathMemberEmpty(
   t_EnvPathLinePtr envPathLine
   ) {
   return envPathLine == NULL;
}

static int GetNextEnvPathMemberX(
   t_EnvPathLinePtr envPathLine
   ) {
   t_EnvPathLinePtr envPathLinePrev = GetEnvPathMemberPrev( envPathLine );
   return GetEnvPathMemberX( envPathLinePrev );
}

static int GetNextEnvPathMemberY(
   t_EnvPathLinePtr envPathLine
   ) {
   t_EnvPathLinePtr envPathLinePrev = GetEnvPathMemberPrev( envPathLine );
   return GetEnvPathMemberY( envPathLinePrev );
}

static int GetDoubleNextEnvPathMemberX(
   t_EnvPathLinePtr envPathLine
   ) {
   t_EnvPathLinePtr envPathLinePrev = GetEnvPathMemberPrev( envPathLine );
   t_EnvPathLinePtr envPathLineDoublePrev = GetEnvPathMemberPrev( envPathLinePrev );
   return GetEnvPathMemberX( envPathLineDoublePrev );
}

static int GetDoubleNextEnvPathMemberY(
   t_EnvPathLinePtr envPathLine
   ) {
   t_EnvPathLinePtr envPathLinePrev = GetEnvPathMemberPrev( envPathLine );
   t_EnvPathLinePtr envPathLineDoublePrev = GetEnvPathMemberPrev( envPathLinePrev );
   return GetEnvPathMemberY( envPathLineDoublePrev );
}

static int DijskstraPriority(
   int costNew
   ) {
   return costNew;
}

static int DepthFirstPriority(
   t_EnvironmentMemberPtr member,
   t_EnvironmentPtr environment
   ) {
   int xHeuristic = GetEnvEndX( environment) - GetEnvMemberX( member );
   int yHeuristic = GetEnvEndY( environment)-  GetEnvMemberY( member );

   xHeuristic = abs( xHeuristic );
   yHeuristic = abs( yHeuristic );
   return xHeuristic + yHeuristic;
}

/* Todo : can add some restraint of keep the same direction with the previous and stay at the same y( or may be the line of start and the end ) */
static int AStarPriority(
   int costNew,
   t_EnvironmentMemberPtr member,
   t_EnvironmentPtr environment
   ) {
   return DijskstraPriority( costNew ) +
      DepthFirstPriority( member, environment );
}


static t_EnvPathLinePtr StoreNewDistributedMembers(
   int xIndex,
   int yIndex,
   t_EnvPathLinePtr envPathLine
   ) {
   t_EnvPathLinePtr memberNew = Malloc( sizeof( struct t_EnvPathLine ) );
   memberNew->m_xIndex = xIndex;
   memberNew->m_yIndex = yIndex;
   memberNew->m_prevPtr = envPathLine;

   return memberNew;
}

static t_EnvPathLinePtr CreateEnvPathLine(
   int x,
   int y
   ) {
   t_EnvPathLinePtr memberNew = Malloc( sizeof( struct t_EnvPathLine ) );
   memberNew->m_xIndex = x;
   memberNew->m_yIndex = y;
   return memberNew;
}

static t_EnvPathLinePtr InsertNewEnvPathLine(
   int x,
   int y,
   t_EnvPathLinePtr envPathLine
   ) {
   t_EnvPathLinePtr memberNew = CreateEnvPathLine( x, y );
   memberNew->m_prevPtr = envPathLine;

   return memberNew;
}

static t_EnvPathLinePtr StoreNewPathMember(
   t_EnvironmentMemberPtr envMember,
   t_EnvPathLinePtr envPathLine
   ) {
   int x = GetEnvMemberX( envMember );
   int y = GetEnvMemberY( envMember );
   return InsertNewEnvPathLine( x, y, envPathLine );
}

static int IsNumEven(
   int num
   ) {
   int half = num/2;
   return half * 2 == num;
}

/* !!This function may need to think it over again */
static t_EnvPathLinePtr StoreDistributedPoints(
   int xMin,
   int yMin,
   int xMax,
   int yMax,
   int xStart,
   int yStart,
   int width
   ) {
   int widthCount = ( yMax - yMin )/width;
   int isGoUp;
   if ( yStart == yMin )
      isGoUp = 1;
   else
      isGoUp = 0;
   if ( ( yMax - yMin ) > width * widthCount )
      widthCount++;              /* need to compensate the rest of the width */

   int itemp_x, jtemp_x;
   if ( IsNumEven( widthCount ) ) {      /* determines the initial x according to the widthCount even or odd */
      itemp_x = xStart;
      jtemp_x = xMin + xMax - itemp_x;
   } else {
      itemp_x = xMin + xMax - xStart;
      jtemp_x = xStart;
   }

   t_EnvPathLinePtr distributedPoints = NULL;
   if ( isGoUp ) {
      int i, j, yTemp;
      for ( i=j=0, yTemp = yMin; i < widthCount - 1; i++, j=j+2, yTemp += width ) {
         distributedPoints = StoreNewDistributedMembers( itemp_x, yTemp, distributedPoints );
         distributedPoints = StoreNewDistributedMembers( jtemp_x, yTemp, distributedPoints );
         SwapNum( &itemp_x, &jtemp_x );
      }
      distributedPoints = StoreNewDistributedMembers( itemp_x, yMax, distributedPoints ); /* take care of the rest */
      distributedPoints = StoreNewDistributedMembers( jtemp_x, yMax, distributedPoints );
   } else {
      int i, j, yTemp;
      for ( i=j=0, yTemp = yMax; i < widthCount - 1; i++, j=j+2, yTemp -= width ) {
         distributedPoints = StoreNewDistributedMembers( itemp_x, yTemp, distributedPoints );
         distributedPoints = StoreNewDistributedMembers( jtemp_x, yTemp, distributedPoints );
         SwapNum( &itemp_x, &jtemp_x );
      }
      distributedPoints = StoreNewDistributedMembers( itemp_x, yMin, distributedPoints );
      distributedPoints = StoreNewDistributedMembers( jtemp_x, yMin, distributedPoints );
   }
   return distributedPoints;
}

/* This routine get the distributed points with the request of the scan search */
static t_EnvPathLinePtr GetDistributedPoints(
   int xStart,
   int yStart,
   int xEnd,
   int yEnd,
   int width
   ) {
   int min_x = xStart < xEnd ? xStart : xEnd;
   int min_y = yStart < yEnd ? yStart : yEnd;
   int max_x = xStart + xEnd - min_x;
   int max_y = yStart + yEnd - min_y;
   t_EnvPathLinePtr distributedPoints = StoreDistributedPoints( min_x, min_y, max_x, max_y, xStart, yStart, width );
   return distributedPoints;
}

static t_FifoQueuePtr SearchOneNode(
   int xIndex,
   int yIndex,
   t_FifoQueuePtr nearQueue,
   t_EnvironmentPtr environment
   ) {
   if ( IsEnvPointInEnv( xIndex, yIndex, environment ) ) {
      t_EnvironmentMemberPtr member = GetEnvMember( xIndex, yIndex, environment );
      if ( IsEnvMemberNotSearched( member ) ) {
         SetEnvMemberSearched( member );
         nearQueue = EnFifoQueue( member, nearQueue );
      }
   }
   return nearQueue;
}

static t_FifoQueuePtr SearchFourNeighbour(
   t_EnvironmentMemberPtr member,
   t_FifoQueuePtr nearQueue,
   t_EnvironmentPtr environment
   ) {
   int x[ 4 ] = { -1, 1, 0, 0 };	/* search in four directions */
   int y[ 4 ] = { 0, 0, -1, 1 };
   int xIndex = GetEnvMemberX( member );
   int yIndex = GetEnvMemberY( member );

   for ( unsigned long i = 0; i < sizeof( x )/sizeof( int ); i++ ) {
      nearQueue = SearchOneNode( xIndex + x[ i ], yIndex + y[ i ], nearQueue, environment );
   }
   return nearQueue;
}

static t_EnvironmentMemberPtr SearchNearestFreePoint(
   int xIndex,
   int yIndex,
   t_EnvironmentPtr environment
   ) {
   if ( environment == NULL )
      return NULL;

   ResetEnvAllNotSearched( environment );
   t_FifoQueuePtr nearQueue =  CreateFifoQueue();
   t_EnvironmentMemberPtr member = GetEnvMember( xIndex, yIndex, environment );

   for ( ; member != NULL && IsEnvMemberObstacle( member ); member = DeFifoQueue( nearQueue ) ) {
      nearQueue = SearchFourNeighbour( member, nearQueue, environment );
   }
   if ( member == NULL ) {
         FreeFifoQueue( nearQueue );
         return NULL;
   } else {
      FreeFifoQueue( nearQueue );
      return member;
   }
}

/* This routine change the current cordinatation of the point to the nearest free point. It return False if no nearest free point exist  */
static int UpdatePoint2NearestFreePoint(
   t_EnvPathLinePtr member,
   t_EnvironmentPtr environment
   ) {
   int xIndex = GetEnvPathMemberX( member );
   int yIndex = GetEnvPathMemberY( member );

   if ( IsEnvMemberObstacle( GetEnvMember( xIndex, yIndex, environment )) ) {
      t_EnvironmentMemberPtr newFreeMember = SearchNearestFreePoint( xIndex, yIndex, environment );
      /* if no nearest free point exist then return False */
      if ( newFreeMember == NULL ) {
         return 0;
      }
      member->m_xIndex = GetEnvMemberX( newFreeMember );
      member->m_yIndex = GetEnvMemberY( newFreeMember );
   }
   return 1;
}

static int ChangeDistributedWithNearestFreePoints(
   t_EnvPathLinePtr distributedPoints,
   t_EnvironmentPtr environment
   ) {
   for ( t_EnvPathLinePtr member = distributedPoints; member != NULL; member = GetEnvPathMemberPrev( member ) ) {
      /* if no nearest free point exist then return False */
      if ( UpdatePoint2NearestFreePoint( member, environment ) == 0 )
         return 0;
   }
   return 1;
}

static t_EnvPathLinePtr AppendEnvPathLine(
   t_EnvPathLinePtr envPathLineFirst,
   t_EnvPathLinePtr envPathLineSecond
   ) {
   if ( envPathLineFirst != NULL ) {
      t_EnvPathLinePtr linePrev = envPathLineFirst;
      for ( ; GetEnvPathMemberPrev( linePrev ) != NULL; linePrev = GetEnvPathMemberPrev( linePrev ) )
         ;
      linePrev->m_prevPtr = envPathLineSecond;
      return envPathLineFirst;
   } else {
      return envPathLineSecond;
   }
}

static t_EnvPathLinePtr StoreEnvPathLine(
   int xEnd,
   int yEnd,
   t_EnvPathLinePtr envPathLine,
   t_EnvironmentPtr environment
   ) {
   t_EnvPathLinePtr envPathLineNew = NULL;

   for ( t_EnvironmentMemberPtr memberPrev = GetEnvMember( xEnd, yEnd, environment );
         memberPrev != NULL;
         memberPrev = GetEnvMemberPrev( memberPrev ) ) {
      DebugCodeDetail(
         static int i = 1;
         printf( "StoreEnvPathLine : %d nd search Searched Member has X %d, Y %d, Cost %d\n", i++, GetEnvMemberX( memberPrev ), GetEnvMemberY( memberPrev ), GetEnvMemberCost( memberPrev ) );
         fflush( stdout );
         );

      envPathLineNew = StoreNewPathMember( memberPrev, envPathLineNew );
   }

   envPathLine = AppendEnvPathLine( envPathLine, envPathLineNew );
   return envPathLine;
}

t_PathLinesPtr CreateGpsPathLine(
   void
   ) {
   t_PathLinesPtr pathLineNew = Malloc( sizeof( struct t_PathLines ) );
   pathLineNew->m_pointCounts = 0;
   pathLineNew->m_pathPoints = NULL;
   return pathLineNew;
}

void InsertNewGpsPathPoint(
   int x,
   int y,
   t_PathLinesPtr pathLine
   ) {
   t_PathPointPtr pointNew = Malloc( sizeof( struct t_PathPoint ) );
   pointNew->m_lon = x;
   pointNew->m_lat = y;
   pointNew->m_next = pathLine->m_pathPoints;
   pathLine->m_pathPoints = pointNew;
   pathLine->m_pointCounts++;
}

void PrintGpsPathLines(
   t_PathLinesPtr pathLine,
   char *str
   ) {
   if ( pathLine != NULL ) {
      int count = GetGpsPathLinePointCount( pathLine );
      t_PathPointPtr pathPoints = GetGpsPathLinePoints( pathLine );

      for ( int i = 0; i < count; i++ ) {
         printf( "%s : the %d nd point x %d y %d\n", str, i, GetGpsPathPointLon( pathPoints ), GetGpsPathPointLat( pathPoints ) );
         pathPoints = GetGpsPathPointNext( pathPoints );
      }
   } else {
      return;
   }
}

static void PrintInEnvPathLines(
   t_PathLinesPtr pathLine
   ) {
   PrintGpsPathLines( pathLine, "The env pass point");
}

static void PrintPosPathLines(
   t_PathLinesPtr pathLine
   ) {
   PrintGpsPathLines( pathLine, "The position path line" );
}

static int IsThreePointInALine(
   int x1,
   int y1,
   int x2,
   int y2,
   int x3,
   int y3
   ) {
   if ( ( x1 == x2 ) && ( x2 == x3 ) ) { /* just take care of the condition that delta of x is zero */
      return 1;
   } else if ( ( x1 == x2 ) || ( x1 == x3 ) || ( x2 == x3 ) ) {
      return 0;
   } else {
      double delta1 = ( double ) abs( y1 - y2 ) / ( double ) ( abs( x1 - x2 ) );
      double delta2 = ( double ) abs( y2 - y3 ) / ( double ) ( abs( x2 - x3 ) );
      return IsDoubleEqual( delta1, delta2 );
   }
}

/* This routine filter the neighbouring members that their cordinatation in a line */
static t_EnvPathLinePtr FilterInALinePathLineMembers(
   t_EnvPathLinePtr envPathLine
   ) {
   t_EnvPathLinePtr currentEnvPathLine = envPathLine;
   while ( IsDoubleNextEnvPathMemberExist( currentEnvPathLine ) ) {
      if ( IsThreePointInALine( GetEnvPathMemberX( currentEnvPathLine ),
                                GetEnvPathMemberY( currentEnvPathLine ),
                                GetNextEnvPathMemberX( currentEnvPathLine ),
                                GetNextEnvPathMemberY( currentEnvPathLine ),
                                GetDoubleNextEnvPathMemberX( currentEnvPathLine ),
                                GetDoubleNextEnvPathMemberY( currentEnvPathLine ) ) ) {
         currentEnvPathLine = SkipPathLineMember( currentEnvPathLine );
      }else {
         break;
      }
   }
   return currentEnvPathLine;
}

static t_PathLinesPtr GetGpsPathLineFromEnvPathLine(
   t_EnvPathLinePtr envPathLine
   ) {
   t_PathLinesPtr pathLine = CreateGpsPathLine();
   DebugCode(
      PrintEntirePathMembers( envPathLine );
      );
   for ( t_EnvPathLinePtr currentEnvPathLine = envPathLine;
         !IsEnvPathMemberEmpty( currentEnvPathLine );
         currentEnvPathLine =  SkipPathLineMember( currentEnvPathLine ) ) {
      int x = GetEnvPathMemberX( currentEnvPathLine );
      int y = GetEnvPathMemberY( currentEnvPathLine );
      DebugCodeDetail(
         printf( "GetGpsPathLineFromEnvPathLine : x %d y %d\n", x, y);
         fflush( stdout );
         );
      InsertNewGpsPathPoint( x, y, pathLine );
      currentEnvPathLine = FilterInALinePathLineMembers( currentEnvPathLine );
   }
   return pathLine;
}

int GetGpsPathPointLon(
   t_PathPointPtr pathPoint
   ) {
   return pathPoint->m_lon;
}

int GetGpsPathPointLat(
   t_PathPointPtr pathPoint
   ) {
   return pathPoint->m_lat;
}

t_PathPointPtr GetGpsPathPointNext(
   t_PathPointPtr pathPoint
   ) {
   return pathPoint->m_next;
}

static void SetPathPointLon(
   int x,
   t_PathPointPtr pathPoint
   ) {
   pathPoint->m_lon = x;
}

static void SetPathPointLat(
   int y,
   t_PathPointPtr pathPoint
   ) {
   pathPoint->m_lat = y;
}

t_PathPointPtr GetGpsPathLinePoints(
   t_PathLinesPtr pathLines
   ) {
   return pathLines->m_pathPoints;
}

int GetGpsPathLinePointCount(
   t_PathLinesPtr pathLines
   ) {
   return pathLines->m_pointCounts;
}

/*
  the element x in the map is bigger than the topleft point and the y is smaller than the topleft point
  the formual: longitude = curlongi + x / ( 111.3 * math.cos( math.radians( curlat )) )/ 1000( parameter is m )
  latitude = curlat - y/111.3/1000
  Remember that the answer is ought to inhance 10^7 like 1200291593, the remember that the angle is like 120.0291593
*/
static int GetGpsLonFromDistance(
   int xRealLength,
   int xTopLeft,
   int yTopLeft
   ) {
   double x = xRealLength * pow( 10, 4 );
   double lona = yTopLeft/pow( 10,7 );
   double shift = x / ( 111 * cos( Angle2Radians( lona )) );
   shift = fabs( shift );
   int xGps = xTopLeft + ( int )shift;
   return xGps;
}

static void TurnEnvX2GpsLon(
   t_PathPointPtr pathPoint,
   t_EnvironmentPtr environment
   ) {
   int x = GetGpsPathPointLon( pathPoint ) * GetEnvLengthOfUnit( environment );
   int xTopLeft = GetEnvTopLeftLon( environment );
   int yTopLeft = GetEnvTopLeftLat( environment );
   int lonGps = GetGpsLonFromDistance( x, xTopLeft, yTopLeft );
   SetPathPointLon( lonGps, pathPoint );
}

static int GetGpsLatFromDistance(
   int yRealLength,
   int yTopLeft
   ) {
   double y = yRealLength * pow( 10, 4 );
   double shift = y / 111;
   int yGps = yTopLeft - ( int ) shift;
   return yGps;
}

static void TurnEnvY2GpsLat(
   t_PathPointPtr pathPoint,
   t_EnvironmentPtr environment
   ) {
   int y = GetGpsPathPointLat( pathPoint ) * GetEnvHeightOfUnit( environment );
   int yTopLeft = GetEnvTopLeftLat( environment );
   int latGps = GetGpsLatFromDistance( y, yTopLeft );
   SetPathPointLat( latGps, pathPoint );
}

static void TurnEnv2GpsPathLine(
   t_PathLinesPtr pathLine,
   t_EnvironmentPtr environment
   ) {
   for ( t_PathPointPtr pathPoints = pathLine->m_pathPoints; pathPoints != NULL; pathPoints = pathPoints->m_next ) {
      TurnEnvX2GpsLon( pathPoints, environment );
      TurnEnvY2GpsLat( pathPoints, environment );
   }
}

static t_PathLinesPtr GetFinalGpsPathLine(
   t_EnvPathLinePtr envPathLine,
   t_EnvironmentPtr environment
   ) {
   t_PathLinesPtr pathLine = GetGpsPathLineFromEnvPathLine( envPathLine );
   DebugCode(
      PrintInEnvPathLines( pathLine );
      );
   TurnEnv2GpsPathLine( pathLine, environment );
   return pathLine;
}

static int GetEnvXFromGpsPathMember(
   t_PathPointPtr positionMember,
   t_EnvironmentPtr environment
   ) {
   int lengthOfUnit = GetEnvLengthOfUnit( environment );
   int topLeftLon = GetEnvTopLeftLon( environment );
   int lonMember = GetGpsPathPointLon( positionMember );
   int latMember = GetGpsPathPointLat( positionMember );
   int disLon = CalGpsDistanceLon( lonMember, latMember, topLeftLon );
   if ( lonMember < topLeftLon ) {
      disLon = -1 *disLon;
   }
   return disLon / lengthOfUnit;
}

static int GetEnvYFromGpsPathMember(
   t_PathPointPtr positionMember,
   t_EnvironmentPtr environment
   ) {
   int heightOfUnit = GetEnvHeightOfUnit( environment );
   int topLeftLat = GetEnvTopLeftLat( environment );
   int lonMember = GetGpsPathPointLon( positionMember );
   int latMember = GetGpsPathPointLat( positionMember );
   int disLat = CalGpsDistanceLat( lonMember, latMember, topLeftLat );
   if ( latMember > topLeftLat ) {	/* if y is > 0 it should < the topleft */
      disLat = -1 *disLat;
   }
   return disLat / heightOfUnit;
}

static t_EnvPathLinePtr GetEnvPosFromGpsPos(
   t_PathLinesPtr positions,
   t_EnvironmentPtr environment
   ) {
   int length = GetGpsPathLinePointCount( positions );
   t_PathPointPtr currentPositionMem = GetGpsPathLinePoints( positions );
   t_EnvPathLinePtr envPathLine = NULL;

   for ( int i = 0; i < length; i++ ) {
      int x = GetEnvXFromGpsPathMember( currentPositionMem, environment );
      int y = GetEnvYFromGpsPathMember( currentPositionMem, environment );
      if ( IsEnvPointInEnv( x, y, environment ) ) {
         envPathLine = InsertNewEnvPathLine( x, y, envPathLine );
         currentPositionMem = GetGpsPathPointNext( currentPositionMem );
      } else {
         /* if the gps pos is not in the rectangle place, then return False */
         FreePathLine( envPathLine );
         return NULL;
      }
   }

   return envPathLine;
}

static int AstarInlocal(
   int xStart,
   int yStart,
   int xEnd,
   int yEnd,
   t_EnvironmentPtr environment
   ) {
   return DoSomePathPlanning( xStart, yStart, xEnd, yEnd, AStarPriority, ComparePriority, environment );
}

/* This routine use the A* algorithm to find the shortest path between the start and the end point with the unclean environment(may be used by the former partialpathsearch). It returns the path line in env if the search is success, otherwise it returns false */
static t_EnvPathLinePtr PartialPathSearch(
   int xStart,
   int yStart,
   int xEnd,
   int yEnd,
   t_EnvPathLinePtr envPathLine,
   t_EnvironmentPtr environment,
   int *isSearchSuccess
   ) {
   ResetEnvironment( environment );

   DebugCodeDetail (
      printf( "PartialPathSearch : Searching Between x %d y %d to x %d y %d\n", xStart, yStart, xEnd, yEnd );
      fflush( stdout );
      );

   if ( AstarInlocal( xStart, yStart, xEnd, yEnd, environment ) == 0 ) {

      DebugCodeDetail (
         printf( "PartialPathSearch : Search Between x %d y %d to x %d y %d is wrong\n", xStart, yStart, xEnd, yEnd );
         fflush( stdout );
         );

      *isSearchSuccess = 0;
      return envPathLine;
   }
   *isSearchSuccess = 1;
   envPathLine = StoreEnvPathLine( xEnd, yEnd, envPathLine, environment );
   return envPathLine;
}

/* This routine determines whether the start and end point is valid ( it's cor is in the environment and the corresponding point is not a obstacle ) */
static int IsStartAndEndPointValid(
   int xStart,
   int yStart,
   int xEnd,
   int yEnd,
   t_EnvironmentPtr environment
   ) {
   if ( IsEnvPointInEnv( xStart, yStart, environment ) &&
        !IsEnvMemberObstacle( GetEnvMember( xStart, yStart, environment ) ) &&
        IsEnvPointInEnv( xEnd, yEnd, environment ) ) {
      return 1;
   } else {
      return 0;
   }
}

/* This routine returns the path line which go through the free points which near the request points most. */
static t_PathLinesPtr GetFreePathLineThroughMultiPoints(
   t_EnvPathLinePtr distributedPoints,
   t_EnvironmentPtr environment
   ) {
   /* if no free points exist, then return false */
   if ( ChangeDistributedWithNearestFreePoints( distributedPoints, environment) == 0 ) {
      return NULL;
   }

   /* Do the path plan search between every neighbouring distribute points */
   t_EnvPathLinePtr member = distributedPoints;
   int xPrev = GetEnvPathMemberX( member );
   int yPrev = GetEnvPathMemberY( member );
   t_EnvPathLinePtr envPathLine = NULL;
   for ( member = GetEnvPathMemberPrev( member );
         member != NULL;
         member = GetEnvPathMemberPrev( member ) ) {
      int isSearchSuccess;
      envPathLine = PartialPathSearch( xPrev, yPrev,
                                       GetEnvPathMemberX( member ), GetEnvPathMemberY( member ),
                                       envPathLine, environment, &isSearchSuccess );
      /* handle the situation of no way */
      if ( isSearchSuccess == 0 ) {
         FreePathLine( envPathLine );
         return NULL;
      }
      xPrev = GetEnvPathMemberX( member );
      yPrev = GetEnvPathMemberY( member );
   }

   t_PathLinesPtr pathLines = GetFinalGpsPathLine( envPathLine, environment );
   FreePathLine( envPathLine );
   return pathLines;
}

/* This routine first get the Distributed Free(not a obstacle) points between the start points and end points, then pathplanning to get the free( no collision with the obstacle ) path line which through the points or some points near it. This routine return the final path line if the scansearch is success, otherwise return NULL */
t_PathLinesPtr ScanSearch(
   int xStart,
   int yStart,
   int xEnd,
   int yEnd,
   int width,
   t_EnvironmentPtr environment
   ) {
   if ( !IsStartAndEndPointValid( xStart, yStart, xEnd, yEnd, environment ) ) {
      return NULL;
   }

   t_EnvPathLinePtr distributedPoints = GetDistributedPoints( xStart, yStart,
                                                             xEnd, yEnd,
                                                             width );
   t_PathLinesPtr finalPathLine = GetFreePathLineThroughMultiPoints( distributedPoints, environment );
   FreePathLine( distributedPoints );
   return finalPathLine;
}

/* This routine return the path line which go through the request gps of the positions, or the points near the positions. It returns the final path line if success, otherwise return NULL */
t_PathLinesPtr MultiGpsPosPathPlan(
   t_PathLinesPtr positions,
   t_EnvironmentPtr environment
   ) {
   DebugCode(
      PrintPosPathLines( positions );
      fflush( stdout );
      );
   t_EnvPathLinePtr passedPoints = GetEnvPosFromGpsPos( positions, environment );
   /* If the giving gps pos is not valid then return flase */
   if ( passedPoints == NULL ) {
      return NULL;
   }
   DebugCode(
      PrintEntirePathMembers( passedPoints );
      fflush( stdout );
      );
   t_PathLinesPtr finalPathLine = GetFreePathLineThroughMultiPoints( passedPoints, environment );
   FreePathLine( passedPoints );
   return finalPathLine;
}
