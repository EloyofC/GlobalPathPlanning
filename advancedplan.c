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
   static int i = 0;

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

/* can add some restraint of keep the same direction with the previous and stay at the same y( or may be the line of start and the end ) */
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

static t_EnvPathLinePtr StoreNewPathMember(
   t_EnvironmentMemberPtr envMember,
   t_EnvPathLinePtr envPathLine
   ) {
   t_EnvPathLinePtr memberNew = Malloc( sizeof( struct t_EnvPathLine ) );
   memberNew->m_xIndex = GetEnvMemberX( envMember );
   memberNew->m_yIndex = GetEnvMemberY( envMember );
   memberNew->m_prevPtr = envPathLine;

   return memberNew;
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
   int width,
   t_EnvironmentPtr environment
   ) {
   assert( IsEnvPointInEnv( xStart, yStart, environment ) &&
           !IsEnvMemberObstacle( GetEnvMember( xStart, yStart, environment ) ) );
   assert( IsEnvPointInEnv( xEnd, yEnd, environment ) &&
           !IsEnvMemberObstacle( GetEnvMember( xStart, yStart, environment ) ) );

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

   for ( ; IsEnvMemberObstacle( member ); member = DeFifoQueue( nearQueue ) ) {
      nearQueue = SearchFourNeighbour( member, nearQueue, environment );
   }
   FreeFifoQueue( nearQueue );
   return member;
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

static t_PathLinesPtr CreateFinalPathLine(
   void
   ) {
   t_PathLinesPtr finalPathLineNew = Malloc( sizeof( struct t_PathLines ) );
   finalPathLineNew->m_pointCounts = 0;
   finalPathLineNew->m_pathPoints = NULL;
   return finalPathLineNew;
}

static void InsertNewFinalPathPoint(
   int x,
   int y,
   t_PathLinesPtr finalPathLine
   ) {
   t_PathPointPtr finalPointNew = Malloc( sizeof( struct t_PathPoint ) );
   finalPointNew->m_lon = x;
   finalPointNew->m_lat = y;
   finalPointNew->m_next = finalPathLine->m_pathPoints;
   finalPathLine->m_pathPoints = finalPointNew;
   finalPathLine->m_pointCounts++;
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

static t_PathLinesPtr GetFinalPathLine(
   t_EnvPathLinePtr envPathLine
   ) {
   t_PathLinesPtr finalPathLine = CreateFinalPathLine();
   DebugCode(
      PrintEntirePathMembers( envPathLine );
      );
   for ( t_EnvPathLinePtr currentEnvPathLine = envPathLine;
         !IsEnvPathMemberEmpty( currentEnvPathLine );
         currentEnvPathLine =  SkipPathLineMember( currentEnvPathLine ) ) {
      int x = GetEnvPathMemberX( currentEnvPathLine );
      int y = GetEnvPathMemberY( currentEnvPathLine );
      printf( "GetFinalPathLine : x %d y %d\n", x, y);
      fflush( stdout );
      InsertNewFinalPathPoint( x, y, finalPathLine );
      currentEnvPathLine = FilterInALinePathLineMembers( currentEnvPathLine );
   }
   return finalPathLine;
}

static int GetFinalPathPointX(
   t_PathPointPtr finalPathPoint
   ) {
   return finalPathPoint->m_lon;
}

static int GetFinalPathPointY(
   t_PathPointPtr finalPathPoint
   ) {
   return finalPathPoint->m_lat;
}

static void SetFinalPathPointX(
   int x,
   t_PathPointPtr finalPathPoint
   ) {
   finalPathPoint->m_lon = x;
}

static void SetFinalPathPointY(
   int y,
   t_PathPointPtr finalPathPoint
   ) {
   finalPathPoint->m_lat = y;
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
   t_PathPointPtr finalPathPoint,
   t_EnvironmentPtr environment
   ) {
   int x = GetFinalPathPointX( finalPathPoint ) * GetEnvLengthOfUnit( environment );
   int xTopLeft = GetEnvTopLeftLon( environment );
   int yTopLeft = GetEnvTopLeftLat( environment );
   int lonGps = GetGpsLonFromDistance( x, xTopLeft, yTopLeft );
   SetFinalPathPointX( lonGps, finalPathPoint );
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
   t_PathPointPtr finalPathPoint,
   t_EnvironmentPtr environment
   ) {
   int y = GetFinalPathPointY( finalPathPoint ) * GetEnvWidthOfUnit( environment );
   int yTopLeft = GetEnvTopLeftLat( environment );
   int latGps = GetGpsLatFromDistance( y, yTopLeft );
   SetFinalPathPointY( latGps, finalPathPoint );
}

static void TurnEnv2GpsPathLine(
   t_PathLinesPtr finalPathLine,
   t_EnvironmentPtr environment
   ) {
   for ( t_PathPointPtr finalPathPoints = finalPathLine->m_pathPoints; finalPathPoints != NULL; finalPathPoints = finalPathPoints->m_next ) {
      TurnEnvX2GpsLon( finalPathPoints, environment );
      TurnEnvY2GpsLat( finalPathPoints, environment );
   }
}

static t_PathLinesPtr GetGpsPathLine(
   t_EnvPathLinePtr envPathLine,
   t_EnvironmentPtr environment
   ) {
   t_PathLinesPtr finalPathLine = GetFinalPathLine( envPathLine );
   DebugCode (
      PrintEnvPathLines( finalPathLine );
      );
   TurnEnv2GpsPathLine( finalPathLine, environment );
   return finalPathLine;
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
   t_EnvironmentPtr environment
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

      return NULL;
   }
   envPathLine = StoreEnvPathLine( xEnd, yEnd, envPathLine, environment );
   return envPathLine;
}
static int IsStartAndEndPointValid(
   int xStart,
   int yStart,
   int xEnd,
   int yEnd,
   t_EnvironmentPtr environment
   ) {
   if ( IsEnvPointInEnv( xStart, yStart, environment ) &&
        !IsEnvMemberObstacle( GetEnvMember( xStart, yStart, environment ) ) &&
        IsEnvPointInEnv( xEnd, yEnd, environment ) &&
        !IsEnvMemberObstacle( GetEnvMember( xEnd, yEnd, environment ) ) ) {
      return 1;
   } else {
      return 0;
   }
}

/* This routine first get the Distributed Free(not a obstacle) points between the start points and end points, then use the path plan search for the every neighbouring distributed points. This routine return the final path line if the scansearch is success, otherwise return NULL */
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
                                                             width, environment );

   /* if no free points exist, then return false */
   if ( ChangeDistributedWithNearestFreePoints( distributedPoints, environment) == 0 ) {
      FreePathLine( distributedPoints );
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
       envPathLine = PartialPathSearch( xPrev, yPrev,
                                        GetEnvPathMemberX( member ), GetEnvPathMemberY( member ),
                                        envPathLine, environment );
       /* handle the situation of no way */
      if ( envPathLine == NULL ) {
         FreePathLine( distributedPoints );
         FreePathLine( envPathLine );
         return NULL;
      }
      xPrev = GetEnvPathMemberX( member );
      yPrev = GetEnvPathMemberY( member );
   }

   FreePathLine( distributedPoints );
   t_PathLinesPtr pathLines = GetGpsPathLine( envPathLine, environment );
   FreePathLine( envPathLine );
   return pathLines;
}
