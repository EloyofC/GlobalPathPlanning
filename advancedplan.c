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
#include "scansearchinrec.h"

#define c_angleDeltaPerRadians 0.04

typedef struct t_EnvPathLine
{
   int m_xIndex;
   int m_yIndex;
   struct t_EnvPathLine *m_next;
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

static t_EnvPathLinePtr GetEnvPathMemberNext(
   t_EnvPathLinePtr envPathLineMember
   ) {
   return envPathLineMember->m_next;
}

static void PrintDistributedPathMembers(
   t_EnvPathLinePtr envPathLine
   ) {
   int i = 0;

   for ( t_EnvPathLinePtr memberNext = envPathLine;
         memberNext != NULL;
         memberNext = memberNext->m_next )
      printf( "The distribute pass point : the %d nd point x %d y %d\n",
              i++, memberNext->m_xIndex, memberNext->m_yIndex );
}

static void PrintDistributedFreePathMembers(
   t_EnvPathLinePtr envPathLine
   ) {
   int i = 0;

   for ( t_EnvPathLinePtr memberNext = envPathLine;
         memberNext != NULL;
         memberNext = memberNext->m_next )
      printf( "The distributed free pass point : the %d nd point x %d y %d\n",
              i++, memberNext->m_xIndex, memberNext->m_yIndex );
}

static void FreePathLine(
   t_EnvPathLinePtr envPathLine
   ) {
   for ( t_EnvPathLinePtr memberNext = envPathLine; memberNext != NULL; ) {
      t_EnvPathLinePtr memberCurrent = memberNext;
      memberNext = GetEnvPathMemberNext( memberNext );
      Free( memberCurrent );
   }
}

static int IsDoubleNextEnvPathMemberExist(
   t_EnvPathLinePtr envPathLine
   ) {
   t_EnvPathLinePtr envPathLineNext = GetEnvPathMemberNext( envPathLine );
   return ( envPathLineNext != NULL ) &&
      ( GetEnvPathMemberNext( envPathLineNext ) != NULL );
}

static t_EnvPathLinePtr SkipNextEnvPathMember(
   t_EnvPathLinePtr envPathLine
   ) {
   assert( envPathLine != NULL );
   t_EnvPathLinePtr memberNext = GetEnvPathMemberNext( envPathLine );
   t_EnvPathLinePtr newEnvPathLine = envPathLine;
   newEnvPathLine->m_next = GetEnvPathMemberNext( memberNext );
   Free( memberNext );
   return newEnvPathLine;
}

static int IsEnvPathMemberEmpty(
   t_EnvPathLinePtr envPathLine
   ) {
   return envPathLine == NULL;
}

static int GetNextEnvPathMemberX(
   t_EnvPathLinePtr envPathLine
   ) {
   t_EnvPathLinePtr envPathLineNext = GetEnvPathMemberNext( envPathLine );
   return GetEnvPathMemberX( envPathLineNext );
}

static int GetNextEnvPathMemberY(
   t_EnvPathLinePtr envPathLine
   ) {
   t_EnvPathLinePtr envPathLineNext = GetEnvPathMemberNext( envPathLine );
   return GetEnvPathMemberY( envPathLineNext );
}

static int GetDoubleNextEnvPathMemberX(
   t_EnvPathLinePtr envPathLine
   ) {
   t_EnvPathLinePtr envPathLineNext = GetEnvPathMemberNext( envPathLine );
   t_EnvPathLinePtr envPathLineDoubleNext = GetEnvPathMemberNext( envPathLineNext );
   return GetEnvPathMemberX( envPathLineDoubleNext );
}

static int GetDoubleNextEnvPathMemberY(
   t_EnvPathLinePtr envPathLine
   ) {
   t_EnvPathLinePtr envPathLineNext = GetEnvPathMemberNext( envPathLine );
   t_EnvPathLinePtr envPathLineDoubleNext = GetEnvPathMemberNext( envPathLineNext );
   return GetEnvPathMemberY( envPathLineDoubleNext );
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
   int yHeuristic = GetEnvEndY( environment) - GetEnvMemberY( member );

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
   memberNew->m_next = envPathLine;

   return memberNew;
}

static t_EnvPathLinePtr CreateEnvPathLine(
   int x,
   int y
   ) {
   t_EnvPathLinePtr memberNew = Malloc( sizeof( struct t_EnvPathLine ) );
   memberNew->m_xIndex = x;
   memberNew->m_yIndex = y;
   memberNew->m_next = NULL;
   return memberNew;
}

t_EnvPathLinePtr InsertNewEnvPathLine(
   int x,
   int y,
   t_EnvPathLinePtr envPathLine
   ) {
   t_EnvPathLinePtr memberNew = CreateEnvPathLine( x, y );
   memberNew->m_next = envPathLine;

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

static t_EnvPathLinePtr StoreDistributedPointsTwoSidesXCor(
   const int xCurrent,
   const int xNext,
   const int yCurrent,
   const t_EnvPathLinePtr distributedPoints
   ) {
   t_EnvPathLinePtr newDistributedPoints = StoreNewDistributedMembers( xCurrent, yCurrent, distributedPoints );
   newDistributedPoints = StoreNewDistributedMembers( xNext, yCurrent, newDistributedPoints );
   return newDistributedPoints;
}

static t_EnvPathLinePtr GetLengthGoingUpDistributedPoints(
   const int xMin,
   const int xMax,
   const int yMax,
   const int xStart,
   const int yStart,
   const int width
   ) {
   /* xASide is the nearest cor to the xStart */
   int xASide;
   if ( abs( xMin - xStart ) <= abs( xMax - xStart )) {
      xASide = xMin;
   } else {
       xASide = xMax;
   }
   int xBSide = xMin + xMax - xASide;
   t_EnvPathLinePtr distributedPoints = NULL;
   /* store the first couple of points */
   distributedPoints = StoreDistributedPointsTwoSidesXCor( xStart, xBSide, yStart, distributedPoints );
   SwapNum( &xASide, &xBSide );

   /* store the members except the first and last */
   int yCurrent = yStart + width;
   for ( ; yCurrent < yMax; yCurrent += width ) {
      distributedPoints = StoreDistributedPointsTwoSidesXCor( xASide, xBSide, yCurrent, distributedPoints );
      SwapNum( &xASide, &xBSide );
   }

   /* store the last couple points */
   distributedPoints = StoreDistributedPointsTwoSidesXCor( xASide, xBSide, yMax, distributedPoints );

   return distributedPoints;
}

static t_EnvPathLinePtr GetLengthGoingDownDistributedPoints(
   const int xMin,
   const int yMin,
   const int xMax,
   const int xStart,
   const int yStart,
   const int width
   ) {
   /* xASide is the nearest cor to the xStart */
   int xASide;
   if ( abs( xMin - xStart ) <= abs( xMax - xStart ) ) {
      xASide = xMin;
   } else {
       xASide = xMax;
   }
   int xBSide = xMin + xMax - xASide;
   t_EnvPathLinePtr distributedPoints = NULL;
   /* store the first couple of points */
   distributedPoints = StoreDistributedPointsTwoSidesXCor( xStart, xBSide, yStart, distributedPoints );
   SwapNum( &xASide, &xBSide );

   /* store the members except the first and last */
   int yCurrent = yStart - width;
   for ( ; yCurrent > yMin; yCurrent -= width ) {
      distributedPoints = StoreDistributedPointsTwoSidesXCor( xASide, xBSide, yCurrent, distributedPoints );
      SwapNum( &xASide, &xBSide );
   }

   /* store the last couple points */
   distributedPoints = StoreDistributedPointsTwoSidesXCor( xASide, xBSide, yMin, distributedPoints );

   return distributedPoints;
}

static t_EnvPathLinePtr StoreDistributedPointsTwoSidesYCor(
   const int xCurrent,
   const int yCurrent,
   const int yNext,
   const t_EnvPathLinePtr distributedPoints
   ) {
   t_EnvPathLinePtr newDistributedPoints = StoreNewDistributedMembers( xCurrent, yCurrent, distributedPoints );
   newDistributedPoints = StoreNewDistributedMembers( xCurrent, yNext, newDistributedPoints );
   return newDistributedPoints;
}

static t_EnvPathLinePtr GetHeightGoingUpDistributedPoints(
   const int yMin,
   const int xMax,
   const int yMax,
   const int xStart,
   const int yStart,
   const int width
   ) {
   int yASide;
   if ( abs( yMin - yStart ) <= abs( yMax - yStart ) ) {
      yASide = yMin;
   } else {
      yASide = yMax;
   }
   int yBSide = yMin + yMax - yASide;
   t_EnvPathLinePtr distributedPoints = NULL;
   distributedPoints = StoreDistributedPointsTwoSidesYCor( xStart, yStart, yBSide, distributedPoints );
   SwapNum( &yASide, &yBSide );

   int xCurrent = xStart + width;
   for ( ; xCurrent < xMax; xCurrent += width ) {
      distributedPoints = StoreDistributedPointsTwoSidesYCor( xCurrent, yASide, yBSide, distributedPoints );
      SwapNum( &yASide, &yBSide );
   }

   distributedPoints = StoreDistributedPointsTwoSidesYCor( xMax, yASide, yBSide, distributedPoints );

   return distributedPoints;
}

static t_EnvPathLinePtr GetHeightGoingDownDistributedPoints(
   const int xMin,
   const int yMin,
   const int yMax,
   const int xStart,
   const int yStart,
   const int width
   ) {
   int yASide;
   if ( abs( yMin - yStart ) <= abs( yMax - yStart ) ) {
      yASide = yMin;
   } else {
      yASide = yMax;
   }
   int yBSide = yMin + yMax - yASide;
   t_EnvPathLinePtr distributedPoints = NULL;
   distributedPoints = StoreDistributedPointsTwoSidesYCor( xStart, yStart, yBSide, distributedPoints );
   SwapNum( &yASide, &yBSide );

   int xCurrent = xStart - width;
   for ( ; xCurrent > xMin; xCurrent -= width ) {
      distributedPoints = StoreDistributedPointsTwoSidesYCor( xCurrent, yASide, yBSide, distributedPoints );
      SwapNum( &yASide, &yBSide );
   }

   distributedPoints = StoreDistributedPointsTwoSidesYCor( xMin, yASide, yBSide, distributedPoints );
   return distributedPoints;
}

static t_EnvPathLinePtr StoreDistributedPoints(
   const int xMin,
   const int yMin,
   const int xMax,
   const int yMax,
   const int xStart,
   const int yStart,
   const int width,
   const unsigned char isScanLineHorizon
   ) {
   int isGoUp;
   if ( abs( yStart - yMin ) <= abs( yStart - yMax ) ) {
      isGoUp = 1;
   } else {
      isGoUp = 0;
   }

   if ( isScanLineHorizon ) {
      if ( isGoUp ) {
         return GetLengthGoingUpDistributedPoints( xMin, xMax, yMax, xStart, yStart, width );
      } else {
         return GetLengthGoingDownDistributedPoints( xMin, yMin, xMax, xStart, yStart, width );
      }
   } else {
      if ( isGoUp ) {
         return GetHeightGoingUpDistributedPoints( yMin, xMax, yMax, xStart, yStart, width );
      } else {
         return GetHeightGoingDownDistributedPoints( xMin, yMin, yMax, xStart, yStart, width );
      }
   }
}

/* This routine get the distributed points with the request of the scan search */
static t_EnvPathLinePtr GetDistributedPoints(
   const int xStart,
   const int yStart,
   const int xEnd,
   const int yEnd,
   const int width,
   const unsigned char isScanLineHorizon
   ) {
   int min_x = xStart < xEnd ? xStart : xEnd;
   int min_y = yStart < yEnd ? yStart : yEnd;
   int max_x = xStart + xEnd - min_x;
   int max_y = yStart + yEnd - min_y;
   t_EnvPathLinePtr distributedPoints = StoreDistributedPoints( min_x, min_y,
                                                                max_x, max_y,
                                                                xStart, yStart,
                                                                width, isScanLineHorizon );
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
   int x[ ] = {
      -1, 1, 0, 0
   };	/* search in four directions */
   int y[ ] = {
      0, 0, -1, 1
   };
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
   assert( environment != NULL );

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

/* This routine change the current cordinatation of the point to the nearest free point.
   It return False if no nearest free point exist  */
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
   for ( t_EnvPathLinePtr member = distributedPoints;
         member != NULL;
         member = GetEnvPathMemberNext( member ) ) {
      /* if no nearest free point exist then return False */
      if ( UpdatePoint2NearestFreePoint( member, environment ) == 0 )
         return 0;
   }
   return 1;
}

/* This routine append the second pathline to the end of the first one */
static t_EnvPathLinePtr AppendEnvPathLine(
   t_EnvPathLinePtr envPathLineFirst,
   t_EnvPathLinePtr envPathLineSecond
   ) {
   if ( envPathLineFirst != NULL ) {
      t_EnvPathLinePtr linePrev = envPathLineFirst;
      for ( ; GetEnvPathMemberNext( linePrev ) != NULL; linePrev = GetEnvPathMemberNext( linePrev ) )
         ;
      linePrev->m_next = envPathLineSecond;
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
         printf( "StoreEnvPathLine : %d nd search Searched Member has X %d, Y %d, Cost %d\n",
                 i++, GetEnvMemberX( memberPrev ), GetEnvMemberY( memberPrev ),
                 GetEnvMemberCost( memberPrev ) );
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
   assert( pathLine != NULL );

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
         printf( "%s : the %d nd point x %d y %d\n",
                 str, i, GetGpsPathPointLon( pathPoints ), GetGpsPathPointLat( pathPoints ) );
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

static int IsTwoPointEqual(
   int x1,
   int y1,
   int x2,
   int y2
   ) {
   return ( ( x1 == x2 ) &&
            ( y1 == y2 ) );
}

static int IsThreePointInALine(
   int x1,
   int y1,
   int x2,
   int y2,
   int x3,
   int y3
   ) {
   if ( ( x1 == x2 ) && ( x2 == x3 ) ) {
      /* just take care of the condition that delta of x is zero */
      return 1;
   } else if ( ( x1 == x2 ) || ( x1 == x3 ) || ( x2 == x3 ) ) {
      if ( IsTwoPointEqual( x1, y1, x2, y2 ) ||
           IsTwoPointEqual( x1, y1, x3, y3 ) ||
           IsTwoPointEqual( x2, y2, x3, y3 ) ) {
         return 1;
      } else {
      return 0;
      }
   } else {
      double delta1 = ( double ) abs( y2 - y1 ) / ( double ) abs( x2 - x1 );
      double delta2 = ( double ) abs( y3 - y2 ) / ( double ) abs( x3 - x2 );
      return IsDoubleEqual( delta1, delta2 );
   }
}

static int IsThreePointInADirection(
   int x1,
   int y1,
   int x2,
   int y2,
   int x3,
   int y3
   ) {
   if ( ( x2 - x1 ) * ( x3 - x2 ) >= 0 &&
        ( y2 - y1 ) * ( y3 - y2 ) >= 0 ) {
      return 1;
   } else {
      return 0;
   }
}

static int IsThreePointInABeam(
   int x1,
   int y1,
   int x2,
   int y2,
   int x3,
   int y3
   ) {
   return ( IsThreePointInALine( x1, y1, x2, y2, x3, y3 ) &&
            IsThreePointInADirection( x1, y1, x2, y2, x3, y3 ) );
}

/* This routine filter the next member that their cordinatation in a line */
static t_EnvPathLinePtr FilterInBeamPathLineNext(
   t_EnvPathLinePtr envPathLine
   ) {
   assert( envPathLine != NULL );

   t_EnvPathLinePtr currentEnvPathLine = envPathLine;
   while ( IsDoubleNextEnvPathMemberExist( currentEnvPathLine ) ) {
      if ( IsThreePointInABeam( GetEnvPathMemberX( currentEnvPathLine ),
                                GetEnvPathMemberY( currentEnvPathLine ),
                                GetNextEnvPathMemberX( currentEnvPathLine ),
                                GetNextEnvPathMemberY( currentEnvPathLine ),
                                GetDoubleNextEnvPathMemberX( currentEnvPathLine ),
                                GetDoubleNextEnvPathMemberY( currentEnvPathLine ) ) ) {
         currentEnvPathLine = SkipNextEnvPathMember( currentEnvPathLine );
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
      PrintDistributedPathMembers( envPathLine );
      );
   for ( t_EnvPathLinePtr currentEnvPathLine = envPathLine;
         !IsEnvPathMemberEmpty( currentEnvPathLine );
         currentEnvPathLine =  GetEnvPathMemberNext( currentEnvPathLine ) ) {
      int x = GetEnvPathMemberX( currentEnvPathLine );
      int y = GetEnvPathMemberY( currentEnvPathLine );
      DebugCodeDetail(
         printf( "GetGpsPathLineFromEnvPathLine : x %d y %d\n", x, y);
         fflush( stdout );
         );
      InsertNewGpsPathPoint( x, y, pathLine );
      currentEnvPathLine = FilterInBeamPathLineNext( currentEnvPathLine );
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
   for ( t_PathPointPtr pathPoints = pathLine->m_pathPoints;
         pathPoints != NULL;
         pathPoints = pathPoints->m_next ) {
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
   int lonMember = GetGpsPathPointLon( positionMember );
   return GetEnvXFromGpsLon( lonMember, environment );
}

static int GetEnvYFromGpsPathMember(
   t_PathPointPtr positionMember,
   t_EnvironmentPtr environment
   ) {
   int latMember = GetGpsPathPointLat( positionMember );
   return GetEnvYFromGpsLat( latMember, environment );
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
         DebugCode(
            printf( "GetEnvPosFromGpsPos : %d nd x %d y %d is not in env\n", i, x, y );
            fflush( stdout );
            );
         FreePathLine( envPathLine );
         return NULL;
      }
   }

   return envPathLine;
}

static int GetGpsCircleCenterLon(
   struct t_ExpectedCruiseCricle circle
   ) {
   return circle.m_lonCircleCenter;
}

static int GetGpsCircleCenterLat(
   struct t_ExpectedCruiseCricle circle
   ) {
   return circle.m_latCircleCenter;
}

static int GetCircleRadius(
   struct t_ExpectedCruiseCricle circle
   ) {
   return circle.m_circleRadius;
}

static int IsCircleClockWise(
   struct t_ExpectedCruiseCricle circle
   ) {
   return circle.m_isClockWise;
}

static int GetCircleCenterX(
   struct t_ExpectedCruiseCricle circle,
   t_EnvironmentPtr environment
   ) {
   int lonCircleCenter = GetGpsCircleCenterLon( circle );
   return GetEnvXFromGpsLon( lonCircleCenter, environment );
}
static int GetCircleCenterY(
   struct t_ExpectedCruiseCricle circle,
   t_EnvironmentPtr environment
   ) {
   int latCircleCenter = GetGpsCircleCenterLat( circle );
   return GetEnvYFromGpsLat( latCircleCenter, environment );
}

/* This routine get the cor of point through solving the equation of :
   1. k = tan(alpha)
   2. y - y0 = k( x - x0 )
   3. ( x - x0 )^2/a^2 + ( y - y0 )^2/b^2 = 1
   The solution is :
   |x| = a * b/sqrt( b^2 + a^2 * k^2 ) + x0
   |y| = k( x - x0 ) + y0
   To be simplity: we declare that
   the value of x and y is according to the quadrant
*/
static void GetPointCorInOval(
   double angleCurrent,
   int circleCenterX,
   int circleCenterY,
   double axisX,
   double axisY,
   int *pointX,
   int *pointY
   ) {
   double gradient = tan( angleCurrent );
   double denominator = sqrt( IntSquare( axisY ) + IntSquare( axisX * gradient ) );
   double deltaX = axisX * axisY / denominator;
   double deltaY = deltaX * fabs( gradient );

   double x, y;
   if ( angleCurrent >= 0 &&
        angleCurrent < c_pi/2 ) {
      /* first quadrant */
      x = circleCenterX + deltaX;
      y = circleCenterY + deltaY;
   } else if ( angleCurrent >= c_pi/2 &&
               angleCurrent < c_pi ) {
      /* second quadrant */
      x = circleCenterX - deltaX;
      y = circleCenterY + deltaY;
   } else if ( angleCurrent >= -1 * c_pi /2 &&
               angleCurrent < 0 ) {
      /* fourth quadrant */
      x = circleCenterX + deltaX;
      y = circleCenterY - deltaY;
   } else {
      /* third quadrant */
      x = circleCenterX - deltaX;
      y = circleCenterY - deltaY;
   }
   *pointX = x;
   *pointY = y;
}

/* close in left and open in right */
static double Turn2NormalRadiansRange(
   double radians
   ) {
   double min = -1 * c_pi;
   while ( ( radians < min ) ) {
      radians += 2 * c_pi;
   }
   double max = c_pi;
   while (( radians >= max ) ) {
      radians -= 2 * c_pi;
   }
   return radians;
}

static t_EnvPathLinePtr GetEnvOvalDistributedPoints(
   double radiansStart,              /* the para is in the radians */
   double radiansDelta,
   int circleCenterX,
   int circleCenterY,
   double axisX,
   double axisY,
   t_EnvironmentPtr environment
   ) {
   int count = 2 * c_pi / fabs( radiansDelta );
   double radiansCurrent = Turn2NormalRadiansRange( radiansStart );
   int firstX, firstY;
   GetPointCorInOval( radiansCurrent, circleCenterX, circleCenterY,
                      axisX, axisY, &firstX, &firstY );
   t_EnvPathLinePtr distributedPoints = NULL;
   for ( int i = 0; i < count; i++ ) {
      int pointX, pointY;
      GetPointCorInOval( radiansCurrent,
                         circleCenterX, circleCenterY,
                         axisX, axisY,
                         &pointX, &pointY );
      DebugCode(
      printf( "GetEnvOvalDistributedPoints : i %d nd radians %f x %d y %d \n",
              i, radiansCurrent, pointX, pointY );
      fflush( stdout );
         );
      if ( !IsEnvPointInEnv( pointX, pointY, environment ) ) {
         DebugCode(
            printf( "GetEnvOvalDistributedPoints : x %d y %d is not in env\n", pointX, pointY );
            fflush( stdout );
            );
         FreePathLine( distributedPoints );
         return NULL;
      }
      distributedPoints = InsertNewEnvPathLine( pointX, pointY, distributedPoints );
      radiansCurrent = Turn2NormalRadiansRange( radiansCurrent + radiansDelta );
   }
   distributedPoints = InsertNewEnvPathLine( firstX, firstY, distributedPoints );
   return distributedPoints;
}

static t_EnvPathLinePtr GetCircleDistributedPoints(
   int circleCenterX,
   int circleCenterY,
   int circleRadius,
   unsigned char isClockWise,
   t_EnvironmentPtr environment
   ) {
   double axisX = ( double )circleRadius / GetEnvLengthOfUnit( environment );
   double axisY = ( double )circleRadius / GetEnvHeightOfUnit( environment );
   DebugCode(
      printf( "GetCircleDistributedPoints : circle center x %d y %d\n",
              circleCenterX, circleCenterY );
      fflush( stdout );
      );
   double radiansDelta = isClockWise == 1 ? ( fabs( c_angleDeltaPerRadians ) * -1 ) :
      fabs( c_angleDeltaPerRadians );
   double radiansStart = 0;     /* may be cal according to the start point */
   return GetEnvOvalDistributedPoints( radiansStart, radiansDelta,
                                       circleCenterX, circleCenterY,
                                       axisX, axisY,
                                       environment );
}

static int AstarInlocal(
   int xStart,
   int yStart,
   int xEnd,
   int yEnd,
   t_EnvironmentPtr environment
   ) {
   return DoSomePathPlanning( xStart, yStart,
                              xEnd, yEnd,
                              AStarPriority, ComparePriority, environment );
}

/* This routine use the A* algorithm to find the shortest path
   between the start and the end point with the unclean environment
   (may be used by the former partialpathsearch).
   It returns the path line in env if the search is success, otherwise it returns false */
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
      printf( "PartialPathSearch : Searching Between x %d y %d to x %d y %d\n",
              xStart, yStart, xEnd, yEnd );
      fflush( stdout );
      );

   if ( AstarInlocal( xStart, yStart, xEnd, yEnd, environment ) == 0 ) {

      DebugCodeDetail (
         printf( "PartialPathSearch : Search Between x %d y %d to x %d y %d is wrong\n",
                 xStart, yStart, xEnd, yEnd );
         fflush( stdout );
         );

      *isSearchSuccess = 0;
      return envPathLine;
   }
   *isSearchSuccess = 1;
   envPathLine = StoreEnvPathLine( xEnd, yEnd, envPathLine, environment );
   return envPathLine;
}

/* This routine determines whether the start and end point is valid
   ( it's cor is in the environment and the corresponding point is not a obstacle ) */
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

/* This routine returns the path line which go through the given points
   which near the request points most. */
static t_PathLinesPtr GetFinalPathLineThroughMultiPoints(
   t_EnvPathLinePtr distributedPoints,
   t_EnvironmentPtr environment
   ) {
   /* Do the path plan search between every neighbouring distribute points */
   t_EnvPathLinePtr member = distributedPoints;
   int xPrev = GetEnvPathMemberX( member );
   int yPrev = GetEnvPathMemberY( member );
   t_EnvPathLinePtr envPathLine = NULL;
   for ( member = GetEnvPathMemberNext( member );
         member != NULL;
         member = GetEnvPathMemberNext( member ) ) {
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

/* This routine first get the Distributed Free(not a obstacle) points
   between the start points and end points, then pathplanning to get the free
   ( no collision with the obstacle ) path line which through the points or some points near it.
   This routine return the final path line if the scansearch is success, otherwise return NULL */
t_PathLinesPtr ScanSearch(
   int xStart,
   int yStart,
   int xEnd,
   int yEnd,
   int width,
   unsigned char isScanLineHorizon,
   t_EnvironmentPtr environment
   ) {
   if ( !IsStartAndEndPointValid( xStart, yStart, xEnd, yEnd, environment ) ) {
      return NULL;
   }

   t_EnvPathLinePtr distributedPoints = GetDistributedPoints( xStart, yStart,
                                                             xEnd, yEnd,
                                                              width, isScanLineHorizon );

   DebugCode(
      PrintDistributedPathMembers( distributedPoints );
      fflush( stdout );
      );
   /* if no free points exist, then return false */
   if ( ChangeDistributedWithNearestFreePoints( distributedPoints, environment) == 0 ) {
      return NULL;
   }

   DebugCode(
      PrintDistributedFreePathMembers( distributedPoints );
      fflush( stdout );
      );

   t_PathLinesPtr finalPathLine = GetFinalPathLineThroughMultiPoints( distributedPoints, environment );
   FreePathLine( distributedPoints );
   return finalPathLine;
}

t_PathLinesPtr ScanSearchWithANN(
   int xStart,
   int yStart,
   unsigned char isScanLineHorizon,
   t_EnvironmentPtr environment
   ) {
   t_EnvPathLinePtr envPathLine = DoScanSearchInRec( xStart, yStart, isScanLineHorizon, environment );

   DebugCode(
      PrintDistributedFreePathMembers( envPathLine );
      fflush( stdout );
      );

   t_PathLinesPtr finalPathLine = GetFinalGpsPathLine( envPathLine, environment );
   FreePathLine( envPathLine );
   return finalPathLine;
}


t_PathLinesPtr CircleCruisePathPlan(
   struct t_ExpectedCruiseCricle circle,
   t_EnvironmentPtr environment
   ) {
   int circleCenterX = GetCircleCenterX( circle, environment );
   int circleCenterY = GetCircleCenterY( circle, environment );
   int circleRadius = GetCircleRadius( circle );
   unsigned char isClockWise = IsCircleClockWise( circle );
   t_EnvPathLinePtr distributedPoints = GetCircleDistributedPoints( circleCenterX, circleCenterY,
                                                                    circleRadius, isClockWise,
                                                                    environment );
   if ( distributedPoints == NULL ) {
      return NULL;
   }

   DebugCode(
      PrintDistributedPathMembers( distributedPoints );
      fflush( stdout );
      );
   if ( ChangeDistributedWithNearestFreePoints( distributedPoints, environment) == 0 ) {
      return NULL;
   }

   DebugCode(
      PrintDistributedFreePathMembers( distributedPoints );
      fflush( stdout );
      );

   t_PathLinesPtr finalPathLine = GetFinalPathLineThroughMultiPoints( distributedPoints, environment );
   FreePathLine( distributedPoints );
   return finalPathLine;
}

/* This routine return the path line which go through the request gps of the positions,
   or the points near the positions.
   It returns the final path line if success, otherwise return NULL */
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
      PrintDistributedPathMembers( passedPoints );
      fflush( stdout );
      );
   /* if no free points exist, then return false */
   if ( ChangeDistributedWithNearestFreePoints( passedPoints, environment) == 0 ) {
      return NULL;
   }

   DebugCode(
      PrintDistributedFreePathMembers( passedPoints );
      fflush( stdout );
      );

   t_PathLinesPtr finalPathLine = GetFinalPathLineThroughMultiPoints( passedPoints, environment );
   FreePathLine( passedPoints );
   return finalPathLine;
}
