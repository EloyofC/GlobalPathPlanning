#include <stdio.h>
#include "publicfun.h"
#include "advancedplan.h"
#include "mission.h"

#define c_IsHorizon 1
#define c_IsVertical 2

static t_PointCorPtr CreatePointArray(
   int size
   ) {
   return ( t_PointCorPtr )Malloc( size * sizeof( struct t_PointCor ));
}

static void SetSingleObstacleCount(
   int count,
   t_SingleObstaclePtr singleObstacle
   ) {
   singleObstacle->m_vertexCounts = count;
}

static int GetObstacleCount(
   t_ObstaclesPtr obstacles
   ) {
   return obstacles->m_obstacleCounts;
}

static t_SingleObstaclePtr CreateSingleObstacleArray(
   int size
   ) {
   return ( t_SingleObstaclePtr )Malloc( size * sizeof( struct t_SingleObstacle ));
}

static t_SingleObstaclePtr GetSingleObstacleArray(
   t_ObstaclesPtr obstacles
   ) {
   return obstacles->m_obstacleMembersPtr;
}

static t_PointCorPtr GetPointArray(
   t_SingleObstaclePtr singleObstacle
   ) {
   return singleObstacle->m_pointsPtr;
}

static t_SingleObstaclePtr GetSingleObstacle(
   int index, t_ObstaclesPtr obstacles
   ) {
   return obstacles->m_obstacleMembersPtr + index;
}

static void FreeSingleObstacle(
   t_SingleObstaclePtr singleObstacle
   ) {
   Free( GetPointArray( singleObstacle ));
}

static t_ObstaclesPtr CreateObstacles(
   void
   ) {
   return ( t_ObstaclesPtr )Malloc( sizeof( struct t_Obstacles ));
}

static void FreeObstacles(
   t_ObstaclesPtr obstacles
   ) {
   if ( obstacles == NULL ) {
      return;
   }

   int count = GetObstacleCount( obstacles );
   t_SingleObstaclePtr singleObstacle;

   for ( int i=0; i < count; i++ ) {
      singleObstacle = GetSingleObstacle( i, obstacles );
      FreeSingleObstacle( singleObstacle );
   }
   Free( GetSingleObstacleArray( obstacles ));
   Free( obstacles );
}

static void SetObstaclesCount(
   int count,
   t_ObstaclesPtr obstacles
   ) {
   obstacles->m_obstacleCounts = count;
}

static t_PointCorPtr GetVertex(
   int i,
   t_SingleObstaclePtr singleObstacle
   ) {
   return singleObstacle->m_pointsPtr + i;
}

static t_PointCorPtr GetNextPoint(
   t_PointCorPtr point
   ) {
   return point + 1;
}

static void InsertPoint(
   int lon,
   int lat,
   t_PointCorPtr point
   ) {
   point->m_lon = lon;
   point->m_lat = lat;
}


static void InsertSingleObstacle(
   unsigned int size,
   int *x,
   int *y,
   t_SingleObstaclePtr singleObstacle
   ) {
   SetSingleObstacleCount( size, singleObstacle );
   t_PointCorPtr point = singleObstacle->m_pointsPtr = CreatePointArray( size );
   for ( unsigned int i = 0; i < size; i++ ) {
      InsertPoint( x[i], y[i], point );
      point = GetNextPoint( point );
   }
}

static struct t_RectangleArea SetInRectangle(
   int lonTopLeft,
   int latTopLeft,
   int lonBottomRight,
   int latBottomRight
   ) {
   struct t_RectangleArea rectangle;
   rectangle.m_lonTopLeft = lonTopLeft;
   rectangle.m_latTopLeft = latTopLeft;
   rectangle.m_lonBottomRight = lonBottomRight;
   rectangle.m_latBottomRight = latBottomRight;
   return rectangle;
}

static struct t_ScanWidthInfo SetInWidthInfo(
   int width,
   int isHorizon
   ) {
   struct t_ScanWidthInfo widthInfo;
   widthInfo.m_width = width;
   widthInfo.m_isHorizon = isHorizon;
   return widthInfo;
}

static int TestTemplateInRec(
   int lonTopLeft,
   int latTopLeft,
   int lonBottomRight,
   int latBottomRight,
   int width,
   int isHorizon,
   t_ObstaclesPtr obstacles
   ) {
   DebugCode(
      printf( "The start point is x %d y %d\n", lonTopLeft, latTopLeft );
      printf( "The end point is x %d y %d\n", lonBottomRight, latBottomRight );
      );
   struct t_RectangleArea rectangle = SetInRectangle( lonTopLeft, latTopLeft,
                                                      lonBottomRight, latBottomRight );
   struct t_ScanWidthInfo widthInfo = SetInWidthInfo( width, isHorizon );
   t_PathLinesPtr pathLines = GetScanLinesInRec( lonTopLeft, latTopLeft,
                                                 lonBottomRight, latBottomRight,
                                                 rectangle, widthInfo, obstacles );
   FreeObstacles( obstacles );
   if ( pathLines != NULL ) {
      DebugCode(
         PrintFinalGpsPathLines( pathLines );
         );
      FreeFinalPathLines( pathLines );
      printf( "Success\n" );
   } else {
      printf( "Sad\n" );
   }
   fflush( stdout );
   return 0;
}

static int TestTemplateWithMultiPos(
   t_PathLinesPtr positions,
   int lonTopLeft,
   int latTopLeft,
   int lonBottomRight,
   int latBottomRight,
   t_ObstaclesPtr obstacles
   ) {
   DebugCode(
      printf( "The start point is x %d y %d\n", lonTopLeft, latTopLeft );
      printf( "The end point is x %d y %d\n", lonBottomRight, latBottomRight );
      );
   struct t_RectangleArea rectangle = SetInRectangle( lonTopLeft, latTopLeft,
                                                      lonBottomRight, latBottomRight );
   t_PathLinesPtr pathLines = GetPointsWithFixedMultiPosition( positions, rectangle, obstacles );
   FreeObstacles( obstacles );
   FreeFinalPathLines( positions );
   if ( pathLines != NULL ) {
      DebugCode(
         PrintFinalGpsPathLines( pathLines );
         );
      FreeFinalPathLines( pathLines );
      printf( "Success\n" );
   } else {
      printf( "Sad\n" );
   }
   fflush( stdout );
   return 0;
}

static int TestNoObstacleInRec(
   void
   ) {
   int lonTopLeft = 1225604106;
   int latTopLeft = 308890180;
   int lonBottomRight = 1227319960;
   int latBottomRight = 307873053;
   int width = 2000;
   t_ObstaclesPtr obstacles = NULL;
   return TestTemplateInRec( lonTopLeft, latTopLeft, lonBottomRight,
                             latBottomRight, width, c_IsHorizon, obstacles );
}

static t_PathLinesPtr GetNoObstaclePosWithMultiPos(
   void
   ) {
   int position1X[] = {
      1225604106, 1227304783, 1227304783, 1225604106,
      1225604106, 1227304783, 1227304783, 1225604106,
      1225604106, 1227304783, 1227304783, 1225604106,
      1227319960
   };
   int position1Y[] = {
      308890180, 308890180, 308710000, 308710000,
      308529820, 308529820, 308349640, 308349640,
      308169460, 308169460, 307881171, 307881171,
      307873053
   };
   t_PathLinesPtr positions = CreateGpsPathLine();
   int length = sizeof( position1X )/sizeof( int );
   for ( int i = 0; i < length; i++ ) {
      InsertNewGpsPathPoint( *( position1X + i ), *( position1Y + i ), positions );
   }
   return positions;
}


static int TestNoObstaclePosWithMultiPos(
   void
   ) {
   int lonTopLeft = 1225604106;
   int latTopLeft = 308890180;
   int lonBottomRight = 1227319960;
   int latBottomRight = 307873053;
   t_ObstaclesPtr obstacles = NULL;
   t_PathLinesPtr positions = GetNoObstaclePosWithMultiPos();
   return TestTemplateWithMultiPos( positions, lonTopLeft, latTopLeft,
                                    lonBottomRight, latBottomRight, obstacles );
}

static t_PathLinesPtr GetNoObstaclePosWithMultiPos2(
   void
   ) {
   int position1X[] = {
      1217936682, 1217124131, 1214439812, 1213931968
   };
   int position1Y[] = {
      379124505, 377513380, 377974053, 379802427
   };
   t_PathLinesPtr positions = CreateGpsPathLine();
   int length = sizeof( position1X )/sizeof( int );
   for ( int i = 0; i < length; i++ ) {
      InsertNewGpsPathPoint( *( position1X + i ), *( position1Y + i ), positions );
   }
   return positions;
}

static int TestNoObstaclePosWithMultiPos2(
   void
   ) {
   int lonTopLeft = 1213931968;
   int latTopLeft = 379802427;
   int lonBottomRight = 1217936682;
   int latBottomRight = 377513380;
   t_ObstaclesPtr obstacles = NULL;
   t_PathLinesPtr positions = GetNoObstaclePosWithMultiPos2();
   return TestTemplateWithMultiPos( positions, lonTopLeft, latTopLeft,
                                    lonBottomRight, latBottomRight, obstacles );
}

static t_ObstaclesPtr GetSingleObstacleInArea(
   int lonTopLeft,
   int latTopLeft,
   int lonBottomRight,
   int latBottomRight
   ) {
   int obs1X[] = {
      1226261133, 1226349743, 1226371716, 1226388919,
      1226387886, 1226388613, 1226401468, 1226419803,
      1226572984, 1226596097, 1226590914, 1226356696,
      1226042259, 1226033024, 1226040696, 1226126296,
      1226147344, 1226168796, 1226261133
   };
   int obs1Y[] = {
      308246224, 308204593, 308207694, 308218633,
      308253659, 308271714, 308287948, 308288467,
      308227486, 308198034, 308180667, 308120648,
      308192785, 308211966, 308226106, 308278474,
      308282459, 308283614, 308246224
   };

   t_ObstaclesPtr obstacles = CreateObstacles();
   SetObstaclesCount( 1, obstacles );
   obstacles->m_obstacleMembersPtr = CreateSingleObstacleArray( 1 );
   InsertSingleObstacle( sizeof( obs1X )/sizeof( int ), obs1X, obs1Y,GetSingleObstacle( 0, obstacles ));
   return obstacles;
}

static int TestSingleObstacleInRec(
   void
   ) {
   int lonTopLeft = 1225604106;
   int latTopLeft = 308890180;
   int lonBottomRight = 1227319960;
   int latBottomRight = 307873053;
   int width = 500;
   t_ObstaclesPtr obstacles = GetSingleObstacleInArea( lonTopLeft, latTopLeft,
                                                       lonBottomRight, latBottomRight );
   return TestTemplateInRec( lonTopLeft, latTopLeft, lonBottomRight,
                             latBottomRight, width, c_IsHorizon, obstacles );
}


static int TestSingleObstaclePosWithMultiPos(
   void
   ) {
   int lonTopLeft = 1225604106;
   int latTopLeft = 308890180;
   int lonBottomRight = 1227319960;
   int latBottomRight = 307873053;
   t_ObstaclesPtr obstacles = GetSingleObstacleInArea( lonTopLeft, latTopLeft,
                                                       lonBottomRight, latBottomRight );
   t_PathLinesPtr positions = GetSingleObstaclePosWithMultiPos();
   return TestTemplateWithMultiPos( positions, lonTopLeft, latTopLeft,
                                    lonBottomRight, latBottomRight, obstacles );
}

static t_ObstaclesPtr GetSingleObstacleInArea2(
   int lonTopLeft,
   int latTopLeft,
   int lonBottomRight,
   int latBottomRight
   ) {
   int obs1X[] = {
      1207011191, 1207011191, 1206720994, 1206720994,
      1207156289, 1207373937, 1207519035, 1207373937,
      1207011191
   };
   int obs1Y[] = {
      378773681, 379061143, 379520871, 379808043,
      379635765, 379233563, 378888670, 378601137,
      378773681
   };

   t_ObstaclesPtr obstacles = CreateObstacles();
   SetObstaclesCount( 1, obstacles );
   obstacles->m_obstacleMembersPtr = CreateSingleObstacleArray( 1 );
   InsertSingleObstacle( sizeof( obs1X )/sizeof( int ), obs1X, obs1Y,GetSingleObstacle( 0, obstacles ));
   return obstacles;
}
static t_PathLinesPtr GetSingleObstaclePosWithMultiPos2(
   void
   ) {
   int position1X[] = {
      1208530163, 1209255655, 1207993299, 1206382708
   };
   int position1Y[] = {
      378548193, 378950742, 379651826, 379157694
   };
   t_PathLinesPtr positions = CreateGpsPathLine();
   int length = sizeof( position1X )/sizeof( int );
   for ( int i = 0; i < length; i++ ) {
      InsertNewGpsPathPoint( *( position1X + i ), *( position1Y + i ), positions );
   }
   return positions;
}

static int TestSingleObstaclePosWithMultiPos2(
   void
   ) {
   int lonTopLeft = 1206382708;
   int latTopLeft = 379651826;
   int lonBottomRight = 1209255655;
   int latBottomRight = 378548193;
   t_ObstaclesPtr obstacles = GetSingleObstacleInArea2( lonTopLeft, latTopLeft,
                                                       lonBottomRight, latBottomRight );
   t_PathLinesPtr positions = GetSingleObstaclePosWithMultiPos2();
   return TestTemplateWithMultiPos( positions, lonTopLeft, latTopLeft,
                                    lonBottomRight, latBottomRight, obstacles );
}

static t_ObstaclesPtr GetTwoObstaclesInArea(
   int lonTopLeft,
   int latTopLeft,
   int lonBottomRight,
   int latBottomRight
   ) {
   int obs1X[] = {
      1226261133, 1226349743, 1226371716, 1226388919,
      1226387886, 1226388613, 1226401468, 1226419803,
      1226572984, 1226596097, 1226590914, 1226356696,
      1226042259, 1226033024, 1226040696, 1226126296,
      1226147344, 1226168796, 1226261133
   };
   int obs1Y[] = {
      308246224, 308204593, 308207694, 308218633,
      308253659, 308271714, 308287948, 308288467,
      308227486, 308198034, 308180667, 308120648,
      308192785, 308211966, 308226106, 308278474,
      308282459, 308283614, 308246224
   };
   int obs2X[] = {
      1226969706, 1226738282, 1226703778, 1226698388,
      1226695271, 1226692477, 1226612887, 1226602727,
      1226623666, 1226855081, 1227028061, 1227028681,
      1227017793, 1227000492, 1226969706
   };
   int obs2Y[] = {
      308543448, 308620885, 308603333, 308585903,
      308567125, 308530188, 308510634, 308495645,
      308471222, 308385008, 308467029, 308505225,
      308518346, 308528398, 308543448
   };

   t_ObstaclesPtr obstacles = CreateObstacles();
   SetObstaclesCount( 2, obstacles );
   obstacles->m_obstacleMembersPtr = CreateSingleObstacleArray( 2 );
   InsertSingleObstacle( sizeof( obs1X )/sizeof( int ), obs1X, obs1Y,GetSingleObstacle( 0, obstacles ));
   InsertSingleObstacle( sizeof( obs2X )/sizeof( int ), obs2X, obs2Y, GetSingleObstacle( 1, obstacles ));
   return obstacles;
}

static int TestTwoObstacleInRec(
   void
   ) {
   int lonTopLeft = 1225604106;
   int latTopLeft = 308890180;
   int lonBottomRight = 1227319960;
   int latBottomRight = 307873053;
   int width = 500;
   t_ObstaclesPtr obstacles = GetTwoObstaclesInArea( lonTopLeft, latTopLeft,
                                                     lonBottomRight, latBottomRight );
   return TestTemplateInRec( lonTopLeft, latTopLeft, lonBottomRight,
                             latBottomRight, width, c_IsHorizon, obstacles );
}

static int TestTwoObstaclePosWithMultiPos(
   void
   ) {
   int lonTopLeft = 1225604106;
   int latTopLeft = 308890180;
   int lonBottomRight = 1227319960;
   int latBottomRight = 307873053;
   t_ObstaclesPtr obstacles = GetTwoObstaclesInArea( lonTopLeft, latTopLeft,
                                                     lonBottomRight, latBottomRight );
   t_PathLinesPtr positions = GetNoObstaclePosWithMultiPos();
   return TestTemplateWithMultiPos( positions, lonTopLeft, latTopLeft,
                                    lonBottomRight, latBottomRight, obstacles );
}

static t_ObstaclesPtr GetFullObstacleInArea(
   int lonTopLeft,
   int latTopLeft,
   int lonBottomRight,
   int latBottomRight
   ) {
   int obs1X[] = {
      1223261133, 1228349743, 1228371716, 1223388919
   };
   int obs1Y[] = {
      308946224, 308904593, 307607694, 307618633
   };

   t_ObstaclesPtr obstacles = CreateObstacles();
   SetObstaclesCount( 1, obstacles );
   obstacles->m_obstacleMembersPtr = CreateSingleObstacleArray( 1 );
   InsertSingleObstacle( sizeof( obs1X )/sizeof( int ), obs1X, obs1Y,GetSingleObstacle( 0, obstacles ) );
   return obstacles;
}

static int TestFullObstacleInRec(
   void
   ) {
   int lonTopLeft = 1225604106;
   int latTopLeft = 308890180;
   int lonBottomRight = 1227319960;
   int latBottomRight = 307873053;
   int width = 500;
   t_ObstaclesPtr obstacles = GetFullObstacleInArea( lonTopLeft, latTopLeft,
                                                     lonBottomRight, latBottomRight );
   return TestTemplateInRec( lonTopLeft, latTopLeft, lonBottomRight,
                             latBottomRight, width, c_IsHorizon, obstacles );
}

static int TestFullObstaclePosWithMultiPos(
   void
   ) {
   int lonTopLeft = 1225604106;
   int latTopLeft = 308890180;
   int lonBottomRight = 1227319960;
   int latBottomRight = 307873053;
   t_ObstaclesPtr obstacles = GetFullObstacleInArea( lonTopLeft, latTopLeft,
                                                     lonBottomRight, latBottomRight );
   t_PathLinesPtr positions = GetNoObstaclePosWithMultiPos();
   return TestTemplateWithMultiPos( positions, lonTopLeft, latTopLeft,
                                    lonBottomRight, latBottomRight, obstacles );
}

int main(
   int argc,
   char *argv[]
   ) {
   return TestNoObstaclePosWithMultiPos2();
}
