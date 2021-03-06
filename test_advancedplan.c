#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <check.h>
#include "publicfun.h"
#include "pretreatment.h"
#include "advancedplan.h"
#include "advancedplan.c"

static void test_Turn2NormalRadiansRangeTemplate(
   double radians,
   double predictRadians
   ) {
   double calRadians = Turn2NormalRadiansRange( radians );
   ck_assert( IsDoubleEqual( calRadians, predictRadians ) );
}

START_TEST( test_Turn2NormalRadiansRange )
{
   test_Turn2NormalRadiansRangeTemplate( c_pi, -1 * c_pi );
   test_Turn2NormalRadiansRangeTemplate( -1 * c_pi, -1 * c_pi );
   test_Turn2NormalRadiansRangeTemplate( 0, 0 );
   test_Turn2NormalRadiansRangeTemplate( 2 * c_pi, 0 );
   test_Turn2NormalRadiansRangeTemplate( -2 * c_pi, 0 );
}
END_TEST

static void test_IsNumEvenTemplate(
   int num,
   int IsFalse
   ) {
  ck_assert_int_eq( IsNumEven( num ), !IsFalse );// "error, isnumeven is not right" );
}

START_TEST( test_IsNumEven )
{
  test_IsNumEvenTemplate( 1, 1 );
  test_IsNumEvenTemplate( 2, 0 );
  test_IsNumEvenTemplate( 0, 0 );
  test_IsNumEvenTemplate( -3, 1 );
}
END_TEST

static void test_IsThreePointsInALineTemplate(
   int x1,
   int y1,
   int x2,
   int y2,
   int x3,
   int y3,
   int IsFalse
   ) {
  ck_assert_int_eq( IsThreePointInALine( x1, y1, x2, y2, x3, y3 ),
                    !IsFalse );
}

START_TEST( test_IsThreePointsInALine )
{
  test_IsThreePointsInALineTemplate( 1, 2, 3, 4, 5, 6, 0 );
  test_IsThreePointsInALineTemplate( 1, 3, 1, 5, 1, 9, 0 );
  test_IsThreePointsInALineTemplate( 1, 3, 1, 9, 1, 4, 0 );
  test_IsThreePointsInALineTemplate( 1, 3, 1, 3, 1, 4, 0 );
  test_IsThreePointsInALineTemplate( 1, 3, 1, 4, 2, 3, 1 );
  test_IsThreePointsInALineTemplate( 1, 3, 1, 3, 2, 4, 0 );
  test_IsThreePointsInALineTemplate( 9, 7, 23, 7, 2, 7, 0 );
  test_IsThreePointsInALineTemplate( 9, 9, 12, 12, 19, 19, 0 );
  test_IsThreePointsInALineTemplate( 4, 9, 8, 10, 9, 11, 1 );
  test_IsThreePointsInALineTemplate( 4, 9, 8, 10, 12, 7, 1 );
}
END_TEST

static void test_IsThreePointsInADirectionTemplate(
   int x1,
   int y1,
   int x2,
   int y2,
   int x3,
   int y3,
   int IsFalse
   ) {
  ck_assert_int_eq( IsThreePointInADirection( x1, y1, x2, y2, x3, y3 ),
                    !IsFalse );
}

START_TEST( test_IsThreePointsInADirection )
{
  test_IsThreePointsInADirectionTemplate( 1, 2, 3, 4, 5, 6, 0 );
  test_IsThreePointsInADirectionTemplate( 1, 3, 1, 5, 1, 9, 0 );
  test_IsThreePointsInADirectionTemplate( 1, 3, 1, 9, 1, 4, 1 );
  test_IsThreePointsInADirectionTemplate( 1, 3, 1, 3, 1, 4, 0 );
  test_IsThreePointsInADirectionTemplate( 1, 3, 1, 4, 2, 3, 1 );
  test_IsThreePointsInADirectionTemplate( 9, 9, 12, 12, 19, 19, 0 );
  test_IsThreePointsInADirectionTemplate( 9, 9, 19, 19, 12, 12, 1 );
  test_IsThreePointsInADirectionTemplate( 4, 9, 8, 10, 9, 11, 0 );
  test_IsThreePointsInADirectionTemplate( 4, 9, 8, 10, 12, 7, 1 );
}
END_TEST

static void test_IsThreePointsInABeamTemplate(
   int x1,
   int y1,
   int x2,
   int y2,
   int x3,
   int y3,
   int IsFalse
   ) {
  ck_assert_int_eq( IsThreePointInABeam( x1, y1, x2, y2, x3, y3 ),
                    !IsFalse );
}

START_TEST( test_IsThreePointsInABeam )
{
  test_IsThreePointsInABeamTemplate( 1, 2, 3, 4, 5, 6, 0 );
  test_IsThreePointsInABeamTemplate( 1, 3, 1, 5, 1, 9, 0 );
  test_IsThreePointsInABeamTemplate( 1, 3, 1, 9, 1, 4, 1 );
  test_IsThreePointsInABeamTemplate( 1, 3, 1, 3, 1, 4, 0 );
  test_IsThreePointsInABeamTemplate( 1, 3, 1, 4, 2, 3, 1 );
  test_IsThreePointsInABeamTemplate( 9, 9, 12, 12, 19, 19, 0 );
  test_IsThreePointsInABeamTemplate( 9, 9, 19, 19, 12, 12, 1 );
  test_IsThreePointsInABeamTemplate( 4, 9, 8, 10, 9, 11, 1 );
  test_IsThreePointsInABeamTemplate( 4, 9, 8, 10, 12, 7, 1 );
}
END_TEST

static void test_GetGpsLonFromDistanceTemplate(
   int xRealLength,
   int xTopLeft,
   int yTopLeft,
   int lon
   ) {
  double lonCal;

  lonCal = ( double )GetGpsLonFromDistance( xRealLength, xTopLeft, yTopLeft );
  ck_assert_msg( IsDoubleEqualWithTolerance( lonCal, ( double )lon, pow( 10, 4 ) ),
                "xReallength %d  xTopleft %d yTopLeft %d lat %d", xRealLength, xTopLeft, yTopLeft, lon );
}

/* Green: the test case is lona: 1200291593 lata: 356995731 lonb: 1201856045 latb: 356052480 xreallength: 14144 yreallength: 10488 */
START_TEST( test_GetGpsLonFromDistance )
{
  test_GetGpsLonFromDistanceTemplate( 14144, 1200291593, 356995731, 1201856045 );
}
END_TEST

static void test_GetGpsLatFromDistanceTemplate(
   int yRealLength,
   int yTopLeft,
   int lat
   ) {
  double latCal;

  latCal = ( double )GetGpsLatFromDistance( yRealLength, yTopLeft );
  ck_assert_msg( IsDoubleEqualWithTolerance( latCal, ( double )lat, pow( 10, 4) ),
                "yReallength %d yTopLeft %d lat %d", yRealLength, yTopLeft, lat );
}

START_TEST( test_GetGpsLatFromDistance )
{
  test_GetGpsLatFromDistanceTemplate( 10488, 356995731, 356052480 );
}
END_TEST

static int IsIntEqualWithTolerant(
   int x,
   int y,
   int epslion
   ) {
   return abs( x - y ) < epslion;
}

static void test_GetPointCorInOvalTemplate(
   double angleCurrent,
   int circleCenterX,
   int circleCenterY,
   double axisX,
   double axisY,
   int predictX,
   int predictY
   ) {
   int calX, calY;
   GetPointCorInOval( angleCurrent, circleCenterX, circleCenterY,
                      axisX, axisY, &calX, &calY );
   /* printf( "test_getpointcorinoval : calx %d predictx %d caly %d predicty %d\n", */
   /*         calX, predictX, calY, predictY ); */
   /* fflush( stdout ); */
   ck_assert( IsIntEqualWithTolerant( calX, predictX, 3 ) );
   ck_assert( IsIntEqualWithTolerant( calY, predictY, 3 ) );
}

START_TEST( test_GetPointCorInOval )
{
   test_GetPointCorInOvalTemplate( 0, 10, 30, 5, 5, 15, 30 );
   test_GetPointCorInOvalTemplate( 0, 10, 30, 9, 12, 19, 30 );
}
END_TEST

Suite *helperfun_suite(
   void
   ) {
  Suite *s;
  TCase *tc_core;

  s = suite_create( "helperfun" );

  tc_core = tcase_create( "Core" );

  tcase_add_test( tc_core, test_GetGpsLatFromDistance );
  tcase_add_test( tc_core, test_GetGpsLonFromDistance );
  tcase_add_test( tc_core, test_IsNumEven );
  tcase_add_test( tc_core, test_IsThreePointsInALine );
  tcase_add_test( tc_core, test_IsThreePointsInADirection );
  tcase_add_test( tc_core, test_IsThreePointsInABeam );
  tcase_add_test( tc_core, test_Turn2NormalRadiansRange );
  tcase_add_test( tc_core, test_GetPointCorInOval );
  suite_add_tcase( s, tc_core );

  return s;
}

static t_EnvPathLinePtr CreateEnvPathLineWithXYArray(
   int size,
   int *x,
   int *y
   ) {
   t_EnvPathLinePtr envPathLine = NULL;
   for ( int i = size - 1; i >= 0; i-- ) {
      envPathLine = InsertNewEnvPathLine( *( x + i ), *( y + i ), envPathLine );
   }
   return envPathLine;
}

static void AssertEnvPathLineMember(
   int size,
   int *x,
   int *y,
   t_EnvPathLinePtr envPathLine
   ) {
  t_EnvPathLinePtr currentEnvPathLineMem = envPathLine;
  for ( int i = 0; i < size; i++ ) {
      int currentX = GetEnvPathMemberX( currentEnvPathLineMem );
      ck_assert_int_eq( currentX, *( x + i ) );
      int currentY = GetEnvPathMemberY( currentEnvPathLineMem );
      ck_assert_int_eq( currentY, *( y + i ) );
      currentEnvPathLineMem = GetEnvPathMemberNext( currentEnvPathLineMem );
   }
}

static void test_FilterInBeamPathLineNextTemplate(
   int size,
   int *envX,
   int *envY,
   t_EnvPathLinePtr envPathLine
   ) {
   t_EnvPathLinePtr filteredEnvPathLine = FilterInBeamPathLineNext( envPathLine );
   AssertEnvPathLineMember( size, envX, envY, filteredEnvPathLine );
}

START_TEST( test_FilterInBeamPathLineNext )
{
   int x1[] = {
      1
   };
   int y1[] = {
      2
   };
   t_EnvPathLinePtr envPathLine1 = CreateEnvPathLineWithXYArray( sizeof( x1 )/sizeof( int ), x1, y1 );
   test_FilterInBeamPathLineNextTemplate( sizeof( x1 )/sizeof( int ), x1, y1, envPathLine1 );

   int x2[] = {
      1, 2
   };
   int y2[] = {
      1, 2
   };
   t_EnvPathLinePtr envPathLine2 = CreateEnvPathLineWithXYArray( sizeof( x2 )/sizeof( int ), x2, y2 );
   test_FilterInBeamPathLineNextTemplate( sizeof( x2 )/sizeof( int ), x2, y2, envPathLine2 );

   int x3[] = {
      1, 2, 3
   };
   int y3[] = {
      1, 2, 3
   };
   t_EnvPathLinePtr envPathLine3 = CreateEnvPathLineWithXYArray( sizeof( x3 )/sizeof( int ), x3, y3 );
   int filteredX3[] = {
      1, 3
   };
   int filteredY3[] = {
      1, 3
   };
   test_FilterInBeamPathLineNextTemplate( sizeof( filteredX3 )/sizeof( int ), filteredX3, filteredY3, envPathLine3 );

   int x4[] = {
      1, 2, 3
   };
   int y4[] = {
      1, 2, 4
   };
   t_EnvPathLinePtr envPathLine4 = CreateEnvPathLineWithXYArray( sizeof( x4 )/sizeof( int ), x4, y4 );
   int filteredX4[] = {
      1, 2, 3
   };
   int filteredY4[] = {
      1, 2, 4
   };
   test_FilterInBeamPathLineNextTemplate( sizeof( filteredX4 )/sizeof( int ), filteredX4, filteredY4, envPathLine4 );
}
END_TEST

static void test_AppendEnvPathLineTemplate(
   int size,
   int *x,
   int *y,
   t_EnvPathLinePtr envPathLine1,
   t_EnvPathLinePtr envPathLine2
   ) {
   t_EnvPathLinePtr envPathLine = AppendEnvPathLine( envPathLine1, envPathLine2 );
   AssertEnvPathLineMember( size, x, y, envPathLine );
   FreePathLine( envPathLine );
}

START_TEST( test_AppendEnvPathLine )
{
   int x11[] = {
      1, 2, 3, 4, 5
   };
   int y11[] = {
      5, 4, 3, 2, 1
   };
   t_EnvPathLinePtr envPathLine11 = CreateEnvPathLineWithXYArray( sizeof( x11 )/sizeof( int ),
                                                    x11, y11 );
   int x21[] = {
      6, 7, 8, 9
   };
   int y21[] = {
      9, 8, 7, 6
   };
   t_EnvPathLinePtr envPathLine21 = CreateEnvPathLineWithXYArray( sizeof( x21 )/sizeof( int ),
                                                                 x21, y21 );
   int totalSize1 = sizeof( x11 )/sizeof( int ) + sizeof( x21 )/sizeof( int );
   int x31[ totalSize1 ], y31[ totalSize1 ];
   memcpy( x31, x11, sizeof( x11 ) );
   memcpy( x31 + sizeof( x11 )/sizeof( int ), x21, sizeof( x21 ) );
   memcpy( y31, y11, sizeof( y11 ) );
   memcpy( y31 + sizeof( y11 )/sizeof( int ), y21, sizeof( y21 ) );
   test_AppendEnvPathLineTemplate( totalSize1, x31, y31, envPathLine11, envPathLine21);

   int x12[] = {
      1, 2, 3, 4, 5
   };
   int y12[] = {
      5, 4, 3, 2, 1
   };
   t_EnvPathLinePtr envPathLine12 = CreateEnvPathLineWithXYArray( sizeof( x12 )/sizeof( int ),
                                                                  x12, y12 );
   int totalSize2 = sizeof( x12 )/sizeof( int );
   test_AppendEnvPathLineTemplate( totalSize2, x12, y12, envPathLine12, NULL );

   int x23[] = {
      6, 7, 8, 9
   };
   int y23[] = {
      9, 8, 7, 6
   };
   t_EnvPathLinePtr envPathLine23 = CreateEnvPathLineWithXYArray( sizeof( x23 )/sizeof( int ),
                                                                 x23, y23 );
   int totalSize3 = sizeof( x23 )/sizeof( int );
   test_AppendEnvPathLineTemplate( totalSize3, x23, y23, NULL, envPathLine23);
 }
END_TEST

static void AssertGpsPathLineMember(
   t_PathLinesPtr pathLine,
   int size,
   int *allLon,
   int *allLat
   ) {
   t_PathPointPtr pathPoints = GetGpsPathLinePoints( pathLine );
   for ( int i = 0; i < size; i++ ) {
      int currentLon = GetGpsPathPointLon( pathPoints );
      ck_assert_int_eq( currentLon, *( allLon + i ) );
      int currentLat = GetGpsPathPointLat( pathPoints );
      ck_assert_int_eq( currentLat, *( allLat + i ) );
      pathPoints = GetGpsPathPointNext( pathPoints );
   }
}

static void test_InsertNewGpsPathPointTemplate(
   int newLon,
   int newLat,
   t_PathLinesPtr pathLine,
   int size,
   int *allLon,
   int *allLat
   ) {
   InsertNewGpsPathPoint( newLon, newLat, pathLine );
   AssertGpsPathLineMember( pathLine, size, allLon, allLat );
}

static void test_InsertNewGpsPathPointArrayTemplate(
   int size,
   int *allLon,
   int *allLat,
   t_PathLinesPtr pathLine
   ) {
   for ( int i = size - 1; i >= 0; i-- ) {
      InsertNewGpsPathPoint( *( allLon + i ), *( allLat + i ), pathLine );
   }

   AssertGpsPathLineMember( pathLine, size, allLon, allLat );
}

START_TEST( test_InsertNewGpsPathPoint )
{
   int lon11 = 1;
   int lat11 = 2;
   t_PathLinesPtr pathLine1 = CreateGpsPathLine();
   test_InsertNewGpsPathPointTemplate( lon11, lat11, pathLine1, 1, &lon11, &lat11 );
   FreeFinalPathLines( pathLine1 );

   int lon12[] = {
      1, 2, 3, 4, 5
   };
   int lat12[] = {
      5, 4, 3, 2, 1
   };
   t_PathLinesPtr pathLine2 = CreateGpsPathLine();
   test_InsertNewGpsPathPointArrayTemplate( sizeof( lon12 )/sizeof( int ),
                                            lon12, lat12, pathLine2 );
   FreeFinalPathLines( pathLine2 );
}
END_TEST

static void test_TurnEnv2GpsPathLineTemplate(
   int size,
   int *allLon,
   int *allLat,
   t_PathLinesPtr pathLine,
   t_EnvironmentPtr environment
   ) {
   TurnEnv2GpsPathLine( pathLine, environment );
   AssertGpsPathLineMember( pathLine, size, allLon, allLat );
}

static void InsertXY2GpsPathLine(
   int size,
   int *allX,
   int *allY,
   t_PathLinesPtr pathLine
   ) {
   for ( int i = size - 1; i >= 0; i-- ) {
      InsertNewGpsPathPoint( *( allX + i ), *( allY + i ), pathLine );
   }
}

START_TEST( test_TurnEnv2GpsPathLine )
{
   t_EnvironmentPtr environment = InitialEnvWithGps( 100, 100,
                                                     1225604106, 308890180,
                                                     1227319960, 307873053 );
   int x1[] = {
      2, 87, 5, 120, 45
   };
   int y1[] = {
      34, 6, 89, 12, 6
   };
   t_PathLinesPtr pathLine1 = CreateGpsPathLine();
   InsertXY2GpsPathLine( sizeof( x1 )/sizeof( int ), x1, y1, pathLine1 );
   int lon1[] = {
      1225625102, 1226517433, 1225656596, 1226863867, 1226076516
   };
   int lat1[] = {
      308583874, 308836126, 308088379, 308782072, 308836126
   };
   test_TurnEnv2GpsPathLineTemplate( sizeof( lon1 )/sizeof( int ), lon1, lat1,
                                     pathLine1, environment );
}
END_TEST

START_TEST( test_GetLengthGoingUpDistributedPoints )
{
   int xMin1 = 1;
   int xMax1 = 18;
   int yMax1 = 20;
   int xStart1 = 2;
   int yStart1 = 2;
   int width1 = 5;
   t_EnvPathLinePtr distributedPoints1 = GetLengthGoingUpDistributedPoints( xMin1, xMax1, yMax1, xStart1, yStart1, width1 );
   int predictDistributedPointsX1[] = {
      18, 1, 1, 18, 18, 1, 1, 18, 18, 2
   };
   int predictDistributedPointsY1[] = {
      20, 20, 17, 17, 12, 12, 7, 7, 2, 2
   };
   AssertEnvPathLineMember( sizeof( predictDistributedPointsX1 )/sizeof( int ),
                            predictDistributedPointsX1, predictDistributedPointsY1,
                            distributedPoints1 );

   int xMin2 = 3;
   int xMax2 = 10;
   int yMax2 = 20;
   int xStart2 = 4;
   int yStart2 = 5;
   int width2 = 5;
   t_EnvPathLinePtr distributedPoints2 = GetLengthGoingUpDistributedPoints( xMin2, xMax2, yMax2, xStart2, yStart2, width2 );
   int predictDistributedPointsX2[] = {
      3, 10, 10, 3, 3, 10, 10, 4
   };
   int predictDistributedPointsY2[] = {
      20, 20, 15, 15, 10, 10, 5, 5
   };
   AssertEnvPathLineMember( sizeof( predictDistributedPointsX2 )/sizeof( int ),
                            predictDistributedPointsX2, predictDistributedPointsY2,
                            distributedPoints2 );
}
END_TEST

START_TEST( test_GetLengthGoingDownDistributedPoints )
{
   int xMin1 = 1;
   int yMin1 = 1;
   int xMax1 = 18;
   int xStart1 = 2;
   int yStart1 = 20;
   int width1 = 5;
   t_EnvPathLinePtr distributedPoints1 = GetLengthGoingDownDistributedPoints( xMin1, yMin1, xMax1, xStart1, yStart1, width1 );
   int predictDistributedPointsX1[] = {
      18, 1, 1, 18, 18, 1, 1, 18, 18, 2
   };
   int predictDistributedPointsY1[] = {
      1, 1, 5, 5, 10, 10, 15, 15, 20, 20
   };
   AssertEnvPathLineMember( sizeof( predictDistributedPointsX1 )/sizeof( int ),
                            predictDistributedPointsX1, predictDistributedPointsY1,
                            distributedPoints1 );

   int xMin2 = 3;
   int yMin2 = 5;
   int xMax2 = 10;
   int xStart2 = 7;
   int yStart2 = 20;
   int width2 = 5;
   t_EnvPathLinePtr distributedPoints2 = GetLengthGoingDownDistributedPoints( xMin2, yMin2, xMax2, xStart2, yStart2, width2 );
   int predictDistributedPointsX2[] = {
      10, 3, 3, 10, 10, 3, 3, 7
   };
   int predictDistributedPointsY2[] = {
      5, 5, 10, 10, 15, 15, 20, 20
   };
   AssertEnvPathLineMember( sizeof( predictDistributedPointsX2 )/sizeof( int ),
                            predictDistributedPointsX2, predictDistributedPointsY2,
                            distributedPoints2 );
}
END_TEST

START_TEST( test_GetHeightGoingUpDistributedPoints )
{
   int yMin1 = 1;
   int xMax1 = 20;
   int yMax1 = 18;
   int xStart1 = 2;
   int yStart1 = 2;
   int width1 = 5;
   t_EnvPathLinePtr distributedPoints1 = GetHeightGoingUpDistributedPoints( yMin1, xMax1, yMax1, xStart1, yStart1, width1 );
   int predictDistributedPointsX1[] = {
      20, 20, 17, 17, 12, 12, 7, 7, 2, 2
   };
   int predictDistributedPointsY1[] = {
      18, 1, 1, 18, 18, 1, 1, 18, 18, 2

   };
   AssertEnvPathLineMember( sizeof( predictDistributedPointsX1 )/sizeof( int ),
                            predictDistributedPointsX1, predictDistributedPointsY1,
                            distributedPoints1 );

   int yMin2 = 3;
   int xMax2 = 20;
   int yMax2 = 10;
   int xStart2 = 5;
   int yStart2 = 4;
   int width2 = 5;
   t_EnvPathLinePtr distributedPoints2 = GetHeightGoingUpDistributedPoints( yMin2, xMax2, yMax2, xStart2, yStart2, width2 );
   int predictDistributedPointsX2[] = {
      20, 20, 15, 15, 10, 10, 5, 5
   };
   int predictDistributedPointsY2[] = {
      3, 10, 10, 3, 3, 10, 10, 4
   };
   AssertEnvPathLineMember( sizeof( predictDistributedPointsX2 )/sizeof( int ),
                            predictDistributedPointsX2, predictDistributedPointsY2,
                            distributedPoints2 );
}
END_TEST

START_TEST( test_GetHeightGoingDownDistributedPoints )
{
   int xMin1 = 1;
   int yMin1 = 1;
   int yMax1 = 18;
   int xStart1 = 20;
   int yStart1 = 2;
   int width1 = 5;
   t_EnvPathLinePtr distributedPoints1 = GetHeightGoingDownDistributedPoints( xMin1, yMin1, yMax1, xStart1, yStart1, width1 );
   int predictDistributedPointsX1[] = {
      1, 1, 5, 5, 10, 10, 15, 15, 20, 20
   };
   int predictDistributedPointsY1[] = {
      18, 1, 1, 18, 18, 1, 1, 18, 18, 2
   };
   AssertEnvPathLineMember( sizeof( predictDistributedPointsX1 )/sizeof( int ),
                            predictDistributedPointsX1, predictDistributedPointsY1,
                            distributedPoints1 );

   int xMin2 = 5;
   int yMin2 = 3;
   int yMax2 = 10;
   int xStart2 = 20;
   int yStart2 = 7;
   int width2 = 5;
   t_EnvPathLinePtr distributedPoints2 = GetHeightGoingDownDistributedPoints( xMin2, yMin2, yMax2, xStart2, yStart2, width2 );
   int predictDistributedPointsX2[] = {
      5, 5, 10, 10, 15, 15, 20, 20
   };
   int predictDistributedPointsY2[] = {
      10, 3, 3, 10, 10, 3, 3, 7
   };
   AssertEnvPathLineMember( sizeof( predictDistributedPointsX2 )/sizeof( int ),
                            predictDistributedPointsX2, predictDistributedPointsY2,
                            distributedPoints2 );
}
END_TEST

static Suite *envPathLine_suite(
   void
   ) {
   Suite *s;
   TCase *tc_core;

   s = suite_create( "envPathLine" );  /*the output name of the suite*/
   tc_core = tcase_create( "Core" );

   tcase_add_test( tc_core, test_AppendEnvPathLine );
   tcase_add_test( tc_core, test_InsertNewGpsPathPoint );
   tcase_add_test( tc_core, test_TurnEnv2GpsPathLine );
   tcase_add_test( tc_core, test_FilterInBeamPathLineNext );
   tcase_add_test( tc_core, test_GetLengthGoingUpDistributedPoints );
   tcase_add_test( tc_core, test_GetLengthGoingDownDistributedPoints );
   tcase_add_test( tc_core, test_GetHeightGoingUpDistributedPoints );
   tcase_add_test( tc_core, test_GetLengthGoingDownDistributedPoints );
   suite_add_tcase( s, tc_core );

   return s;
}

int main(
   int argc,
   char **argv
   ) {
  int number_failed;
  Suite *s;
  SRunner *sr;

  s = helperfun_suite();
  sr = srunner_create( s );
  srunner_add_suite( sr, envPathLine_suite() );

  srunner_run_all( sr, CK_NORMAL );
  number_failed = srunner_ntests_failed( sr );
  srunner_free( sr );
  return ( number_failed == 0 ) ? EXIT_SUCCESS : EXIT_FAILURE;
}
