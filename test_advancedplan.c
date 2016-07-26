#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <check.h>
#include "publicfun.c"
#include "pretreatment.h"
#include "advancedplan.h"
#include "advancedplan.c"


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
  test_IsThreePointsInALineTemplate( 1, 3, 1, 9, 1, 4, 0 );
  test_IsThreePointsInALineTemplate( 9, 7, 23, 7, 2, 7, 0 );
  test_IsThreePointsInALineTemplate( 4, 9, 8, 10, 9, 11, 1 );
  test_IsThreePointsInALineTemplate( 4, 9, 8, 10, 12, 7, 1 );
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

  srunner_run_all( sr, CK_NORMAL );
  number_failed = srunner_ntests_failed( sr );
  srunner_free( sr );
  return ( number_failed == 0 ) ? EXIT_SUCCESS : EXIT_FAILURE;
}
