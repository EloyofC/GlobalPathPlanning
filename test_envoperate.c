#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <check.h>
#include "publicfun.h"
#include "envoperate.h"
#include "envoperate.c"

static void test_GetEnvXFromGpsLonTemplate(
   int lon,
   t_EnvironmentPtr environment,
   int predictX
   ) {
   int calX = GetEnvXFromGpsLon( lon, environment );
   ck_assert_int_eq( calX, predictX );
}

START_TEST( test_GetEnvXFromGpsLon )
{
   t_EnvironmentPtr environment = InitialEnvWithGps( 100, 100,
                                                     1225604106, 308890180,
                                                     1227319960, 307873053 );

   test_GetEnvXFromGpsLonTemplate( 1225604106, environment, 0 );
   test_GetEnvXFromGpsLonTemplate( 1227319960, environment, 163 );
   test_GetEnvXFromGpsLonTemplate( 1223388919, environment, -211 );

   DeleteEnvironment( environment );
}
END_TEST

static void test_GetEnvYFromGpsLatTemplate(
   int lat,
   t_EnvironmentPtr environment,
   int predictY
   ) {
   int calY = GetEnvYFromGpsLat( lat, environment );
   ck_assert_int_eq( calY, predictY );
}

START_TEST( test_GetEnvYFromGpsLat )
{
   t_EnvironmentPtr environment = InitialEnvWithGps( 100, 100,
                                                     1225604106, 308890180,
                                                     1227319960, 307873053 );

   test_GetEnvYFromGpsLatTemplate( 308890180, environment, 0 );
   test_GetEnvYFromGpsLatTemplate( 307873053, environment, 113 );
   test_GetEnvYFromGpsLatTemplate( 309999999, environment, -123 );

   DeleteEnvironment( environment );
}
END_TEST

static Suite *envoperate_suite(
   void
   ) {
   Suite *s;
   TCase *tc_core;

   s = suite_create( "envoperate" );  /*the output name of the suite*/
   tc_core = tcase_create( "Core" );

   tcase_add_test( tc_core, test_GetEnvXFromGpsLon );
   tcase_add_test( tc_core, test_GetEnvYFromGpsLat );
   suite_add_tcase( s, tc_core );

   return s;
}

int main(
   int argc,
   char *argv[]
   ) {
   int number_failed;
   Suite *s;
   SRunner *sr;

   s = envoperate_suite();        /*get the name of the suite*/
   sr = srunner_create( s );

   srunner_run_all( sr, CK_NORMAL );
   number_failed= srunner_ntests_failed( sr );
   srunner_free( sr );
   return ( number_failed == 0 ) ? EXIT_SUCCESS : EXIT_FAILURE;
}
