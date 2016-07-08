#include <stdio.h>
#include <stdlib.h>
#include <check.h>
#include "publicfun.h"
#include "publicfun.c"

static int Caldistancetolerance(int calDistance, int distance)
{
  return (distance + calDistance)/ 200;
}

static char* test_SwapNumTemplate(int a1, int b1)
{
  int a2 = a1, b2 = b1;
  SwapNum(&a2, &b2);
  ck_assert_int_eq(a1, b2);
  ck_assert_int_eq(b1, a2);
}

START_TEST(test_SwapNum)
{
  test_SwapNumTemplate(-1, 1);
  test_SwapNumTemplate(0, 0);
}
END_TEST

static char* test_IsDoubleEqualTemplate(double x1, double x2, int isFalse)
{
  ck_assert_int_eq(IsDoubleEqual(x1, x2), !isFalse);
}

START_TEST(test_IsDoubleEqual)
{
  test_IsDoubleEqualTemplate(10.00000002, 10.00000001, 0);
  test_IsDoubleEqualTemplate(19.0, 19.02, 1);
}
END_TEST

static char* test_IsDoubleEqualWithToleranceTemplate(double x1, double x2, double epsilon, int isFalse)
{
  ck_assert_int_eq(IsDoubleEqualWithTolerance(x1, x2, epsilon), !isFalse);
}

START_TEST(test_IsDoubleEqualWithTolerance)
{
  test_IsDoubleEqualWithToleranceTemplate(10.2, 10.1, 0.2, 0);
  test_IsDoubleEqualWithToleranceTemplate(19.00, 19.02, 0.01, 1);
}
END_TEST

static char* test_Angle2RadiansTemplate(double angle, double radians)
{
  double calRadians = Angle2Radians(angle);
  ck_assert(IsDoubleEqualWithTolerance(calRadians, radians, 0.01));
}

START_TEST(test_Angle2Radians)
{
  test_Angle2RadiansTemplate(0, 0);
  test_Angle2RadiansTemplate(30, 0.523599);
  test_Angle2RadiansTemplate(60, 1.0471976);
}
END_TEST

static char* test_CalGpsDistanceTemplate(int lonFirst, int latFirst, int lonSecond, int latSecond, int distance)
{
  int calDistance = CalGpsDistance(lonFirst, latFirst, lonSecond, latSecond);
  ck_assert(abs(calDistance - distance) < Caldistancetolerance(calDistance, distance));
}

START_TEST(test_CalGpsDistance)
{
  test_CalGpsDistanceTemplate(1206480644, 380044038, 1208053971, 378817969, 19418);
}
END_TEST

static char* test_CalGpsDistanceLonTemplate(int lonFirst, int latFirst, int lonSecond, int latSecond, int lonDistance)
{
  int calLonDistance = CalGpsDistanceLon(lonFirst, latFirst, lonSecond);
  ck_assert(abs(calLonDistance - lonDistance) < Caldistancetolerance(calLonDistance, lonDistance));
}

START_TEST(test_CalGpsDistanceLon){
  test_CalGpsDistanceLonTemplate(1206480644, 380044038, 1208053971, 378817969, 13800);
}
END_TEST

static char* test_CalGpsDistanceLatTemplate(int lonFirst, int latFirst, int lonSecond, int latSecond, int latDistance)
{
  int calLatDistance = CalGpsDistanceLat(lonFirst, latFirst, latSecond);
  ck_assert(abs(calLatDistance - latDistance) < Caldistancetolerance(calLatDistance, latDistance));
}

START_TEST(test_CalGpsDistanceLat)
{
  test_CalGpsDistanceLatTemplate(1206480644, 380044038, 1208053971, 378817969, 13648);
}
END_TEST

Suite *publicfun_suite(void)
{
  Suite *s;
  TCase *tc_core;

  s = suite_create("publicfun");
  tc_core = tcase_create("Core");

  tcase_add_test(tc_core, test_IsDoubleEqual);
  tcase_add_test(tc_core, test_IsDoubleEqualWithTolerance);
  tcase_add_test(tc_core, test_CalGpsDistance);
  tcase_add_test(tc_core, test_CalGpsDistanceLon);
  tcase_add_test(tc_core, test_CalGpsDistanceLat);
  suite_add_tcase(s, tc_core);

  return s;
}

int main(int argc, char *argv[])
{
  int number_failed;
  Suite *s;
  SRunner *sr;

  s = publicfun_suite();
  sr = srunner_create(s);

  srunner_run_all(sr, CK_NORMAL);
  number_failed = srunner_ntests_failed(sr);
  srunner_free(sr);
  return (number_failed == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}
