#include <stdio.h>
#include <stdlib.h>
#include "miniunit.h"
#include "publicfun.h"
#include "publicfun.c"

int tests_run = 0;

static char* test_SwapNumTemplate(int a1, int b1)
{
  int a2 = a1, b2 = b1;
  SwapNum(&a2, &b2);
  mu_assert("error, swapnum is not right", (a1 == b2) && (b1 == a2));
  return 0;
}

static char* test_SwapNum(void)
{
  test_SwapNumTemplate(-1, 1);
  test_SwapNumTemplate(0, 0);
  return 0;
}

static char* test_IsDoubleEqualTemplate(double x1, double x2, int isFalse)
{
  mu_assert("error, isdoubleequal is not right", IsDoubleEqual(x1, x2) == !isFalse);
  return 0;
}

static char* test_IsDoubleEqual(void)
{
  test_IsDoubleEqualTemplate(10.00000002, 10.00000001, 0);
  test_IsDoubleEqualTemplate(19.0, 19.02, 1);
  return 0;
}

static char* test_IsDoubleEqualWithToleranceTemplate(double x1, double x2, double epsilon, int isFalse)
{
  mu_assert("error, isdoubleequal is not right", IsDoubleEqualWithTolerance(x1, x2, epsilon) == !isFalse);
  return 0;
}

static char* test_IsDoubleEqualWithTolerance(void)
{
  test_IsDoubleEqualWithToleranceTemplate(10.2, 10.1, 0.2, 0);
  test_IsDoubleEqualWithToleranceTemplate(19.00, 19.02, 0.01, 1);
  return 0;
}

static char* test_Angle2RadiansTemplate(double angle, double radians)
{
  double calRadians = Angle2Radians(angle);
  mu_assert("error, angle2radians is not right", IsDoubleEqualWithTolerance(calRadians, radians, 0.01));
  return 0;
}

static char* test_Angle2Radians(void)
{
  test_Angle2RadiansTemplate(0, 0);
  test_Angle2RadiansTemplate(30, 0.523599);
  test_Angle2RadiansTemplate(60, 1.0471976);
  return 0;
}

static char* test_CalGpsDistanceTemplate(int lonFirst, int latFirst, int lonSecond, int latSecond, int distance)
{
  int calDistance = CalGpsDistance(lonFirst, latFirst, lonSecond, latSecond);
  mu_assert("error, calgpsdistance is not right", abs(calDistance - distance) < 5);
  return 0;
}

static char* test_CalGpsDistance(void)
{
  test_CalGpsDistanceTemplate(1206480644, 380044038, 1208053971, 378817969, 19418);
  return 0;
}

static char* test_CalGpsDistanceLonTemplate(int lonFirst, int latFirst, int lonSecond, int latSecond, int lonDistance)
{
  int calLonDistance = CalGpsDistanceLon(lonFirst, latFirst, lonSecond);
  mu_assert("error, calgpsdistancelon is not right", abs(calLonDistance - lonDistance) < 5);
  return 0;
}

static char* test_CalGpsDistanceLon(void){
  test_CalGpsDistanceLonTemplate(1206480644, 380044038, 1208053971, 378817969, 13800);
  return 0;
}

static char* test_CalGpsDistanceLatTemplate(int lonFirst, int latFirst, int lonSecond, int latSecond, int latDistance)
{
  int calLatDistance = CalGpsDistanceLat(lonFirst, latFirst, latSecond);
  mu_assert("error, calgpsdistancelon is not right", abs(calLatDistance - latDistance) < 5);
  return 0;
}

static char* test_CalGpsDistanceLat(void){
  test_CalGpsDistanceLatTemplate(1206480644, 380044038, 1208053971, 378817969, 13648);
  return 0;
}

static char* all_tests(void)
{
  mu_run_test(test_SwapNum);
  mu_run_test(test_IsDoubleEqual);
  mu_run_test(test_IsDoubleEqualWithTolerance);
  mu_run_test(test_Angle2Radians);
  mu_run_test(test_CalGpsDistance);
  mu_run_test(test_CalGpsDistanceLon);
  mu_run_test(test_CalGpsDistanceLat);
  return 0;
}

int main(int argc, char **argv) {
  char *result = all_tests();
  if (result != 0) {
    printf("%s\n", result);
  }
  else {
    printf("ALL TESTS PASSED\n");
  }
  printf("Tests run: %d\n", tests_run);

  return result != 0;
}
