/*
  Description: This file is for the public function for all functions to use
  Author: Green
  Date: 16/6/2
*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include "PublicFun.h"

#define c_earthRadius 6371004.0
#define c_pi 3.1415926
#define c_doubleEqualEpsilon 0.001

static int CalGpsDistance(int lonFirst, int latFirst, int lonSecond, int latSecond);
static double StupidAngle2Radians(double angle);

void *
Malloc(size_t size)
{
  void *MallocResult;

  MallocResult = malloc(size);
  if (MallocResult == NULL) {
    fprintf(stderr, "Out of Space");
    exit(0);
  } else {
    return MallocResult;
  }
}

void
Free(void *p)
{
  free(p);
}

/*  The original gps is * 10^7
  We use the formula: C = sin(LatA) * sin(LatB) + cos(LatA) * cos(LatB) * cos(LonA - LonB)
  D = R * arccos(C)*/
static int
CalGpsDistance(int lonFirst, int latFirst, int lonSecond, int latSecond)
{
  double pointALon, pointALat, pointBLon, pointBLat;
  double C, D;

  pointALon = StupidAngle2Radians(lonFirst);
  pointALat = StupidAngle2Radians(latFirst);
  pointBLon = StupidAngle2Radians(lonSecond);
  pointBLat = StupidAngle2Radians(latSecond);

  C = sin(pointALat) * sin(pointBLat) + cos(pointALat) * cos(pointBLat) * cos(pointALon - pointBLon);
  D = c_earthRadius * acos(C);
  return (int) D;
}


int
CalGpsDistanceLon(int lonFirst, int latFirst, int lonSecond, int latSecond)
{
  return CalGpsDistance(lonFirst, latFirst, lonSecond, latFirst);
}

int
CalGpsDistanceLat(int lonFirst, int latFirst, int lonSecond, int latSecond)
{
  return CalGpsDistance(lonFirst, latFirst, lonFirst, latSecond);
}


/* stupid angle is * already * 10^7 */
static double
StupidAngle2Radians(double angle)
{
  double newangle, radians;

  newangle = angle / pow(10, 7);
  radians = Angle2Radians(newangle);

  return radians;
}

double
Angle2Radians(double angle)
{
  assert(angle <= 180 && angle >= -180);
  return angle * c_pi / 180;
}

void
SwapNum(int *a, int *b)
{
  int temp;

  temp = *a;
  *a = *b;
  *b = temp;
}

int
IsDoubleEqual(double x1, double x2)
{
  return IsDoubleEqualWithTolerance(x1, x2, c_doubleEqualEpsilon);
}

int
IsDoubleEqualWithTolerance(double x1, double x2, double epsilon)
{
  return fabs(x1 - x2) < epsilon;
}
