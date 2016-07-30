/*
  Description: This file is for the public function for all functions to use
  Author: Green
  Date: 16/6/2
*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include "publicfun.h"

#define c_earthRadius 6371004.0
#define c_doubleEqualEpsilon 0.001

/* This routine is a wrap function of malloc, with the handling of out of space error by
 print to the stderr*/
void * Malloc(
   size_t size
   ) {
  void *MallocResult = malloc(size);
  if ( MallocResult == NULL ) {
    fprintf( stderr, "Out of Space" );
    exit( 0 );
  } else {
    return MallocResult;
  }
}

void Free(
   void *p
   ) {
  free(p);
}

int IntSquare(
   int x
   ) {
   return pow( x, 2 );
}

static double SevenPowersLargerAngle2Radians(
   double angle                 /* the angle is larger than normal angle 10^7 powers */
   ) {
  double newangle = angle / pow( 10, 7 );
  double radians = Angle2Radians( newangle );

  return radians;
}

/*  The original gps is larger than normal gps cor 10^7
  We use the formula: C = sin(LatA) * sin(LatB) + cos(LatA) * cos(LatB) * cos(LonA - LonB)
  D = R * arccos(C)*/
static int CalGpsDistance(
   int lonFirst,
   int latFirst,
   int lonSecond,
   int latSecond
   ) {
   double pointALon = SevenPowersLargerAngle2Radians( lonFirst );
   double pointALat = SevenPowersLargerAngle2Radians( latFirst );
   double pointBLon = SevenPowersLargerAngle2Radians( lonSecond );
   double pointBLat = SevenPowersLargerAngle2Radians( latSecond );

   double C = sin( pointALat ) * sin( pointBLat ) +
      cos( pointALat ) * cos( pointBLat ) * cos( pointALon - pointBLon );
   double D = c_earthRadius * acos( C );
   return ( int ) D;
}

int CalGpsDistanceLon(
   int lonFirst,                /* the lon and lat is larger than normal lon by 10^7 */
   int latFirst,
   int lonSecond
   ) {
  return CalGpsDistance( lonFirst, latFirst, lonSecond, latFirst );
}

int CalGpsDistanceLat(
   int lonFirst,
   int latFirst,
   int latSecond
   ) {
  return CalGpsDistance( lonFirst, latFirst, lonFirst, latSecond );
}

double Angle2Radians(
   double angle
   ) {
  assert( angle <= 180 && angle >= -180 );
  return angle * c_pi / 180;
}

void SwapNum(
   int *a,
   int *b
   ) {
  int temp = *a;
  *a = *b;
  *b = temp;
}

int IsDoubleEqual(
   double x1, double x2
   ) {
  return IsDoubleEqualWithTolerance( x1, x2, c_doubleEqualEpsilon );
}

int IsDoubleEqualWithTolerance(
   double x1,
   double x2,
   double epsilon
   ) {
  return fabs( x1 - x2 ) < epsilon;
}
