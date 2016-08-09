/*
  Description: This file is for the public function for all functions to use
  Author: Green
  Date: 16/6/2
*/

#ifndef _Public_Fun_H
#define _Public_Fun_H

#include <stddef.h>

//#define DEBUG
#if defined( DEBUG )
#define DebugCode( code_fragment ) { code_fragment }
#define DebugCodeDetail( code_fragment ) { code_fragment }
#else
#define DebugCode( code_fragment )
#define DebugCodeDetail( code_fragment )
#endif

#define c_pi 3.1415926

void *Malloc( size_t size );
void Free( void *p );
int IntSquare( int x );
int CalGpsDistanceLon( int lonFirst, int latFirst, int lonSecond );
int CalGpsDistanceLat( int lonFirst, int latFirst, int latSecond );
double Angle2Radians( double angle );
void SwapNum( int *a, int *b );
int IsDoubleEqual( double x1, double x2 );
int IsDoubleEqualWithTolerance( double x1, double x2, double epsilon );

#endif
