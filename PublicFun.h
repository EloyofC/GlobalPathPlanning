/*
  Description: This file is for the public function for all functions to use
  Author: Green
  Date: 16/6/2
*/

#ifndef _Public_Fun_H

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>

//#define DEBUG
#if defined( DEBUG )
#define DebugCode( code_fragment ) { code_fragment }
#else
#define DebugCode( code_fragment )
#endif

void *Malloc(size_t size);

#endif
