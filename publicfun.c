/*
  Description: This file is for the public function for all functions to use
  Author: Green
  Date: 16/6/2
*/

#include "PublicFun.h"

void *
Malloc(size_t size)
{
  void *MallocResult;

  MallocResult = malloc(size);
  if (MallocResult == NULL) {
    fprintf(stderr, "Out of space");
    exit(0);
  } else {
    return MallocResult;
  }
}
