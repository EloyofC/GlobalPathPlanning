#ifndef obsctclelabel_H

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "global_planning.h"

typedef struct BwConnectMem *BwConnMems;
struct PixelLists;
struct SinglePixelList;

/* free the BwConnectMem Resourse when u don't need it */
void DeleteLabelRes(BwConnMems ConnectInfo);

/* pass the image and get the label information and u need to also pass the image width image height
 and the mode of 4 or 8 connectivity
 to get the details, see the struct
*/
BwConnMems LabObstacles(const Environment GlobalEnv, const int mode); /* get the obstacles */
BwConnMems LabObstacleborders(const Environment GlobalEnv, const int mode); /* get the borders of the obstacles */
void PrintAllConnInfo(const BwConnMems ConnectInfo);

#endif
/*
  struct BwConnectMem
  {
  int ConnKind;
  int ImagWidth;
  int ImageHeight;
  int ObjectCounts;
  struct PixelLists *ConnectGroup;
  };

  struct PixelLists
  {
  int CurrentObjectCounts;
  struct SinglePixelList *ConnectMem;
  struct PixelLists *next;
  };

  struct SinglePixelList
  {
  int cor_x;
  int cor_y;
  struct SinglePixelList *next;
  };

*/
