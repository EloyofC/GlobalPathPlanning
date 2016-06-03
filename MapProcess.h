/*
  Descreption: This file is for the interferce to do with map
  Author: Green
  Date: 16/6/2
*/

#ifndef _Map_Process_H

#define XINITIAL 0
#define YINITIAL 0
struct t_ObstacleMaps;
typedef struct t_ObstacleMaps *t_ObstacleMap;
typedef int *t_ObstacleMapPoint;
struct t_EnvironmentInfo;
typedef struct t_EnvironmentInfo *t_EnvironmentPtr;

t_ObstacleMap ReadObstacleMap(FILE *ifp);
void DeleteObstacleMap(t_ObstacleMap map);
t_EnvironmentPtr ObstacleMap2Enviroment(t_ObstacleMap data, int length, int height, int xStart, int yStart, int xEnd, int yEnd);

#endif
