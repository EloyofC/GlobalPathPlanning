/*
  Description: This file is the interferce with the internal environment of obstacle
  Author: Green
  Date: 16/06/02
*/

#ifndef _PreTreatMent_H
#define _PreTreatMent_H

struct t_Environment;
typedef struct t_Environment *t_EnvironmentPtr;
typedef struct t_Obstacles *t_ObstaclesPtr;

void SetObstaclesInEnvironment( t_ObstaclesPtr obstacles, t_EnvironmentPtr newEnvironment );

#endif
