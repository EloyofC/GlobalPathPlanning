#include <stdio.h>
#include "MapProcess.h"
#include "ComPlan.h"
#include "GetChangeEnv.h"

#define c_length 1
#define c_height 1

int main(int argc, char *argv[])
{
  t_ObstacleMap newObstacleMap;
  t_EnvironmentPtr newEnvironment;


  newObstacleMap = ReadObstacleMap(stdin);
  newEnvironment = ObstacleMap2Enviroment(newObstacleMap, c_length, c_height, 0, 0, -1, -1);
  // OutputEnvironment(NewEnvironment);
  ScanSearch(0, 0, 19, 19, 5, newEnvironment);
  PrintAndFreePathLine();
  DeleteEnvironment(newEnvironment);
  DeleteObstacleMap(newObstacleMap);

  return 0;
}
