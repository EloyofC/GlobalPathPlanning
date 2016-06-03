/*
  Description: This file is about the declarition and functions about the pretreatment of pathplanning such as transform file to map, transform map to Environment model.
  Author: Green
  Date: 15/12/15
*/

#include "GetChangeEnv.h"
#include "MapProcess.h"
#include "PublicFun.h"
#include "PathAndShow.h"

#define c_lineLength 100000

static t_ObstacleMapPoint PointInObstacleMap(t_ObstacleMap maps, int xIndex, int yIndex);
static int IsMapObstacle(t_ObstacleMapPoint xIndex);
static int AreaHaveObstacle(t_ObstacleMap maps, int xMin, int xMax, int yMin, int yMax);
static int CalRectangleIndex(int xIndex, int yIndex, int length, int height);
static int InitialStartOrEndDirection(int start, int end, int adir);
static void ResetEnvironmentMember(int xIndex, int yIndex, t_EnvironmentPtr environment);
static int SkipBlankAndLine(FILE *ifp);
static void *CreateObstacleMapdatas(int charcount);
static void PrintEnvironmentMember(int xIndex, int yIndex, t_EnvironmentPtr environment);
static void InitialEnvironmentMember(int xIndex, int yIndex, t_ObstacleMap maps, int xMin, int xMax, int yMin, int yMax, t_EnvironmentPtr environment);

enum e_States{States_unvisited, States_alive, States_dead};

struct t_EnvMember
{
  int m_xIndex;
  int m_yIndex;
  int m_obstacle;
  int m_cost;
  int m_priority;
  int m_flag;                     /* a m_flag for some function who need it  */
  enum e_States m_state;
  t_EnvironmentMemberPtr m_prevPtr;
};

struct t_EnvironmentInfo
{
  int m_envLength;
  int m_envHeight;
  int m_xInitial;
  int m_yInitial;
  int m_xGoal;
  int m_yGoal;
  t_EnvironmentMemberPtr m_envMembersPtr;
};

struct t_ObstacleMaps
{
  int m_obstacleMapLength;
  int m_obstacleMapHeight;
  struct t_ObstacleMapDatas *m_obstacleMapDatas;
};

struct t_ObstacleMapDatas
{
  int *m_obstacleMapData;
  struct t_ObstacleMapDatas *m_obstacleMapDatasNext;
};

static int
SkipBlank(FILE *ifp)
{
  int c;

  while (((c = getc(ifp)) == ' ') || (c == '\t')) {
    // printf("Skip c is %d\n", c);
    c = getc(ifp);
  }
  //printf("skiped c is %d\n", c);
  return c;
}

static int
SkipBlankAndLine(FILE *ifp)
{
  int c;

  while (((c = getc(ifp)) == ' ') || (c == '\t') || (c == '\n'))
    c = getc(ifp);
  return c;
}

static void*
CreateObstacleMapdatas(int charIndex)
{
  struct t_ObstacleMapDatas *newObstacleMapDatas;

  newObstacleMapDatas = Malloc(sizeof(struct t_ObstacleMapDatas));
  newObstacleMapDatas->m_obstacleMapData = Malloc(sizeof(int) * charIndex);

  newObstacleMapDatas->m_obstacleMapDatasNext = NULL;
  return newObstacleMapDatas;
}

/* assume the map is square and the zero point is in the left top*/
t_ObstacleMap
ReadObstacleMap(FILE *ifp)
{
  t_ObstacleMap NewObstacleMap;
  struct t_ObstacleMapDatas *newObstacleMapDatas, *currentObstacleMapDatas;
  int temp[c_lineLength];
  int charIndex, lineIndex, i, c;

  for (charIndex=0; (c = SkipBlank(ifp)) != '\n'; charIndex++) {
    //    printf("First line is: %c\n", c);
    temp[charIndex] = c -'0';
  }

  NewObstacleMap = Malloc(sizeof(struct t_ObstacleMaps));

  NewObstacleMap->m_obstacleMapLength = charIndex;
  currentObstacleMapDatas = newObstacleMapDatas = NewObstacleMap->m_obstacleMapDatas = CreateObstacleMapdatas(charIndex);
   for (i=0; i < charIndex; i++)
    *(newObstacleMapDatas->m_obstacleMapData + i) = temp[i];
  lineIndex = 1;

  while (((c = SkipBlankAndLine(ifp)) != EOF)) {
    newObstacleMapDatas = CreateObstacleMapdatas(charIndex);
    currentObstacleMapDatas->m_obstacleMapDatasNext = newObstacleMapDatas;
    currentObstacleMapDatas = newObstacleMapDatas;
    i = 0;
    while (1) {
      *(newObstacleMapDatas->m_obstacleMapData + i) = c - '0';
      i++;
      if (((c = SkipBlank(ifp)) == '\n') || (c == EOF))
        break;
    }
    if (i != charIndex) {
      fprintf(stderr, "Some line counts of the input Map format is not equal");
      exit(0);
    }
    lineIndex++;
    if (c == EOF) {
      break;
    }
  }
  //printf("The charIndex is:%d, lineIndex is: %d\n", charIndex, lineIndex);
  NewObstacleMap->m_obstacleMapHeight = lineIndex;

  return NewObstacleMap;
}

static int
IsMapObstacle(t_ObstacleMapPoint xIndex)
{
  return *xIndex > 0;
}

static t_ObstacleMapPoint
PointInObstacleMap(t_ObstacleMap maps, int xIndex, int yIndex)
{
  int i;
  struct t_ObstacleMapDatas *currentObstacleMap;

  currentObstacleMap = maps->m_obstacleMapDatas;
  for (i=0; i < yIndex; i++) {
    currentObstacleMap = currentObstacleMap->m_obstacleMapDatasNext;
  }
  return currentObstacleMap->m_obstacleMapData + xIndex;
}

void
DeleteObstacleMap(t_ObstacleMap map)
{
  struct t_ObstacleMapDatas *current, *temp;

  for (current = map->m_obstacleMapDatas; current != NULL;) {
    temp = current->m_obstacleMapDatasNext;
    free(current);
    current = temp;
  }
  free(map);
}

static int
AreaHaveObstacle(t_ObstacleMap maps, int xMin, int xMax, int yMin, int yMax)
{
  int i, j;

  for (i = xMin; i < xMax; i++)
    for (j = yMin; j < yMax; j++)
      if (IsMapObstacle(PointInObstacleMap(maps, i, j)))
        return 1;
  return 0;
}

/* Assume the relation between cordinatation of the environment and index is top and left to 0 and right and down to 1 */
t_EnvironmentPtr
CreateEnvironment(int length, int width, int xStart, int yStart, int xEnd, int yEnd)
{
  t_EnvironmentPtr p;
  int xIndexLast, yIndexLast;

  p = Malloc(sizeof(struct t_EnvironmentInfo));
  p->m_envMembersPtr = Malloc(sizeof(struct t_EnvMember) * length * width);

  p->m_envLength = length;
  p->m_envHeight = width;
  xIndexLast = length - 1;
  yIndexLast = width - 1;
  p->m_xInitial = InitialStartOrEndDirection(XINITIAL, xIndexLast, xStart);
  p->m_yInitial = InitialStartOrEndDirection(YINITIAL, yIndexLast, yStart);
  p->m_xGoal = InitialStartOrEndDirection(XINITIAL, xIndexLast, xEnd);
  p->m_yGoal = InitialStartOrEndDirection(YINITIAL, xIndexLast, yEnd);

  return p;
}

static int
InitialStartOrEndDirection(int start, int end, int adir)
{
  if (adir <= 0 && adir > -1) {
    return start;
  } else if (adir == -1 || adir >= end) {
    return end;
  } else {
    return adir;
  }
}

void
DeleteEnvironment(t_EnvironmentPtr environment)
{
  t_EnvironmentMemberPtr p;

  p = environment->m_envMembersPtr;
  free(p);
  free(environment);
}

/* If the size of environment is not equal length * height, the rest will in the last row and col, it means the last will be big */
t_EnvironmentPtr
ObstacleMap2Enviroment(t_ObstacleMap maps, int LengthOfUnit, int HeightOfUnit, int xStart, int yStart, int xEnd, int yEnd)
{
  int i, j;
  int xIndex, yIndex;
  int xRemained, yRemained;
  int ObstacleMapLength, ObstacleMapHeight;
  t_EnvironmentPtr environment;

  ObstacleMapLength = maps->m_obstacleMapLength;
  ObstacleMapHeight = maps->m_obstacleMapHeight;
  xIndex = ObstacleMapLength / LengthOfUnit - 1;
  yIndex = ObstacleMapHeight / HeightOfUnit - 1;
  xRemained = ObstacleMapLength - LengthOfUnit * (xIndex + 1);
  yRemained = ObstacleMapHeight - HeightOfUnit * (yIndex + 1);

  environment = CreateEnvironment(xIndex + 1, yIndex + 1, xStart, yStart, xEnd, yEnd);
  for (i = 0; i < xIndex; i++)
    for (j = 0; j < yIndex; j++)
      InitialEnvironmentMember(i, j, maps, i * LengthOfUnit, (i + 1) * LengthOfUnit, j * HeightOfUnit, (j + 1) * HeightOfUnit, environment);

  for (i = 0, j = yIndex; i < xIndex; i++)
    InitialEnvironmentMember(i, j, maps, i * LengthOfUnit, (i + 1) * LengthOfUnit + xRemained, j * HeightOfUnit, (j + 1) * HeightOfUnit, environment);
  for (j = 0, i = xIndex; j < yIndex; j++)
    InitialEnvironmentMember(i, j, maps, i * LengthOfUnit, (i + 1) * LengthOfUnit, j * HeightOfUnit, (j + 1) * HeightOfUnit + yRemained, environment);
  InitialEnvironmentMember(xIndex, yIndex, maps, xIndex * LengthOfUnit, ObstacleMapLength, yIndex * HeightOfUnit, ObstacleMapHeight, environment);

  return environment;
}

static void
InitialEnvironmentMember(int xIndex, int yIndex, t_ObstacleMap maps, int xMin, int xMax, int yMin, int yMax, t_EnvironmentPtr environment)
{
  t_EnvironmentMemberPtr member;

  member = GetEnvMember(xIndex, yIndex, environment);
  member->m_xIndex = xIndex;
  member->m_yIndex = yIndex;
  member->m_prevPtr = NULL;
  member->m_cost = 0;
  member->m_priority = 0;
  member->m_flag = 0;
  member->m_state = States_unvisited;
  member->m_obstacle = AreaHaveObstacle(maps, xMin, xMax, yMin, yMax);
}

static void
ResetEnvironmentMember(int xIndex, int yIndex, t_EnvironmentPtr environment)
{
  t_EnvironmentMemberPtr member;

  member = GetEnvMember(xIndex, yIndex, environment);
  member->m_prevPtr = NULL;
  member->m_cost = 0;
  member->m_priority = 0;
  member->m_flag = 0;
  member->m_state = States_unvisited;
}

static void
PrintEnvironmentMember(int xIndex, int yIndex, t_EnvironmentPtr environment)
{
  t_EnvironmentMemberPtr member;

  member = GetEnvMember(xIndex, yIndex, environment);
  printf("The EnvironmentMember of x:%d y:%d is ", xIndex, yIndex);
  if (member->m_obstacle)
    printf("with obstacle\n");
  else
    printf("no obstacle\n");
}

void
PrintEnvironment(t_EnvironmentPtr environment)
{
  int i,j;

  printf("The Environment with Length:%d, Height:%d\n", environment->m_envLength, environment->m_envHeight);
  for (i=0; i < environment->m_envLength; i++)
    for (j=0; j < environment->m_envHeight; j++)
      PrintEnvironmentMember(i, j, environment);
}

void
ResetEnvironment(t_EnvironmentPtr environment)
{
  int i,j;

  for (i=0; i < environment->m_envLength; i++)
    for (j=0; j < environment->m_envHeight; j++)
      ResetEnvironmentMember(i, j, environment);
  /* for (i = environment->m_envLength; i > -1; i--) */
  /*   for (j = environment->m_envHeight; j > -1; j--) { */
  /*     ResetEnvironmentMember(environment, i, j); */
  /*   } */
}

static int
CalRectangleIndex(int x, int y, int length, int height)
{
  return x + y * length;
}

t_EnvironmentMemberPtr
GetEnvMember(int xIndex, int yIndex, t_EnvironmentPtr environment)
{
  return (environment->m_envMembersPtr + CalRectangleIndex(xIndex, yIndex, environment->m_envLength, environment->m_envHeight));
}

int
IsEnvMemberObstacle(t_EnvironmentMemberPtr member)
{
  return member->m_obstacle;
}

int
IsEnvPointValid(int xIndex, int yIndex, t_EnvironmentPtr environment)
{
  if (xIndex > 0 && xIndex < environment->m_envLength && yIndex > 0 && yIndex < environment->m_envHeight) {
    return 1;
  } else {
    return 0;
  }
}

int
GetEnvironmentLength(t_EnvironmentPtr environment)
{
  return environment->m_envLength;
}

int
GetEnvironmentHeight(t_EnvironmentPtr environment)
{
  return environment->m_envHeight;
}

int
IsEnvMemberFlagNotSet(t_EnvironmentMemberPtr member)
{
  return member->m_flag == 0;
}

void
ResetEnvAllFlag(t_EnvironmentPtr environment)
{
  int i, j;
  t_EnvironmentMemberPtr member;

  for (i=0; i < environment->m_envLength; i++)
    for (j=0; j < environment->m_envHeight; j++) {
      member = GetEnvMember(i, j, environment);
      member->m_flag = 0;
    }
}

void
SetEnvMemberFlag(t_EnvironmentMemberPtr member)
{
  member->m_flag = 1;
}

int
GetEnvMemberX(t_EnvironmentMemberPtr member)
{
  return member->m_xIndex;
}

int
GetEnvMemberY(t_EnvironmentMemberPtr member)
{
  return member->m_yIndex;
}

int
GetEnvMemberCost(t_EnvironmentMemberPtr member)
{
  return member->m_cost;
}

int
GetEnvMemberPriority(t_EnvironmentMemberPtr member)
{
  return member->m_priority;
}

t_EnvironmentMemberPtr
GetEnvMemberPrev(t_EnvironmentMemberPtr member)
{
  return member->m_prevPtr;
}

int
GetEnvXInitial(t_EnvironmentPtr environment)
{
  return environment->m_xInitial;
}

int
GetEnvYInitial(t_EnvironmentPtr environment)
{
  return environment->m_yInitial;
}

int
GetEnvXGoal(t_EnvironmentPtr environment)
{
  return environment->m_xGoal;
}

int
GetEnvYGoal(t_EnvironmentPtr environment)
{
  return environment->m_yGoal;
}

void
SetEnvMemberCost(int cost, t_EnvironmentMemberPtr member)
{
  member->m_cost = cost;
}

void
SetEnvMemberPriority(int priority, t_EnvironmentMemberPtr member)
{
  member->m_priority = priority;
}

void
SetEnvMemberDead(t_EnvironmentMemberPtr member)
{
  member->m_state = States_dead;
}

void
SetEnvMemberAlive(t_EnvironmentMemberPtr member)
{
  member->m_state = States_alive;
}

void
SetEnvMemberPrev(t_EnvironmentMemberPtr memberPrev, t_EnvironmentMemberPtr member)
{
  member->m_prevPtr = memberPrev;
}

/* IsEnvMemLegal : means the member is in environment and is unvisted */
int
IsEnvMemberLegal(int xIndex, int yIndex, t_EnvironmentPtr environment)
{
  return IsEnvMemberValid(xIndex, yIndex, environment) && (!IsEnvMemberDead(GetEnvMember(xIndex, yIndex, environment)));
}

int
IsEnvMemberValid(int xIndex, int yIndex, t_EnvironmentPtr environment)
{
    return xIndex >= environment->m_xInitial && xIndex <= environment->m_xGoal && yIndex >= environment->m_yInitial && yIndex <= environment->m_yGoal;
}

int
IsSearchEnd(int xGoal, int yGoal, t_EnvironmentMemberPtr member, t_EnvironmentPtr environment)
{
  return (member->m_xIndex == xGoal) && (member->m_yIndex == yGoal);
}

int
IsEnvMemberUnvisited(t_EnvironmentMemberPtr member)
{
  return member->m_state == States_unvisited;
}

int
IsEnvMemberAlive(t_EnvironmentMemberPtr member)
{
  return member->m_state == States_alive;
}

int
IsEnvMemberDead(t_EnvironmentMemberPtr member)
{
  return member->m_state == States_dead;
}
