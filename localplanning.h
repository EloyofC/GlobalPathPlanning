/*
  Description: This file is for the interferce of localplanning
  Author: Green
  Date: 16/07/06
*/

#ifndef _Local_Planning_H
#define _Local_Planning_H

struct t_EnvMember;
typedef struct t_EnvMember *t_EnvironmentMemberPtr;
struct t_Environment;
typedef struct t_Environment *t_EnvironmentPtr;
typedef t_EnvironmentMemberPtr t_ElementTypePtr;

int DoSomePathPlanning( int xStart, int yStart, int xEnd, int yEnd, int (* compute)( int, t_EnvironmentMemberPtr, t_EnvironmentPtr ), int (* cmp)( t_ElementTypePtr, t_ElementTypePtr ), t_EnvironmentPtr m_environment ); /* do the pathplanning between the start point to the goal using the environment map, and u can uncomment the printpathplan in localplanning.c to check the output*/
int ComparePriority( t_EnvironmentMemberPtr memberFirst, t_EnvironmentMemberPtr memberSecond ); /* the function to compare to environmentmember , as a parameter for dosomepathplanning*/

#endif
