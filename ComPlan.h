/*
  Description: This file is for the interferce of advancedplanning;
  Author: Green
  Date: 16/06/02
 */
#ifndef _Com_Plan_H


#define c_costNormal 1

struct t_EnvMember;
typedef struct t_EnvMember *t_EnvironmentMemberPtr;
struct t_EnvironmentInfo;
typedef struct t_EnvironmentInfo *t_EnvironmentPtr;
typedef t_EnvironmentMemberPtr t_ElementTypePtr;

void DoSomePathPlanning(int xStart, int yStart, int xGoal, int yGoal, int (* compute)(int, t_EnvironmentMemberPtr, t_EnvironmentPtr), int (* cmp)(t_ElementTypePtr, t_ElementTypePtr), t_EnvironmentPtr m_environment); /* do the pathplanning between the start point to the goal using the environment map, and u can uncomment the printpathplan in localplanning.c to check the output*/
void ResetEnvironment(t_EnvironmentPtr environment); /* Reset all the members of the environment to initial value for another use */
int ComparePriority(t_EnvironmentMemberPtr memberFirst, t_EnvironmentMemberPtr memberSecond); /* the function to compare to environmentmember , as a parameter for dosomepathplanning*/

void PrintAndFreePathLine();
void ScanSearch(int xStart, int yStart, int xGoal, int yGoal, int width, t_EnvironmentPtr m_environment);
t_EnvironmentMemberPtr SearchNearestFreePoint(int xIndex, int yIndex, t_EnvironmentPtr m_environment);

#endif
