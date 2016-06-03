/*
  Description: This file is the interferce with the internal environment of obstacle
  Author: Green
  Date: 16/06/02
*/

#ifndef _GetChangeEnv_H

#include <stdio.h>
#include <stdlib.h>

struct t_EnvMember;
typedef struct t_EnvMember *t_EnvironmentMemberPtr;
struct t_EnvironmentInfo;
typedef struct t_EnvironmentInfo *t_EnvironmentPtr;

t_EnvironmentPtr CreateEnvironment(int length, int width, int xStart, int yStart, int xEnd, int YEnd); /* Create the environment which we are dealing with in program, details in struct t_EnvironmentPtrInfo */
void PrintEnvironment(t_EnvironmentPtr environment);
int GetEnvironmentLength(t_EnvironmentPtr environment); /* Get the length of the environment map */
int GetEnvironmentHeight(t_EnvironmentPtr environment); /* Get the height of the environment map */
t_EnvironmentMemberPtr GetEnvMemberPrev(t_EnvironmentMemberPtr member);
int GetEnvXInitial(t_EnvironmentPtr environment);
int GetEnvYInitial(t_EnvironmentPtr environment);
int GetEnvXGoal(t_EnvironmentPtr environment);
int GetEnvYGoal(t_EnvironmentPtr environment);
t_EnvironmentMemberPtr GetEnvMember(int xIndex, int yIndex, t_EnvironmentPtr environment); /* Get the Env Member for using */
int GetEnvMemberX(t_EnvironmentMemberPtr member);
int GetEnvMemberY(t_EnvironmentMemberPtr member);
int GetEnvMemberCost(t_EnvironmentMemberPtr member);
int GetEnvMemberPriority(t_EnvironmentMemberPtr member);
void DeleteEnvironment(t_EnvironmentPtr environment);   /* Delete the t_EnvironmentPtr when we almost finish the program */

void SetEnvMemberCost(int cost, t_EnvironmentMemberPtr member);
void SetEnvMemberPriority(int priority, t_EnvironmentMemberPtr member);
void SetEnvMemberDead(t_EnvironmentMemberPtr member);
void SetEnvMemberAlive(t_EnvironmentMemberPtr member);
void SetEnvMemberPrev(t_EnvironmentMemberPtr memberPrev, t_EnvironmentMemberPtr member);
void SetEnvMemberFlag(t_EnvironmentMemberPtr member);     /* set to envmem 1 */
int IsEnvMemberLegal(int xIndex, int yIndex, t_EnvironmentPtr environment);
int IsEnvMemberValid(int xIndex, int yIndex, t_EnvironmentPtr environment);
int IsEnvMemberUnvisited(t_EnvironmentMemberPtr member);
int IsEnvMemberAlive(t_EnvironmentMemberPtr member);
int IsEnvMemberDead(t_EnvironmentMemberPtr member);
int IsEnvMemberFlagNotSet(t_EnvironmentMemberPtr member); /* to check whether the flag of this environmentmember is not set */
int IsEnvMemberObstacle(t_EnvironmentMemberPtr member); /* To see if the currentenvmember is obstacle */

void ResetEnvAllFlag(t_EnvironmentPtr environment); /* flag is for potantial use, if u need to use it, this function set all the flag in environment to 1 */
int IsSearchEnd(int xGoal, int yGoal, t_EnvironmentMemberPtr member, t_EnvironmentPtr environment);

int IsEnvPointValid(int xIndex, int yIndex, t_EnvironmentPtr environment); /* to see if the point is not out of the env */
#endif
