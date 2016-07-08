/*
  Description: This file is the interferce with the internal environment of obstacle
  Author: Green
  Date: 16/06/02
*/

#ifndef _GetChangeEnv_H
#define _GetChangeEnv_H

struct t_EnvMember;
typedef struct t_EnvMember *t_EnvironmentMemberPtr;
struct t_EnvironmentInfo;
typedef struct t_EnvironmentInfo *t_EnvironmentPtr;
typedef struct t_Obstacles *t_ObstaclesPtr;

t_EnvironmentMemberPtr CreateEnvMemberWithCost(int cost); /* for the test function */
void FreeEnvMember(t_EnvironmentMemberPtr member);        /* for test function */
t_EnvironmentPtr InitialEnvWithGps(int lengthOfUnit, int widthOfUnit, int lonTopLeft, int latTopLeft, int lonBottomRight, int latBottomRight);
void SetObstaclesInEnvironment(t_ObstaclesPtr obstacles, t_EnvironmentPtr newEnvironment);
void PrintEnvironment(t_EnvironmentPtr environment);
int GetEnvLength(t_EnvironmentPtr environment); /* Get the length of the environment map */
int GetEnvWidth(t_EnvironmentPtr environment); /* Get the width of the environment map */
int GetEnvEndX(t_EnvironmentPtr environment);
int GetEnvEndY(t_EnvironmentPtr environment);
int GetEnvStartY(t_EnvironmentPtr environment);
int GetEnvStartX(t_EnvironmentPtr environment);
int GetEnvLengthOfUnit(t_EnvironmentPtr environment);
int GetEnvWidthOfUnit(t_EnvironmentPtr environment);
t_EnvironmentMemberPtr GetEnvMemberPrev(t_EnvironmentMemberPtr member);
int GetEnvTopLeftX(t_EnvironmentPtr environment);
int GetEnvTopLeftY(t_EnvironmentPtr environment);
int GetEnvBottomRightX(t_EnvironmentPtr environment);
int GetEnvBottomRightY(t_EnvironmentPtr environment);
t_EnvironmentMemberPtr GetEnvMember(int xIndex, int yIndex, t_EnvironmentPtr environment); /* Get the Env Member for using */
int GetEnvMemberX(t_EnvironmentMemberPtr member);
int GetEnvMemberY(t_EnvironmentMemberPtr member);
int GetEnvMemberCost(t_EnvironmentMemberPtr member);
int GetEnvMemberPriority(t_EnvironmentMemberPtr member);
void DeleteEnvironment(t_EnvironmentPtr environment);   /* Delete the t_EnvironmentPtr when we almost finish the program */

void SetEnvStartX(int xStart, t_EnvironmentPtr environment);
void SetEnvStartY(int yStart, t_EnvironmentPtr environment);
void SetEnvEndX(int xEnd, t_EnvironmentPtr environment);
void SetEnvEndY(int yEnd, t_EnvironmentPtr environment);
void SetEnvStartAndEnd(int xStart, int yStart, int xEnd, int yEnd, t_EnvironmentPtr environment);
void SetEnvMemberCost(int cost, t_EnvironmentMemberPtr member);
void SetEnvMemberPriority(int priority, t_EnvironmentMemberPtr member);
void SetEnvMemberDead(t_EnvironmentMemberPtr member);
void SetEnvMemberAlive(t_EnvironmentMemberPtr member);
void SetEnvMemberPrev(t_EnvironmentMemberPtr memberPrev, t_EnvironmentMemberPtr member);
void SetEnvMemberFlag(t_EnvironmentMemberPtr member);     /* set to envmem 1 */
void SetEnvMemberObstacle(t_EnvironmentMemberPtr member);
int IsEnvMemberLegal(int xIndex, int yIndex, t_EnvironmentPtr environment);
int IsEnvMemberValid(int xIndex, int yIndex, t_EnvironmentPtr environment);
int IsEnvMemberUnvisited(t_EnvironmentMemberPtr member);
int IsEnvMemberAlive(t_EnvironmentMemberPtr member);
int IsEnvMemberDead(t_EnvironmentMemberPtr member);
int IsEnvMemberFlagNotSet(t_EnvironmentMemberPtr member); /* to check whether the flag of this environmentmember is not set */
int IsEnvMemberObstacle(t_EnvironmentMemberPtr member); /* To see if the currentenvmember is obstacle */

void ResetEnvAllFlag(t_EnvironmentPtr environment); /* flag is for potantial use, if u need to use it, this function set all the flag in environment to 1 */
void ResetEnvironment(t_EnvironmentPtr environment); /* Reset all the members of the environment to initial value for another use */
int IsSearchEnd(t_EnvironmentMemberPtr member, t_EnvironmentPtr environment);

int IsEnvPointValid(int xIndex, int yIndex, t_EnvironmentPtr environment); /* to see if the point is not out of the env */
#endif