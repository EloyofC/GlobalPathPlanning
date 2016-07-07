/*
  Description: This file is for the interferce of advancedplanning;
  Author: Green
  Date: 16/06/02
 */
#ifndef _Advanced_Plan_H
#define _Advanced_Plan_H

#define c_costNormal 1

struct t_EnvMember;
typedef struct t_EnvMember *t_EnvironmentMemberPtr;
struct t_EnvironmentInfo;
typedef struct t_EnvironmentInfo *t_EnvironmentPtr;
typedef t_EnvironmentMemberPtr t_ElementTypePtr;
struct t_PathLines;
typedef struct t_PathLines *t_PathLinesPtr;

int PartialPathSearch(int xStart, int yStart, int xEnd, int yEnd, t_EnvironmentPtr environment);
void PrintAndFreePathLine(void);
int ScanSearch(int xStart, int yStart, int xEnd, int yEnd, int width, t_EnvironmentPtr m_environment);
t_EnvironmentMemberPtr SearchNearestFreePoint(int xIndex, int yIndex, t_EnvironmentPtr m_environment);
void PrintEnvPathLine(void);
t_PathLinesPtr GetGpsPathLines(t_EnvironmentPtr environment);

#endif
