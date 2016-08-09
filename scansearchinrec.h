#ifndef _ScanSearc_InRec_H
#define _ScanSearc_InRec_H

struct t_EnvPathLine;
typedef struct t_EnvPathLine *t_EnvPathLinePtr;

struct t_Environment;
typedef struct t_Environment *t_EnvironmentPtr;

struct t_EnvMember;
typedef struct t_EnvMember *t_EnvironmentMemberPtr;

t_EnvPathLinePtr DoScanSearchInRec( int xStart, int yStart, int xEnd, int yEnd, unsigned char isScanLineHorizon, t_EnvironmentPtr environment );

int GetEnvLength( t_EnvironmentPtr environment );
int GetEnvHeight( t_EnvironmentPtr environment );

int IsEnvPointInEnv( int xIndex, int yIndex, t_EnvironmentPtr environment );
t_EnvironmentMemberPtr GetEnvMember( int xIndex, int yIndex, t_EnvironmentPtr environment );
int IsEnvMemberObstacle( t_EnvironmentMemberPtr member );

t_EnvPathLinePtr InsertNewEnvPathLine( int x, int y, t_EnvPathLinePtr envPathLine );

#endif /* _ScanSearc_InRec_H */
