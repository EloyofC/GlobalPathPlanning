#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "publicfun.h"
#include "envoperate.h"

#define c_neighbourNum 4

enum e_States{ States_unvisited, States_alive, States_dead };
enum e_Obstacle{ Obstacle_obstacle, Obstacle_free };

struct t_EnvMember
{
   int m_xIndex;
   int m_yIndex;
   int m_cost;
   int m_priority;
   int m_searched;
   enum e_Obstacle m_obstacle;
   enum e_States m_state;
   t_EnvironmentMemberPtr m_prevPtr;
};

struct t_Environment
{
   int m_envLength;
   int m_envHeight;
   int m_lengthOfUnit;
   int m_heightOfUnit;
   int m_xStart;
   int m_yStart;
   int m_xEnd;
   int m_yEnd;
   int m_lonTopLeft;
   int m_latTopLeft;
   int m_lonBottomRight;
   int m_latBottomRight;
   t_EnvironmentMemberPtr m_envMembersPtr;
};


static void InitialEnvironment(
   int length,
   int height,
   int lonTopLeft,
   int latTopLeft,
   int lonBottomRight,
   int latBottomRight,
   int lengthOfUnit,
   int heightOfUnit,
   t_EnvironmentPtr environment
   ) {
   environment->m_envLength = length;
   environment->m_envHeight = height;
   environment->m_lengthOfUnit = lengthOfUnit;
   environment->m_heightOfUnit = heightOfUnit;
   environment->m_xStart = 0;
   environment->m_yStart = 0;
   environment->m_xEnd = 0;
   environment->m_yEnd = 0;
   environment->m_lonTopLeft = lonTopLeft;
   environment->m_latTopLeft = latTopLeft;
   environment->m_lonBottomRight = lonBottomRight;
   environment->m_latBottomRight = latBottomRight;
}

static void InitialEnvironmentMember(
   int xIndex,
   int yIndex,
   t_EnvironmentPtr environment
   ) {
   t_EnvironmentMemberPtr member = GetEnvMember( xIndex, yIndex, environment );
   member->m_xIndex = xIndex;
   member->m_yIndex = yIndex;
   member->m_prevPtr = NULL;
   member->m_cost = 0;
   member->m_priority = 0;
   member->m_searched = 0;
   member->m_state = States_unvisited;
   member->m_obstacle = Obstacle_free;
}

/* Assume the relation between cordinatation of the environment and index is top and left to 0 and right and down to 1 */
static t_EnvironmentPtr CreateEnvironment(
   int length,
   int height
   ) {
   t_EnvironmentPtr environment = Malloc( sizeof( struct t_Environment ) );
   environment->m_envMembersPtr = Malloc( sizeof( struct t_EnvMember ) * length * height );
   return environment;
}

/* If the size of environment is not equal length * height, the rest will in the last row and col, it means the last will be big
   and the TopLeft and BottomRight index is gps data */
static t_EnvironmentPtr InitialEnvWithCell(
   int length,
   int height,
   int lengthOfUnit,
   int heightOfUnit,
   int lonTopLeft,
   int latTopLeft,
   int lonBottomRight,
   int latBottomRight
   ) {
   int xIndex = length / lengthOfUnit + 1; /* cos the length is indeed to be double type so plus 1 */
   int yIndex = height / heightOfUnit + 1;

   DebugCode (
      printf( "InitialEnvWithCell : x %d y %d\n", xIndex, yIndex );
      );
   t_EnvironmentPtr environment = CreateEnvironment( xIndex, yIndex );
   InitialEnvironment( xIndex, yIndex, lonTopLeft, latTopLeft, lonBottomRight, latBottomRight, lengthOfUnit, heightOfUnit, environment );
   for ( int i = 0; i < xIndex; i++ )
      for ( int j = 0; j < yIndex; j++ )
         InitialEnvironmentMember( i, j, environment );

   return environment;
}

t_EnvironmentPtr InitialEnvWithGps(
   int lengthOfUnit,
   int heightOfUnit,
   int lonTopLeft,
   int latTopLeft,
   int lonBottomRight,
   int latBottomRight
   ) {
   assert( lonTopLeft <= lonBottomRight ); /* confirm the relation */
   assert( latTopLeft >= latBottomRight ); /* need to confirm the number range */

   int length = CalGpsDistanceLon( lonTopLeft, latTopLeft, lonBottomRight );
   int height = CalGpsDistanceLat( lonTopLeft, latTopLeft, latBottomRight );

   DebugCode (
      printf( "InitialEnvWithGps : length %d height %d\n", length, height );
      fflush( stdout );
      );

   t_EnvironmentPtr environmentNew = InitialEnvWithCell( length, height, lengthOfUnit, heightOfUnit, lonTopLeft, latTopLeft, lonBottomRight, latBottomRight );

   return environmentNew;
}


void DeleteEnvironment(
   t_EnvironmentPtr environment
   ) {
   t_EnvironmentMemberPtr envMembers = environment->m_envMembersPtr;
   free( envMembers );
   free( environment );
}


static void ResetEnvironmentMember(
   int xIndex,
   int yIndex,
   t_EnvironmentPtr environment
   ) {
   t_EnvironmentMemberPtr member = GetEnvMember( xIndex, yIndex, environment );
   member->m_prevPtr = NULL;
   member->m_cost = 0;
   member->m_priority = 0;
   member->m_searched = 0;
   member->m_state = States_unvisited;
}

static void PrintEnvironmentMember(
   int xIndex,
   int yIndex,
   t_EnvironmentPtr environment
   ) {
   printf( "PrintEnvironmentMember : The EnvironmentMember of x %d y %d is ",
           xIndex, yIndex );
   t_EnvironmentMemberPtr member = GetEnvMember( xIndex, yIndex, environment );
   if ( IsEnvMemberObstacle( member ) )
      printf( "with obstacle\n" );
   else
      printf( "no obstacle\n" );
}

void PrintEnvironment(
   t_EnvironmentPtr environment
   ) {
   printf( "PrintEnvironment : The Environment with length %d height %d\n",
           environment->m_envLength, environment->m_envHeight );
   for ( int i = 0; i < environment->m_envLength; i++ )
      for ( int j = 0; j < environment->m_envHeight; j++ )
         PrintEnvironmentMember( i, j, environment );
}

/* may need to take care of the off-by-one errro */
void ResetEnvironment(
   t_EnvironmentPtr environment
   ) {
    for ( int i = 0; i < environment->m_envLength; i++ )
      for ( int j = 0; j < environment->m_envHeight; j++ )
         ResetEnvironmentMember( i, j, environment );
}

static int CalRectangleIndex(
   int x,
   int y,
   int length
   ) {
   return x + y * length;
}

t_EnvironmentMemberPtr GetEnvMember(
   int xIndex,
   int yIndex,
   t_EnvironmentPtr environment
   ) {
     /* may need to add something to confirm the validation of environment and the index */
   assert( environment != NULL );
   t_EnvironmentMemberPtr member = environment->m_envMembersPtr + CalRectangleIndex( xIndex, yIndex, environment->m_envLength );
   assert( environment != NULL );
   return member;
}

int IsEnvMemberObstacle(
   t_EnvironmentMemberPtr member
   ) {
   return ( member->m_obstacle == Obstacle_obstacle );
}

static int IsEnvMemberFree(
   t_EnvironmentMemberPtr member
   ) {
   return member->m_obstacle == Obstacle_free;
}

static void SetEnvMemberFree(
   t_EnvironmentMemberPtr member
   ) {
   member->m_obstacle = Obstacle_free;
}

int GetEnvLength(
   t_EnvironmentPtr environment
   ) {
   return environment->m_envLength;
}

int GetEnvHeight(
   t_EnvironmentPtr environment
   ) {
   return environment->m_envHeight;
}

int GetEnvStartX(
   t_EnvironmentPtr environment
   ) {
   return environment->m_xStart;
}

int GetEnvEndX(
   t_EnvironmentPtr environment
   ) {
   return environment->m_xEnd;
}

int GetEnvStartY(
   t_EnvironmentPtr environment
   ) {
   return environment->m_yStart;
}

int GetEnvEndY(
   t_EnvironmentPtr environment
   ) {
   return environment->m_yEnd;
}

int IsEnvMemberNotSearched(
   t_EnvironmentMemberPtr member
   ) {
   return member->m_searched == 0;
}

void ResetEnvAllNotSearched(
   t_EnvironmentPtr environment
   ) {
   for ( int i = 0; i < environment->m_envLength; i++ )
      for ( int j = 0; j < environment->m_envHeight; j++ ) {
          t_EnvironmentMemberPtr member = GetEnvMember( i, j, environment );
         member->m_searched = 0;
      }
}

void SetEnvMemberSearched(
   t_EnvironmentMemberPtr member
   ) {
   member->m_searched = 1;
}

int GetEnvMemberX(
   t_EnvironmentMemberPtr member
   ) {
   return member->m_xIndex;
}

int GetEnvMemberY(
   t_EnvironmentMemberPtr member
   ) {
   return member->m_yIndex;
}

int GetEnvMemberCost(
   t_EnvironmentMemberPtr member
   ) {
   return member->m_cost;
}

int GetEnvMemberPriority(
   t_EnvironmentMemberPtr member
   ) {
   return member->m_priority;
}

t_EnvironmentMemberPtr GetEnvMemberPrev(
   t_EnvironmentMemberPtr member
   ) {
   return member->m_prevPtr;
}

int GetEnvTopLeftLon(
   t_EnvironmentPtr environment
   ) {
   return environment->m_lonTopLeft;
}

int GetEnvTopLeftLat(
   t_EnvironmentPtr environment
   ) {
   return environment->m_latTopLeft;
}

int GetEnvBottomRightLon(
   t_EnvironmentPtr environment
   ) {
   return environment->m_lonBottomRight;
}

int GetEnvBottomRightLat(
   t_EnvironmentPtr environment
   ) {
   return environment->m_latBottomRight;
}

void SetEnvStartX(
   int xStart,
   t_EnvironmentPtr environment
   ) {
   environment->m_xStart= xStart;
}

void SetEnvStartY(
   int yStart,
   t_EnvironmentPtr environment
   ) {
   environment->m_yStart = yStart;
}

void SetEnvEndX(
   int xEnd,
   t_EnvironmentPtr environment
   ) {
   environment->m_xEnd = xEnd;
}

void SetEnvEndY(
   int yEnd,
   t_EnvironmentPtr environment
   ) {
   environment->m_yEnd = yEnd;
}

void SetEnvStartAndEnd(
   int xStart,
   int yStart,
   int xEnd,
   int yEnd,
   t_EnvironmentPtr environment
   ) {
   SetEnvStartX( xStart, environment );
   SetEnvStartY( yStart, environment );
   SetEnvEndX( xEnd, environment );
   SetEnvEndY( yEnd, environment );
}

void SetEnvMemberCost(
   int cost,
   t_EnvironmentMemberPtr member
   ) {
   member->m_cost = cost;
}

void SetEnvMemberPriority(
   int priority,
   t_EnvironmentMemberPtr member
   ) {
   member->m_priority = priority;
}

void SetEnvMemberDead(
   t_EnvironmentMemberPtr member
   ) {
   member->m_state = States_dead;
}

void SetEnvMemberAlive(
   t_EnvironmentMemberPtr member
   ) {
   member->m_state = States_alive;
}

void SetEnvMemberObstacle(
   t_EnvironmentMemberPtr member
   ) {
   member->m_obstacle = Obstacle_obstacle;
}


void SetEnvMemberPrev(
   t_EnvironmentMemberPtr memberPrev,
   t_EnvironmentMemberPtr member
   ) {
   member->m_prevPtr = memberPrev;
}

int IsEnvPointInEnv(
   int xIndex,
   int yIndex,
   t_EnvironmentPtr environment
   ) {
   return xIndex >= 0 && xIndex < GetEnvLength( environment ) &&
      yIndex >= 0 && yIndex < GetEnvHeight( environment );
}

int IsSearchEnd(
   t_EnvironmentMemberPtr member,
   t_EnvironmentPtr environment
   ) {
   return ( member->m_xIndex == GetEnvEndX( environment )) &&
      ( member->m_yIndex == GetEnvEndY( environment ) );
}

int IsEnvMemberUnvisited(
   t_EnvironmentMemberPtr member
   ) {
   return member->m_state == States_unvisited;
}

int IsEnvMemberAlive(
   t_EnvironmentMemberPtr member
   ) {
   return member->m_state == States_alive;
}

int IsEnvMemberDead(
   t_EnvironmentMemberPtr member
   ) {
   return member->m_state == States_dead;
}

int GetEnvLengthOfUnit(
   t_EnvironmentPtr environment
   ) {
   return environment->m_lengthOfUnit;
}

int GetEnvHeightOfUnit(
   t_EnvironmentPtr environment
   ) {
   return environment->m_heightOfUnit;
}

void FreeEnvMember(
   t_EnvironmentMemberPtr member
   ) {
   free( member );
}

t_EnvironmentMemberPtr CreateEnvMemberWithCost(
   int cost
   ) {
   t_EnvironmentMemberPtr member = Malloc( sizeof( struct t_EnvMember ) );
   member->m_cost = cost;
   member->m_prevPtr = NULL;
   return member;
}


int GetEnvXFromGpsLon(
   int lon,
   t_EnvironmentPtr environment
   ) {
   int topLeftLon = GetEnvTopLeftLon( environment );
   int topLeftLat = GetEnvTopLeftLat( environment );
   int lengthOfUnit = GetEnvLengthOfUnit( environment );
   int disLon = CalGpsDistanceLon( topLeftLon, topLeftLat, lon );
   if ( lon < topLeftLon ) {
      disLon = -1 * disLon;
   }
   return disLon / lengthOfUnit;
}


int GetEnvYFromGpsLat(
   int lat,
   t_EnvironmentPtr environment
   ) {
   int topLeftLon = GetEnvTopLeftLon( environment );
   int topLeftLat = GetEnvTopLeftLat( environment );
   int heightOfUnit = GetEnvHeightOfUnit( environment );
   int disLat = CalGpsDistanceLat( topLeftLon, topLeftLat, lat );
   if ( lat > topLeftLat ) {
      disLat = -1 * disLat;
   }
   return disLat / heightOfUnit;
}
