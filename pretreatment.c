/*
  Description: This file is about the declarition and functions about the pretreatment of pathplanning such as transform file to map, transform map to Environment model.
  Author: Green
  Date: 15/12/15
*/

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "pretreatment.h"
#include "mission.h"
#include "publicfun.h"
#include "envoperate.h"

typedef struct t_AdaptPoint	/* just convert the index of the gps cor and add the first point to the end of the link */
{
   int m_xIndex;
   int m_yIndex;
   struct t_AdaptPoint * m_next;
} *t_AdaptPointPtr;

typedef struct t_AllEdge
{
   int m_x1;
   int m_y1;
   int m_x2;
   int m_y2;
} *t_AllEdgePtr;

typedef struct t_AllEdgeArray
{
   int m_counts;
   t_AllEdgePtr m_edgeArrays;
} *t_AllEdgeArrayPtr;

typedef struct t_Edge
{
   int m_yMax;
   int m_yMin;
   double m_x;                  /* this  */
   double m_ratio;
} *t_EdgePtr;

/* The EdgelistNode is a double linked non-cycle list */
typedef struct t_EdgeListNode
{
   t_EdgePtr m_edge;
   struct t_EdgeListNode *m_prev;
   struct t_EdgeListNode *m_next;
} *t_EdgeListNodePtr;

typedef struct t_EdgeList
{
   int m_length;
   t_EdgeListNodePtr m_edgeNodes;
} *t_EdgeListPtr;

typedef struct t_EdgeTable
{
   int m_height;
   t_EdgeListPtr *m_edgeArrays;  /* get the edgelist by height */
} *t_EdgeTablePtr;

/* The activeedges have the head of the edgelist cos it is frequently using append and remove */
typedef struct t_ActiveEdgeTable
{
   t_EdgeTablePtr m_edgeTable;
   t_EdgeListPtr m_activeEdges;
} *t_ActiveEdgeTablePtr;


static int GetObstaclesCounts(
   t_ObstaclesPtr obstacles
   ) {
   return obstacles->m_obstacleCounts;
}

static t_SingleObstaclePtr GetObstacleArray(
   t_ObstaclesPtr obstacles
   ) {
   return obstacles->m_obstacleMembersPtr;
}

static t_SingleObstaclePtr GetSingleObstacle(
   t_SingleObstaclePtr singleObstacle,
   int index
   ) {
   return singleObstacle + index;
}

static int GetVertexCounts(
   t_SingleObstaclePtr singleObstacle
   ) {
   return singleObstacle->m_vertexCounts;
}

static t_PointCorPtr GetSinglePoint(
   t_PointCorPtr pointArray,
   int index
   ) {
   return pointArray + index;
}

static t_PointCorPtr GetPointArray(
   t_SingleObstaclePtr singleObstacle
   ) {
   return singleObstacle->m_pointsPtr;
}

static int GetPointCorLon(
   t_PointCorPtr point
   ) {
   return point->m_lon;
}

static int GetPointCorLat(
   t_PointCorPtr point
   ) {
   return point->m_lat;
}


/* can be < 0 number */
static int CalGpsPointXInEnv(
   t_PointCorPtr obstaclePoint,
   t_EnvironmentPtr environment
   ) {
   int lengthOfUnit = GetEnvLengthOfUnit( environment );
   int lonPoint = GetPointCorLon( obstaclePoint );
   int latPoint = GetPointCorLat( obstaclePoint );
   int topLeftLon = GetEnvTopLeftLon( environment );
   int disLon = CalGpsDistanceLon( lonPoint, latPoint, topLeftLon );
   if ( lonPoint < topLeftLon ) {
      disLon = -1 *disLon;
   }
   return disLon / lengthOfUnit;
}

static int CalGpsPointYInEnv(
   t_PointCorPtr obstaclePoint,
   t_EnvironmentPtr environment
   ) {
   int widthOfUnit = GetEnvWidthOfUnit( environment );
   int lonPoint = GetPointCorLon( obstaclePoint );
   int latPoint = GetPointCorLat( obstaclePoint );
   int topLeftLat =  GetEnvTopLeftLat( environment );
   int disLat = CalGpsDistanceLat( lonPoint, latPoint, topLeftLat );
   if ( latPoint > topLeftLat ) {	/* if y is > 0 it should < the topleft */
      disLat = -1 *disLat;
   }
   return disLat / widthOfUnit;
}

static int GetAdaptPointX(
   t_AdaptPointPtr point
   ) {
   return point->m_xIndex;
}

static int GetAdaptPointY(
   t_AdaptPointPtr point
   ) {
   return point->m_yIndex;
}

static t_AdaptPointPtr GetNextAdaptPoint(
   t_AdaptPointPtr point
   ) {
   return point->m_next;
}

static t_AdaptPointPtr InsertAdaptPoint(
   int x,
   int y,
   t_AdaptPointPtr point
   ) {
   t_AdaptPointPtr pointNew = Malloc( sizeof( struct t_AdaptPoint ) );
   pointNew->m_xIndex = x;
   pointNew->m_yIndex = y;
   pointNew->m_next = point;
   return pointNew;
}

static void FreeAdaptPoints(
   t_AdaptPointPtr points
   ) {
   t_AdaptPointPtr pointNext;

   for ( ; points != NULL; points = pointNext ) {
      pointNext = points->m_next;
      free( points );
   }
}

static void PrintAdaptPoints(
   t_AdaptPointPtr points
   ) {
   t_AdaptPointPtr pointNext;

   for ( ; points != NULL; points = pointNext ) {
      pointNext = points->m_next;
      printf( "PrintAdaptPoints : x %d y %d\n", GetAdaptPointX( points ), GetAdaptPointY( points ) );
   }
}

static t_AdaptPointPtr AdjustAndInsertAdapt(
   t_PointCorPtr obstaclePoint,
   t_EnvironmentPtr environment,
   t_AdaptPointPtr adaptPoints
   ) {
   int x = CalGpsPointXInEnv( obstaclePoint, environment );
   int y = CalGpsPointYInEnv( obstaclePoint, environment );
   t_AdaptPointPtr adaptPointsNew = InsertAdaptPoint( x, y, adaptPoints );
   assert( adaptPointsNew != NULL );
   return adaptPointsNew;
}

/* to ensure the closure of the obstacle if the two crosspoint is in two different lines get the corner vertex into the pointsLink */
static t_AdaptPointPtr GetAdaptPoints(
   t_SingleObstaclePtr singleObstacle,
   t_EnvironmentPtr environment
   ) {
   assert( singleObstacle != NULL );

   int pointCounts = GetVertexCounts( singleObstacle );
   t_AdaptPointPtr adaptPoints = NULL;
   t_PointCorPtr currentPoint;
   t_PointCorPtr pointArray = GetPointArray( singleObstacle );
   t_PointCorPtr firstPoint = GetSinglePoint( pointArray, 0 );

   for ( int i = 0; i < pointCounts; i++ ) {
      currentPoint = GetSinglePoint( pointArray, i );
      adaptPoints = AdjustAndInsertAdapt( currentPoint, environment, adaptPoints );
   }

   /* add the first one to the end */
   adaptPoints = AdjustAndInsertAdapt( firstPoint, environment, adaptPoints );

   return adaptPoints;
}

static int CalYMax(
   int y1,
   int y2
   ) {
   return y1 > y2 ? y1 : y2;
}

static int CalYMinPositive(
   int y1,
   int y2
   ) {
   if ( y1 < y2 ) {
      if ( y1 < 0 ) {
         return 0;
      } else {
         return y1;
      }
   } else {
      if ( y2 < 0 ) {
         return 0;
      } else {
         return y2;
      }
   }
}

/* This routine return x cor with the ymin and if y < 0 it will be left
 to be zero*/
static double CalXWithYPositiveMin(
   int x1,
   int y1,
   int x2,
   int y2
   ) {
   if ( y1 < 0 || y2 < 0 ) {
      double x;
      x = ( -1.0 * ( double ) ( x2 - x1 ) ) / ( double ) ( y2 - y1 ) * y2 + x2;
      return x;
   } else if ( y1 < y2 ) {
      return ( double ) x1;
   } else {
      return ( double ) x2;
   }
}

static double CalRevRatio(
   int x1,
   int y1,
   int x2,
   int y2
   ) {
   assert( y1 != y2 );

   if ( y1 < y2 ) {
      return ( double )( x2 - x1 ) / ( double )( y2 - y1 );
   } else {
      return ( double )( x1 - x2 ) / ( double )( y1 - y2 );
   }
}

static t_EdgePtr CreateAEdge(
   int x1,
   int y1,
   int x2,
   int y2
   ) {
   assert( y1 != y2 );

   t_EdgePtr newEdge = Malloc( sizeof( struct t_Edge ) );
   newEdge->m_yMax = CalYMax( y1, y2 );
   newEdge->m_yMin = CalYMinPositive( y1, y2 );
   newEdge->m_x = CalXWithYPositiveMin( x1, y1, x2, y2 );
   newEdge->m_ratio = CalRevRatio( x1, y1, x2, y2 );
   return newEdge;
}

static t_EdgePtr CloneAEdge(
   t_EdgePtr edge
   ) {
   t_EdgePtr newEdge = Malloc( sizeof( struct t_Edge ) );
   newEdge->m_yMax = edge->m_yMax;
   newEdge->m_yMin = edge->m_yMin;
   newEdge->m_x = edge->m_x;
   newEdge->m_ratio = edge->m_ratio;
   return newEdge;
}

static void FreeAEdge(
   t_EdgePtr edge
   ) {
   Free( edge );
}

static double GetEdgeX(
   t_EdgePtr edge
   ) {
   return edge->m_x;
}

static int GetEdgeYMin(
   t_EdgePtr edge
   ) {
   return edge->m_yMin;
}

static int GetEdgeYMax(
   t_EdgePtr edge
   ) {
   return edge->m_yMax;
}

static void UpdateEdgeXWithRatio(
   t_EdgePtr edge,
   double ratio
   ) {
   edge->m_x += ratio;
}

static void UpdateEdgeX(
   t_EdgePtr edge
   ) {
   UpdateEdgeXWithRatio( edge, edge->m_ratio );
}

static double GetEdgeRevRatio(
   t_EdgePtr edge
   ) {
   return edge->m_ratio;
}

static t_EdgeListNodePtr CreateEdgeListNode(
   t_EdgePtr edge
   ) {
   t_EdgeListNodePtr edgeNode = Malloc( sizeof( struct t_EdgeListNode ) );
   edgeNode->m_edge = edge;
   edgeNode->m_prev = NULL;
   edgeNode->m_next = NULL;
   return edgeNode;
}

static t_EdgeListNodePtr CreateEdgeListNodeWithNewEdge(
   t_EdgePtr edge
   ) {
   t_EdgePtr newEdge = CloneAEdge( edge );
   t_EdgeListNodePtr newEdgeNode = CreateEdgeListNode( newEdge );
   return newEdgeNode;
}

static t_EdgeListNodePtr GetEdgeListNodeNext(
   t_EdgeListNodePtr edgeNode
   ) {
   return edgeNode->m_next;
}

static t_EdgePtr GetEdgeListNodeEdge(
   t_EdgeListNodePtr edgeNode
   ) {
   return edgeNode->m_edge;
}

static t_EdgeListNodePtr GetEdgeListNodePrev(
   t_EdgeListNodePtr edgeNode
   ) {
   return edgeNode->m_prev;
}

static void SetEdgeListNodeNext(
   t_EdgeListNodePtr edgeNodeNext,
   t_EdgeListNodePtr edgeNode
   ) {
   edgeNode->m_next = edgeNodeNext;
}

static void SetEdgeListNodePrev(
   t_EdgeListNodePtr edgeNodePrev,
   t_EdgeListNodePtr edgeNode
   ) {
   edgeNode->m_prev = edgeNodePrev;
}

static void SetEdgeListNodeEdge(
   t_EdgePtr edge,
   t_EdgeListNodePtr edgeNode
   ) {
   edgeNode->m_edge = edge;
}

static t_EdgeListPtr CreateEdgeList(
   void
   ) {
   t_EdgeListPtr edgeList = Malloc( sizeof( struct t_EdgeList ) );
   edgeList->m_length = 0;
   edgeList->m_edgeNodes = NULL;
   return edgeList;
}

static int GetEdgeListLength(
   t_EdgeListPtr edgeList
   ) {
   return edgeList->m_length;
}

static void SetEdgeListLength(
   int length,
   t_EdgeListPtr edgeList
   ) {
   edgeList->m_length = length;
}

static void IncreaseEdgeListLength(
   t_EdgeListPtr edgeList
   ) {
   int length = GetEdgeListLength( edgeList );
   SetEdgeListLength( length + 1, edgeList );
}

static void DecreaseEdgeListLength(
   t_EdgeListPtr edgeList
   ) {
   int length = GetEdgeListLength( edgeList );
   assert( length > 0 );
   SetEdgeListLength( length -1, edgeList );
}

static t_EdgeListNodePtr GetEdgeListNodes(
   t_EdgeListPtr edgeList
   ) {
   return edgeList->m_edgeNodes;
}

static void SetEdgeListNodes(
   t_EdgeListNodePtr edgeListNodes,
   t_EdgeListPtr edgeList
   ) {
   edgeList->m_edgeNodes = edgeListNodes;
}

static void FreeEdgeListNode(
   t_EdgeListNodePtr edgeListNode
   ) {
   t_EdgePtr edge = GetEdgeListNodeEdge( edgeListNode );
   FreeAEdge( edge );
   Free( edgeListNode );
}

static void FreeEdgeList(
   t_EdgeListPtr edgeList
   ) {
   Free( edgeList );
}

static void FreeEdgeListAndMems(
   t_EdgeListPtr edgeList
   ) {
   int length = GetEdgeListLength( edgeList );
   t_EdgeListNodePtr edgeNodes = GetEdgeListNodes( edgeList );

   for ( int i = 0; i < length; i++ ) {
      t_EdgeListNodePtr edgeNodeNext = GetEdgeListNodeNext( edgeNodes );
      FreeEdgeListNode( edgeNodes );
      edgeNodes = edgeNodeNext;
   }
   FreeEdgeList( edgeList );
}

static t_EdgeListPtr * CreateEdgeListArrays(
   int length
   ) {
   t_EdgeListPtr *edgeListArrays = Malloc( length * sizeof( t_EdgeListPtr ) );

   /* initial the member of edgeListArrays */
   for ( int i = 0; i < length; i++ ) {
      *( edgeListArrays + i ) = CreateEdgeList();
   }
   return edgeListArrays;
}

static t_EdgeTablePtr CreateEdgeTable(
   int height
   ) {
   t_EdgeTablePtr edgeTable = Malloc( sizeof( struct t_EdgeTable ) );
   edgeTable->m_height = height;
   edgeTable->m_edgeArrays = CreateEdgeListArrays( height );
   return edgeTable;
}

static int GetEdgeTableHeight(
   t_EdgeTablePtr edgeTable
   ) {
   return edgeTable->m_height;
}

static t_EdgeListPtr *GetEdgeTableListArray(
   t_EdgeTablePtr edgeTable
   ) {
   return edgeTable->m_edgeArrays;
}

static t_EdgeListPtr GetEdgeTableListByHeight(
   int height,
   t_EdgeTablePtr edgeTable
   ) {
   assert( ( height >= 0 ) && ( height <= GetEdgeTableHeight( edgeTable )) );
   return *( GetEdgeTableListArray( edgeTable ) + height );
}

static void FreeEdgetableAndMems(
   t_EdgeTablePtr edgeTable
   ) {
   t_EdgeListPtr edgeList;
   int height = GetEdgeTableHeight( edgeTable );

   for ( int i = 0; i < height; i++ ) {
      edgeList = GetEdgeTableListByHeight( i, edgeTable );
      FreeEdgeListAndMems( edgeList );
   }
   t_EdgeListPtr *edgeTableListArray = GetEdgeTableListArray( edgeTable );
   Free( edgeTableListArray );
   Free( edgeTable );
}

static void InsertEdgeInEdgeList(
   t_EdgePtr edge,
   t_EdgeListPtr edgeList
   ) {
   t_EdgeListNodePtr newEdgeNode = CreateEdgeListNode( edge );

   if ( GetEdgeListNodes( edgeList ) == NULL ) {
      /* the edgeList is empty */
      SetEdgeListNodeNext( NULL, newEdgeNode );
      SetEdgeListNodePrev( NULL, newEdgeNode );
   } else {
      SetEdgeListNodeNext( GetEdgeListNodes( edgeList ), newEdgeNode );
      SetEdgeListNodePrev( NULL, newEdgeNode );
      t_EdgeListNodePtr newEdgeNodeNext = GetEdgeListNodeNext( newEdgeNode );
      SetEdgeListNodePrev( newEdgeNode, newEdgeNodeNext );
   }
   SetEdgeListNodes( newEdgeNode, edgeList );
   IncreaseEdgeListLength( edgeList );
}

static void InsertEdgeInEdgeTable(
   t_EdgePtr edge,
   t_EdgeTablePtr edgeTable
   ) {
   int height = GetEdgeYMin( edge );
   t_EdgeListPtr edgeList = GetEdgeTableListByHeight( height, edgeTable );
   InsertEdgeInEdgeList( edge, edgeList );
}

static t_ActiveEdgeTablePtr CreateActiveEdgeTable(
   t_EdgeTablePtr edgeTable
   ) {
   t_ActiveEdgeTablePtr activeEdgeTable;
   activeEdgeTable = Malloc( sizeof( struct t_ActiveEdgeTable ) );
   activeEdgeTable->m_edgeTable = edgeTable;
   activeEdgeTable->m_activeEdges = CreateEdgeList();
   return activeEdgeTable;
}

static t_EdgeListPtr GetActiveEdgeList(
   t_ActiveEdgeTablePtr activeEdgeTable
   ) {
   return activeEdgeTable->m_activeEdges;
}

static void SetActiveEdgeList(
   t_EdgeListPtr edgeTableList,
   t_ActiveEdgeTablePtr activeEdgeTable
   ) {
   activeEdgeTable->m_activeEdges = edgeTableList;
}

static void FreeActiveEdgeTableAndMems(
   t_ActiveEdgeTablePtr activeEdgeTable
   ) {
   t_EdgeListPtr edgeList = GetActiveEdgeList( activeEdgeTable );
   FreeEdgeListAndMems( edgeList );
   Free( activeEdgeTable );
}

static t_EdgeTablePtr GetActiveEdgeTableAssociateTable(
   t_ActiveEdgeTablePtr activeEdgeTable
   ) {
   return activeEdgeTable->m_edgeTable;
}

/* This routine append the second edge node list to the first one */
static t_EdgeListNodePtr AppendEdgeNodes(
   t_EdgeListNodePtr edgeNode1,
   t_EdgeListNodePtr edgeNode2
   ) {
   t_EdgeListNodePtr edgeNodeFirst = edgeNode1;
   for ( t_EdgeListNodePtr edgeNodeCurrent = edgeNode2;
         edgeNodeCurrent != NULL;
         edgeNodeCurrent= GetEdgeListNodeNext( edgeNodeCurrent ) ) {
      t_EdgeListNodePtr newEdgeNode = CreateEdgeListNodeWithNewEdge( GetEdgeListNodeEdge( edgeNodeCurrent ) );
      SetEdgeListNodeNext( edgeNodeFirst, newEdgeNode);
      SetEdgeListNodePrev( NULL, newEdgeNode );

      if ( edgeNodeFirst != NULL ) {
         SetEdgeListNodePrev( newEdgeNode, edgeNodeFirst );
      }
      edgeNodeFirst = newEdgeNode;
   }
   return edgeNodeFirst;
}

/* This append needs to create the copy and insert to the front of the first list, and make empty the second list */
static t_EdgeListPtr AppendEdgeList(
   t_EdgeListPtr edgeList1,
   t_EdgeListPtr edgeList2
   ) {

   int totalLength = GetEdgeListLength( edgeList1 ) +
      GetEdgeListLength( edgeList2 );
   SetEdgeListLength( totalLength, edgeList1 );
   t_EdgeListNodePtr appendedList = AppendEdgeNodes( GetEdgeListNodes( edgeList1 ),
                                                     GetEdgeListNodes( edgeList2 ) );
   SetEdgeListNodes( appendedList, edgeList1 );

   return edgeList1;
}

static void UpdateNewActiveEdges(
   int yCurrent,
   t_ActiveEdgeTablePtr activeEdgeTable
   ) {
   t_EdgeTablePtr edgeTable = GetActiveEdgeTableAssociateTable( activeEdgeTable );
   t_EdgeListPtr edgeTableList = GetEdgeTableListByHeight( yCurrent, edgeTable );
   t_EdgeListPtr newEdgeList = AppendEdgeList( GetActiveEdgeList( activeEdgeTable ), edgeTableList );
   SetActiveEdgeList( newEdgeList, activeEdgeTable );
}

static int IsEdgeDead(
   t_EdgePtr edge,
   int yCurrent
   ) {
   assert( GetEdgeYMax( edge ) >= yCurrent );
   return GetEdgeYMax( edge ) == yCurrent;
}

/* This routine remove the edgeListNodeCurrent from the edgeList and update the edgeListNodePrevPtr and the edgeListNodeCurrentPtr */
static void RemoveDeadEdge(
   t_EdgeListPtr edgeList,
   t_EdgeListNodePtr *edgeListNodePrevPtr,
   t_EdgeListNodePtr *edgeListNodeCurrentPtr
   ) {
   assert( *edgeListNodeCurrentPtr != NULL );
   DecreaseEdgeListLength( edgeList );

   if ( *edgeListNodePrevPtr == NULL ) {
      /* the dead is the first of the edgelist */
      SetEdgeListNodes( GetEdgeListNodeNext( *edgeListNodeCurrentPtr ),
                        edgeList);
      t_EdgeListNodePtr edgeListNodeNext = GetEdgeListNodeNext( * edgeListNodeCurrentPtr );
      if ( edgeListNodeNext != NULL ) {
         SetEdgeListNodePrev( NULL, edgeListNodeNext );
      }
      FreeEdgeListNode( *edgeListNodeCurrentPtr );
      *edgeListNodeCurrentPtr = GetEdgeListNodes( edgeList );
   } else {
      SetEdgeListNodeNext( GetEdgeListNodeNext( *edgeListNodeCurrentPtr ),
                           *edgeListNodePrevPtr );
      t_EdgeListNodePtr edgeListNodeNext = GetEdgeListNodeNext( * edgeListNodeCurrentPtr );
      if ( edgeListNodeNext != NULL ) {
         SetEdgeListNodePrev( *edgeListNodePrevPtr, edgeListNodeNext );
      }
      FreeEdgeListNode( *edgeListNodeCurrentPtr );
      *edgeListNodeCurrentPtr = GetEdgeListNodeNext( *edgeListNodePrevPtr );
   }
}

static void RemoveDeadActiveEdges(
   int yCurrent,
   t_ActiveEdgeTablePtr activeEdgeTable
   ) {
   t_EdgeListPtr edgeList = GetActiveEdgeList( activeEdgeTable );
   t_EdgeListNodePtr edgeListNodeCurrent = GetEdgeListNodes( edgeList );

   t_EdgeListNodePtr edgeListNodePrev = NULL;
   while ( edgeListNodeCurrent != NULL ) {
      t_EdgePtr edgeCurrent = GetEdgeListNodeEdge( edgeListNodeCurrent );
      if ( IsEdgeDead( edgeCurrent, yCurrent ) ) {
         RemoveDeadEdge( edgeList, &edgeListNodePrev, &edgeListNodeCurrent );
      } else {
         edgeListNodePrev = edgeListNodeCurrent;
         edgeListNodeCurrent = GetEdgeListNodeNext( edgeListNodeCurrent );
      }
   }
}

static void UpdateAllXOfActiveEdge(
   t_ActiveEdgeTablePtr activeEdgeTable
   ) {
   t_EdgeListPtr edgeList = GetActiveEdgeList( activeEdgeTable );
   for ( t_EdgeListNodePtr edgeListNode = GetEdgeListNodes( edgeList ); edgeListNode != NULL; edgeListNode = GetEdgeListNodeNext( edgeListNode ) ) {
      UpdateEdgeX( GetEdgeListNodeEdge( edgeListNode ) );
   }
}

/* This routine copy the length of edgelist from edgelistnode to the xarray */
static void CopyXEdgeListNode(
   t_EdgeListNodePtr edgeListNode,
   int length,
   int *xArray
   ) {
   t_EdgeListNodePtr edgeListNodeCurrent = edgeListNode;
   for ( int i = 0; i < length; i++ ) {
      t_EdgePtr edgeCurrent = GetEdgeListNodeEdge( edgeListNodeCurrent );
      *( xArray + i ) = ( int ) GetEdgeX( edgeCurrent );
      edgeListNodeCurrent = GetEdgeListNodeNext( edgeListNodeCurrent );
   }
}

/* This routine get all the x of the edge list in active edge */
static int * GetXOfActiveEdge(
   t_ActiveEdgeTablePtr activeEdgeTable,
   int *size
   ) {
   t_EdgeListPtr edgeList = GetActiveEdgeList( activeEdgeTable );
   int length = GetEdgeListLength( edgeList );
   *size = length;

   if ( length == 0 ) {
      return NULL;
   }

   int *xArray = Malloc( length * sizeof( int ) );
   CopyXEdgeListNode( GetEdgeListNodes(edgeList), length,  xArray );
   return xArray;
}

static void FreeXOfActiveEdge(
   int *xList
   ) {
   Free( xList );
}

static int IntCompare(
   const void *a,
   const void *b
   ) {
   return *( int * )a > *( int * )b;
}

static void SortIntXArray(
   int length,
   int *xArray
   ) {
   assert( ( length > 0 ) && ( xArray != NULL ) );
   qsort( xArray, length, sizeof( int ), IntCompare );
}

typedef struct t_XPair
{
   int m_xMin;
   int m_xMax;
} *t_XPairPtr;

static int GetXPairXMin(
   t_XPairPtr xPair
   ) {
   return xPair->m_xMin;
}

static int GetXPairXMax(
   t_XPairPtr xPair
   ) {
   return xPair->m_xMax;
}

static void SetXPairXMin(
   int xMin,
   t_XPairPtr xPair
   ) {
   xPair->m_xMin = xMin;
}

static void SetXPairXMax(
   int xMax,
   t_XPairPtr xPair
   ) {
   xPair->m_xMax = xMax;
}

/* This routine store sorted x to the xPairs and append the pair of the first x and the last one to the last */
static void StoreXPairs(
   int arrayLength,
   int *xSorted,
   t_XPairPtr xPairs
   ) {
   int firstPoint = *xSorted;
   int lastShift = arrayLength - 1;

   for ( int i = 0; i < lastShift; i++ ) {
      t_XPairPtr xPairCurrent = xPairs + i;
      SetXPairXMin( *( xSorted + i ), xPairCurrent );
      SetXPairXMax( *( xSorted + i + 1 ), xPairCurrent );
   }

   t_XPairPtr xPairLast = xPairs + lastShift;
   SetXPairXMin( firstPoint, xPairLast );
   SetXPairXMax( *( xSorted + lastShift ), xPairLast );
}

/* This routine set every obstacle between the xMin and xMax with the yCurrent in the environment */
static void ScanLineSetObstacle(
   int xMin,
   int xMax,
   int yCurrent,
   t_EnvironmentPtr environment
   ) {
   for ( int i = xMin; i < xMax; i++ ) {
      if ( IsEnvPointInEnv( i, yCurrent, environment ) ) {
         SetEnvMemberObstacle( GetEnvMember( i, yCurrent, environment ) );
      }
   }
}

/* Fill the obstacle in the environment with the height of yCurrent */
static void FillHeightWithScanLine(
   int yCurrent,
   int arrayLength,
   int *xArray,
   t_EnvironmentPtr environment
   ) {
   if ( arrayLength < 2) {
      return;
   }

   SortIntXArray( arrayLength, xArray );
   struct t_XPair xPairs[ arrayLength ];
   StoreXPairs( arrayLength, xArray, xPairs );

   /* set the obstacle between every pairs of the x of current height */
   for ( int i = 0; i < arrayLength; i++ ) {
      t_XPairPtr xPairCurrent = xPairs + i;
      ScanLineSetObstacle( GetXPairXMin( xPairCurrent ), GetXPairXMax( xPairCurrent ), yCurrent, environment );
   }
}

/* This routine judge whether the edge is valid ( the edge is not horizontal and at least one of the height is in the environment ) */
static int IsValidEdge(
   int y1,
   int y2,
   int height
   ) {
   if ( ( y1 == y2 ) ||
        ( y1 < 0 && y2 < 0 ) ||
        ( y1 >= height && y2 >= height ) ) {
      return 0;
   } else {
      return 1;
   }
}


static t_EdgeTablePtr GetAllEnvEdges(
   int height,
   t_AdaptPointPtr adaptPoints /* assume the number of identical adaptpoints should be more than 2 */
   ) {
   t_EdgeTablePtr edgeTable = CreateEdgeTable( height );

   /* Insert all valid edge into edge table */
   t_AdaptPointPtr prevPoint = adaptPoints;
   for ( t_AdaptPointPtr currentPoint = GetNextAdaptPoint( prevPoint );
         currentPoint != NULL;
         currentPoint = GetNextAdaptPoint( currentPoint ) ) {
      if ( IsValidEdge( GetAdaptPointY( prevPoint ),
                        GetAdaptPointY( currentPoint ),
                        height ) ) {
         t_EdgePtr edge = CreateAEdge( GetAdaptPointX( prevPoint ),
                                       GetAdaptPointY( prevPoint ),
                                       GetAdaptPointX( currentPoint ),
                                       GetAdaptPointY( currentPoint ) );
         InsertEdgeInEdgeTable( edge, edgeTable );
      } /* if -- at the end of valid edge */
      prevPoint = currentPoint;
   } /* for */

   return edgeTable;
}

/* This routine set every obstacle in the environment with the scanline algorithm */
static void ScanLineSetAllEnvWithAET(
   int height,
   t_EnvironmentPtr environment,
   t_ActiveEdgeTablePtr activeEdgeTable
   ) {
   for ( int i = 0; i < height; i++ ) {
      UpdateNewActiveEdges( i, activeEdgeTable );
      RemoveDeadActiveEdges( i, activeEdgeTable );
      int size;
      int *xList = GetXOfActiveEdge( activeEdgeTable, &size );
      FillHeightWithScanLine( i, size, xList, environment );
      FreeXOfActiveEdge( xList );
      UpdateAllXOfActiveEdge( activeEdgeTable );
   }
}

/* core algorithm for set environment */
static void SetSingleObstacleInEnvironment(
   t_SingleObstaclePtr singleObstacle,
   t_EnvironmentPtr environment
   ) {
   assert( singleObstacle != NULL );

   t_AdaptPointPtr adaptPoints = GetAdaptPoints( singleObstacle, environment );
   int height = GetEnvWidth( environment );
   t_EdgeTablePtr edgeTable = GetAllEnvEdges( height, adaptPoints );
   FreeAdaptPoints( adaptPoints );
   t_ActiveEdgeTablePtr activeEdgeTable = CreateActiveEdgeTable( edgeTable );
   ScanLineSetAllEnvWithAET( height, environment, activeEdgeTable );

   FreeActiveEdgeTableAndMems( activeEdgeTable );
   FreeEdgetableAndMems( edgeTable );
}

void SetObstaclesInEnvironment(
   t_ObstaclesPtr obstacles,
   t_EnvironmentPtr newEnvironment
   ) {
   if ( obstacles == NULL ) {
      return;
   }

   int obstacleCounts = GetObstaclesCounts( obstacles );
   t_SingleObstaclePtr singleObstacle = GetObstacleArray( obstacles );
   for ( int i = 0; i < obstacleCounts; i++ ) {
      SetSingleObstacleInEnvironment( GetSingleObstacle( singleObstacle, i ), newEnvironment );
   }

   DebugCode (
      PrintEnvironment( newEnvironment );
      );

   return;
}


