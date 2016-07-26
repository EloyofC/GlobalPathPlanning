#include <stdio.h>
#include <stdlib.h>
#include <check.h>
#include "pretreatment.h"
#include "pretreatment.c"
#include "publicfun.h"

static void test_CalXWithYPositiveMinTemplate(
   int x1,
   int y1,
   int x2,
   int y2,
   double predict
   ) {
   double cal = CalXWithYPositiveMin( x1, y1, x2, y2 );
   /* printf( "test_calxminypositive of x1 %d y1 %d x2 %d y2 %d cal : %f predict : %f\n", x1, y1, x2, y2, cal, predict ); */
   /* fflush(stdout); */
   ck_assert( IsDoubleEqual( cal, predict) );
}

START_TEST( test_CalXWithYPositiveMin )
{
   test_CalXWithYPositiveMinTemplate( -1, -3, 2, 3, 0.5 );
   test_CalXWithYPositiveMinTemplate( 1, 2, 3, 4, 1.0 );
   test_CalXWithYPositiveMinTemplate( 3, 4, 1, 2, 1.0 );
   test_CalXWithYPositiveMinTemplate( -1, -2, -3, -4, 1.0 );
   test_CalXWithYPositiveMinTemplate( 2, 3, -1, -3, 0.5 );
}
END_TEST

static void test_CalYMaxTemplate(
   int y1,
   int y2,
   int predict
   ) {
   ck_assert_int_eq( CalYMax( y1, y2 ), predict );
}

START_TEST( test_CalYMax )
{
   test_CalYMaxTemplate( 1, 3, 3 );
   test_CalYMaxTemplate( 3, 1, 3 );
   test_CalYMaxTemplate( 0, 0, 0 );
   test_CalYMaxTemplate( -2, 3, 3 );
   test_CalYMaxTemplate( 3, -2, 3 );
}
END_TEST

static void test_CalYMinPositiveTemplate(
   int y1,
   int y2,
   int predict
   ) {
   ck_assert_int_eq( CalYMinPositive( y1, y2 ), predict );
}

START_TEST( test_CalYMinPositive )
{
   test_CalYMinPositiveTemplate( 1, 3, 1 );
   test_CalYMinPositiveTemplate( -2, 9, 0 );
   test_CalYMinPositiveTemplate( 2, -9, 0 );
   test_CalYMinPositiveTemplate( 2, 1, 1 );
}
END_TEST

static void test_CalRevRatioTemplate(
   int x1,
   int y1,
   int x2,
   int y2,
   double predict
   ) {
   ck_assert( IsDoubleEqual ( CalRevRatio( x1, y1, x2, y2 ), predict ) );
}

START_TEST( test_CalRevRatio )
{
   test_CalRevRatioTemplate( 16, 6, 8, 1, 1.6 );
   test_CalRevRatioTemplate( 8, 1, 16, 6, 1.6 );
}
END_TEST

static void test_IsValidEdgeTemplate(
   int y1,
   int y2,
   int height,
   int isFalse
   ) {
   int isValid = IsValidEdge( y1, y2, height );
   /* printf( "test_isvalidedge y1 %d y2 %d height %d cal isvalid %d should be false %d\n", y1, y2, height, isValid, isFalse ); */
   /* fflush(stdout); */
   ck_assert( isValid == !isFalse );
}

START_TEST( test_IsValidEdge )
{
   test_IsValidEdgeTemplate( 1, 2, 5, 0 );
   test_IsValidEdgeTemplate( 1, 1, 5, 1 );
   test_IsValidEdgeTemplate( -1, 0, 5, 0 );
   test_IsValidEdgeTemplate( -2, -4, 5, 1 );
   test_IsValidEdgeTemplate( 6, 3, 5, 0 );
   test_IsValidEdgeTemplate( 8, 19, 5, 1 );
}
END_TEST

static void test_SortIntXArrayTemplate(
   int length,
   int *xArray,
   int *xSorted
   ) {
   SortIntXArray( length, xArray );

   for ( int i = 0; i < length; i++ ) {
      ck_assert_int_eq( *xArray, *xSorted );
   }
}

START_TEST( test_SortIntXArray )
{
   int xArray1[ 5 ] = { 1, 3, 5, 7, 9 };
   int xSorted1[ 5 ] = { 1, 3, 5, 7, 9 };
   test_SortIntXArrayTemplate( sizeof( xArray1 )/sizeof( int ), xArray1, xSorted1 );

   int xArray2[ 5 ] = { 9, 7, 5, 3, 1 };
   int xSorted2[ 5 ] = { 1, 3, 5, 7, 9 };
   test_SortIntXArrayTemplate( sizeof( xArray2 )/sizeof( int ), xArray2, xSorted2 );

   int xArray3[ 5 ] = { 5, 1, 9, 3, 7 };
   int xSorted3[ 5 ] = { 1, 3, 5, 7, 9 };
   test_SortIntXArrayTemplate( sizeof( xArray3 )/sizeof( int ), xArray3, xSorted3 );

   int xArray4[ 5 ] = { 1, 3, 5, 3, 1 };
   int xSorted4[ 5 ] = { 1, 1, 3, 3, 5 };
   test_SortIntXArrayTemplate( sizeof( xArray4 )/sizeof( int ), xArray4, xSorted4 );

   int xArray5[ 5 ] = { -1, -3, 3, -1, 9 };
   int xSorted5[ 5 ] = { -3, -1, -1, 3, 9 };
     test_SortIntXArrayTemplate( sizeof( xArray5 )/sizeof( int ), xArray5, xSorted5 );
}
END_TEST

static void test_StoreXPairsTemplate(
   int arrayLength,
   int *xSorted,
   int *xMin,
   int *xMax
   ) {
   struct t_XPair xPairs[ arrayLength ];
   StoreXPairs(arrayLength, xSorted, xPairs );

   for ( int i = 0; i < arrayLength; i++ ) {
      int xPairsMin = GetXPairXMin( xPairs + i );
      ck_assert_int_eq( xPairsMin, *( xMin + i ) );
      int xPairsMax = GetXPairXMax( xPairs + i );
      ck_assert_int_eq( xPairsMax, *( xMax + i ) );
   }
}

START_TEST( test_StoreXPairs )
{
   int xSorted1[ 5 ] = { 1, 3, 5, 7, 9 };
   int xMin1[ 5 ] = { 1, 3, 5, 7, 1 };
   int xMax1[ 5 ] = { 3, 5, 7, 9, 9 };
   test_StoreXPairsTemplate( sizeof( xSorted1 )/sizeof( int ), xSorted1, xMin1, xMax1 );

   int xSorted2[ 5 ] = { 1, 1, 3, 5, 5 };
   int xMin2[ 5 ] = { 1, 1, 3, 5, 1 };
   int xMax2[ 5 ] = { 1, 3, 5, 5, 5 };
   test_StoreXPairsTemplate( sizeof( xSorted2 )/sizeof( int ), xSorted2, xMin2, xMax2 );
}
END_TEST

static void test_AdaptPointTemplate(
   int arrayLength,
   int *x,
   int *y
   ) {
   t_AdaptPointPtr adaptPoints = NULL;
   for ( int i = 0; i < arrayLength; i++ ) {
      adaptPoints = InsertAdaptPoint( *( x + i ), *( y + i ), adaptPoints );
   }
   int i = arrayLength - 1;
   for ( t_AdaptPointPtr currentPoint = adaptPoints;
         currentPoint != NULL;
         currentPoint = GetNextAdaptPoint( currentPoint ) ) {
      int currentX = GetAdaptPointX( currentPoint );
      ck_assert_int_eq( currentX, *( x + i ) );
      int currentY = GetAdaptPointY( currentPoint );
      ck_assert_int_eq( currentY, *( y + i ) );
      i--;
   }
   FreeAdaptPoints( adaptPoints );
}

START_TEST( test_AdaptPoint )
{
   int x1[ 5 ] = { 1, 2, 3, 4, 5 };
   int y1[ 5 ] = { 6, 7, 8, 9, 1 };
   test_AdaptPointTemplate( sizeof( x1 )/sizeof( int ), x1, y1 );

   int x2[ 5 ] = { 1, 3, 1, 2, 9 };
   int y2[ 5 ] = { 9, 4, 10, 4, 2 };
   test_AdaptPointTemplate( sizeof( x2 )/sizeof( int ), x2, y2 );
}
END_TEST

static void test_CreateAEdgeTemplate(
   int x1,
   int y1,
   int x2,
   int y2,
   int yMax,
   int yMin,
   int x,
   int ratio
   ) {
   t_EdgePtr edge = CreateAEdge( x1, y1, x2, y2 );
   int calYMax = GetEdgeYMax( edge );
   ck_assert_int_eq( calYMax, yMax );
   int calYMin = GetEdgeYMin( edge );
   ck_assert_int_eq( calYMin, yMin );
   int calX = GetEdgeX( edge );
   ck_assert_int_eq( calX, x );
   int calRatio = GetEdgeRevRatio( edge );
   ck_assert_int_eq( calRatio, ratio );
   FreeAEdge( edge );
}

START_TEST( test_CreateAEdge )
{
   test_CreateAEdgeTemplate( 1, 2, 3, 4, 4, 2, 1, 1 );
}
END_TEST

static void test_IsEdgeDeadTemplate(
   int x1,
   int y1,
   int x2,
   int y2,
   int yCurrent,
   int isEdgeDeadFalse
   ) {
   t_EdgePtr edge = CreateAEdge( x1, y1, x2, y2 );
   int calEdgeDead = IsEdgeDead( edge, yCurrent );
   ck_assert_int_eq( calEdgeDead, !isEdgeDeadFalse );
   FreeAEdge( edge );
}

START_TEST( test_IsEdgeDead )
{
   test_IsEdgeDeadTemplate( 1, 4, 9, 20, 20, 0 );
   test_IsEdgeDeadTemplate( 1, 4, 9, 20, 19, 1 );
}
END_TEST

static t_EdgeListPtr CreateEdgeListWithNodes(
   int arrayLength,
   int *x1,
   int *y1,
   int *x2,
   int *y2
   ) {
   t_EdgeListPtr edgeList = CreateEdgeList();
   for ( int i = 0; i < arrayLength; i++ ) {
      t_EdgePtr newEdge = CreateAEdge( *( x1 + i ), *( y1 + i ),
                                 *( x2 + i ), *( y2 + i ));
      InsertEdgeInEdgeList( newEdge, edgeList );
   }
   return edgeList;
}

static void test_CopyXEdgeListNodeTemplate(
   int arrayLength,
   int *x1,
   int *y1,
   int *x2,
   int *y2,
   int *xArray
   ) {
   t_EdgeListPtr edgeList = CreateEdgeListWithNodes( arrayLength, x1, y1, x2, y2 );
   int length = GetEdgeListLength( edgeList );
   ck_assert_int_eq( length, arrayLength );
   t_EdgeListNodePtr edgeListNode = GetEdgeListNodes( edgeList );
   int xCopyed[ length ];
   CopyXEdgeListNode( edgeListNode, length, xCopyed );
   FreeEdgeListAndMems( edgeList );

   for ( int i = 0; i < arrayLength; i++ ) {
      ck_assert_int_eq( *( xArray + i ), *( xCopyed + i ) );
   }
}

START_TEST( test_CopyXEdgeListNode )
{
   int x11[ 5 ] = { 1, 3, 5, 7, 9 };
   int y11[ 5 ] = { 2, 4, 6, 8, 10 };
   int x21[ 5 ] = { 3, 1, 5, 7, 9 };
   int y21[ 5 ] = { 1, 6, 10, 9, 6 };
   int xArray1[ 5 ] = { 9, 7, 5, 3, 3  };
   test_CopyXEdgeListNodeTemplate( sizeof( x11 )/sizeof( int ), x11, y11, x21, y21, xArray1 );
}
END_TEST


static void test_AppendEdgeListTemplate(
   int arrayLength1,
   int *x11,
   int *y11,
   int *x21,
   int *y21,
   int arrayLength2,
   int *x12,
   int *y12,
   int *x22,
   int *y22,
   int *yMax
   ) {
   t_EdgeListPtr edgeList1 = CreateEdgeListWithNodes( arrayLength1, x11, y11, x21, y21 );
   t_EdgeListPtr edgeList2 = CreateEdgeListWithNodes( arrayLength2, x12, y12, x22, y22 );

   t_EdgeListPtr edgeListWhole = CreateEdgeList();
   AppendEdgeList( edgeListWhole, edgeList1 );
   AppendEdgeList( edgeListWhole, edgeList2 );
   int length = GetEdgeListLength( edgeListWhole );
   ck_assert_int_eq( length, arrayLength1 + arrayLength2 );
   t_EdgeListNodePtr edgeListNodes = GetEdgeListNodes( edgeListWhole );
   for ( int i = 0; i < length; i++ ) {
      t_EdgePtr edge = GetEdgeListNodeEdge( edgeListNodes );
      ck_assert_int_eq( *( yMax + i ), GetEdgeYMax( edge ) );
      edgeListNodes = GetEdgeListNodeNext( edgeListNodes );
   }
   FreeEdgeList( edgeListWhole );
   FreeEdgeListAndMems( edgeList1 );
   FreeEdgeListAndMems( edgeList2 );
}

START_TEST( test_AppendEdgeList )
{
   int x11[ 5 ] = { 1, 3, 5, 7, 9 };
   int y11[ 5 ] = { 2, 4, 6, 8, 10 };
   int x21[ 5 ] = { 3, 1, 9, 8, 5 };
   int y21[ 5 ] = { 3, 7, 8, 5, 8 };
   int x12[ 3 ] = { 4, 7, 5 };
   int y12[ 3 ] = { 5, 8, 6 };
   int x22[ 3 ] = { 2, 3, 1 };
   int y22[ 3 ] = { 3, 5, 7 };
   int yMax1[ 8 ] = { 5, 8, 7, 3, 7, 8, 8, 10 }; /* each of the ymax revert twice */
   test_AppendEdgeListTemplate( sizeof( x11 )/sizeof( int ), x11, y11, x21, y21,
                                sizeof( x12 )/sizeof( int ), x12, y12, x22, y22,
                                yMax1 );

   int x13[ 5 ] = { 1, 3, 5, 7, 9 };
   int y13[ 5 ] = { 2, 4, 6, 8, 10 };
   int x23[ 5 ] = { 3, 1, 9, 8, 5 };
   int y23[ 5 ] = { 3, 7, 8, 5, 8 };
   int yMax2[ 5 ] = { 3, 7, 8, 8, 10 };
   test_AppendEdgeListTemplate( sizeof( x13 )/sizeof( int ), x13, y13, x23, y23,
                                0, NULL, NULL, NULL, NULL,
                                yMax2 );

   int x14[ 3 ] = { 4, 7, 5 };
   int y14[ 3 ] = { 5, 8, 6 };
   int x24[ 3 ] = { 2, 3, 1 };
   int y24[ 3 ] = { 3, 5, 7 };
   int yMax3[ 8 ] = { 5, 8, 7 };
   test_AppendEdgeListTemplate( 0, NULL, NULL, NULL, NULL,
                                sizeof( x14 )/sizeof( int ), x14, y14, x24, y24,
                                yMax3 );
}
END_TEST

static void test_RemoveDeadEdgeTemplate(
   int index,                   /* the element count start from 1 */
   int arrayLength,
   int *x1,
   int *y1,
   int *x2,
   int *y2,
   int *yMax
   ) {
   t_EdgeListPtr edgeList = CreateEdgeListWithNodes( arrayLength, x1, y1, x2, y2 );
   t_EdgeListNodePtr edgeListNodePrev = NULL;
   t_EdgeListNodePtr edgeListNodeCurrent = GetEdgeListNodes( edgeList );
   int indexInList = arrayLength - index;
   assert( indexInList >= 0 );
   for ( int i = 0; i < indexInList; i++ ) {
      edgeListNodePrev = edgeListNodeCurrent;
      edgeListNodeCurrent = GetEdgeListNodeNext( edgeListNodeCurrent );
   }

   RemoveDeadEdge( edgeList, &edgeListNodePrev, &edgeListNodeCurrent );

   int length = GetEdgeListLength( edgeList );
   ck_assert_int_eq( length, arrayLength - 1 );
   t_EdgeListNodePtr edgeListNode = GetEdgeListNodes( edgeList );
   for ( int i = length - 1; i >= 0; i-- ) {
      t_EdgePtr edge = GetEdgeListNodeEdge( edgeListNode );
      ck_assert_int_eq( *( yMax + i ), GetEdgeYMax( edge ) );
      edgeListNode = GetEdgeListNodeNext( edgeListNode );
   }
   FreeEdgeListAndMems( edgeList );
}

START_TEST( test_RemoveDeadEdge )
{
   int x11[ 5 ] = { 1, 3, 5, 7, 9 };
   int y11[ 5 ] = { 2, 4, 6, 8, 10 };
   int x21[ 5 ] = { 3, 1, 9, 8, 5 };
   int y21[ 5 ] = { 3, 7, 8, 5, 8 };
   int yMax1[ 4 ] = { 3, 7, 8, 8 };
   test_RemoveDeadEdgeTemplate( 5, sizeof( x11 )/sizeof( int ), x11, y11, x21, y21, yMax1 );

   int x12[ 5 ] = { 1, 3, 5, 7, 9 };
   int y12[ 5 ] = { 2, 4, 6, 8, 10 };
   int x22[ 5 ] = { 3, 1, 9, 8, 5 };
   int y22[ 5 ] = { 3, 7, 8, 5, 8 };
   int yMax2[ 4 ] = { 7, 8, 8, 10 };
   test_RemoveDeadEdgeTemplate( 1, sizeof( x12 )/sizeof( int ), x12, y12, x22, y22, yMax2 );

   int x13[ 5 ] = { 1, 3, 5, 7, 9 };
   int y13[ 5 ] = { 2, 4, 6, 8, 10 };
   int x23[ 5 ] = { 3, 1, 9, 8, 5 };
   int y23[ 5 ] = { 3, 7, 8, 5, 8 };
   int yMax3[ 4 ] = { 3, 7, 8, 10 };
   test_RemoveDeadEdgeTemplate( 4, sizeof( x13 )/sizeof( int ), x13, y13, x23, y23, yMax3 );
}
END_TEST

static t_EdgeTablePtr CreateEdgeTableWithEdgeListNodes(
   int arrayLength,
   int *x1,
   int *y1,
   int *x2,
   int *y2,
   int height
   ) {
   t_EdgeTablePtr edgeTable = CreateEdgeTable( height );
   for ( int i = 0; i < arrayLength; i++ ) {
      t_EdgePtr edge = CreateAEdge( *( x1 + i ), *( y1 + i ),
                                    *( x2 + i ), *( y2 + i ) );
      InsertEdgeInEdgeTable( edge, edgeTable );
   }
   return edgeTable;
}

static void test_EdgeTableTemplate(
   int arrayLength,
   int *x1,
   int *y1,
   int *x2,
   int *y2,
   int height,
   int index,
   int yMaxLength,
   int *yMaxForIndex
   ) {
   t_EdgeTablePtr edgeTable = CreateEdgeTableWithEdgeListNodes( arrayLength, x1, y1, x2, y2, height );
   t_EdgeListPtr edgeList = GetEdgeTableListByHeight( index, edgeTable );
   int length = GetEdgeListLength( edgeList );
   ck_assert_int_eq( length, yMaxLength );
   t_EdgeListNodePtr edgeListNodes = GetEdgeListNodes( edgeList );
   for ( int i = length - 1; i >= 0; i-- ) {
      t_EdgePtr edge = GetEdgeListNodeEdge( edgeListNodes );
      ck_assert_int_eq( *( yMaxForIndex + i ), GetEdgeYMax( edge ) );
      edgeListNodes = GetEdgeListNodeNext( edgeListNodes );
   }
   FreeEdgetableAndMems( edgeTable );
}

START_TEST( test_EdgeTable )
{
   int x11[ 5 ] = { 1, 3, 5, 7, 9 };
   int y11[ 5 ] = { 4, 4, 3, 8, 3 };
   int x21[ 5 ] = { 3, 1, 9, 8, 5 };
   int y21[ 5 ] = { 3, 7, 8, 3, 9 };
   int yMax1[ 4 ] = { 4, 8, 8, 9 };
   test_EdgeTableTemplate( sizeof( x11 )/sizeof( int ), x11, y11, x21, y21, 10, 3, sizeof( yMax1 )/sizeof( int ), yMax1 );

   int x12[ 5 ] = { 1, 3, 5, 7, 9 };
   int y12[ 5 ] = { 4, 4, 3, 8, 3 };
   int x22[ 5 ] = { 3, 1, 9, 8, 5 };
   int y22[ 5 ] = { 3, 7, 8, 3, 9 };
   int yMax2[ 1 ] = { 7 };
   test_EdgeTableTemplate( sizeof( x12 )/sizeof( int ), x12, y12, x22, y22, 10, 4, sizeof( yMax2 )/sizeof( int ), yMax2 );

   int x13[ 5 ] = { 1, 3, 5, 7, 9 };
   int y13[ 5 ] = { 4, 4, 3, 8, 3 };
   int x23[ 5 ] = { 3, 1, 9, 8, 5 };
   int y23[ 5 ] = { 3, 7, 8, 3, 9 };
   test_EdgeTableTemplate( sizeof( x13 )/sizeof( int ), x13, y13, x23, y23, 10, 1, 0, NULL );
}
END_TEST

static t_ActiveEdgeTablePtr CreateActiveEdgeTableWithEdgeListNodes(
   int arrayLength,
   int *x1,
   int *y1,
   int *x2,
   int *y2,
   int height
   ) {
   t_EdgeTablePtr edgeTable =  CreateEdgeTableWithEdgeListNodes( arrayLength, x1, y1, x2, y2, height );
   t_ActiveEdgeTablePtr activeEdgeTable = CreateActiveEdgeTable( edgeTable );
   return activeEdgeTable;
}

static test_UpdateNewActiveEdgesTemplate(
   int arrayLength,
   int *x1,
   int *y1,
   int *x2,
   int *y2,
   int height,
   int yCurrentLength,
   int *yCurrent,
   int xArrayLength,
   int *xArray
   ) {
   t_ActiveEdgeTablePtr activeEdgeTable = CreateActiveEdgeTableWithEdgeListNodes( arrayLength, x1, y1, x2, y2, height );

   for ( int i = 0; i < yCurrentLength; i++ ) {
      UpdateNewActiveEdges( *( yCurrent + i ), activeEdgeTable );
   }

   int size = 0;
   int *xSorted = GetXOfActiveEdge( activeEdgeTable, &size );
   ck_assert_int_eq( size, xArrayLength );
   if ( size > 0 ) {
      SortIntXArray( size, xSorted );
      for ( int i = 0; i < xArrayLength; i++ ) {
         ck_assert_int_eq( *( xArray + i ), *( xSorted + i ) );
      }
      FreeXOfActiveEdge( xSorted );
   }
   t_EdgeTablePtr associatedEdgeTable = GetActiveEdgeTableAssociateTable( activeEdgeTable );
   FreeEdgetableAndMems( associatedEdgeTable );
   FreeActiveEdgeTableAndMems( activeEdgeTable );
}

START_TEST( test_UpdateNewActiveEdges )
{
   int x11[ 5 ] = { 1, 4, 5, 7, 9 };
   int y11[ 5 ] = { 4, 4, 3, 1, 3 };
   int x21[ 5 ] = { 3, 1, 9, 9, 5 };
   int y21[ 5 ] = { 3, 1, 8, 3, 9 };
   int yCurrent1[ 3 ] = { 0, 1, 2 };
   int xArray1[ 2 ] = { 1, 7 };
   test_UpdateNewActiveEdgesTemplate( sizeof( x11 )/sizeof( int ), x11, y11, x21, y21, 10, sizeof( yCurrent1 )/sizeof( int ), yCurrent1, sizeof( xArray1 )/sizeof( int ), xArray1 );

   int x12[ 5 ] = { 1, 4, 5, 7, 9 };
   int y12[ 5 ] = { 4, 4, 3, 1, 3 };
   int x22[ 5 ] = { 3, 1, 9, 9, 5 };
   int y22[ 5 ] = { 3, 1, 8, 3, 9 };
   int yCurrent2[ 4 ] = { 0, 1, 2, 3 };
   int xArray2[ 5 ] = { 1, 3, 5, 7, 9 };
   test_UpdateNewActiveEdgesTemplate( sizeof( x12 )/sizeof( int ), x12, y12, x22, y22, 10, sizeof( yCurrent2 )/sizeof( int ), yCurrent2, sizeof( xArray2 )/sizeof( int ), xArray2 );

   int x13[ 5 ] = { 1, 4, 5, 7, 9 };
   int y13[ 5 ] = { 4, 4, 3, 1, 3 };
   int x23[ 5 ] = { 3, 1, 9, 9, 5 };
   int y23[ 5 ] = { 3, 1, 8, 3, 9 };
   int yCurrent3[ 1 ] = { 0 };
   test_UpdateNewActiveEdgesTemplate( sizeof( x13 )/sizeof( int ), x13, y13, x23, y23, 10, sizeof( yCurrent3 )/sizeof( int ), yCurrent3, 0, NULL);
}
END_TEST

static void test_UpdateAllXOfActiveEdgeTemplate(
   int arrayLength,
   int *x1,
   int *y1,
   int *x2,
   int *y2,
   int height,
   int yCurrent,
   int updatedCount,
   int xArrayLength,
   int *xArray
   ) {
   t_ActiveEdgeTablePtr activeEdgeTable = CreateActiveEdgeTableWithEdgeListNodes( arrayLength, x1, y1, x2, y2, height );

   UpdateNewActiveEdges( yCurrent, activeEdgeTable );

   for ( int i = 0; i < updatedCount; i++ ) {
      UpdateAllXOfActiveEdge( activeEdgeTable );
   }

   int size = 0;
   int *xSorted = GetXOfActiveEdge( activeEdgeTable, &size );
   ck_assert_int_eq( size, xArrayLength );
   if ( size > 0 ) {
      SortIntXArray( size, xSorted );
      for ( int i = 0; i < xArrayLength; i++ ) {
         ck_assert_int_eq( *( xArray + i ), *( xSorted + i ) );
      }
      FreeXOfActiveEdge( xSorted );
   }
   t_EdgeTablePtr associatedEdgeTable = GetActiveEdgeTableAssociateTable( activeEdgeTable );
   FreeEdgetableAndMems( associatedEdgeTable );
   FreeActiveEdgeTableAndMems( activeEdgeTable );
}

START_TEST( test_UpdateAllXOfActiveEdge )
{
   int x11[ 5 ] = { 1, 4, 5, 7, 9 };
   int y11[ 5 ] = { 4, 4, 3, 1, 3 };
   int x21[ 5 ] = { 3, 1, 9, 9, 5 };
   int y21[ 5 ] = { 3, 1, 8, 3, 9 };
   int yCurrent1 = 1;
   int updatedCount1 = 1;
   int xArray1[ 2 ] = { 2, 8 };
   test_UpdateAllXOfActiveEdgeTemplate( sizeof( x11 )/sizeof( int ), x11, y11, x21, y21, 10, yCurrent1, updatedCount1, sizeof( xArray1 )/sizeof( int ), xArray1 );

   int x12[ 5 ] = { 1, 4, 5, 7, 9 };
   int y12[ 5 ] = { 4, 4, 3, 1, 3 };
   int x22[ 5 ] = { 3, 1, 9, 9, 5 };
   int y22[ 5 ] = { 3, 1, 8, 3, 9 };
   int yCurrent2 = 1;
   int updatedCount2 = 2;
   int xArray2[ 2 ] = { 3, 9 };
   test_UpdateAllXOfActiveEdgeTemplate( sizeof( x12 )/sizeof( int ), x12, y12, x22, y22, 10, yCurrent2, updatedCount2, sizeof( xArray2 )/sizeof( int ), xArray2 );

   int x13[ 5 ] = { 1, 4, 5, 7, 9 };
   int y13[ 5 ] = { 4, 4, 3, 1, 3 };
   int x23[ 5 ] = { 3, 1, 9, 9, 5 };
   int y23[ 5 ] = { 3, 1, 8, 3, 9 };
   int yCurrent3 = 0;
   int updatedCount3 = 1;
   test_UpdateAllXOfActiveEdgeTemplate( sizeof( x13 )/sizeof( int ), x13, y13, x23, y23, 10, yCurrent3, updatedCount3, 0, NULL );
}
END_TEST

static void test_RemoveDeadActiveEdgesTemplate(
   int arrayLength,
   int *x1,
   int *y1,
   int *x2,
   int *y2,
   int height,
   int yCurrentLength,
   int *yCurrent,
   int removeDeadLength,
   int *removeDeadArray,
   int xArrayLength,
   int *xArray
   ) {
   t_ActiveEdgeTablePtr activeEdgeTable = CreateActiveEdgeTableWithEdgeListNodes( arrayLength, x1, y1, x2, y2, height );

   for ( int i = 0; i < yCurrentLength; i++ ) {
      UpdateNewActiveEdges( *( yCurrent + i ), activeEdgeTable );
   }

   for ( int i = 0; i < removeDeadLength; i++ ) {
      RemoveDeadActiveEdges( *( removeDeadArray + i ), activeEdgeTable );
   }

   int size = 0;
   int *xSorted = GetXOfActiveEdge( activeEdgeTable, &size );
   ck_assert_int_eq( size, xArrayLength );
   if ( size > 0 ) {
      SortIntXArray( size, xSorted );
      for ( int i = 0; i < xArrayLength; i++ ) {
         ck_assert_int_eq( *( xArray + i ), *( xSorted + i ) );
      }
      FreeXOfActiveEdge( xSorted );
   }
   t_EdgeTablePtr associatedEdgeTable = GetActiveEdgeTableAssociateTable( activeEdgeTable );
   FreeEdgetableAndMems( associatedEdgeTable );
   FreeActiveEdgeTableAndMems( activeEdgeTable );
}

START_TEST( test_RemoveDeadActiveEdges )
{
   int x11[ 5 ] = { 1, 4, 5, 7, 9 };
   int y11[ 5 ] = { 4, 4, 3, 1, 3 };
   int x21[ 5 ] = { 3, 1, 9, 9, 5 };
   int y21[ 5 ] = { 3, 1, 8, 3, 9 };
   int yCurrent1[ 4 ] = { 0, 1, 2, 3 };
   int removeDeadArray1[ 4 ] = { 1, 2, 3, 4 };
   int xArray1[ 2 ] = { 5, 9 };
   test_RemoveDeadActiveEdgesTemplate( sizeof( x11 )/sizeof( int ), x11, y11, x21, y21, 10, sizeof( yCurrent1 )/sizeof( int ), yCurrent1, sizeof( removeDeadArray1 )/sizeof( int ), removeDeadArray1, sizeof( xArray1 )/sizeof( int ), xArray1 );

   int x12[ 5 ] = { 1, 4, 5, 7, 9 };
   int y12[ 5 ] = { 4, 4, 3, 1, 3 };
   int x22[ 5 ] = { 3, 1, 9, 9, 5 };
   int y22[ 5 ] = { 3, 1, 8, 3, 9 };
   int yCurrent2[ 4 ] = { 0, 1, 2, 3 };
   int removeDeadArray2[ 4 ] = { 1, 2 };
   int xArray2[ 5 ] = { 1, 3, 5, 7, 9 };
   test_RemoveDeadActiveEdgesTemplate( sizeof( x12 )/sizeof( int ), x12, y12, x22, y22, 10, sizeof( yCurrent2 )/sizeof( int ), yCurrent2, sizeof( removeDeadArray2)/sizeof( int ), removeDeadArray2, sizeof( xArray2 )/sizeof( int ), xArray2 );
}
END_TEST

static Suite *pretreatment_suite(
   void
   ) {
   Suite *s;
   TCase *tc_core;

   s = suite_create( "pretreatment" );  /*the output name of the suite*/
   tc_core = tcase_create( "Core" );

   tcase_add_test( tc_core, test_CalXWithYPositiveMin );
   tcase_add_test( tc_core, test_CalYMax );
   tcase_add_test( tc_core, test_CalYMinPositive );
   tcase_add_test( tc_core, test_CalRevRatio );

   tcase_add_test( tc_core, test_IsValidEdge );
   tcase_add_test( tc_core, test_SortIntXArray );
   tcase_add_test( tc_core, test_StoreXPairs );

   tcase_add_test( tc_core, test_AdaptPoint );

   tcase_add_test( tc_core, test_CreateAEdge );
   tcase_add_test( tc_core, test_IsEdgeDead );
   tcase_add_test( tc_core, test_CopyXEdgeListNode );
   tcase_add_test( tc_core, test_AppendEdgeList );
   tcase_add_test( tc_core, test_RemoveDeadEdge );

   tcase_add_test( tc_core, test_EdgeTable );
   tcase_add_test( tc_core, test_UpdateNewActiveEdges );
   tcase_add_test( tc_core, test_UpdateAllXOfActiveEdge );
   tcase_add_test( tc_core, test_RemoveDeadActiveEdges );
   suite_add_tcase( s, tc_core );

   return s;
}

int main(
   int argc,
   char *argv[]
   ) {
   int number_failed;
   Suite *s;
   SRunner *sr;

   s = pretreatment_suite();        /*get the name of the suite*/
   sr= srunner_create( s );

   srunner_run_all( sr, CK_NORMAL );
   number_failed= srunner_ntests_failed( sr );
   srunner_free( sr );
   return ( number_failed == 0 ) ? EXIT_SUCCESS : EXIT_FAILURE;
   return 0;
}
