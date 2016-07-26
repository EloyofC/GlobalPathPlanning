#include <stdlib.h>
#include <stdio.h>
#include <check.h>
#include "fifoqueue.h"
#include "publicfun.h"
#include "envoperate.h"
#include "fifoqueue.c"


START_TEST( test_IsQueueFull )
{
   t_FifoQueuePtr queue;
   t_FifoElementTypePtr memIn, memOut;

   queue = CreateFifoQueueWithSize(1);
   memIn = CreateEnvMemberWithCost(0);
   queue = EnFifoQueue( memIn, queue );
   ck_assert( IsQueueFull( queue ) );
   memOut = DeFifoQueue( queue );
   ck_assert_ptr_eq( memIn, memOut );
   FreeEnvMember( memIn );
   FreeFifoQueue( queue );
}
END_TEST

static void test_FifoQueueOneTurnInOut(
   int *test,
   unsigned long length,
   t_FifoQueuePtr queue
   ) {
   t_FifoElementTypePtr mem[ length ];
   unsigned long i;

   for ( i = 0; i < length; i++ ) {
      mem[ i ] = CreateEnvMemberWithCost( test[ i ] );
   }
   for ( i = 0; i < length; i++ ) {
      queue = EnFifoQueue( mem[ i ], queue );
   }
   t_FifoElementTypePtr memOut;
   for ( i = 0; i < length; i++ ) {
      memOut = DeFifoQueue( queue );
      ck_assert_msg( memOut == mem[ i ], "The number before %d - %d - after %d is not right\n", test[ i - 1 ], test[ i ], test[ i - 1 ] );
   }

   for ( i = 0; i < length; i++ ) {
      FreeEnvMember( mem[ i ] );
   }
}

static void test_FifoQueueAllInAllOutTemplate(
   int *test,
   unsigned long length
   ) {
   t_FifoQueuePtr queue = CreateFifoQueue();

   ck_assert( IsQueueEmpty( queue ) );
   test_FifoQueueOneTurnInOut(test, length, queue);
   ck_assert( IsQueueEmpty( queue ) );

   FreeFifoQueue( queue );
}

START_TEST( test_FifoQueueAllInAllOut )
{
   int test1[ 8 ] = { 1, 2, 3, 4, 5, 6, 7, 8 };
   test_FifoQueueAllInAllOutTemplate( test1, sizeof( test1 )/sizeof( int ) );

   int test2[ 8 ] = { 8, 7, 6, 5, 4, 3, 2, 1 };
   test_FifoQueueAllInAllOutTemplate( test2, sizeof( test2 )/sizeof( int ) );

   int test3[ 8 ] = { 3, 3, 8, 0, 9, 8, 0, 4 };
   test_FifoQueueAllInAllOutTemplate( test3, sizeof( test3 )/sizeof( int ) );

   int test4[ 8 ] = { -4, -9, -4, 0, 99, 3, 8, -192 };
   test_FifoQueueAllInAllOutTemplate( test4, sizeof( test4 )/sizeof( int ) );

}
END_TEST

static void test_FifoQueueSizeInSizeOutTemplate(
   int *test,
   unsigned long length,
   int oneTimeSize
   ) {
   t_FifoQueuePtr queue = CreateFifoQueue();

   ck_assert( IsQueueEmpty( queue ) );
   int count = length / oneTimeSize;
   for ( int i = 0; i < count; i++ ) {
      int index = i * oneTimeSize;
      test_FifoQueueOneTurnInOut( test + index, oneTimeSize, queue );
   }
   int index = count * oneTimeSize;
   int rest = length - index;
   test_FifoQueueOneTurnInOut( test + index, rest, queue );
   ck_assert( IsQueueEmpty( queue ) );

   FreeFifoQueue( queue );
}

START_TEST( test_FifoQueueSizeInSizeOut )
{
   int test1[ 8 ] = { 1, 2, 3, 4, 5, 6, 7, 8 };
   test_FifoQueueSizeInSizeOutTemplate( test1, sizeof( test1 )/sizeof( int ), 2 );

   int test2[ 8 ] = { 8, 7, 6, 5, 4, 3, 2, 1 };
   test_FifoQueueSizeInSizeOutTemplate( test2, sizeof( test2 )/sizeof( int ), 2 );

   int test3[ 8 ] = { 3, 3, 8, 0, 9, 8, 0, 4 };
   test_FifoQueueSizeInSizeOutTemplate( test3, sizeof( test3 )/sizeof( int ), 2 );

   int test4[ 8 ] = { -4, -9, -4, 0, 99, 3, 8, -192 };
   test_FifoQueueSizeInSizeOutTemplate( test4, sizeof( test4 )/sizeof( int ), 2 );
}
END_TEST

static void test_FifoQueueIncreaseSizeInOutTemplate(
   int *test,
   unsigned long length
   ) {
   t_FifoQueuePtr queue = CreateFifoQueue();

   ck_assert( IsQueueEmpty( queue ) );
   int index = 0;
   for ( int i = 1; index + i < (int)length; i++ ) {
      test_FifoQueueOneTurnInOut( test + index, i, queue );
      index += i;
   }
   int rest = length - index;
   test_FifoQueueOneTurnInOut( test + index, rest, queue );
   ck_assert( IsQueueEmpty( queue ) );

   FreeFifoQueue( queue );
}

START_TEST( test_FifoQueueIncreaseSizeInOut )
{
   int test1[ 8 ] = { 1, 2, 3, 4, 5, 6, 7, 8 };
   test_FifoQueueIncreaseSizeInOutTemplate( test1, sizeof( test1 )/sizeof( int ) );

   int test2[ 8 ] = { 8, 7, 6, 5, 4, 3, 2, 1 };
   test_FifoQueueIncreaseSizeInOutTemplate( test2, sizeof( test2 )/sizeof( int ) );

   int test3[ 8 ] = { 3, 3, 8, 0, 9, 8, 0, 4 };
   test_FifoQueueIncreaseSizeInOutTemplate( test3, sizeof( test3 )/sizeof( int ) );

   int test4[ 8 ] = { -4, -9, -4, 0, 99, 3, 8, -192 };
   test_FifoQueueIncreaseSizeInOutTemplate( test4, sizeof( test4 )/sizeof( int ) );
}
END_TEST

static Suite *fifoqueue_suite(
   void
   ) {
   Suite *s;
   TCase *tc_core;

   s = suite_create( "fifoqueue" );
   tc_core = tcase_create( "Core" );

   tcase_add_test( tc_core, test_FifoQueueAllInAllOut );
   tcase_add_test( tc_core, test_FifoQueueSizeInSizeOut );
   tcase_add_test( tc_core, test_FifoQueueIncreaseSizeInOut );
   tcase_add_test( tc_core, test_IsQueueFull );
   suite_add_tcase( s, tc_core );

   return s;
}


int main(
   int argc,
   char **argv
   ) {
   int number_failed;
   Suite *s;
   SRunner *sr;

   s = fifoqueue_suite();
   sr = srunner_create( s );

   srunner_run_all( sr, CK_NORMAL );
   number_failed = srunner_ntests_failed( sr );
   srunner_free( sr );
   return ( number_failed == 0 ) ? EXIT_SUCCESS : EXIT_FAILURE;
}
