#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <check.h>
#include "heapqueue.h"
#include "publicfun.h"
#include "pretreatment.h"
#include "heapqueue.c"


static int MemberCompare(t_EnvironmentMemberPtr member1, t_EnvironmentMemberPtr member2)
{
  int cost1, cost2;

  cost1 = GetEnvMemberCost(member1);
  cost2 = GetEnvMemberCost(member2);
  return cost1 > cost2;
}

static int NormalCompare(const void *a, const void *b)
{
  return *(int *)a > *(int *)b;
}

static char* test_GetFatherTemplate(int child, int father)
{
  int calFather = GetFather(child);
  ck_assert_int_eq(calFather, father);
}

START_TEST(test_GetFather)
{
  int father1 = 0;
  int leftChild1 = 1, rightChild1 = 2;
  test_GetFatherTemplate(leftChild1, father1);
  test_GetFatherTemplate(rightChild1, father1);

  int father2 = 3;
  int leftChild2 = 7, rightChild2 = 8;
  test_GetFatherTemplate(leftChild2, father2);
  test_GetFatherTemplate(rightChild2, father2);
}
END_TEST

static char* test_GetLeftChildTemplate(int child, int father)
{
  int calLeftChild = GetLeftChild(father);
  ck_assert_int_eq(calLeftChild, child);
}

START_TEST(test_GetLeftChild)
{
  int father1 = 0;
  int leftChild1 = 1;
  test_GetLeftChildTemplate(leftChild1, father1);

  int father2 = 3;
  int leftChild2 = 7;
  test_GetLeftChildTemplate(leftChild2, father2);

}
END_TEST

START_TEST(test_IsQueueFull)
{
  t_PriorityQueuePtr queue;
  t_EnvironmentMemberPtr memIn, memOut;

  queue = InitializeWithSize(1);
  memIn = CreateEnvMemberWithCost(0);
  queue = InsertPriorityQueue(MemberCompare, memIn, queue);
  ck_assert(IsQueueFull(queue));
  memOut = DeleteMinPriorityQueue(MemberCompare, queue);
  ck_assert_ptr_eq(memIn, memOut);
  FreeEnvMember(memIn);
  DestroyPriorityQueue(queue);
}
END_TEST

/* test case template for initial to be empty, the order out of the queue is right, and the end queue is empty is true */
static char* test_PriorityQueueTemplate(int *test, unsigned long length, int *sorted)
{
  t_PriorityQueuePtr queue;
  t_EnvironmentMemberPtr mem[length];
  unsigned long i;

  memcpy(sorted, test, length * sizeof(int));
  qsort(sorted, length, sizeof(int), NormalCompare);
  queue = InitializePriorityQueue();
  ck_assert(IsQueueEmpty(queue));
  for (i=0; i < length; i++) {
    mem[i] = CreateEnvMemberWithCost(test[i]);
  }
  for (i = 0; i < length; i++) {
    queue = InsertPriorityQueue(MemberCompare, mem[i], queue);
  }
  int cost;
  for (i = 0; i < length; i++) {
    cost = GetEnvMemberCost(DeleteMinPriorityQueue(MemberCompare, queue));
    ck_assert_int_eq(sorted[i], cost);
  }
  ck_assert(IsQueueEmpty(queue));
  for (i = 0; i < length; i++) {
    FreeEnvMember(mem[i]);
  }
  DestroyPriorityQueue(queue);
}

START_TEST(test_PriorityQueue)
{
  int test1[8] = {1, 3, 5, 7, 9, 35, 78, 89};
  int sorted1[8];
  test_PriorityQueueTemplate(test1,  sizeof(test1)/sizeof(int), sorted1);

  int test2[8] = {89, 78, 35, 9, 7, 5, 3, 1};
  int sorted2[8];
  test_PriorityQueueTemplate(test2, sizeof(test2)/sizeof(int), sorted2);

  int test3[8] = {35, 9, 5, 1, 7, 3, 89, 78};
  int sorted3[8];
  test_PriorityQueueTemplate(test3, sizeof(test3)/sizeof(int), sorted3);

  int test4[8] = {1, 1, 2, 3, 4, 2, 5, 6};
  int sorted4[8];
  test_PriorityQueueTemplate(test4, sizeof(test4)/sizeof(int), sorted4);

  int test5[8] = {-1, -89,  0, -78, -35, -12, -9, -5};
  int sorted5[8];
  test_PriorityQueueTemplate(test5, sizeof(test5)/sizeof(int), sorted5);
}
END_TEST

Suite *heapqueue_suite(void)
{
  Suite *s;
  TCase *tc_core;

  s = suite_create("heapqueue");
  tc_core = tcase_create("Core");

  tcase_add_test(tc_core, test_PriorityQueue);
  tcase_add_test(tc_core, test_GetFather);
  tcase_add_test(tc_core, test_GetLeftChild);
  tcase_add_test(tc_core, test_IsQueueFull);
  suite_add_tcase(s, tc_core);

  return s;
}

int main(int argc, char *argv[])
{
  int number_failed;
  Suite *s;
  SRunner *sr;

  s = heapqueue_suite();
  sr = srunner_create(s);

  srunner_run_all(sr, CK_NORMAL);
  number_failed = srunner_ntests_failed(sr);
  srunner_free(sr);
  return (number_failed == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
  return 0;
}
