#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "miniunit.h"
#include "HeapQueue.h"
#include "PublicFun.h"
#include "GetChangeEnv.h"

int tests_run = 0;

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

static char* test_PriorityQueueTemplate(int *test, unsigned long length, int *sorted) {
  t_PriorityQueuePtr queue;
  t_EnvironmentMemberPtr mem[length];
  unsigned long i;

  memcpy(sorted, test, length * sizeof(int));
  qsort(sorted, length, sizeof(int), NormalCompare);
  queue = InitializePriorityQueue();
  for (i=0; i < length; i++) {
    mem[i] = CreateEnvMemberWithCost(test[i]);
  }
  for (i = 0; i < length; i++) {
    queue = InsertPriorityQueue(MemberCompare, mem[i], queue);
  }
  int cost;
  for (i = 0; i < length; i++) {
    cost = GetEnvMemberCost(DeleteMinPriorityQueue(MemberCompare, queue));
    mu_assert("error, deleteminpriorityqueue is not the min", sorted[i] == cost);
  }
  return 0;
  for (i = 0; i < length; i++) {
    FreeEnvMember(mem[i]);
  }
}

static char* test_PriorityQueue(){
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

static char* all_tests(){
  mu_run_test(test_PriorityQueue);
  return 0;
}

int main(int argc, char **argv) {
  char *result = all_tests();
  if (result != 0) {
    printf("%s\n", result);
  }
  else {
    printf("ALL TESTS PASSED\n");
  }
  printf("Tests run: %d\n", tests_run);

  return result != 0;
}
