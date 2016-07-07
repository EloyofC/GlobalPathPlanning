#include <stdlib.h>
#include <stdio.h>
#include "miniunit.h"
#include "fifoqueue.h"
#include "publicfun.h"
#include "pretreatment.h"
#include "fifoqueue.c"

int tests_run = 0;

static char* test_IsQueueFull()
{
  t_FifoQueuePtr queue;
  t_EnvironmentMemberPtr memIn, memOut;

  queue = InitialFifoQueueWithSize(1);
  memIn = CreateEnvMemberWithCost(0);
  queue = EnFifoQueue(memIn, queue);
  mu_assert("error, fifoqueue should be full", IsQueueFull(queue));
  memOut = DeFifoQueue(queue);
  mu_assert("error, fifoqueue change the in member", memIn == memOut);
  FreeEnvMember(memIn);
  FreeFifoQueue(queue);
  return 0;
}

static char* test_FifoQueueTemplate(int *test, unsigned long length)
{
  t_FifoQueuePtr queue;
  t_EnvironmentMemberPtr mem[length];
  unsigned long i;

  queue = InitialFifoQueue();
  mu_assert("error, the initial queue is not empty", IsQueueEmpty(queue));
  for (i = 0; i < length; i++) {
    mem[i] = CreateEnvMemberWithCost(test[i]);
  }
  for (i = 0; i < length; i++) {
    queue = EnFifoQueue(mem[i], queue);
  }
  int memOut;
  for (i = 0; i < length; i++) {
    memOut = DeFifoQueue(queue);
    mu_assert("error, fifoqueue is not the same as before", memOut == mem[i]);
  }
  mu_assert("error, the fifoqueue after deleting all is not empty", IsQueueEmpty(queue));
  for (i = 0; i < length; i++) {
    FreeEnvMember(mem[i]);
  }
  FreeFifoQueue(queue);
  return 0;
}

static char* test_FifoQueue()
{
  int test1[8] = {1, 2, 3, 4, 5, 6, 7, 8};
  test_FifoQueueTemplate(test1, sizeof(test1)/sizeof(int));

  int test2[8] = {8, 7, 6, 5, 4, 3, 2, 1};
  test_FifoQueueTemplate(test2, sizeof(test2)/sizeof(int));

  int test3[8] = {3, 3, 8, 0, 9, 8, 0, 4};
  test_FifoQueueTemplate(test3, sizeof(test3)/sizeof(int));

  int test4[8] = {-4, -9, -4, 0, 99, 3, 8, -192};
  test_FifoQueueTemplate(test4, sizeof(test4)/sizeof(int));
  return 0;
}

static char* all_tests()
{
  mu_run_test(test_IsQueueFull);
  mu_run_test(test_FifoQueue);
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
