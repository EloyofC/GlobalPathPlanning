CC = gcc -g -Wall -Wextra -Wstrict-prototypes -Wmissing-prototypes
DEBUG ?= 0
ifeq ($(DEBUG), 1)
	CFLAGS = -O1 -DDEBUG
else
	CFLAGS = -O2 -DNDEBUG
endif

objects = advancedplan.o localplanning.o pretreatment.o heaparray.o publicfun.o mission.o
testsample = test.c
all : libmission.a test
test : $(testsample) PathAndShow.h libmission.a
	$(CC) -o $@ $(CFLAGS) test.c -L. -lmission -lm

libmission.a : $(objects)
	ar -rc $@ $^
mission.o : GetChangeEnv.h ComPlan.h PublicFun.h PathAndShow.h localplanning.o publicfun.o pretreatment.o
advancedplan.o : GetChangeEnv.h ComPlan.h PublicFun.h localplanning.o heaparray.o publicfun.o
localplanning.o : GetChangeEnv.h ComPlan.h HeapQueue.h heaparray.o publicfun.o pretreatment.o
pretreatment.o : GetChangeEnv.h PublicFun.h publicfun.o
heaparray.o : HeapQueue.h PublicFun.h publicfun.o
publicfun.o : PublicFun.h


.PHONY : clean
clean :
	rm test $(objects) libmission.a

.PHONY : run
run :
	./test
