CC = gcc -g -Wall -Wextra -Wstrict-prototypes -Wmissing-prototypes -fPIC
DEBUG ?= 0
ifeq ($(DEBUG), 1)
	CFLAGS = -O1 -DDEBUG
else
	CFLAGS = -O2 -DNDEBUG
endif

objects = advancedplan.o localplanning.o pretreatment.o heaparray.o publicfun.o mission.o
testsample = test.c
all : libmission.so test
test : $(testsample) PathAndShow.h
	$(CC) -o $@ $(CFLAGS) test.c -lmission -lm

libmission.so : $(objects)
	gcc -shared -o $@ $^
mission.o : GetChangeEnv.h ComPlan.h PublicFun.h PathAndShow.h localplanning.o publicfun.o pretreatment.o
advancedplan.o : GetChangeEnv.h ComPlan.h PublicFun.h localplanning.o heaparray.o publicfun.o
localplanning.o : GetChangeEnv.h ComPlan.h HeapQueue.h heaparray.o publicfun.o pretreatment.o
pretreatment.o : GetChangeEnv.h MapProcess.h PublicFun.h publicfun.o
heaparray.o : HeapQueue.h PublicFun.h publicfun.o
publicfun.o : PublicFun.h


.PHONY : clean
clean :
	rm test $(objects) libmission.a

.PHONY : run
run :
	./test
