CC = gcc -g -Wall -Wextra -Wstrict-prototypes -Wmissing-prototypes
DEBUG ?= 0			#usage: make DEBUG=\d
ifeq ($(DEBUG), 1)
	CFLAGS = -O1 -DDEBUG
else
	CFLAGS = -O2 -DNDEBUG
endif

objects = advancedplan.o localplanning.o pretreatment.o heaparray.o publicfun.o mission.o
testsample = test.c

all : libmission.a test test_heaparray
test : $(testsample) PathAndShow.h libmission.a
	$(CC) -o $@ $(CFLAGS) test.c -L. -lmission -lm

libmission.a : $(objects)
	ar -rc $@ $^

test_heaparray : test_heaparray.o libmission.a
	$(CC) -o $@ $(CFLAGS) test_heaparray.o -L. -lmission -lm

mission.o : GetChangeEnv.h ComPlan.h PublicFun.h PathAndShow.h localplanning.o publicfun.o pretreatment.o
advancedplan.o : GetChangeEnv.h ComPlan.h PublicFun.h localplanning.o heaparray.o publicfun.o
localplanning.o : GetChangeEnv.h ComPlan.h HeapQueue.h heaparray.o publicfun.o pretreatment.o
pretreatment.o : GetChangeEnv.h PublicFun.h publicfun.o
heaparray.o : HeapQueue.h PublicFun.h publicfun.o
publicfun.o : PublicFun.h
test_heaparray.o : heaparray.o HeapQueue.h PublicFun.h

.PHONY : clean
clean :
	rm test $(objects) libmission.a

.PHONY : cleanall
cleanall :
	rm test $(objects) libmission.a out1 out2 test.log

.PHONY : run
run :
	./test

.PHONY : confirm
confirm :
	./test > out1

.PHONY : regresstest
regresstest :
	./regtest.sh

.PHONY : showenv
showenv :
	./test > test.log
	./testglobalenv.py test.log

.PHONY : leakcheck
leakcheck :
	valgrind --leak-check=full --track-fds=yes ./test
