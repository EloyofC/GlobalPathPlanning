CC = gcc -g -Wall -Wextra -Wstrict-prototypes -Wmissing-prototypes
LINKFLAGS = -L. -lmission -lm
CHECK = -lcheck -lrt -lsubunit -pthread
DEBUG ?= 0			#usage: make DEBUG=\d
ifeq ($(DEBUG), 1)
	CFLAGS = -O1 -DDEBUG
else
	CFLAGS = -O2 -DNDEBUG
endif

objects = advancedplan.o localplanning.o pretreatment.o heapqueue.o publicfun.o mission.o fifoqueue.o
unittests = test_heapqueue.o test_publicfun.o test_fifoqueue.o test_advancedplan.o
testsample = test.c

all : libmission.a test test_heapqueue test_publicfun test_fifoqueue test_advancedplan
test : $(testsample) libmission.a
	$(CC) -o $@ $(CFLAGS) test.c $(LINKFLAGS)

libmission.a : $(objects)
	ar -rc $@ $^

test_heapqueue : test_heapqueue.o libmission.a
	$(CC) -o $@ $(CFLAGS) test_heapqueue.o $(LINKFLAGS) $(CHECK)

test_publicfun : test_publicfun.o libmission.a
	$(CC) -o $@ $(CFLAGS) test_publicfun.o $(LINKFLAGS) $(CHECK)

test_fifoqueue : test_fifoqueue.o libmission.a
	$(CC) -o $@ $(CFLAGS) test_publicfun.o $(LINKFLAGS) $(CHECK)

test_advancedplan : test_advancedplan.o libmission.a
	$(CC) -o $@ $(CFLAGS) test_advancedplan.o $(LINKFLAGS) $(CHECK)

mission.o : pretreatment.h localplanning.h advancedplan.h publicfun.h mission.h localplanning.o publicfun.o pretreatment.o
advancedplan.o : pretreatment.h localplanning.h fifoqueue.h advancedplan.h publicfun.h localplanning.o heapqueue.o publicfun.o fifoqueue.o
localplanning.o : pretreatment.h localplanning.h advancedplan.h heapqueue.h heapqueue.o publicfun.o pretreatment.o
pretreatment.o : pretreatment.h publicfun.h publicfun.o
fifoqueue.o : fifoqueue.h publicfun.h publicfun.o
heapqueue.o : heapqueue.h publicfun.h publicfun.o
publicfun.o : publicfun.h
test_heapqueue.o : heapqueue.o heapqueue.h publicfun.h
test_publicfun.o : publicfun.o publicfun.h
test_fifoqueue.o : publicfun.h fifoqueue.o fifoqueue.h pretreatment.h
test_advancedplan.o : publicfun.h pretreatment.h advancedplan.h advancedplan.o

.PHONY : clean
clean :
	rm test $(objects) $(unittests) libmission.a

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
