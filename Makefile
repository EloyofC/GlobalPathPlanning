CC = gcc -g -Wall -Wextra -Wstrict-prototypes -Wmissing-prototypes -std=c99
LINKFLAGS = -L. -lmission -lm
CHECK = -lcheck -lrt -lsubunit -pthread
DEBUG ?= 0			#usage: make DEBUG=\d
ifeq ($(DEBUG), 1)
	CFLAGS = -O1 -DDEBUG
else
	CFLAGS = -O2 -DNDEBUG
endif

OBJECTS = advancedplan.o localplanning.o pretreatment.o envoperate.o heapqueue.o publicfun.o mission.o fifoqueue.o
TESTSAMPLE = test.c
UNITTESTTARGETS = $(basename $(wildcard test_*.c))
TARGETS = test

default : libmission.a test
all : libmission.a $(TARGETS) $(UNITTESTTARGETS)
test : $(TESTSAMPLE) libmission.a
	$(CC) -o $@ $(CFLAGS) test.c $(LINKFLAGS)

libmission.a : $(OBJECTS)
	ar -rc $@ $^

$(UNITTESTTARGETS) : % : %.c libmission.a
	$(CC) -o $@ $(CFLAGS) $< $(LINKFLAGS) $(CHECK)

mission.o : pretreatment.h localplanning.h advancedplan.h publicfun.h mission.h
advancedplan.o : pretreatment.h localplanning.h fifoqueue.h advancedplan.h publicfun.h
localplanning.o : pretreatment.h localplanning.h advancedplan.h heapqueue.h
pretreatment.o : pretreatment.h publicfun.h
envoperate.o : envoperate.h publicfun.h
fifoqueue.o : fifoqueue.h publicfun.h
heapqueue.o : heapqueue.h publicfun.h
publicfun.o : publicfun.h

.PHONY : clean
clean :
	rm $(OBJECTS) $(UNITTESTTARGETS) $(TARGETS) libmission.a

.PHONY : cleanall
cleanall :
	rm $(OBJECTS) $(UNITTESTTARGETS) $(TARGETS) libmission.a out1 out2 test.log

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
