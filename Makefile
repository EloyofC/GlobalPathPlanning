CC = gcc -g -Wall
DEBUG ?= 1
ifeq ($(DEBUG), 1)
	CFLAGS = -O1 -DDEBUG
else
	CFLAGS = -O2 -DNDEBUG
endif

objects = advancedplan.o localplanning.o pretreatment.o heaparray.o publicfun.o

atest : $(objects) main.o
	$(CC) -o atest $(objects) main.o $(CFLAGS)

main.o : MapProcess.h GetChangeEnv.h ComPlan.h $(objects)
advancedplan.o : GetChangeEnv.h ComPlan.h PublicFun.h localplanning.o heaparray.o publicfun.o
localplanning.o : GetChangeEnv.h ComPlan.h HeapQueue.h heaparray.o publicfun.o pretreatment.o
pretreatment.o : GetChangeEnv.h MapProcess.h PublicFun.h publicfun.o
heaparray.o : HeapQueue.h PublicFun.h publicfun.o
publicfun.o : PublicFun.h

.PHONY : clean
clean :
	rm atest $(objects) main.o

.PHONY : testwhole
testwhole :
	./atest < ./map1
