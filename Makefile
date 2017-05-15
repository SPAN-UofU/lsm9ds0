CC      = gcc
CFLAGS  = -Wall -O2
LDFLAGS = -lm

lsm9ds0-objs := lsm9ds0.o

all: cleanest basic_test

basic_test: basic_test.o $(lsm9ds0-objs)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

.PHONY: clean cleanest

clean:
	rm -f *.o

cleanest: clean
	rm -f basic_test
