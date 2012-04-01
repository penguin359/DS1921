CC ?= gcc
CFLAGS ?= -Wall -Werror -g
MAKE ?= make

all: dumplog

dumplog: dumplog.o
	$(CC) $(CFLAGS) -o $@ $^

dumplog.o: dumplog.c
	$(CC) $(CFLAGS) -c $^

#test: dumplog
#	./dumplog testfile

clean:
	-rm -fr dumplog.dSYM dumplog *.o

.PHONY: all clean test
