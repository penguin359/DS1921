CC ?= gcc
CFLAGS ?= -Wall -Werror -g
MAKE ?= make

all: dumplog test-xbee

dumplog: dumplog.o
	$(CC) $(CFLAGS) -o $@ $^

dumplog.o: dumplog.c
	$(CC) $(CFLAGS) -c $<

xbee.o: xbee.c xbee.h
	$(CC) $(CFLAGS) -c $<

test-xbee.o: test-xbee.c xbee.h
	$(CC) $(CFLAGS) -c $<

test-xbee: test-xbee.o xbee.o
	$(CC) $(CFLAGS) -o $@ $^

test: all
	./test-xbee testfile

clean:
	-rm -fr testfile dumplog.dSYM dumplog test-xbee.dSYM test-xbee *.o

.PHONY: all clean test
