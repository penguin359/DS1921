CC ?= gcc
CFLAGS ?= -Wall -Werror -g
MAKE ?= make

all: dumplog xbee

dumplog: dumplog.o
	$(CC) $(CFLAGS) -o $@ $^

dumplog.o: dumplog.c
	$(CC) $(CFLAGS) -c $^

xbee: xbee.o
	$(CC) $(CFLAGS) -o $@ $^

xbee.o: xbee.c
	$(CC) $(CFLAGS) -c $^

test: all
	./xbee testfile

clean:
	-rm -fr testfile dumplog.dSYM dumplog xbee.dSYM xbee *.o

.PHONY: all clean test
