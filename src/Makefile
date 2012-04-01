CC ?= gcc
CFLAGS ?= -Wall -Werror -g
MAKE ?= make

all: dumplog test-xbee

serial.o: serial.c serial.h
	$(CC) $(CFLAGS) -c $<

dumplog.o: dumplog.c serial.h
	$(CC) $(CFLAGS) -c $<

dumplog: dumplog.o serial.o
	$(CC) $(CFLAGS) -o $@ $^

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
