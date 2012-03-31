CC ?= gcc
CFLAGS ?= -Wall -Werror -g
MAKE ?= make

all: xbee rrd
	$(MAKE) -C rrd all

xbee: xbee.o
	$(CC) $(CFLAGS) -o $@ $^

xbee.o: xbee.c
	$(CC) $(CFLAGS) -c $^

test: xbee
	./xbee testfile

clean:
	-rm -fr testfile xbee.dSYM xbee *.o
	$(MAKE) -C rrd clean

.PHONY: all clean test
