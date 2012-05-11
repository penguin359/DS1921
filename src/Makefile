CC ?= gcc
CFLAGS ?= -Wall -Werror -g
MAKE ?= make

all: dumplog http test-xbee xbee-monitor

serial.o: serial.c serial.h
	$(CC) $(CFLAGS) -c $<

dumplog.o: dumplog.c serial.h
	$(CC) $(CFLAGS) -c $<

dumplog: dumplog.o serial.o
	$(CC) $(CFLAGS) -o $@ $^

http.o: http.c
	$(CC) $(CFLAGS) -c $<

http: http.o
	$(CC) $(CFLAGS) -o $@ $^

xbee-monitor.o: xbee-monitor.c serial.h xbee.h sensor.h
	$(CC) $(CFLAGS) -c $<

xbee-monitor: xbee-monitor.o serial.o xbee.o sensor.o
	$(CC) $(CFLAGS) -o $@ $^

xbee.o: xbee.c serial.h xbee.h
	$(CC) $(CFLAGS) -c $<

test-xbee.o: test-xbee.c xbee.h sensor.h
	$(CC) $(CFLAGS) -c $<

test-xbee: test-xbee.o serial.o xbee.o sensor.o
	$(CC) $(CFLAGS) -o $@ $^

test: all
	./test-xbee testfile

tags:
	ctags *.[ch]

clean:
	-rm -fr testfile dumplog.dSYM dumplog tags test-xbee.dSYM test-xbee xbee-monitor *.o

.PHONY: all clean test
