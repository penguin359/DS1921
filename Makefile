CC = gcc
CFLAGS = -Wall -Werror -g

all: xbee

xbee: xbee.o
	$(CC) $(CFLAGS) -o $@ $^

xbee.o: xbee.c
	$(CC) $(CFLAGS) -c $^

test: xbee
	./xbee testfile

clean:
	-rm -fr testfile xbee.dSYM xbee *.o

.PHONY: all clean test
