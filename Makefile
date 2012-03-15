CC = gcc
CFLAGS = -Wall -Werror -g

xbee: xbee.c
	$(CC) $(CFLAGS) -o $@ $^

test: xbee
	./xbee testfile
