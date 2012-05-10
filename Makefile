CC ?= gcc
CFLAGS ?= -Wall -Werror -g
MAKE ?= make

all:
	$(MAKE) -C src all

test:
	$(MAKE) -C src test

clean:
	$(MAKE) -C src clean

.PHONY: all clean test
