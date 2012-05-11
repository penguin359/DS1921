CC ?= gcc
CFLAGS ?= -Wall -Werror -g
MAKE ?= make
SRC_FILES = serial.h xbee.h xbee.c sensor.h sensor.c

all:
	$(MAKE) -C src all
	for i in $(SRC_FILES); do \
		if [ ! -L "$$i" ]; then \
			ln -s src/"$$i"; \
		fi; \
	done
	#cat src/xbee.h src/sensor.h > src.h

test:
	$(MAKE) -C src test

clean:
	$(MAKE) -C src clean
	-rm -fr $(SRC_FILES)

.PHONY: all clean test
