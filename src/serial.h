#ifndef _SERIAL_H_
#define _SERIAL_H_

#ifndef __AVR__
#include <termios.h>
#endif

typedef enum {
	FILE_SERIAL_TYPE,
	CHAR_SERIAL_TYPE,
} serialType_t;

typedef struct {
#ifdef __AVR__
	void *stream;
#else
	int fd;
	serialType_t type;
	struct termios savedTermios;
#endif
} serial_t;

int readSerial(serial_t *serial);
int writeSerial(serial_t *serial, unsigned char c);
int closeSerial(serial_t *serial);
serial_t *openSerial(char *file);

#endif /* _SERIAL_H_ */
