#include <Arduino.h>

#include "serial.h"


int readSerial(serial_t *serial)
{
	Stream *stream = (Stream *)serial->stream;
	return stream->read();
}

int writeSerial(serial_t *serial, unsigned char c)
{
	Stream *stream = (Stream *)serial->stream;
	return stream->write(c);
}

int closeSerial(serial_t *serial)
{
	Serial.end();
	return 0;
}

serial_t *openSerial(char *file)
{
	static serial_t serialStore;
	serial_t *serial;

	serial = &serialStore;

	Serial.begin(9600);
	serial->stream = &Serial;

	return serial;
}
