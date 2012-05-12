#include <Arduino.h>

#include "serial.h"


extern "C" int readSerial(serial_t *serial)
{
	Stream *stream = (Stream *)serial->stream;
	return stream->read();
}

extern "C" int writeSerial(serial_t *serial, unsigned char c)
{
	Stream *stream = (Stream *)serial->stream;
	return stream->write(c);
}

extern "C" int closeSerial(serial_t *serial)
{
	Serial.end();
	return 0;
}

extern "C" serial_t *openSerial(char *file)
{
	static serial_t serialStore;
	serial_t *serial;

	serial = &serialStore;

	Serial.begin(9600);
	serial->stream = &Serial;

	return serial;
}
