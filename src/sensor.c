#include <stdio.h>
#include <time.h>

#include "xbee.h"
#include "sensor.h"

int queryTime(xbee_t *xbee, macAddr64_t *addr64)
{
	char buf[1];

	buf[0] = QUERY_TIME_SENSOR_CMD;
	return sendTx(xbee, addr64, buf, sizeof(buf));
}

int sendTime(xbee_t *xbee, macAddr64_t *addr64)
{
	char buf[5];

	printf("Setting time on node...\n");
	buf[0] = SEND_TIME_SENSOR_CMD;
	time((time_t *)&buf[1]);
	return sendTx(xbee, addr64, buf, sizeof(buf));
}

int querySensor(xbee_t *xbee, macAddr64_t *addr64, int sensor)
{
	char buf[2];

	buf[0] = QUERY_SENSOR_SENSOR_CMD;
	buf[1] = sensor;
	return sendTx(xbee, addr64, buf, sizeof(buf));
}
