#include <stdio.h>
#include <time.h>

#include "xbee.h"
#include "sensor.h"

#define DEBUG_SENSOR_API

int queryTime(xbee_t *xbee, macAddr64_t *addr64)
{
	char buf[1];

#ifdef DEBUG_SENSOR_API
	printf("Querying time on %s\n", strMacAddr64(addr64));
#endif
	buf[0] = QUERY_TIME_SENSOR_CMD;
	return sendTx(xbee, addr64, buf, sizeof(buf));
}

int sendTime(xbee_t *xbee, macAddr64_t *addr64)
{
	char buf[5];

#ifdef DEBUG_SENSOR_API
	printf("Sending time to %s\n", strMacAddr64(addr64));
#endif
	buf[0] = SEND_TIME_SENSOR_CMD;
	time((time_t *)&buf[1]);
	return sendTx(xbee, addr64, buf, sizeof(buf));
}

int querySensor(xbee_t *xbee, macAddr64_t *addr64, int sensor)
{
	char buf[2];

#ifdef DEBUG_SENSOR_API
	printf("Sending querying sensor %d on node %s\n", sensor, strMacAddr64(addr64));
#endif
	buf[0] = QUERY_SENSOR_SENSOR_CMD;
	buf[1] = sensor;
	return sendTx(xbee, addr64, buf, sizeof(buf));
}
