#ifndef _SENSOR_H_
#define _SENSOR_H_
#include <stdint.h>

#include "xbee.h"


#define QUERY_TIME_SENSOR_CMD	1
#define SEND_TIME_SENSOR_CMD	2
#define QUERY_SENSOR_SENSOR_CMD	3
#define DEBUG_SENSOR_CMD	4

struct queryTimeRequest {
	uint8_t cmd;
} __attribute((packed));

struct queryTimeResponse {
	uint8_t cmd;
	uint32_t time;
} __attribute((packed));

struct sendTimeRequest {
	uint8_t cmd;
	uint32_t time;
} __attribute((packed));

struct querySensorRequest {
	uint8_t cmd;
	uint8_t sensor;
} __attribute((packed));

struct querySensorResponse {
	uint8_t cmd;
	uint8_t sensor;
	uint8_t type;
	uint32_t time;
	union {
		uint8_t data8[4];
		uint16_t data16[2];
		uint32_t data32[1];
	};
} __attribute((packed));

struct debugResponse {
	uint8_t cmd;
	char debug[0];
} __attribute((packed));

int queryTime(xbee_t *xbee, macAddr64_t *addr64);
int sendTime(xbee_t *xbee, macAddr64_t *addr64);
int querySensor(xbee_t *xbee, macAddr64_t *addr64, int sensor);

#endif /* _SENSOR_H_ */
