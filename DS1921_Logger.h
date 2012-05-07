typedef enum {
	NONE_SENSOR_TYPE,
	ANALOG_SENSOR_TYPE,
	LM75_SENSOR_TYPE,
	HIH6130_SENSOR_TYPE,
	TC_SENSOR_TYPE,
	ONE_WIRE_SENSOR_TYPE,
} sensorType_t;

typedef enum {
	START_SENSOR_STATE,
	WAIT_SENSOR_STATE,
	READ_SENSOR_STATE,
	COMPLETED_SENSOR_STATE,
} sensorState_t;

#define ALARM_SENSOR_FLAG	0x01
typedef uint8_t sensorFlags_t;

typedef struct {
	sensorType_t type;
	sensorState_t state;
	sensorFlags_t flags;
	unsigned long waitTime;
	union {
		uint8_t data8[4];
		uint16_t data16[2];
		uint32_t data32[1];
	};
	uint8_t addr;
} sensor_t;

typedef float temp_t;
#define ERROR_TEMP	-256.f

#define CELSIUS_TO_KELVIN	273
