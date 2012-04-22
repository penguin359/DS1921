#include <OneWire.h>
#include <XBee.h>
#include <Wire.h>

// OneWire DS18S20, DS18B20, DS1822 Temperature Example
//
// http://www.pjrc.com/teensy/td_libs_OneWire.html
//
// The DallasTemperature library can do all this work for you!
// http://milesburton.com/Dallas_Temperature_Control_Library


#define SELF_POWERED			1


#define DS18B20_CONVERT_TEMP		0x44
#define DS18B20_COPY_SCRATCHPAD		0x48
#define DS18B20_READ_SCRATCHPAD		0xBE
#define DS18B20_WRITE_SCRATCHPAD	0x4E
#define DS18B20_RECALL_EE		0xB8
#define DS18B20_READ_POWER_SUPPLY	0xB4

#define DS1921_CONVERT_TEMP		0x44
#define DS1921_COPY_SCRATCHPAD		0x55
#define DS1921_READ_SCRATCHPAD		0xAA
#define DS1921_WRITE_SCRATCHPAD		0x0F
#define DS1921_CLEAR_MEMORY		0x3C
#define DS1921_READ_MEMORY		0xF0

#define DS1921_RTC_REGISTER		0x0200
#define DS1921_RTC_ALARM_REGISTER	0x0207
#define DS1921_CONTROL_REGISTER		0x020E
#define DS1921_STATUS_REGISTER		0x0214
#define DS1921_SAMPLE_REGISTER		0x020D
#define DS1921_DEVICE_SAMPLES_COUNTER	0x021D
#define DS1921_MISSION_SAMPLES_COUNTER	0x021A
#define DS1921_MISSION_TIMESTAMP	0x0215
#define DS1921_MISSION_START_DELAY	0x0212
#define DS1921_TEMPERATURE		0x0211
#define DS1921_TEMPERATURE_LOW_ALARM	0x020B
#define DS1921_TEMPERATURE_HIGH_ALARM	0x020C
#define DS1921_TEMP_HISTOGRAM		0x0800
#define DS1921_DATA_LOG			0x1000

/*
 * Device Samples Counter: 24-bit unsigned little-endian
 * Mission Samples Counter: 24-bit unsigned little-endian
 * Mission Timestamp:
 *   BCD minutes
 *   BCD hours (24-hour)
 *   BCD day of month
 *   BCD year without century
 */


#define DS1921_CONTROL_nEOSC		0x80
#define DS1921_CONTROL_EMCLR		0x40
#define DS1921_CONTROL_nEM		0x10
#define DS1921_CONTROL_RO		0x08
#define DS1921_CONTROL_TLS		0x04
#define DS1921_CONTROL_THS		0x02
#define DS1921_CONTROL_TAS		0x01

#define DS1921_STATUS_nTCB		0x80
#define DS1921_STATUS_MEMCLR		0x40
#define DS1921_STATUS_MIP		0x20
#define DS1921_STATUS_SIP		0x10
#define DS1921_STATUS_TLF		0x04
#define DS1921_STATUS_THF		0x02
#define DS1921_STATUS_TAF		0x01


#define LED_PIN				11
#define ledOn()				digitalWrite(LED_PIN, HIGH);
#define ledOff()			digitalWrite(LED_PIN, LOW);


OneWire  ds(21);  // on pin 10

class Dummy {
	public:
		void print(char *c) {}
		void print(char *c, int i) {}
		void print(int c) {}
		void print(int c, int i) {}
		void println(char *c) {}
		void println(char *c, int i) {}
		void println(int c) {}
		void println(int c, int i) {}
		int available() {return 0;}
		int read() {return -1;}
};

//HardwareSerial Uart = HardwareSerial();
//#define debug Uart
Dummy Dummy;
//#define debug Dummy
#define debug Serial

XBee xbee = XBee();
XBeeResponse response = XBeeResponse();
ZBRxResponse rx = ZBRxResponse();
ModemStatusResponse msr = ModemStatusResponse();
XBeeAddress64 coordinator = XBeeAddress64(0x0, 0x0);
uint8_t payload[] = { 'H', 'i' };

unsigned long clock = 0;
//elapsedMillis clockTick;
unsigned long  clockTick;

#define D0 5
#define D1 6

void setup(void) {
  //debug.begin(9600);
  xbee.begin(9600);
  delay(5000);
  //debug.println("Hello, World!");
  pinMode(LED_PIN, OUTPUT);
  analogReference(DEFAULT);
  pinMode(A1, INPUT);
  digitalWrite(A1, LOW);
  pinMode(D0, INPUT);
  pinMode(D1, INPUT);
  digitalWrite(D0, HIGH); /* Turn on pull-ups */
  digitalWrite(D1, HIGH);
  Wire.begin();
}

void writeDS18B20(byte *addr, byte high, byte low, byte config) {
  byte status;

  ds.reset();
  ds.select(addr);
  ds.write(DS18B20_WRITE_SCRATCHPAD);
  ds.write(high);
  ds.write(low);
  ds.write(config);

  ds.reset();
  ds.select(addr);
  ds.write(DS18B20_READ_SCRATCHPAD);
  ds.read();
  ds.read();
  if(ds.read() != high ||
     ds.read() != low ||
     ds.read() != config) {
	  /* TODO: record failure to configure */
	  return;
  }

  ds.reset();
  ds.select(addr);
  ds.write(DS18B20_COPY_SCRATCHPAD);
#ifdef SELF_POWERED
  /* Wait for finished writing */
  while(!ds.read_bit())
	  ;
#else
  /* TODO: enable strong pull-up */
  delay(10);
  /* TODO: disable strong pull-up */
#endif
  /* TODO: replace delay with better solution */
}

void writeDS1921(byte *addr, int target, byte *data, int len) {
  byte status;

  ds.reset();
  ds.select(addr);
  ds.write(DS1921_WRITE_SCRATCHPAD);
  ds.write(target & 0x00FF);
  ds.write(target >> 8 & 0x00FF);
  while(len-- > 0)
    ds.write(*data++);
  /* TODO: write til end of page for CRC-16 */

  ds.reset();
  ds.select(addr);
  ds.write(DS1921_READ_SCRATCHPAD);
  target  = (int)ds.read();
  target |= (int)ds.read() << 8;
  status = ds.read();
  /* TODO: check AA and PF flags */
  /* TODO: read back and verify data */
  /* TODO: read til end of page for CRC-16 */

  ds.reset();
  ds.select(addr);
  ds.write(DS1921_COPY_SCRATCHPAD);
  ds.write(target & 0x00FF);
  ds.write(target >> 8 & 0x00FF);
  ds.write(status);
  delay(1);
  /* TODO: replace delay with better solution */
}

void readDS1921(byte *addr, int target, byte *data, int len) {
  ds.reset();
  ds.select(addr);
  ds.write(DS1921_READ_MEMORY);
  ds.write(target & 0x00FF);
  ds.write(target >> 8 & 0x00FF);
  while(len-- > 0)
    *data++ = ds.read();
}

void clearDS1921(byte *addr) {
  byte control;

  debug.println("  Clearing memory...");
  readDS1921(addr, DS1921_STATUS_REGISTER, &control, sizeof(control));
  if(control & DS1921_STATUS_MEMCLR)
    debug.println("  Memory previously cleared.");

  readDS1921(addr, DS1921_CONTROL_REGISTER, &control, sizeof(control));

  debug.print("  Control = ");
  debug.println(control, HEX);
  control |= DS1921_CONTROL_EMCLR;
  control |= DS1921_CONTROL_RO;

  writeDS1921(addr, DS1921_CONTROL_REGISTER, &control, sizeof(control));

  ds.reset();
  ds.select(addr);
  ds.write(DS1921_CLEAR_MEMORY);
  delayMicroseconds(550);

  readDS1921(addr, DS1921_STATUS_REGISTER, &control, sizeof(control));
  debug.print("  Status = ");
  debug.println(control, HEX);
  if(control & DS1921_STATUS_MEMCLR)
    debug.println("  Memory cleared successfully.");
  else
    debug.println("  Failed to clear memory!");
}

void stopMission(byte *addr) {
  byte zero = 0;
  writeDS1921(addr, DS1921_TEMPERATURE, &zero, sizeof(zero));
}

enum {
  HOME_STATE,
  D_STATE,
  C_STATE,
  R_STATE,
  RT_STATE,
  RTC_STATE,
};

byte addr[8];
byte buf[11];
int bufCount = 0;
long val;

void parseSerial(char c) {
  static int state = HOME_STATE;
  byte rtcBuf[7];

  int i;

  debug.print("S=");
  debug.print(state, DEC);
  debug.print(": ");
  debug.println(c);
  switch(state) {
    case HOME_STATE:
      if(c == 'D') {
	state = D_STATE;
	break;
      } else if(c == 'C') {
	state = C_STATE;
	bufCount = 0;
	val = 0;
	break;
      } else if(c == 'R') {
	state = R_STATE;
	break;
      }

      state = HOME_STATE;
      break;

    case D_STATE:
      if(c == '\n') {
	debug.println("STARTLOG");
	readDS1921(addr, DS1921_MISSION_TIMESTAMP, rtcBuf, 5);
	debug.print("Time=20");
	debug.print(rtcBuf[4], HEX);
	debug.print("-");
	debug.print(rtcBuf[3], HEX);
	debug.print("-");
	debug.print(rtcBuf[2], HEX);
	debug.print("T");
	debug.print(rtcBuf[1], HEX);
	debug.print(":");
	debug.print(rtcBuf[0], HEX);
	debug.println(":00Z");

	long count = 0;
	byte countBytes[3];
	readDS1921(addr, DS1921_MISSION_SAMPLES_COUNTER, countBytes, sizeof(countBytes));
	count = countBytes[0] << 0 | countBytes[1] << 8 | countBytes[2] << 16;
	debug.print("Count=");
	debug.println(count, DEC);

	readDS1921(addr, DS1921_DATA_LOG, NULL, 0);
	debug.println("LOG");
	if(count > 2048)
		count = 2048;
	while(count-- > 0) {
	  debug.print("    ");
	  debug.println(ds.read(), DEC);
	}
	debug.println("ENDLOG");
	state = HOME_STATE;
	break;
      }

      state = HOME_STATE;
      break;

    case C_STATE:
      if(isdigit(c) && bufCount < sizeof(buf)-1) {
	      val = val*10 + c - '0';
	      bufCount++;
	      //buf[bufCount] = c;
	      break;
      }

      clock = val;
      debug.print("Time set to ");
      debug.println(clock, DEC);
      state = HOME_STATE;
      break;

    case R_STATE:
      if(c == 'T') {
	state = RT_STATE;
	break;
      }

      state = HOME_STATE;
      break;

    case RT_STATE:
      if(c == 'C') {
	state = RTC_STATE;
	break;
      }

      state = HOME_STATE;
      break;

    case RTC_STATE:
      if(c == '\n') {
	debug.println("rtc time is ...");
	readDS1921(addr, DS1921_RTC_REGISTER, rtcBuf, sizeof(rtcBuf));
	debug.print("20");
	debug.print((unsigned char)rtcBuf[6], HEX);
	debug.print("-");
	debug.print((unsigned char)rtcBuf[5] & ~0x80, HEX);
	debug.print("-");
	debug.print((unsigned char)rtcBuf[4], HEX);
	debug.print("T");
	debug.print((unsigned char)rtcBuf[2], HEX);
	debug.print(":");
	debug.print((unsigned char)rtcBuf[1], HEX);
	debug.print(":");
	debug.print((unsigned char)rtcBuf[0], HEX);
	debug.println("Z");
	state = HOME_STATE;
	break;
      } else {
	memset(rtcBuf, 0, sizeof(rtcBuf));
	/* discard 2 */
	while(!debug.available())
	  ;
	c = debug.read();
	/* discard 0 */
	while(!debug.available())
	  ;
	c = debug.read();
	rtcBuf[6] = (c & ~0x30) << 4;
	while(!debug.available())
	  ;
	c = debug.read();
	rtcBuf[6] |= (c & ~0x30);
	while(!debug.available())
	  ;
	c = debug.read();
	/* discard - */
	while(!debug.available())
	  ;
	c = debug.read();
	rtcBuf[5] = (c & ~0x30) << 4;
	while(!debug.available())
	  ;
	c = debug.read();
	rtcBuf[5] |= (c & ~0x30);
	rtcBuf[5] |= 0x80;
	while(!debug.available())
	  ;
	c = debug.read();
	/* discard - */
	while(!debug.available())
	  ;
	c = debug.read();
	rtcBuf[4] = (c & ~0x30) << 4;
	while(!debug.available())
	  ;
	c = debug.read();
	rtcBuf[4] |= (c & ~0x30);
	while(!debug.available())
	  ;
	c = debug.read();
	/* discard T */
	while(!debug.available())
	  ;
	c = debug.read();
	rtcBuf[2] = (c & ~0x30) << 4;
	while(!debug.available())
	  ;
	c = debug.read();
	rtcBuf[2] |= (c & ~0x30);
	while(!debug.available())
	  ;
	c = debug.read();
	/* discard : */
	while(!debug.available())
	  ;
	c = debug.read();
	rtcBuf[1] = (c & ~0x30) << 4;
	while(!debug.available())
	  ;
	c = debug.read();
	rtcBuf[1] |= (c & ~0x30);
	while(!debug.available())
	  ;
	c = debug.read();
	/* discard : */
	while(!debug.available())
	  ;
	c = debug.read();
	rtcBuf[0] = (c & ~0x30) << 4;
	while(!debug.available())
	  ;
	c = debug.read();
	rtcBuf[0] |= (c & ~0x30);
	while(!debug.available())
	  ;
	c = debug.read();
	/* discard '\n' */
	rtcBuf[3] = 1; /* day of week (1-7) */
	debug.print("  Set RTC = 20");
	for(i = 6; i >= 0; i--) {
	  debug.print(rtcBuf[i], HEX);
	  debug.print(" ");
	}
	debug.println("");
	debug.println("  Updating RTC...");
	writeDS1921(addr, DS1921_RTC_REGISTER, rtcBuf, sizeof(rtcBuf));
      }

      state = HOME_STATE;
      break;

    default:
      state = HOME_STATE;
      break;
  }
}

/* 2011-12-25T05:22:40 Sun */
byte rtc[] = {
  0x40,	  /* BCD seconds */
  0x22,	  /* BCD minutes */
  0x05,	  /* BCD hours (24-hour) */
  0x01,	  /* day of week (1-7) */
  0x25,	  /* BCD day of month */
  0x92,	  /* century bit + BCD month */
  0x11,	  /* BCD year without century */
};

int writeRtc = 0;

void loop(void) {
  static int alarm = 0;
  byte i;
  byte present = 0;
  byte type_s, type_19;
  byte data[12];
  //byte addr[8];
  float celsius, fahrenheit;

  unsigned long currentMillis = millis();
  if(currentMillis - clockTick >= 1000UL) {
	  clockTick += 1000UL;
	  clock++;
	  debug.print("UTime=");
	  debug.println(clock, DEC);
	  ZBTxRequest zbTx = ZBTxRequest(coordinator, payload, sizeof(payload));
	  xbee.send(zbTx);
  }

  if(debug.available()) {
    parseSerial(debug.read());
  }

  xbee.readPacket();
  if(xbee.getResponse().isAvailable()) {
      switch(xbee.getResponse().getApiId()) {
      case ZB_RX_RESPONSE:
	      debug.print("ZB RZ");
        if (rx.getOption() == ZB_PACKET_ACKNOWLEDGED) {
		debug.print(" (ACK)");
	}
	debug.println("");
	break;

      case MODEM_STATUS_RESPONSE:
	debug.println("MODEM");
	xbee.getResponse().getModemStatusResponse(msr);
	if(msr.getStatus() == ASSOCIATED)
		debug.println("ASSOCIATED");
	break;

      default:
	debug.println("Unknown");
	break;
      }
  }

#if 1
  delay(2000);
  long val = analogRead(A1);
  celsius = ((float)val / 1023. * 5000. - 500.)/10.;
  debug.print("ADC Val: ");
  debug.println(val, DEC);

  Wire.beginTransmission(0x4f);
  Wire.write(0);
  Wire.endTransmission();
  Wire.requestFrom(0x4f, 2);
  val = Wire.read() << 8;
  val |= Wire.read();
  val >>= 4;
  debug.print("I2C Val: ");
  debug.println(val, DEC);
  celsius = (float)val * 0.0625;
#if 0
  if ( !ds.search(addr)) {
    //debug.println("No more addresses.");
    debug.println();
    ds.reset_search();
    delay(250);
    delay(1250);
    return;
  }

#if 0
  debug.print("ROM =");
  for( i = 0; i < 8; i++) {
    debug.write(' ');
    debug.print(addr[i], HEX);
  }
#endif

  if (OneWire::crc8(addr, 7) != addr[7]) {
      debug.println("CRC is not valid!");
      return;
  }
  debug.println();
 
   type_19 = 0;
  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      debug.println("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
      debug.println("  Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
      debug.println("  Chip = DS1822");
      type_s = 0;
      break;
    case 0x21:
      debug.println("  Chip = DS1921G");
      type_19 = 1;
      break;
    default:
#if 0
      debug.println("Device is not a DS18x20 family device.");
#endif
      return;
  } 

  ds.reset();
  ds.select(addr);
  ds.write(DS1921_CONVERT_TEMP, 1);         // start conversion, with parasite power on at the end
  
  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.
  
  present = ds.reset();
  ds.select(addr);
  if(type_19) {
    ds.write(DS1921_READ_MEMORY);
#if 0
    ds.write(0x00);
    ds.write(0x02);
    debug.print("  Clock =");
    for(i = 0x00; i <= 0x0A; i++) {
      debug.print(" ");
      debug.print(ds.read(), HEX);
    }
    debug.println("");
    for( ; i < 0x11; i++)
      ds.read();
#else
    ds.write(0x11);
    ds.write(0x02);
#endif
    data[0] = ds.read();
#if 0
    debug.print("  Data = ");
    debug.print(present,HEX);
    debug.print(" ");
    debug.println(data[0], HEX);
#endif
    celsius = (float)data[0] / 2.0f - 40.0f;

#if 0
    if(!writeRtc) {
      byte sampleRate;
      readDS1921(addr, DS1921_SAMPLE_REGISTER, &sampleRate, sizeof(sampleRate));
      debug.print("  sample rate = ");
      debug.println(sampleRate, DEC);
      readDS1921(addr, DS1921_STATUS_REGISTER, &sampleRate, sizeof(sampleRate));
      if(sampleRate & DS1921_STATUS_MIP) {
	debug.println("  Mission in progress.");
	writeRtc = 1;
	return;
      }
      clearDS1921(addr);
      debug.println("  Starting a mission.");
      sampleRate = 1;
      writeDS1921(addr, DS1921_SAMPLE_REGISTER, &sampleRate, sizeof(sampleRate));
      //debug.println("  Updating RTC...");
      //writeDS1921(addr, DS1921_RTC_REGISTER, rtc, sizeof(rtc));
      writeRtc = 1;
    }

    long count = 0;
    byte countBytes[3];

    readDS1921(addr, DS1921_DEVICE_SAMPLES_COUNTER, countBytes, sizeof(countBytes));
    count = countBytes[0] << 0 | countBytes[1] << 8 | countBytes[2] << 16;
    debug.print("  Device samples = ");
    debug.println(count, DEC);

    readDS1921(addr, DS1921_MISSION_SAMPLES_COUNTER, countBytes, sizeof(countBytes));
    count = countBytes[0] << 0 | countBytes[1] << 8 | countBytes[2] << 16;
    debug.print("  Mission samples = ");
    debug.println(count, DEC);
    readDS1921(addr, DS1921_MISSION_TIMESTAMP, rtc, 5);
    debug.print("  Mission timestamp = 20");
    //for(i = 4; i >= 0; i--) {
    //  debug.print(rtc[i], HEX);
    //  debug.print(" ");
    //}
    for(i = 0; i < 5; i++) {
      debug.print(rtc[4-i], HEX);
      debug.print(" ");
    }
    debug.println("");
    readDS1921(addr, DS1921_DATA_LOG, NULL, 0);
    debug.println("  Data log:");
    if(count > 2048)
	    count = 2048;
    count = 4;
    while(count-- > 0) {
      debug.print("    ");
      debug.println(ds.read(), DEC);
    }
    debug.println("");
#endif
  } else {
    ds.write(DS18B20_READ_SCRATCHPAD);

#if 0
  debug.print("  Data = ");
  debug.print(present,HEX);
  debug.print(" ");
#endif
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
#if 0
    debug.print(data[i], HEX);
    debug.print(" ");
#endif
  }
#if 0
  debug.print(" CRC=");
  debug.print(OneWire::crc8(data, 8), HEX);
  debug.println();
#endif

  // convert the data to actual temperature

  unsigned int raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // count remain gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    if (cfg == 0x00) raw = raw << 3;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw << 2; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw << 1; // 11 bit res, 375 ms
    // default is 12 bit resolution, 750 ms conversion time
  }
  celsius = (float)raw / 16.0;
  }
#endif
  if(celsius < 8. || celsius > 58.) {
	  debug.println("ALARM!");
	  alarm = 1;
	  ledOn();
  } else {
	  ledOff();
  }
  fahrenheit = celsius * 1.8 + 32.0;
  //debug.print("  ATemperature = ");
  debug.print("  T=");
  debug.print(celsius);
  //debug.print(" Celsius, ");
  debug.print("C, ");
  debug.print(fahrenheit);
  //debug.println(" Fahrenheit");
  debug.println("F");
#endif
}
