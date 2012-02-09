#include <OneWire.h>

// OneWire DS18S20, DS18B20, DS1822 Temperature Example
//
// http://www.pjrc.com/teensy/td_libs_OneWire.html
//
// The DallasTemperature library can do all this work for you!
// http://milesburton.com/Dallas_Temperature_Control_Library


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


OneWire  ds(21);  // on pin 10
//HardwareSerial Uart = HardwareSerial();
//#define Serial Uart

void setup(void) {
  Serial.begin(9600);
  delay(5000);
  Serial.println("Hello, World!");
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

  Serial.println("  Clearing memory...");
  readDS1921(addr, DS1921_STATUS_REGISTER, &control, sizeof(control));
  if(control & DS1921_STATUS_MEMCLR)
    Serial.println("  Memory previously cleared.");

  readDS1921(addr, DS1921_CONTROL_REGISTER, &control, sizeof(control));

  Serial.print("  Control = ");
  Serial.println(control, HEX);
  control |= DS1921_CONTROL_EMCLR;
  control |= DS1921_CONTROL_RO;

  writeDS1921(addr, DS1921_CONTROL_REGISTER, &control, sizeof(control));

  ds.reset();
  ds.select(addr);
  ds.write(DS1921_CLEAR_MEMORY);
  delayMicroseconds(550);

  readDS1921(addr, DS1921_STATUS_REGISTER, &control, sizeof(control));
  Serial.print("  Status = ");
  Serial.println(control, HEX);
  if(control & DS1921_STATUS_MEMCLR)
    Serial.println("  Memory cleared successfully.");
  else
    Serial.println("  Failed to clear memory!");
}

void stopMission(byte *addr) {
  byte zero = 0;
  writeDS1921(addr, DS1921_TEMPERATURE, &zero, sizeof(zero));
}

enum {
  HOME_STATE,
  R_STATE,
  RT_STATE,
  RTC_STATE,
};

byte addr[8];

void parseSerial(char c) {
  static int state = HOME_STATE;
  byte rtcBuf[7];
  int i;

  switch(state) {
    case HOME_STATE:
      if(c == 'R') {
	state = R_STATE;
	break;
      }

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
	Serial.println("rtc time is ...");
	readDS1921(addr, DS1921_RTC_REGISTER, rtcBuf, sizeof(rtcBuf));
	Serial.print("20");
	Serial.print((unsigned char)rtcBuf[6], HEX);
	Serial.print("-");
	Serial.print((unsigned char)rtcBuf[5] & ~0x80, HEX);
	Serial.print("-");
	Serial.print((unsigned char)rtcBuf[4], HEX);
	Serial.print("T");
	Serial.print((unsigned char)rtcBuf[2], HEX);
	Serial.print(":");
	Serial.print((unsigned char)rtcBuf[1], HEX);
	Serial.print(":");
	Serial.print((unsigned char)rtcBuf[0], HEX);
	Serial.println("Z");
	state = HOME_STATE;
	break;
      } else {
	memset(rtcBuf, 0, sizeof(rtcBuf));
	/* discard 2 */
	while(!Serial.available())
	  ;
	c = Serial.read();
	/* discard 0 */
	while(!Serial.available())
	  ;
	c = Serial.read();
	rtcBuf[6] = (c & ~0x30) << 4;
	while(!Serial.available())
	  ;
	c = Serial.read();
	rtcBuf[6] |= (c & ~0x30);
	while(!Serial.available())
	  ;
	c = Serial.read();
	/* discard - */
	while(!Serial.available())
	  ;
	c = Serial.read();
	rtcBuf[5] = (c & ~0x30) << 4;
	while(!Serial.available())
	  ;
	c = Serial.read();
	rtcBuf[5] |= (c & ~0x30);
	rtcBuf[5] |= 0x80;
	while(!Serial.available())
	  ;
	c = Serial.read();
	/* discard - */
	while(!Serial.available())
	  ;
	c = Serial.read();
	rtcBuf[4] = (c & ~0x30) << 4;
	while(!Serial.available())
	  ;
	c = Serial.read();
	rtcBuf[4] |= (c & ~0x30);
	while(!Serial.available())
	  ;
	c = Serial.read();
	/* discard T */
	while(!Serial.available())
	  ;
	c = Serial.read();
	rtcBuf[2] = (c & ~0x30) << 4;
	while(!Serial.available())
	  ;
	c = Serial.read();
	rtcBuf[2] |= (c & ~0x30);
	while(!Serial.available())
	  ;
	c = Serial.read();
	/* discard : */
	while(!Serial.available())
	  ;
	c = Serial.read();
	rtcBuf[1] = (c & ~0x30) << 4;
	while(!Serial.available())
	  ;
	c = Serial.read();
	rtcBuf[1] |= (c & ~0x30);
	while(!Serial.available())
	  ;
	c = Serial.read();
	/* discard : */
	while(!Serial.available())
	  ;
	c = Serial.read();
	rtcBuf[0] = (c & ~0x30) << 4;
	while(!Serial.available())
	  ;
	c = Serial.read();
	rtcBuf[0] |= (c & ~0x30);
	while(!Serial.available())
	  ;
	c = Serial.read();
	/* discard '\n' */
	rtcBuf[3] = 1; /* day of week (1-7) */
	Serial.print("  Set RTC = 20");
	for(i = 6; i >= 0; i--) {
	  Serial.print(rtcBuf[i], HEX);
	  Serial.print(" ");
	}
	Serial.println("");
	Serial.println("  Updating RTC...");
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
  byte i;
  byte present = 0;
  byte type_s, type_19;
  byte data[12];
  //byte addr[8];
  float celsius, fahrenheit;
  
  if(Serial.available()) {
    parseSerial(Serial.read());
  }

  if ( !ds.search(addr)) {
    Serial.println("No more addresses.");
    Serial.println();
    ds.reset_search();
    delay(250);
    return;
  }
  
  Serial.print("ROM =");
  for( i = 0; i < 8; i++) {
    Serial.write(' ');
    Serial.print(addr[i], HEX);
  }

  if (OneWire::crc8(addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return;
  }
  Serial.println();
 
   type_19 = 0;
  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      Serial.println("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
      Serial.println("  Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
      Serial.println("  Chip = DS1822");
      type_s = 0;
      break;
    case 0x21:
      Serial.println("  Chip = DS1921G");
      type_19 = 1;
      break;
    default:
      Serial.println("Device is not a DS18x20 family device.");
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
    ds.write(0x00);
    ds.write(0x02);
    Serial.print("  Clock =");
    for(i = 0x00; i <= 0x0A; i++) {
      Serial.print(" ");
      Serial.print(ds.read(), HEX);
    }
    Serial.println("");
    for( ; i < 0x11; i++)
      ds.read();
    //ds.write(0x11);
    //ds.write(0x02);
    data[0] = ds.read();
    Serial.print("  Data = ");
    Serial.print(present,HEX);
    Serial.print(" ");
    Serial.println(data[0], HEX);
    celsius = (float)data[0] / 2.0f - 40.0f;

    if(!writeRtc) {
      byte sampleRate;
      readDS1921(addr, DS1921_SAMPLE_REGISTER, &sampleRate, sizeof(sampleRate));
      Serial.print("  sample rate = ");
      Serial.println(sampleRate, DEC);
      readDS1921(addr, DS1921_STATUS_REGISTER, &sampleRate, sizeof(sampleRate));
      if(sampleRate & DS1921_STATUS_MIP) {
	Serial.println("  Mission in progress.");
	writeRtc = 1;
	return;
      }
      clearDS1921(addr);
      Serial.println("  Starting a mission.");
      sampleRate = 1;
      writeDS1921(addr, DS1921_SAMPLE_REGISTER, &sampleRate, sizeof(sampleRate));
      //Serial.println("  Updating RTC...");
      //writeDS1921(addr, DS1921_RTC_REGISTER, rtc, sizeof(rtc));
      writeRtc = 1;
    }

    long count = 0;
    byte countBytes[3];

    readDS1921(addr, DS1921_DEVICE_SAMPLES_COUNTER, countBytes, sizeof(countBytes));
    count = countBytes[0] << 0 | countBytes[1] << 8 | countBytes[2] << 16;
    Serial.print("  Device samples = ");
    Serial.println(count, DEC);

    readDS1921(addr, DS1921_MISSION_SAMPLES_COUNTER, countBytes, sizeof(countBytes));
    count = countBytes[0] << 0 | countBytes[1] << 8 | countBytes[2] << 16;
    Serial.print("  Mission samples = ");
    Serial.println(count, DEC);
    readDS1921(addr, DS1921_MISSION_TIMESTAMP, rtc, 5);
    Serial.print("  Mission timestamp = 20");
    //for(i = 4; i >= 0; i--) {
    //  Serial.print(rtc[i], HEX);
    //  Serial.print(" ");
    //}
    for(i = 0; i < 5; i++) {
      Serial.print(rtc[4-i], HEX);
      Serial.print(" ");
    }
    Serial.println("");
    readDS1921(addr, DS1921_DATA_LOG, NULL, 0);
    Serial.println("  Data log:");
    if(count > 2048)
	    count = 2048;
    count = 4;
    while(count-- > 0) {
      Serial.print("    ");
      Serial.println(ds.read(), DEC);
    }
    Serial.println("");
  } else {
    ds.write(0xBE);         // Read Scratchpad

  Serial.print("  Data = ");
  Serial.print(present,HEX);
  Serial.print(" ");
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.print(" CRC=");
  Serial.print(OneWire::crc8(data, 8), HEX);
  Serial.println();

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
  fahrenheit = celsius * 1.8 + 32.0;
  Serial.print("  ATemperature = ");
  Serial.print(celsius);
  Serial.print(" Celsius, ");
  Serial.print(fahrenheit);
  Serial.println(" Fahrenheit");
}
