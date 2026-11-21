/*
  Minimal PCF85063 / PCF85063A RTC driver using the Arduino Wire library.
  The RTC always holds UTC (seconds since 1970); conversion to local time is
  left to the caller / the ESP32 system clock.
*/

#ifndef PCF85063_H
#define PCF85063_H

#include <Arduino.h>
#include <Wire.h>
#include <time.h>

#define PCF85063_I2C_ADDRESS 0x51

bool pcf85063Begin(TwoWire &wire = Wire);
bool pcf85063ReadTime(time_t &utc);
bool pcf85063WriteTime(time_t utc);

// Copy RTC -> system clock (settimeofday) and TimeLib
bool pcf85063LoadSystemTime();
// Copy system clock (UTC) -> RTC
bool pcf85063SaveSystemTime();

#endif
