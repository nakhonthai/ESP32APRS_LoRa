/*
  Minimal PCF85063 / PCF85063A RTC driver using the Arduino Wire library.
*/

#include "pcf85063.h"

#ifdef PCF85063

#include <sys/time.h>
#include <TimeLib.h>

#define PCF85063_REG_CONTROL1 0x00
#define PCF85063_REG_SECONDS 0x02
#define PCF85063_REG_MINUTES 0x03
#define PCF85063_REG_HOURS 0x04
#define PCF85063_REG_DAYS 0x05
#define PCF85063_REG_DATE 0x06
#define PCF85063_REG_MONTHS 0x07
#define PCF85063_REG_YEARS 0x08

#define PCF85063_CTRL1_STOP 0x20
#define PCF85063_SEC_OS 0x80
#define PCF85063_HOUR_12H 0x40
#define PCF85063_HOUR_PM 0x20

extern SemaphoreHandle_t i2c0_mutex;

static TwoWire *rtcWire = &Wire;

static inline uint8_t dec2bcd(uint8_t v)
{
    return (uint8_t)(((v / 10) << 4) | (v % 10));
}

static inline uint8_t bcd2dec(uint8_t v)
{
    return (uint8_t)(((v >> 4) * 10) + (v & 0x0F));
}

// The RTC shares the PMU/sensor I2C bus, so serialize access with i2c0_mutex
static bool busLock()
{
    if (i2c0_mutex == NULL)
        return true;
    return xSemaphoreTake(i2c0_mutex, pdMS_TO_TICKS(200)) == pdTRUE;
}

static void busUnlock()
{
    if (i2c0_mutex != NULL)
        xSemaphoreGive(i2c0_mutex);
}

static bool writeRegs(uint8_t reg, const uint8_t *data, size_t len)
{
    rtcWire->beginTransmission(PCF85063_I2C_ADDRESS);
    rtcWire->write(reg);
    for (size_t i = 0; i < len; i++)
        rtcWire->write(data[i]);
    return rtcWire->endTransmission() == 0;
}

static bool readRegs(uint8_t reg, uint8_t *data, size_t len)
{
    rtcWire->beginTransmission(PCF85063_I2C_ADDRESS);
    rtcWire->write(reg);
    if (rtcWire->endTransmission(false) != 0)
        return false;
    if (rtcWire->requestFrom((uint8_t)PCF85063_I2C_ADDRESS, (uint8_t)len) != len)
        return false;
    for (size_t i = 0; i < len; i++)
        data[i] = rtcWire->read();
    return true;
}

bool pcf85063Begin(TwoWire &wire)
{
    rtcWire = &wire;

    uint8_t ctrl1 = 0;
    if (!busLock())
        return false;
    bool ok = readRegs(PCF85063_REG_CONTROL1, &ctrl1, 1);
    if (ok && (ctrl1 & PCF85063_CTRL1_STOP))
    {
        ctrl1 &= ~PCF85063_CTRL1_STOP;
        ok = writeRegs(PCF85063_REG_CONTROL1, &ctrl1, 1);
    }
    busUnlock();

    if (!ok)
        log_e("PCF85063 not found on I2C bus");
    return ok;
}

bool pcf85063ReadTime(time_t &utc)
{
    uint8_t b[7];
    if (!busLock())
        return false;
    bool ok = readRegs(PCF85063_REG_SECONDS, b, 7);
    busUnlock();
    if (!ok)
        return false;

    // Bit 7 of the seconds register is set when the oscillator has stopped,
    // which means the stored time is no longer valid.
    if (b[0] & PCF85063_SEC_OS)
        return false;

    tmElements_t tm;
    tm.Second = bcd2dec(b[0] & 0x7F);
    tm.Minute = bcd2dec(b[1] & 0x7F);
    if (b[2] & PCF85063_HOUR_12H)
    {
        tm.Hour = bcd2dec(b[2] & 0x1F);
        if (tm.Hour == 12)
            tm.Hour = 0;
        if (b[2] & PCF85063_HOUR_PM)
            tm.Hour += 12;
    }
    else
    {
        tm.Hour = bcd2dec(b[2] & 0x3F);
    }
    tm.Wday = (b[3] & 0x07) + 1; // TimeLib: Sunday = 1
    tm.Day = bcd2dec(b[4] & 0x3F);
    tm.Month = bcd2dec(b[5] & 0x1F);
    uint8_t year = bcd2dec(b[6]);

    if (tm.Month < 1 || tm.Month > 12 || tm.Day < 1 || tm.Day > 31 ||
        tm.Hour > 23 || tm.Minute > 59 || tm.Second > 59)
        return false;

    tm.Year = CalendarYrToTm(2000 + year);
    utc = makeTime(tm);
    return utc > 0;
}

bool pcf85063WriteTime(time_t utc)
{
    tmElements_t tm;
    breakTime(utc, tm);
    int year = tmYearToCalendar(tm.Year);
    if (year < 2020 || year > 2099)
        return false;

    uint8_t b[7];
    b[0] = dec2bcd(tm.Second); // writing seconds also clears the OS flag
    b[1] = dec2bcd(tm.Minute);
    b[2] = dec2bcd(tm.Hour); // force 24-hour mode
    b[3] = tm.Wday - 1;      // PCF85063: 0 = Sunday
    b[4] = dec2bcd(tm.Day);
    b[5] = dec2bcd(tm.Month);
    b[6] = dec2bcd(year % 100);

    if (!busLock())
        return false;
    bool ok = writeRegs(PCF85063_REG_SECONDS, b, 7);
    busUnlock();
    return ok;
}

bool pcf85063LoadSystemTime()
{
    time_t utc;
    if (!pcf85063ReadTime(utc))
        return false;

    timeval tv = {utc, 0};
    settimeofday(&tv, NULL);
    setTime(utc);
    return true;
}

bool pcf85063SaveSystemTime()
{
    time_t utc = time(NULL);
    if (utc < 1700000000) // system clock not synced yet
        return false;
    return pcf85063WriteTime(utc);
}

#endif // PCF85063
