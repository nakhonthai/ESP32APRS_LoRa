/*
 * lvgl_ui.cpp - LVGL 8.3 display/touch port for the two CYD variants.
 *
 *  ESP32_3248S035 (CYD 3.5", 320x480 panel)
 *      TFT_eSPI ST7796_DRIVER, rotation 3 -> 480x320 landscape
 *      MISO 12 / MOSI 13 / SCLK 14 / CS 15 / DC 2 / RST -1 / BL 27
 *      Touch: GT911 capacitive on I2C (SDA 33 / SCL 32 / RST 25, addr 0x5D)
 *      Everything copied from the working NAWS_Display reference firmware.
 *
 *  ESP32_2432S028 (CYD 2.8", 240x320 panel)  - default
 *      TFT_eSPI ILI9341_2_DRIVER, rotation 1 -> 320x240 landscape
 *      MISO 12 / MOSI 13 / SCLK 14 / CS 15 / DC 2 / RST -1 / BL 21
 *      Touch: XPT2046 on its own SPI bus (CLK 25 / MOSI 32 / MISO 39 /
 *      CS 33 / IRQ 36), never through TFT_eSPI.
 *
 * The SquareLine UI was authored on a 480x320 canvas; the generated files in
 * this folder are scaled by 2/3, which suits the 2.8" panel directly.
 */
#ifdef USE_LVGL_UI

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <lvgl.h>
#include <TFT_eSPI.h>

#if !defined(ESP32_3248S035)
#include <XPT2046_Touchscreen.h>
#endif

#include "lvgl_ui.h"
#include "ui.h"
#include "ui_bind.h"

/* ------------------------------------------------------------------ */
/* board profile                                                       */
/* ------------------------------------------------------------------ */
#if defined(ESP32_3248S035)

#define SCR_W 480
#define SCR_H 320
#define SCR_ROTATION 3 /* same as the NAWS_Display reference firmware */

/* GT911 capacitive touch (NAWS_Display pinout) */
#ifndef GT911_SDA
#define GT911_SDA 33
#endif
#ifndef GT911_SCL
#define GT911_SCL 32
#endif
#ifndef GT911_RST
#define GT911_RST 25
#endif
#define GT911_ADDR 0x5D

#else /* ESP32_2432S028 and any other CYD variant */

#define SCR_W 320
#define SCR_H 240
#define SCR_ROTATION 1

#ifndef TFT_BL
#define TFT_BL 21
#endif

/* ---- XPT2046 raw range (tune from the [TOUCH] log) ---- */
#ifndef TOUCH_RAW_X_MIN
#define TOUCH_RAW_X_MIN 300
#endif
#ifndef TOUCH_RAW_X_MAX
#define TOUCH_RAW_X_MAX 3800
#endif
#ifndef TOUCH_RAW_Y_MIN
#define TOUCH_RAW_Y_MIN 300
#endif
#ifndef TOUCH_RAW_Y_MAX
#define TOUCH_RAW_Y_MAX 3800
#endif
#ifndef TOUCH_INVERT_X
#define TOUCH_INVERT_X 0
#endif
#ifndef TOUCH_INVERT_Y
#define TOUCH_INVERT_Y 0
#endif
#ifndef TOUCH_ROTATION
#define TOUCH_ROTATION 1
#endif

#endif /* board profile */

#ifndef TOUCH_DEBUG
#define TOUCH_DEBUG 1
#endif

static TFT_eSPI tft;

#if !defined(ESP32_3248S035)
static XPT2046_Touchscreen ts(TOUCH_CS, TOUCH_IRQ);
#endif

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf1[SCR_W * 10];

/* ------------------------------------------------------------------ */
/* display                                                             */
/* ------------------------------------------------------------------ */
static void disp_flush(lv_disp_drv_t *drv, const lv_area_t *a, lv_color_t *c)
{
    uint32_t w = a->x2 - a->x1 + 1;
    uint32_t h = a->y2 - a->y1 + 1;
    tft.startWrite();
    tft.setAddrWindow(a->x1, a->y1, w, h);
    tft.pushColors((uint16_t *)&c->full, w * h, true);
    tft.endWrite();
    lv_disp_flush_ready(drv);
}

/* ------------------------------------------------------------------ */
/* touch: GT911 over I2C (CYD 3.5")                                    */
/* ------------------------------------------------------------------ */
#if defined(ESP32_3248S035)

static bool gt911_write(uint16_t reg, const uint8_t *buf, uint8_t len)
{
    Wire.beginTransmission(GT911_ADDR);
    Wire.write((uint8_t)(reg >> 8));
    Wire.write((uint8_t)(reg & 0xFF));
    for (uint8_t i = 0; i < len; i++)
        Wire.write(buf[i]);
    return Wire.endTransmission() == 0;
}

static bool gt911_read(uint16_t reg, uint8_t *buf, uint8_t len)
{
    Wire.beginTransmission(GT911_ADDR);
    Wire.write((uint8_t)(reg >> 8));
    Wire.write((uint8_t)(reg & 0xFF));
    if (Wire.endTransmission(false) != 0)
        return false;
    if (Wire.requestFrom((uint8_t)GT911_ADDR, len) != len)
        return false;
    for (uint8_t i = 0; i < len; i++)
        buf[i] = Wire.read();
    return true;
}

static void gt911_begin(void)
{
    pinMode(GT911_RST, OUTPUT);
    digitalWrite(GT911_RST, LOW);
    delay(10);
    digitalWrite(GT911_RST, HIGH);
    delay(50);

    Wire.begin(GT911_SDA, GT911_SCL, 400000);
    Wire.setClock(400000);

    uint8_t id[4] = {0, 0, 0, 0};
    if (gt911_read(0x8140, id, sizeof(id)))
        log_d("GT911 touch id %02X %02X %02X %02X", id[0], id[1], id[2], id[3]);
    else
        log_e("GT911 not answering on SDA %d / SCL %d", GT911_SDA, GT911_SCL);

    uint8_t clear = 0;
    gt911_write(0x814E, &clear, 1);
}

/* panel reports X 0..320 / Y 0..480; map like the reference firmware */
static bool gt911_point(uint16_t *sx, uint16_t *sy)
{
    uint8_t status = 0;
    if (!gt911_read(0x814E, &status, 1))
        return false;

    if ((status & 0x80) == 0)
        return false; /* no new sample */

    uint8_t count = status & 0x0F;
    uint8_t clear = 0;
    if (count == 0 || count > 5)
    {
        gt911_write(0x814E, &clear, 1);
        return false;
    }

    uint8_t buf[8] = {0};
    bool ok = gt911_read(0x814F, buf, sizeof(buf));
    gt911_write(0x814E, &clear, 1);
    if (!ok)
        return false;

    uint16_t px = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);
    uint16_t py = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);

    long x = map((long)py, 0, 480, 480, 0);
    long y = map((long)px, 0, 320, 0, 320);
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    if (x > SCR_W - 1) x = SCR_W - 1;
    if (y > SCR_H - 1) y = SCR_H - 1;

    *sx = (uint16_t)x;
    *sy = (uint16_t)y;

#if TOUCH_DEBUG
    static uint32_t lastLog = 0;
    if (millis() - lastLog > 400)
    {
        lastLog = millis();
        log_d("TOUCH gt911 raw=%u,%u -> %d,%d", px, py, (int)x, (int)y);
    }
#endif
    return true;
}

#else /* XPT2046 on the 2.8" board */

static bool xpt_point(uint16_t *sx, uint16_t *sy)
{
    if (!ts.touched())
        return false;

    TS_Point p = ts.getPoint();
    long x = map((long)p.x, TOUCH_RAW_X_MIN, TOUCH_RAW_X_MAX, 0, SCR_W - 1);
    long y = map((long)p.y, TOUCH_RAW_Y_MIN, TOUCH_RAW_Y_MAX, 0, SCR_H - 1);
#if TOUCH_INVERT_X
    x = (SCR_W - 1) - x;
#endif
#if TOUCH_INVERT_Y
    y = (SCR_H - 1) - y;
#endif
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    if (x > SCR_W - 1) x = SCR_W - 1;
    if (y > SCR_H - 1) y = SCR_H - 1;

    *sx = (uint16_t)x;
    *sy = (uint16_t)y;

#if TOUCH_DEBUG
    static uint32_t lastLog = 0;
    if (millis() - lastLog > 400)
    {
        lastLog = millis();
        log_d("TOUCH xpt raw=%d,%d z=%d -> %d,%d", p.x, p.y, p.z, (int)x, (int)y);
    }
#endif
    return true;
}

#endif /* board profile */

static void touch_read(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    static uint16_t lastX = SCR_W / 2, lastY = SCR_H / 2;
    uint16_t x = 0, y = 0;

#if defined(ESP32_3248S035)
    bool pressed = gt911_point(&x, &y);
#else
    bool pressed = xpt_point(&x, &y);
#endif

    if (pressed)
    {
        lastX = x;
        lastY = y;
        data->state = LV_INDEV_STATE_PR;
    }
    else
    {
        data->state = LV_INDEV_STATE_REL;
    }

    data->point.x = lastX;
    data->point.y = lastY;
}

/* ------------------------------------------------------------------ */
/* init / loop                                                         */
/* ------------------------------------------------------------------ */
void lvgl_ui_init()
{
    log_d("LVGL: init start, heap %u", (unsigned)ESP.getFreeHeap());

#if defined(ESP32_3248S035)
    /* the reference firmware drives GPIO21/22 before the panel comes up */
    pinMode(21, OUTPUT);
    pinMode(22, OUTPUT);
    digitalWrite(21, HIGH);
    digitalWrite(22, LOW);
#endif

#ifdef TFT_BL
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);
#endif

    log_d("LVGL: tft.begin() ...");
    tft.begin();
    log_d("LVGL: tft.begin() ok");
    tft.setRotation(SCR_ROTATION);
    tft.fillScreen(TFT_BLACK);
#ifdef LCD_INVERT_COLORS
    tft.invertDisplay(true);
    log_d("LVGL: colour inversion on");
#endif
    log_d("LVGL: tft ready %dx%d rot %d", SCR_W, SCR_H, SCR_ROTATION);

#if defined(ESP32_3248S035)
    gt911_begin();
#else
    pinMode(TOUCH_CS, OUTPUT);
    digitalWrite(TOUCH_CS, HIGH);
    SPI.begin(TOUCH_CLK, TOUCH_MISO, TOUCH_MOSI, TOUCH_CS);
    ts.begin();
    ts.setRotation(TOUCH_ROTATION);
#endif
    log_d("LVGL: touch ready");

    log_d("LVGL: lv_init() ...");
    lv_init();
    log_d("LVGL: lv_init() ok");

    lv_disp_draw_buf_init(&draw_buf, buf1, NULL, SCR_W * 10);

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = SCR_W;
    disp_drv.ver_res = SCR_H;
    disp_drv.flush_cb = disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = touch_read;
    lv_indev_drv_register(&indev_drv);
    log_d("LVGL: drivers registered");

    log_d("LVGL: ui_init() ...");
    ui_init();
    log_d("LVGL: ui_init done, heap %u", (unsigned)ESP.getFreeHeap());

    log_d("LVGL: ui_bind_init() ...");
    ui_bind_init();
    log_d("LVGL: ui_bind done, heap %u", (unsigned)ESP.getFreeHeap());
}

void lvgl_ui_loop()
{
    lv_timer_handler();
    ui_bind_tick();
}

#endif /* USE_LVGL_UI */
