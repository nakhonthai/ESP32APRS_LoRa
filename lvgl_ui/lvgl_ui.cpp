#ifdef USE_LVGL_UI
#include <Arduino.h>
#include <SPI.h>
#include <lvgl.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
#include "lvgl_ui.h"

#define TOUCH_CLK  25
#define TOUCH_MOSI 32
#define TOUCH_MISO 39
#define TOUCH_CS   33
#define TOUCH_IRQ  36

static const uint16_t SCR_W = 320;   // แนวนอน
static const uint16_t SCR_H = 240;

static TFT_eSPI tft;
static SPIClass touchSPI(VSPI);
static XPT2046_Touchscreen ts(TOUCH_CS, TOUCH_IRQ);

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf1[SCR_W * 20];

static void disp_flush(lv_disp_drv_t *drv, const lv_area_t *a, lv_color_t *c) {
  uint32_t w = a->x2 - a->x1 + 1;
  uint32_t h = a->y2 - a->y1 + 1;
  tft.startWrite();
  tft.setAddrWindow(a->x1, a->y1, w, h);
  tft.pushColors((uint16_t *)&c->full, w * h, true);
  tft.endWrite();
  lv_disp_flush_ready(drv);
}

static void touch_read(lv_indev_drv_t *drv, lv_indev_data_t *data) {
  if (ts.tirqTouched() && ts.touched()) {
    TS_Point p = ts.getPoint();
    // ค่า raw ประมาณ 200-3700 / 240-3800 ปรับสอบเทียบตามจอจริง
    int x = map(p.x, 200, 3700, 0, SCR_W);
    int y = map(p.y, 240, 3800, 0, SCR_H);
    data->point.x = constrain(x, 0, SCR_W - 1);
    data->point.y = constrain(y, 0, SCR_H - 1);
    data->state = LV_INDEV_STATE_PR;
  } else {
    data->state = LV_INDEV_STATE_REL;
  }
}

void lvgl_ui_init() {
  lv_init();

  tft.begin();
  tft.setRotation(1);

  touchSPI.begin(TOUCH_CLK, TOUCH_MISO, TOUCH_MOSI, TOUCH_CS);
  ts.begin(touchSPI);
  ts.setRotation(1);

  lv_disp_draw_buf_init(&draw_buf, buf1, NULL, SCR_W * 20);

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

  // ทดสอบหน้าจอ
  lv_obj_t *label = lv_label_create(lv_scr_act());
  lv_label_set_text(label, "ESP32 APRS LoRa\nLVGL on CYD");
  lv_obj_center(label);
}

void lvgl_ui_loop() {
  lv_timer_handler();
}
#endif